"""Tone-power sweeps: acquisition, fitting, power selection, and balancing.

Workflow
--------
1. Acquire a run (hardware attached), or reload a previous one::

       from souk_readout_tools import power_sweep as ps

       run = ps.run_power_sweep(client, centers, spans, powers_dbm,
                                output_dir="kid_power_sweep")
       run = ps.load_power_sweep("kid_power_sweep")

2. Analyse it — fit every tone at every power, write the fit archive and
   CSV summary, choose a best readout power per tone, render the plots —
   or reload all of that from disk without recomputing anything::

       analysis = ps.analyse_power_sweep(run, n_jobs=-1)
       analysis = ps.load_analysis("kid_power_sweep")

       arrays = ps.best_power_arrays(analysis["best_power"])

3. Optionally balance the comb — re-allocate the per-tone powers under
   bifurcation caps and TX/RX spread constraints::

       balanced = ps.balance_tone_powers(analysis["best_power"])
       ps.write_balanced_power(balanced, "kid_power_sweep")

Pieces
------
The procedures above are assembled from these, all callable directly:

======================  =====================================================
fit_power_sweep         fit every tone at every power -> ``fit_data``
parameter_series        one tone's fitted parameters vs power, as 1D arrays
fit_summary_array       ``fit_data`` as one flat structured table
write_fit_results /     the fit archive (``analysis/fit_results.pkl``)
load_fit_results
write_fit_summary /     the flat table (``analysis/fit_summary.csv``)
load_fit_summary
find_best_power         choose a readout power per tone from the summary
best_power_arrays       best-power rows -> per-tone arrays
write_best_power /      the chosen powers (``analysis/best_power.json``)
load_best_power
balance_tone_powers     re-allocation under bifurcation/TX/RX constraints
balanced_power_arrays   balanced result -> per-tone arrays
write_balanced_power /  the balanced powers (``analysis/balanced_power.json``)
load_balanced_power
accumulator_level_db    per-tone RX levels (``rx_offsets_db`` for balancing)
plot_power_sweep        per-tone fit overlays and parameter-vs-power PNGs
plot_best_power         per-tone ANL-vs-power selection diagnostics
======================  =====================================================

On disk a run directory holds ``measurement.json`` (the manifest),
``data/`` (one sweep npz per power step), ``analysis/`` (the four artifact
files above), and ``plots/``.
"""

from __future__ import annotations

import csv
import json
import pickle
import textwrap
import time
import traceback
import warnings
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import replace
from pathlib import Path

import numpy as np

from .fitting import (
    FIT_SUMMARY_KEYS,
    FIT_UNCERTAINTY_KEYS,
    FitResult,
    batch_fit,
    extract_parameters,
    fit_result_summary_row,
    fit_sweep_stack,
    _resolve_n_jobs,
)
from .plotting import plot_fits, plot_sweep
from .plotting._common import (
    _apply_compact_scientific_ticks,
    _validate_reference_plane,
)
from .resonator import estimate_resonance_empirical
from .measurement import (
    MANIFEST_FILE,
    _format_duration,
    _jsonify,
    _load_manifest,
    _load_npz,
    _save_npz,
    _timestamp,
    _write_manifest,
)


SCHEMA_VERSION = 1
FIT_SUMMARY_FILE = "analysis/fit_summary.csv"
FIT_RESULTS_FILE = "analysis/fit_results.pkl"
BEST_POWER_FILE = "analysis/best_power.json"
BALANCED_POWER_FILE = "analysis/balanced_power.json"

SUMMARY_KEYS = FIT_SUMMARY_KEYS
UNCERTAINTY_KEYS = FIT_UNCERTAINTY_KEYS

# Fit-summary keys that ``find_best_power`` interpolates against power_dbm
# to estimate parameter-like values at the chosen readout power. Booleans,
# solver diagnostics, timing, and fit-quality diagnostics are excluded because
# interpolating them is not meaningful.
_INTERPOLATABLE_FIT_KEYS = tuple(
    k for k in FIT_SUMMARY_KEYS
    if k not in {
        "success",
        "nfev",
        "fit_duration_s",
        "noise_only",
        "residual_rms",
        "weighted_rms",
        "reduced_chi2",
    }
)
_CHOSEN_PARAM_ARRAY_KEYS = ("power_dbm",) + _INTERPOLATABLE_FIT_KEYS + (
    "extrapolated",
    "n_rows_used",
    "fr_source",
)

# Chosen-parameter keys that are physically non-negative. ``find_best_power``
# interpolates/extrapolates each fit-summary field against power, and linear
# extrapolation past the measured range can drive a positive-but-decreasing
# quantity (e.g. a linewidth or Q) negative. ``best_power_arrays`` replaces any
# such negative entry with the cross-tone median of the valid values for that
# key. Signed quantities (skew, nonlinear detuning, coupling/gain phases) are
# deliberately excluded because negatives are legitimate there.
_NONNEGATIVE_PARAM_KEYS = tuple(
    k for k in (
        "Ql", "Qi", "Qc", "Qc_abs", "a", "anl",
        "empirical_linewidth_hz", "empirical_Ql", "empirical_Qc", "empirical_Qi",
        "empirical_dip_depth_db",
    )
    if k in _CHOSEN_PARAM_ARRAY_KEYS
)

# Default per-parameter validity ranges used by ``find_best_power`` to exclude
# rows whose fit pegged a parameter at (or outside) a fitter bound. Mirrors the
# static parts of the broad physical defaults in :mod:`souk_readout_tools.fitting`.
# Data-dependent fitter bounds (``fr``, ``a``, ``tau``) are intentionally omitted
# — callers can supply their own (low, high) entries for those if needed. ``phi``
# and ``alpha`` are tightened by 1e-4 so values pinned at the fitter's
# ``±π/2 - eps`` / ``±π`` clamps are caught. ``anl``'s lower bound (1e-6) is
# loose-by-design: anything that many orders of magnitude below the fitter's
# 1e-8 log clamp is indistinguishable from zero in practice.
DEFAULT_PARAM_VALID_RANGES = {
    "Qi": (10.0, 1e8),
    "Qc": (10.0, 1e8),
    "phi": (-np.pi / 2.0 + 1e-4, np.pi / 2.0 - 1e-4),
    "alpha": (-np.pi + 1e-4, np.pi - 1e-4),
    "anl": (1e-6, 90.0),
}


def _normalise_param_valid_ranges(param_valid_ranges):
    """Validate and normalise a param-range dict from the public API."""
    if param_valid_ranges is None:
        return dict(DEFAULT_PARAM_VALID_RANGES)
    if not isinstance(param_valid_ranges, dict):
        raise TypeError("param_valid_ranges must be a dict of {name: (low, high)}")
    out = {}
    for name, pair in param_valid_ranges.items():
        if pair is None:
            continue
        try:
            lo, hi = pair
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"param_valid_ranges[{name!r}] must be a (low, high) pair"
            ) from exc
        lo = -np.inf if lo is None else float(lo)
        hi = np.inf if hi is None else float(hi)
        if not (lo < hi):
            raise ValueError(
                f"param_valid_ranges[{name!r}] must satisfy low < high"
            )
        out[str(name)] = (lo, hi)
    return out

__all__ = [
    # On-disk artifact locations
    "MANIFEST_FILE",
    "FIT_RESULTS_FILE",
    "FIT_SUMMARY_FILE",
    "BEST_POWER_FILE",
    "BALANCED_POWER_FILE",
    # Whole procedures
    "run_power_sweep",
    "load_power_sweep",
    "analyse_power_sweep",
    "load_analysis",
    # Fitting and the fit-summary table
    "fit_power_sweep",
    "parameter_series",
    "fit_summary_array",
    "write_fit_results",
    "load_fit_results",
    "write_fit_summary",
    "load_fit_summary",
    # Best readout power per tone
    "find_best_power",
    "best_power_arrays",
    "write_best_power",
    "load_best_power",
    # Comb balancing
    "balance_tone_powers",
    "balanced_power_arrays",
    "write_balanced_power",
    "load_balanced_power",
    "accumulator_level_db",
    # Plotting
    "plot_power_sweep",
    "plot_best_power",
]


def _format_power_summary(powers):
    powers = np.asarray(powers, dtype=float).ravel()
    tone_word = "tone" if powers.size == 1 else "tones"
    if powers.size == 1 or np.allclose(powers, powers[0], atol=1e-9, rtol=0.0):
        return f"{powers[0]:.1f} dBm ({powers.size} {tone_word})"
    return (
        f"{np.nanmin(powers):.1f} to {np.nanmax(powers):.1f} dBm "
        f"({powers.size} {tone_word})"
    )


def _progress(verbose, message):
    if verbose:
        print(message, flush=True)


def _verbose_level(verbose):
    """Normalize bool/int verbose values."""
    if verbose is None:
        return 0
    if isinstance(verbose, bool):
        return 1 if verbose else 0
    return int(verbose)


def _print_progress_line(message, final=False):
    """Print progress without filling the terminal with one line per item."""
    print(message, end="\n" if final else "\r", flush=True)


def _progress_report_every(total):
    """Return an update cadence that gives roughly ten progress lines."""
    return max(1, int(np.ceil(max(int(total), 1) / 10.0)))


def _progress_should_report(completed, total, report_every):
    """Return True when a compact progress line should be emitted."""
    return (
        completed == 1
        or completed == int(total)
        or completed % int(report_every) == 0
    )


def _format_plot_progress(
    label, completed, total, elapsed, final=False, detail=""
):
    """Return a compact plotting progress/summary line."""
    verb = "Finished" if final else "Plotting"
    completed = int(completed)
    total = int(total)
    elapsed = float(elapsed)
    rate = completed / elapsed if elapsed > 0.0 else np.nan
    remaining = (
        (total - completed) / rate
        if np.isfinite(rate) and rate > 0.0 and completed < total
        else 0.0
    )
    message = (
        f"{verb} {completed}/{max(total, 1)} {label} "
        f"({100.0 * completed / max(total, 1):.0f}%), "
        f"elapsed={_format_duration(elapsed)}, "
        f"rate={rate:.2f}/s"
    )
    if not final and completed < total:
        message += f", remaining={_format_duration(remaining)}"
    if detail:
        message += f", {detail}"
    return message


def _find_dip_centers(sweep_data, centers, spans, follow_min_depth_db):
    """Empirical-dip recentering used by ``run_power_sweep``.

    Each regular (non-blind) tone is recentered on the deepest dip in its full
    sweep trace.  A dip shallower than ``follow_min_depth_db`` is rejected so a
    tone never locks onto noise.  Finally, since tones are ordered by frequency,
    any neighbouring pair whose new centers would swap order (both chasing the
    same resonance) is reverted to its previous centers.

    Returns ``(next_centers, record)`` where ``record`` is the per-tone
    diagnostic dict (candidate frequencies, depths, accept flags, reasons,
    next centers, and the blind-tone indices that were skipped).
    """
    sweep_f = np.atleast_2d(np.asarray(sweep_data["sweep_f"], dtype=float))
    sweep_i = np.atleast_2d(np.asarray(sweep_data["sweep_i"], dtype=float))
    sweep_q = np.atleast_2d(np.asarray(sweep_data["sweep_q"], dtype=float))
    centers = np.asarray(centers, dtype=float).ravel()
    spans = np.asarray(spans, dtype=float).ravel()
    if spans.size == 1:
        spans = np.full(centers.size, spans[0], dtype=float)
    if spans.size != centers.size:
        raise ValueError("spans must be scalar or have one value per tone.")
    tones = (sweep_data.get("tone_metadata") or {}).get("blind_indices")
    if tones is None:
        tones = ((sweep_data.get("info") or {}).get("tones") or {}).get("blind_indices", [])
    blind = {int(i) for i in tones if 0 <= int(i) < centers.size}
    accepted = np.zeros(centers.size, dtype=bool)
    candidates = np.full(centers.size, np.nan)
    depths = np.full(centers.size, np.nan)
    reasons = ["blind_tone" if i in blind else "not_found" for i in range(centers.size)]

    for tone_index in range(centers.size):
        if tone_index in blind or tone_index >= sweep_f.shape[1]:
            continue
        f = sweep_f[:, tone_index]
        z = sweep_i[:, tone_index] + 1j * sweep_q[:, tone_index]
        good = np.isfinite(f) & np.isfinite(z.real) & np.isfinite(z.imag)
        if np.count_nonzero(good) < 3:
            reasons[tone_index] = "too_few_points"
            continue

        f = f[good]
        z = z[good]
        order = np.argsort(f)
        f = f[order]
        z = z[order]
        # Recenter on the deepest point of the whole trace.
        log_mag = 20.0 * np.log10(np.maximum(np.abs(z), 1e-300))
        estimate = estimate_resonance_empirical(f, z, peak_index=int(np.argmin(log_mag)))
        if not np.isfinite(estimate.fr):
            continue
        candidates[tone_index] = estimate.fr
        depths[tone_index] = estimate.dip_depth_db
        if follow_min_depth_db is not None:
            min_depth = float(follow_min_depth_db)
            if (
                not np.isfinite(estimate.dip_depth_db)
                or estimate.dip_depth_db < min_depth
            ):
                reasons[tone_index] = "dip_too_shallow"
                continue
        accepted[tone_index] = True
        reasons[tone_index] = "accepted"

    next_centers = centers.copy()
    next_centers[accepted] = candidates[accepted]
    # Tones are ordered by frequency; if recentering swapped any neighbouring
    # pair (both chasing the same resonance), revert both to their old centers.
    for left, right in zip(np.argsort(centers)[:-1], np.argsort(centers)[1:]):
        if next_centers[left] >= next_centers[right]:
            accepted[left] = accepted[right] = False
            next_centers[left], next_centers[right] = centers[left], centers[right]
            reasons[left] = f"conflict_with_tone_{int(right)}"
            reasons[right] = f"conflict_with_tone_{int(left)}"

    record = {
        "candidate_centers_hz": candidates.tolist(),
        "candidate_dip_depth_db": depths.tolist(),
        "accepted": accepted.tolist(),
        "reasons": reasons,
        "next_centers_hz": next_centers.tolist(),
        "blind_indices": sorted(blind),
    }
    return next_centers, record


def _normalise_power_steps(powers_dbm, tone_count):
    """Return a list of per-tone power rows."""
    powers = np.asarray(powers_dbm, dtype=float)
    if powers.ndim == 0:
        return [np.full(tone_count, float(powers), dtype=float)]
    if powers.ndim == 1:
        return [np.full(tone_count, float(power), dtype=float) for power in powers]
    if powers.ndim == 2 and powers.shape[1] == tone_count:
        return [row.astype(float, copy=True) for row in powers]
    raise ValueError("powers_dbm must be scalar, 1D, or shaped (step, tone).")


# --- On-disk run record ------------------------------------------------------
# A power-sweep run is a directory holding ``measurement.json`` (a plain manifest
# dict listing the swept parameters and one entry per step) alongside the saved
# per-step sweep ``.npz`` files under ``data/``.  The manifest/npz helpers used
# here (``_write_manifest``, ``_load_manifest``, ``_save_npz``, ``_load_npz``,
# ``_timestamp``, ``_jsonify``) live in :mod:`souk_readout_tools.measurement` and
# are imported at the top of this module; the acquisition loop lives in
# ``run_power_sweep`` below.


def _save_system_info(manifest, root, client, label, sections):
    """Best-effort: save a ``client.get_info()`` snapshot as a JSON file in the
    run directory and record it in ``manifest``.  A client without ``get_info``
    or a failure fetching it is noted as a warning rather than raised, so it
    never aborts a run."""
    if client is None or not hasattr(client, "get_info"):
        return
    try:
        info = client.get_info(sections)
    except Exception as exc:  # best-effort only
        manifest["metadata"].setdefault("warnings", []).append(
            f"Could not capture system info {label}: {exc}"
        )
        return
    rel_path = f"system_info_{label}.json"
    with (Path(root) / rel_path).open("w", encoding="utf-8") as handle:
        json.dump(_jsonify(info), handle, indent=2)
    manifest["artifacts"].append({
        "name": f"system_info_{label}",
        "kind": "system_info",
        "path": rel_path,
        "format": "json",
        "metadata": {"label": label, "sections": sections},
        "created": _timestamp(),
    })


def _nan_placeholder_sweep(centers, spans, points, samples_per_point,
                           reference_plane, requested, reason, phases=None):
    """Build a sweep-data dict shaped like a real targeted sweep but with NaN
    I/Q, used to record a power step that was skipped or failed to be set.

    The frequency axis is the grid the sweep would have used so plots show a
    gap at the right frequencies; all magnitudes are NaN so the fitter returns
    a ``noise_only`` (failed) result rather than a fabricated value.  A
    ``skipped=True`` marker and ``skip_reason`` are carried through so the run
    view and any reloaded run can tell real data from a placeholder."""
    centers = np.asarray(centers, dtype=float).ravel()
    spans = np.asarray(spans, dtype=float).ravel()
    tones = centers.size
    pts = int(points)
    sweep_f = np.empty((pts, tones), dtype=float)
    for t in range(tones):
        sweep_f[:, t] = np.linspace(
            centers[t] - spans[t] / 2.0, centers[t] + spans[t] / 2.0, pts
        )
    nan = np.full((pts, tones), np.nan, dtype=float)
    data = {
        "date": _timestamp(),
        "num_tones": int(tones),
        "num_points": pts,
        "samples_per_point": int(samples_per_point),
        "info": {},
        "sweep_f": sweep_f,
        "sweep_i": nan.copy(),
        "sweep_q": nan.copy(),
        "sweep_ei": nan.copy(),
        "sweep_eq": nan.copy(),
        "telescope_time": np.zeros(pts, dtype=np.uint64),
        "skipped": True,
        "skip_reason": str(reason),
        "requested_tone_powers_dbm": np.asarray(requested, dtype=float).ravel().copy(),
        "readback_tone_powers_dbm": np.full(tones, np.nan),
        "tone_powers_reference_plane": reference_plane,
        "sweep_centers_hz": centers.copy(),
    }
    if phases is not None:
        data["tone_phases_rad"] = np.asarray(phases, dtype=float).ravel()
    return data


def _preflight_feasible_steps(client, power_steps, reference_plane,
                              optimise_dynamic_range, rx_policy, verbose):
    """Find which power steps the hardware can actually reach.

    Probes the most demanding step (highest peak tone power) and, if it cannot
    be set, the next most demanding, and so on, until one succeeds — that step
    fixes the maximum achievable power.  Every step whose peak power is at or
    below that ceiling is marked feasible; the rest are marked infeasible so
    :func:`run_power_sweep` can skip them (as NaN) instead of aborting.

    There is no dry-run, so each probe physically applies that step's powers to
    the hardware; the tones must already be configured (so the achievability
    check sees the right tone count).  Returns ``(feasible_mask, record)`` where
    ``feasible_mask`` has one bool per step and ``record`` captures the probe
    outcome for the manifest.  If no step is achievable, every entry is
    ``False`` (the run then records NaN for every step rather than raising)."""
    step_peak = np.array([float(np.max(step)) for step in power_steps])
    order = np.argsort(step_peak)[::-1]        # most demanding first
    feasible = np.ones(len(power_steps), dtype=bool)
    ceiling_index = None
    failures = []
    probed = set()
    for idx in order:
        peak = float(step_peak[idx])
        # Skip re-probing a peak we've already resolved (steps can share a peak).
        if any(abs(peak - step_peak[i]) <= 1e-6 for i in probed):
            continue
        probed.add(int(idx))
        _progress(
            verbose,
            f"  Pre-flight probe: peak {peak:.1f} dBm at {reference_plane}",
        )
        response = client.set_tone_powers(
            power_steps[idx], reference_plane=reference_plane,
            optimise_dynamic_range=optimise_dynamic_range,
            rx_policy=rx_policy, verbose=False,
        )
        failed = isinstance(response, dict) and response.get("status") != "success"
        if not failed:
            ceiling_index = int(idx)
            break
        message = (
            response.get("message", "set_tone_powers failed")
            if isinstance(response, dict) else "set_tone_powers failed"
        )
        failures.append({"step_index": int(idx), "peak_dbm": peak, "message": message})
        _progress(verbose, f"    infeasible: {message}")

    if ceiling_index is None:
        feasible[:] = False
        ceiling_peak = None
    else:
        ceiling_peak = float(step_peak[ceiling_index])
        feasible = step_peak <= ceiling_peak + 1e-6
    record = {
        "reference_plane": reference_plane,
        "step_peak_dbm": step_peak.tolist(),
        "feasible": feasible.tolist(),
        "ceiling_peak_dbm": ceiling_peak,
        "probe_failures": failures,
    }
    return feasible, record


def run_power_sweep(
    client,
    centers,
    spans,
    powers_dbm,
    output_dir,
    points=201,
    samples_per_point=10,
    direction="up",
    phases="newman",
    reference_plane="detector",
    optimise_dynamic_range=False,
    rx_policy="protect",
    preflight=True,
    settle_time=0.5,
    refresh_adc_cal=True,
    adc_cal_settle_time=2.0,
    file_format="npz",
    follow_dips=True,
    follow_min_depth_db=0.5,
    search_for_center=None,
    verbose=True,
    search_span_factor=2.0,
    capture_system_info=True,
    info_sections="all",
):
    """Step tone power and save each targeted sweep.

    By default the run first performs an unsaved center-search sweep at the
    first requested power, recenters the first saved sweep on the empirical
    dips, and then follows dips between power steps.  For each saved power
    step this function programs tone frequencies, sets phases, applies the
    requested powers via :py:meth:`ReadoutClient.set_tone_powers`, settles,
    runs a targeted sweep, parses the result, and exports it to disk.  The
    JSON manifest (:data:`MANIFEST_FILE`) is rewritten after every step so
    an interrupted run can still be inspected.

    Parameters
    ----------
    client : ReadoutClient
        Connected readout client.
    centers : array-like of float
        Tone center frequencies in Hz, one per tone.
    spans : float or array-like of float
        Sweep span in Hz.  A scalar broadcasts to all tones; otherwise one
        value per tone is required.
    powers_dbm : float or array-like
        Tone power schedule, in dBm at ``reference_plane``.  Three shapes
        are accepted:

        - scalar — one sweep at that common power,
        - 1D ``(n_steps,)`` — one common power per step shared across all
          tones,
        - 2D ``(n_steps, n_tones)`` — independent power per step and tone.
    output_dir : str or Path
        Directory for the manifest and per-step sweep files (created if
        missing).
    points : int, optional
        Frequency points per sweep (default ``201``).
    samples_per_point : int, optional
        Averaging samples per sweep point (default ``10``).
    direction : {'up', 'down'}, optional
        Sweep direction in frequency (default ``'up'``).
    phases : {'newman', None} or array-like, optional
        Tone phase scheme:

        - ``'newman'`` (default) — regenerate Newman phases each step from
          the current centers; minimises crest factor and is recommended
          for many-tone runs.
        - ``None`` — leave phases unset; the server will warn about zero
          phases.
        - array-like of length ``n_tones`` — fixed phases (radians)
          reused for every step.
    reference_plane : {'dac', 'rf_output', 'detector'}, optional
        Plane where ``powers_dbm`` is interpreted (default ``'detector'``).
        See :py:meth:`ReadoutClient.set_tone_powers`.
    optimise_dynamic_range : bool, optional
        If ``True``, maximise DAC bit utilisation at every power step.
        This invokes the slow path inside ``set_tone_powers`` (adjusts the
        programmable attenuator, amp bypass, and PSB scale as needed).
        Default ``False`` for speed, assuming any DAC headroom configured
        before the sweep remains valid.
    rx_policy : {'protect', 'compensate', 'maximise', 'raise', 'none'}, optional
        How the RX path is managed when TX power changes:

        - ``'protect'`` (default) — if the ADC saturates, raise RX
          attenuation/DSA to clear it and warn.
        - ``'compensate'`` — mirror TX power changes onto the RX path to
          keep round-trip power constant.
        - ``'maximise'`` — run ``maximise_rx_power()`` after the TX
          change to optimise RX attenuation, DSA, RX amp, and PFB FFT
          shift.
        - ``'raise'`` — raise an error if the ADC saturates.
        - ``'none'`` — do not touch the RX path.
    preflight : bool, optional
        If ``True`` (default), before any sweeps probe the most demanding
        power step against the hardware and, if it cannot be set, the next
        most demanding, and so on, until one succeeds — establishing the
        maximum achievable power.  Steps whose peak tone power exceeds that
        ceiling are skipped during the run and recorded as NaN-filled
        placeholders instead of aborting.  Each probe physically applies that
        step's powers (there is no dry-run), so the highest feasible power is
        momentarily set before the run proper begins.  Set ``False`` to skip
        the probe and rely solely on the per-step graceful skip below (any
        step that fails to set is still warned and recorded as NaN).  Either
        way, ``run_power_sweep`` never raises on an infeasible or failed step
        and always returns whatever data it collected.
    settle_time : float, optional
        Seconds to sleep after applying a power step, before the sweep
        starts (default ``0.5``).  Set to ``0`` to skip.
    refresh_adc_cal : bool, optional
        If ``True`` (default), refresh the ADC calibration before each
        sweep (unfreeze, settle, freeze) so it adapts to the current tone
        configuration.  If ``False``, the calibration is left frozen at
        whatever it was on entry.
    adc_cal_settle_time : float, optional
        Seconds to let the ADC calibration settle after unfreezing
        (default ``2.0``).  Ignored when ``refresh_adc_cal`` is ``False``.
    file_format : str, optional
        Artifact format for each saved sweep.  Sweeps are stored as ``'npz'``;
        any other value raises.
    follow_dips : bool, optional
        If ``True``, after every sweep step refind the deepest empirical dip
        in each regular tone's full trace and use those frequencies as the
        next step's centers.  Blind tones (per ``client.get_tone_metadata``)
        are always left at their current frequency.  Default ``True``.
    follow_min_depth_db : float or None, optional
        Minimum dip depth in dB required for a candidate to be accepted
        by ``follow_dips`` (default ``0.5`` dB, matching
        :py:func:`fit_power_sweep`'s ``min_dip_depth_db`` default).  Set to
        ``None`` to accept any depth.  Tones whose candidate falls below
        the threshold keep their previous center and are flagged with
        ``"dip_too_shallow"`` in the saved follow-dips record.  Neighbouring
        tones whose new centers would swap frequency order (both chasing the
        same resonance) are both reverted and flagged ``"conflict_with..."``.
    search_for_center : bool or None, optional
        If ``True``, run one extra sweep at the first requested power
        before the recorded run begins, find the empirical dips, and use
        those frequencies as the centers for step 0, so the first saved
        sweep is already on resonance.  The search sweep itself is not
        saved and can be used even when ``follow_dips=False``.  ``None``
        (default) follows ``follow_dips``.
    search_span_factor : float, optional
        Multiplier applied to ``spans`` for the initial center-search sweep
        only.  The saved sweeps still use ``spans``.  Default ``2.0`` helps
        recover dips that would otherwise be clipped at the edge of the
        requested saved-sweep span.
    verbose : bool, optional
        Print step-by-step progress (default ``True``).
    capture_system_info : bool, optional
        If ``True`` (default), save a ``client.get_info()`` snapshot as a
        JSON file in the run directory at the start and end of the run.
    info_sections : optional
        Which info sections to capture, forwarded to ``client.get_info``
        (default ``"all"``).  Ignored when ``capture_system_info`` is
        ``False``.

    Returns
    -------
    run : dict
        The analysis-view dict for the run (see :func:`load_power_sweep`).
        Per-step sweep data are saved as ``.npz`` files under ``data/`` and
        the run is described by ``measurement.json`` (:data:`MANIFEST_FILE`);
        the returned dict can be passed directly to :py:func:`fit_power_sweep`
        or :py:func:`plot_power_sweep`.
    """
    if str(file_format).lstrip(".") != "npz":
        raise ValueError("run_power_sweep stores sweep artifacts as npz.")

    # --- Put the sweep inputs into the shapes the client uses: one center and
    # span per tone, and one requested power vector per sweep step. ---
    centers = np.asarray(centers, dtype=float).ravel()
    spans = np.asarray(spans, dtype=float).ravel()
    if centers.size == 0:
        raise ValueError("centers must contain at least one tone.")
    if spans.size == 1:
        spans = np.full(centers.size, spans[0], dtype=float)
    if spans.size != centers.size:
        raise ValueError("spans must be scalar or have one value per tone.")
    power_steps = _normalise_power_steps(powers_dbm, centers.size)
    if not power_steps:
        raise ValueError("powers_dbm must contain at least one power step.")

    follow_dips = bool(follow_dips)
    if search_for_center is None:
        search_for_center = follow_dips
    search_for_center = bool(search_for_center)
    search_span_factor = float(search_span_factor)
    if not np.isfinite(search_span_factor) or search_span_factor <= 0.0:
        raise ValueError("search_span_factor must be a positive finite number.")

    # Newman phases are regenerated after any recentering so the phase set
    # always matches the current tone frequencies; fixed phases are reused.
    if isinstance(phases, str) and phases.lower() == "newman":
        phase_mode = "newman"
        phase_values = None
    elif phases is None:
        phase_mode = None
        phase_values = None
    else:
        phase_mode = "fixed"
        phase_values = np.asarray(phases, dtype=float).ravel()
        if phase_values.size != centers.size:
            raise ValueError("phases must have one value per tone.")

    output_dir = Path(output_dir).resolve()
    initial_centers = centers.copy()

    (output_dir / "data").mkdir(parents=True, exist_ok=True)
    manifest = {
        "kind": "tone_power_sweep",
        "schema_version": SCHEMA_VERSION,
        "root": str(output_dir),
        "created": _timestamp(),
        "finished": None,
        "status": "running",
        "error": None,
        "parameters": {
            "initial_centers_hz": initial_centers.tolist(),
            "spans_hz": spans.tolist(),
            "powers_dbm": [row.tolist() for row in power_steps],
            "points": int(points),
            "samples_per_point": int(samples_per_point),
            "direction": direction,
            "phase_mode": phase_mode,
            "reference_plane": reference_plane,
            "optimise_dynamic_range": bool(optimise_dynamic_range),
            "rx_policy": rx_policy,
            "settle_time_s": float(settle_time),
            "refresh_adc_cal": bool(refresh_adc_cal),
            "adc_cal_settle_time_s": float(adc_cal_settle_time),
            "follow_dips": follow_dips,
            "search_for_center": search_for_center,
            "search_span_factor": search_span_factor,
            "tone_count": int(initial_centers.size),
        },
        "metadata": {
            "description": "Targeted resonator sweeps across tone power.",
            "current_centers_hz": centers.tolist(),
        },
        "artifacts": [],
        "steps": [],
    }
    _write_manifest(manifest, output_dir)
    if capture_system_info:
        _save_system_info(manifest, output_dir, client, "start", info_sections)
        _write_manifest(manifest, output_dir)

    # Feasibility pre-flight: probe the hardware to learn which power steps are
    # achievable, so infeasible ones are skipped (recorded as NaN) rather than
    # aborting the whole run.  Best-effort — any unexpected failure here just
    # leaves every step marked feasible and lets the per-step graceful skip in
    # the loop below handle it.
    step_feasible = np.ones(len(power_steps), dtype=bool)
    if preflight:
        _progress(verbose, "Pre-flight: checking power feasibility")
        try:
            _require_success(
                client.set_tone_frequencies(centers - spans / 2.0),
                "set_tone_frequencies failed",
            )
            step_feasible, feasibility_record = _preflight_feasible_steps(
                client, power_steps, reference_plane, optimise_dynamic_range,
                rx_policy, verbose,
            )
            manifest["metadata"]["feasibility"] = feasibility_record
            n_skip = int(np.sum(~step_feasible))
            if n_skip:
                peaks = np.round(
                    np.asarray(feasibility_record["step_peak_dbm"])[~step_feasible],
                    1,
                ).tolist()
                ceiling = feasibility_record["ceiling_peak_dbm"]
                ceiling_note = (
                    f"maximum achievable peak is {ceiling:.1f} dBm; "
                    if ceiling is not None
                    else "no power step is achievable; "
                )
                msg = (
                    f"{n_skip}/{len(power_steps)} power step(s) exceed the maximum "
                    f"achievable power at reference_plane={reference_plane!r} and "
                    f"will be skipped (recorded as NaN). {ceiling_note}"
                    f"skipped peaks: {peaks} dBm. Reduce powers_dbm to measure them."
                )
                warnings.warn(msg, stacklevel=2)
                _progress(verbose, f"  WARNING: {msg}")
            _write_manifest(manifest, output_dir)
        except Exception as exc:  # never let the pre-flight abort the run
            step_feasible = np.ones(len(power_steps), dtype=bool)
            warn_msg = (
                "Pre-flight feasibility check failed; proceeding and skipping "
                f"any step that cannot be set at run time: {exc}"
            )
            warnings.warn(warn_msg, stacklevel=2)
            _progress(verbose, f"  WARNING: {warn_msg}")

    def _record_skip(step, index, requested, reason, step_centers, phases):
        """Fill a step with a NaN placeholder sweep so the loop never breaks and
        downstream analyses see one NaN-filled row for the missing power."""
        placeholder = _nan_placeholder_sweep(
            step_centers, spans, points, samples_per_point,
            reference_plane, requested, reason, phases=phases,
        )
        rel_path = f"data/step_{index:04d}_sweep.npz"
        _save_npz(output_dir / rel_path, placeholder)
        centers_list = np.asarray(step_centers, dtype=float).ravel().tolist()
        readback_list = placeholder["readback_tone_powers_dbm"].tolist()
        step["readback"]["tone_power_dbm"] = readback_list
        step["metadata"]["next_centers_hz"] = centers_list
        step["metadata"]["skipped"] = True
        step["metadata"]["skip_reason"] = str(reason)
        step["artifacts"] = [{
            "name": f"step_{index:04d}_sweep",
            "kind": "sweep",
            "path": rel_path,
            "format": "npz",
            "metadata": {
                "reference_plane": reference_plane,
                "requested_tone_powers_dbm": np.asarray(
                    requested, dtype=float).ravel().tolist(),
                "readback_tone_powers_dbm": readback_list,
                "centers_hz": centers_list,
                "spans_hz": spans.tolist(),
                "skipped": True,
                "skip_reason": str(reason),
            },
            "created": _timestamp(),
        }]
        step["status"] = "skipped"
        step["error"] = str(reason)
        step["finished"] = _timestamp()

    try:
        # Optional center search: one discarded sweep (at the first requested
        # power, optionally over wider spans) used only to recenter on the
        # empirical dips before the first saved step.  The park / power / sweep
        # sequence here is the same one each saved step runs below.
        if search_for_center:
          try:
            search_spans = spans * search_span_factor
            span_note = (
                ""
                if np.isclose(search_span_factor, 1.0)
                else f" using {search_span_factor:g}x spans"
            )
            _progress(
                verbose,
                f"Searching for centers at {_format_power_summary(power_steps[0])} "
                f"at {reference_plane}{span_note}",
            )
            search_phases = (
                np.asarray(client.generate_newman_phases(centers), dtype=float)
                if phase_mode == "newman"
                else phase_values
            )
            # Park tones off-resonance (sweep low edge) before set_tone_powers so
            # any rx optimisation sees the highest rx level the sweep reaches, not
            # the on-resonance dip.  perform_sweep restores them before sweeping.
            _progress(verbose, f"  Parking {centers.size} tones at sweep low edge")
            _require_success(
                client.set_tone_frequencies(centers - search_spans / 2.0),
                "set_tone_frequencies failed",
            )
            if search_phases is not None:
                _progress(verbose, "  Setting tone phases")
                _require_success(
                    client.set_tone_phases(search_phases), "set_tone_phases failed"
                )
            _progress(
                verbose,
                "  Optimising dynamic range and applying tone powers"
                if optimise_dynamic_range
                else "  Applying tone powers",
            )
            _require_success(
                client.set_tone_powers(
                    power_steps[0],
                    reference_plane=reference_plane,
                    optimise_dynamic_range=optimise_dynamic_range,
                    rx_policy=rx_policy,
                    verbose=False,
                ),
                "set_tone_powers failed",
            )
            if settle_time:
                _progress(verbose, f"  Settling for {float(settle_time):g} s")
                time.sleep(float(settle_time))
            _progress(verbose, "  Running targeted sweep")
            _require_success(
                client.perform_sweep(
                    centers, search_spans, points=int(points),
                    samples_per_point=int(samples_per_point), direction=direction,
                    phases=search_phases, refresh_adc_cal=refresh_adc_cal,
                    adc_cal_settle_time=adc_cal_settle_time,
                ),
                "perform_sweep failed",
            )
            client.wait_for_sweep(progress_bar=verbose)
            search_sweep = client.parse_sweep_data(client.get_sweep_data())
            search_sweep["tone_metadata"] = client.get_tone_metadata()

            new_centers, search_record = _find_dip_centers(
                search_sweep, centers, search_spans, follow_min_depth_db
            )
            blind_count = len(search_record["blind_indices"])
            _progress(
                verbose,
                f"  Search recentered {int(np.sum(search_record['accepted']))}/"
                f"{centers.size - blind_count} tones",
            )
            centers = new_centers
            manifest["metadata"]["search_record"] = search_record
            manifest["metadata"]["search_spans_hz"] = search_spans.tolist()
            manifest["metadata"]["search_centers_hz"] = centers.tolist()
            manifest["metadata"]["current_centers_hz"] = centers.tolist()
            _write_manifest(manifest, output_dir)
          except Exception as exc:
            # A failed center search must not abort the run: fall back to the
            # requested centers and continue to the recorded power steps.
            warn_msg = (
                f"Center search failed ({exc}); using the requested centers."
            )
            warnings.warn(warn_msg, stacklevel=2)
            _progress(verbose, f"  WARNING: {warn_msg}")
            manifest["metadata"].setdefault("warnings", []).append(warn_msg)

        # Step through the power schedule, saving one targeted sweep per step.
        # A step the pre-flight flagged infeasible, or one that fails to set /
        # sweep at run time, is recorded as a NaN placeholder and the loop
        # continues — the run never aborts on a single bad power level.
        for index, requested in enumerate(power_steps):
            feasible = bool(step_feasible[index])
            _progress(
                verbose,
                f"[{index + 1}/{len(power_steps)}] tone powers "
                f"{_format_power_summary(requested)} at {reference_plane}"
                + ("" if feasible else "  [SKIPPED: exceeds max achievable power]"),
            )
            step = {
                "index": index,
                "axis": {"tone_power_dbm": requested.tolist()},
                "readback": {},
                "metadata": {
                    "reference_plane": reference_plane,
                    "points": int(points),
                    "samples_per_point": int(samples_per_point),
                    "direction": direction,
                    "centers_hz": centers.tolist(),
                    "spans_hz": spans.tolist(),
                },
                "artifacts": [],
                "status": "running",
                "started": _timestamp(),
                "finished": None,
                "error": None,
            }
            manifest["steps"].append(step)

            if not feasible:
                reason = (
                    "requested power exceeds maximum achievable power "
                    "(pre-flight); recorded as NaN"
                )
                _record_skip(step, index, requested, reason, centers, None)
                _progress(verbose, "  Recorded NaN placeholder (skipped)")
                manifest["metadata"]["current_centers_hz"] = centers.tolist()
                _write_manifest(manifest, output_dir)
                continue

            try:
                step_phases = (
                    np.asarray(client.generate_newman_phases(centers), dtype=float)
                    if phase_mode == "newman"
                    else phase_values
                )
                if step_phases is not None:
                    step["metadata"]["tone_phases_rad"] = step_phases.tolist()

                # Park, power, settle, sweep (same sequence as the center search).
                _progress(
                    verbose, f"  Parking {centers.size} tones at sweep low edge"
                )
                _require_success(
                    client.set_tone_frequencies(centers - spans / 2.0),
                    "set_tone_frequencies failed",
                )
                if step_phases is not None:
                    _progress(verbose, "  Setting tone phases")
                    _require_success(
                        client.set_tone_phases(step_phases),
                        "set_tone_phases failed",
                    )
                _progress(
                    verbose,
                    "  Optimising dynamic range and applying tone powers"
                    if optimise_dynamic_range
                    else "  Applying tone powers",
                )
                _require_success(
                    client.set_tone_powers(
                        requested,
                        reference_plane=reference_plane,
                        optimise_dynamic_range=optimise_dynamic_range,
                        rx_policy=rx_policy,
                        verbose=False,
                    ),
                    "set_tone_powers failed",
                )
                if settle_time:
                    _progress(verbose, f"  Settling for {float(settle_time):g} s")
                    time.sleep(float(settle_time))
                _progress(verbose, "  Running targeted sweep")
                _require_success(
                    client.perform_sweep(
                        centers, spans, points=int(points),
                        samples_per_point=int(samples_per_point),
                        direction=direction,
                        phases=step_phases, refresh_adc_cal=refresh_adc_cal,
                        adc_cal_settle_time=adc_cal_settle_time,
                    ),
                    "perform_sweep failed",
                )
                client.wait_for_sweep(progress_bar=verbose)
                sweep_data = client.parse_sweep_data(client.get_sweep_data())
                sweep_data["tone_metadata"] = client.get_tone_metadata()

                readback = np.asarray(
                    client.get_tone_powers(reference_plane=reference_plane),
                    dtype=float,
                ).ravel()
                step["readback"]["tone_power_dbm"] = readback.tolist()
                sweep_data["requested_tone_powers_dbm"] = requested.copy()
                sweep_data["readback_tone_powers_dbm"] = readback.copy()
                sweep_data["tone_powers_reference_plane"] = reference_plane
                sweep_data["sweep_centers_hz"] = centers.copy()

                # If requested, find the measured dip in each regular trace and
                # use it as the next step's center.  Blind tones are left fixed.
                next_centers = centers.copy()
                if follow_dips:
                    next_centers, follow_record = _find_dip_centers(
                        sweep_data, centers, spans, follow_min_depth_db
                    )
                    sweep_data["follow_dips"] = follow_record
                    sweep_data["next_sweep_centers_hz"] = next_centers.copy()
                    blind_count = len(follow_record["blind_indices"])
                    _progress(
                        verbose,
                        f"  Following dips: {int(np.sum(follow_record['accepted']))}/"
                        f"{centers.size - blind_count} centers updated",
                    )
                step["metadata"]["next_centers_hz"] = next_centers.tolist()

                rel_path = f"data/step_{index:04d}_sweep.npz"
                _save_npz(output_dir / rel_path, sweep_data)
                step["artifacts"].append({
                    "name": f"step_{index:04d}_sweep",
                    "kind": "sweep",
                    "path": rel_path,
                    "format": "npz",
                    "metadata": {
                        "reference_plane": reference_plane,
                        "requested_tone_powers_dbm": requested.tolist(),
                        "readback_tone_powers_dbm": readback.tolist(),
                        "centers_hz": centers.tolist(),
                        "spans_hz": spans.tolist(),
                        "system_info_source": "embedded",
                    },
                    "created": _timestamp(),
                })
                _progress(verbose, f"  Saved {rel_path}")

                step["status"] = "success"
                step["finished"] = _timestamp()
                centers = next_centers
            except Exception as exc:
                # A power level that can't be set (or a sweep that fails) must
                # not break the loop: warn loudly, record a NaN placeholder, and
                # keep the current centers for the next step.
                reason = "".join(
                    traceback.format_exception_only(type(exc), exc)
                ).strip()
                warn_msg = (
                    f"Power step {index} "
                    f"({_format_power_summary(requested)} at {reference_plane}) "
                    f"failed and was recorded as NaN: {reason}"
                )
                warnings.warn(warn_msg, stacklevel=2)
                _progress(verbose, f"  WARNING: {warn_msg}")
                _record_skip(step, index, requested, reason, centers, None)

            manifest["metadata"]["current_centers_hz"] = centers.tolist()
            # Rewrite the manifest after every step so an interrupted run can
            # still be inspected.
            _write_manifest(manifest, output_dir)

        manifest["metadata"]["final_centers_hz"] = centers.tolist()
        n_skipped = sum(
            1 for s in manifest["steps"] if s.get("status") == "skipped"
        )
        manifest["metadata"]["skipped_step_count"] = int(n_skipped)
        manifest["status"] = "success" if n_skipped == 0 else "partial"
    except Exception as exc:
        # Never raise: record the error and fall through so the caller still
        # receives the partial run (whatever steps completed before the fault).
        manifest["status"] = "failed"
        manifest["error"] = "".join(
            traceback.format_exception_only(type(exc), exc)
        ).strip()
        if manifest["steps"] and manifest["steps"][-1]["status"] == "running":
            manifest["steps"][-1]["status"] = "failed"
            manifest["steps"][-1]["error"] = manifest["error"]
            manifest["steps"][-1]["finished"] = _timestamp()
        warnings.warn(
            f"run_power_sweep stopped early and returned partial data: "
            f"{manifest['error']}",
            stacklevel=2,
        )
        print(traceback.format_exc(), flush=True)
    finally:
        manifest["finished"] = _timestamp()
        if capture_system_info:
            _save_system_info(manifest, output_dir, client, "end", info_sections)
        _write_manifest(manifest, output_dir)
    return _power_sweep_run_view(manifest)


def _require_success(response, message):
    """Raise if a client call returned a ``{'status': ...}`` failure dict."""
    if isinstance(response, dict) and response.get("status") != "success":
        raise RuntimeError(response.get("message", message))


def _power_sweep_run_view(run):
    """Return the array view the fit/plot helpers work on.

    Accepts an already-built view dict (returned unchanged), a run manifest
    dict (as built by :func:`run_power_sweep`), or a path to a run directory
    or ``measurement.json`` file.  Fits attached to the returned dict by
    :func:`fit_power_sweep` are therefore visible to a later
    :func:`plot_power_sweep` call handed the same dict.
    """
    if isinstance(run, dict):
        if "sweeps" in run:          # already a built analysis view
            return run
        manifest = run               # a freshly built or loaded manifest
    else:
        manifest = _load_manifest(run)

    root = Path(manifest.get("root", "."))
    parameters = manifest.get("parameters", {})
    metadata = manifest.get("metadata", {})

    # Walk the steps once, pulling the per-step arrays the analysis code wants
    # (the loaded sweep, requested/readback powers, and centers) into parallel
    # lists indexed by step.
    sweeps, files, powers, readback, centers, next_centers, steps = (
        [], [], [], [], [], [], []
    )
    for step in manifest.get("steps", []):
        step_meta = step.get("metadata", {})
        # Each step's sweep is stored as an npz artifact; load it (or keep a
        # None placeholder so list positions stay aligned with the steps).
        sweep_artifact = next(
            (art for art in step.get("artifacts", []) if art.get("kind") == "sweep"),
            None,
        )
        if sweep_artifact is not None:
            sweep_path = Path(sweep_artifact["path"])
            if not sweep_path.is_absolute():
                sweep_path = root / sweep_path
            sweeps.append(_load_npz(sweep_path))
            files.append(str(sweep_path))
            sweep_file = sweep_artifact["path"]
        else:
            sweeps.append(None)
            files.append("")
            sweep_file = None
        power = np.asarray(
            step.get("axis", {}).get("tone_power_dbm", []), dtype=float
        ).ravel()
        power_readback = np.asarray(
            step.get("readback", {}).get("tone_power_dbm", []), dtype=float
        ).ravel()
        step_centers = np.asarray(
            step_meta.get("centers_hz", parameters.get("initial_centers_hz", [])),
            dtype=float,
        ).ravel()
        step_next_centers = np.asarray(
            step_meta.get("next_centers_hz", step_centers), dtype=float
        ).ravel()
        powers.append(power)
        readback.append(power_readback)
        centers.append(step_centers)
        next_centers.append(step_next_centers)
        steps.append({
            "index": int(step.get("index", len(steps))),
            "power_dbm": power.tolist(),
            "readback_power_dbm": power_readback.tolist(),
            "centers_hz": step_centers.tolist(),
            "next_centers_hz": step_next_centers.tolist(),
            "sweep_file": sweep_file,
            "metadata": step_meta,
        })

    # Infer the tone count from the first real sweep (a wideband or single-row
    # sweep is one tone; otherwise it is the number of columns), falling back to
    # the recorded parameter when no sweep loaded.
    first_sweep = next((sweep for sweep in sweeps if isinstance(sweep, dict)), None)
    if first_sweep is not None:
        first_f = np.atleast_2d(np.asarray(first_sweep["sweep_f"]))
        if first_sweep.get("wideband_sweep", False) or first_f.shape[0] == 1:
            tone_count = 1
        else:
            tone_count = first_f.shape[1]
    else:
        tone_count = int(parameters.get("tone_count", 0))

    initial_centers = parameters.get("initial_centers_hz", [])
    return {
        "root": str(root),
        "manifest_file": str(root / MANIFEST_FILE),
        "manifest": manifest,
        "initial_centers_hz": np.asarray(initial_centers, dtype=float),
        "centers_hz": np.asarray(initial_centers, dtype=float),
        "final_centers_hz": np.asarray(
            metadata.get(
                "final_centers_hz",
                metadata.get("current_centers_hz", initial_centers),
            ),
            dtype=float,
        ),
        "spans_hz": np.asarray(parameters.get("spans_hz", []), dtype=float),
        "steps": steps,
        "sweeps": sweeps,
        "files": files,
        "powers_dbm": powers,
        "readback_powers_dbm": readback,
        "centers_by_step_hz": centers,
        "next_centers_by_step_hz": next_centers,
        "tone_count": int(tone_count),
    }


def load_power_sweep(path):
    """Load a tone-power sweep run.

    Parameters
    ----------
    path : str or Path
        A power-sweep run directory or its ``measurement.json`` manifest.
        Raises if the loaded run is not a ``"tone_power_sweep"``.

    Returns
    -------
    run : dict
        Analysis-view dict ready for :py:func:`fit_power_sweep`,
        :py:func:`plot_power_sweep`, and :py:func:`analyse_power_sweep`.
    """
    manifest = _load_manifest(path)
    kind = manifest.get("kind")
    if kind != "tone_power_sweep":
        raise ValueError(
            f"{path} is a {kind!r} measurement, not a tone-power sweep."
        )
    return _power_sweep_run_view(manifest)


def fit_power_sweep(
    run,
    tone_index=None,
    nonlinear=True,
    sweep_direction="up",
    n_jobs=1,
    verbose=True,
    min_dip_depth_db=0.5,
    **fit_kwargs,
):
    """Fit all tones at each power, or one tone across the power axis.

    Two operating modes:

    - ``tone_index=None`` (per-power) calls :py:func:`batch_fit` on each
      saved server sweep.  Returned as ``fits_by_power``: one inner list
      per power step, each containing the regular tones fit from that
      sweep with original ``FitResult.tone_index`` values preserved.
    - ``tone_index=<int>`` (per-tone stack) extracts that one tone from
      every saved sweep and passes the resulting arrays to
      :py:func:`fit_sweep_stack`.  Returned as a single ``fits`` list
      ordered by sweep/power step, convenient for one resonator's
      parameter trend.  Stack rows are fit independently — the previous
      power's fit is not chained into the next fit's initial guess.

    The per-sweep ``sweep_ei`` / ``sweep_eq`` arrays are forwarded as
    ``z_err`` in both modes (per-power via :py:func:`batch_fit`'s
    auto-pickup, per-tone via the explicit ``z_err_stack`` argument), so
    the resulting ``FitResult.parameter_uncertainties`` reflect
    measurement noise rather than the residual size of each individual
    fit. ``min_dip_depth_db`` rejects traces whose empirical dip is too
    shallow; use ``min_dip_depth_db=None`` to force every optimiser run.

    Parameters
    ----------
    run : dict or str or Path
        Run dict from :py:func:`run_power_sweep` or
        :py:func:`load_power_sweep`, or a path to a power-sweep directory
        / manifest (which is loaded on the fly).
    tone_index : int or None, optional
        ``None`` (default) fits every regular tone at every power.  An
        ``int`` fits only that tone across the power axis as a stack.
    nonlinear : bool or {'auto'}, optional
        How to handle the Duffing nonlinearity parameter ``anl`` (default
        ``True``, so ``anl`` is fitted at every power -- needed for ``anl``
        versus power studies; see
        :func:`souk_readout_tools.fitting.fit_resonance`). ``False`` fits the
        linear seven-parameter model only. ``'auto'`` fits each power/tone with
        the linear model and only refines the ones it cannot explain, which is
        faster but drops ``anl`` for low powers whose nonlinearity is below the
        single-sweep detection floor.
    sweep_direction : {'up', 'down'}, optional
        Direction the saved sweeps were taken in.  Affects nonlinear
        bifurcation handling (default ``'up'``).
    n_jobs : int, optional
        joblib worker count for parallel fitting: ``1`` (default) serial,
        ``-1`` all CPUs, ``-2`` all but one.  Parallel fitting uses
        separate processes.
    verbose : bool, optional
        Print per-fit progress (default ``True``).
    min_dip_depth_db : float or None, optional
        Minimum empirical dip depth in dB required before running the
        optimiser (default ``0.5``). Fits below this threshold are returned as
        ``success=False`` and ``noise_only=True``. Set ``None`` to force every
        fit.
    **fit_kwargs
        Forwarded to :py:func:`batch_fit` (per-power mode) or
        :py:func:`fit_sweep_stack` (per-tone mode).  Common kwargs:
        ``initial_guess`` (dict), ``param_bounds`` (dict of
        ``(low, high)``), ``param_fixed`` (iterable of names),
        ``data_format``, ``window_fwhm``, ``z_err``, ``use_error_weights``,
        ``max_points``, ``skip_blind``.

    Returns
    -------
    fit_data : dict
        Per-power mode returns ``{'run': run, 'fits_by_power': [...]}``;
        per-tone mode returns
        ``{'run': run, 'tone_index': int, 'fits': [...]}``.  The same
        dict includes ``summary_array``, a structured NumPy array with the
        same columns and values as the fit-summary CSV (see
        :py:func:`fit_summary_array`).  It is also stored on ``run['fits']``
        so subsequent :py:func:`plot_power_sweep` calls can omit it.
    """
    run = _power_sweep_run_view(run)

    # Per-power mode keeps the server sweep boundary intact: every saved sweep
    # goes through batch_fit, so blind-tone handling and original tone indices
    # match normal one-sweep fitting.
    if tone_index is None:
        fits_by_power = [
            batch_fit(
                sweep,
                nonlinear=nonlinear,
                sweep_direction=sweep_direction,
                n_jobs=n_jobs,
                verbose=verbose,
                min_dip_depth_db=min_dip_depth_db,
                **fit_kwargs,
            )
            for sweep in run["sweeps"]
        ]
        fit_data = {"run": run, "fits_by_power": fits_by_power}
        fit_data["summary_array"] = fit_summary_array(fit_data)
        run["fits"] = fit_data
        return fit_data

    # Single-tone mode lines up one resonator's trace from each power and uses
    # the lower-level array stack fitter. This changes the output shape from
    # power -> tone to one tone -> power; rows are not chained as initial guesses.
    f_stack, z_stack, e_stack = [], [], []
    have_error_stack = True
    for sweep in run["sweeps"]:
        f = np.atleast_2d(np.asarray(sweep["sweep_f"], dtype=float))
        i = np.atleast_2d(np.asarray(sweep["sweep_i"], dtype=float))
        q = np.atleast_2d(np.asarray(sweep["sweep_q"], dtype=float))
        f_stack.append(f[:, tone_index])
        z_stack.append(i[:, tone_index] + 1j * q[:, tone_index])
        if "sweep_ei" in sweep and "sweep_eq" in sweep:
            ei = np.atleast_2d(np.asarray(sweep["sweep_ei"], dtype=float))
            eq = np.atleast_2d(np.asarray(sweep["sweep_eq"], dtype=float))
            e_stack.append(ei[:, tone_index] + 1j * eq[:, tone_index])
        else:
            have_error_stack = False

    fits = fit_sweep_stack(
        np.asarray(f_stack),
        np.asarray(z_stack),
        z_err_stack=np.asarray(e_stack) if have_error_stack else None,
        nonlinear=nonlinear,
        sweep_direction=sweep_direction,
        n_jobs=n_jobs,
        verbose=verbose,
        min_dip_depth_db=min_dip_depth_db,
        **fit_kwargs,
    )
    for fit in fits:
        fit.tone_index = int(tone_index)
    fit_data = {"run": run, "tone_index": int(tone_index), "fits": list(fits)}
    fit_data["summary_array"] = fit_summary_array(fit_data)
    run["fits"] = fit_data
    return fit_data


def parameter_series(
    fit_data,
    tone_index,
    parameters=None,
    *,
    use_readback=True,
    include_uncertainties=True,
    drop_missing=False,
):
    """Return one tone's fit parameters as arrays over the power axis.

    This is the interactive-inspection companion to
    :py:func:`fit_summary_array`: it keeps the native power-step ordering
    from :py:func:`fit_power_sweep` and fills missing fits with ``NaN`` by
    default.  It works with both ``fits_by_power`` output and the
    ``tone_index=<int>`` single-tone output.

    Parameters
    ----------
    fit_data : dict
        Output of :py:func:`fit_power_sweep`.
    tone_index : int
        Original tone index to extract.
    parameters : iterable of str, str, or None, optional
        Fit-result fields to include.  Valid names are the keys returned
        by :py:func:`souk_readout_tools.fitting.extract_parameters`, such
        as ``'fr'``, ``'Qi'``, ``'Qc'``, ``'anl'`` and the
        ``'empirical_*'`` diagnostics.  ``None`` (default) includes the
        standard summary fields.
    use_readback : bool, optional
        ``True`` (default) makes ``series['power_dbm']`` use the readback
        tone power when available, falling back to requested power.
        ``False`` uses requested power first.  The raw requested and
        readback values are always returned as ``requested_power_dbm`` and
        ``readback_power_dbm``.
    include_uncertainties : bool, optional
        Include ``'<name>_err'`` arrays for parameters with propagated
        uncertainties stored on the ``FitResult`` (default ``True``).
    drop_missing : bool, optional
        If ``True``, remove power steps where this tone has no fit.
        Default ``False`` preserves one row per power step.

    Returns
    -------
    series : dict
        Dictionary of 1D arrays.  Always includes ``sweep_index``,
        ``power_dbm``, ``requested_power_dbm`` and
        ``readback_power_dbm``, ``tone_index`` and ``fit_present``.
        Requested fit parameters are added under their own names, so an
        interactive plot can be as direct as::

            s = parameter_series(fits, tone_index=0)
            plt.plot(s["power_dbm"], s["Qc"] / s["Qi"], marker="o")
    """
    run = fit_data["run"]
    tone_index = int(tone_index)
    if "tone_count" in run:
        _normalise_tone_indices(tone_index, run["tone_count"])

    if parameters is None:
        parameters = SUMMARY_KEYS
    elif isinstance(parameters, str):
        parameters = (parameters,)
    else:
        parameters = tuple(parameters)

    valid_parameters = set(extract_parameters(()))
    invalid = [name for name in parameters if name not in valid_parameters]
    if invalid:
        valid = ", ".join(sorted(valid_parameters))
        raise ValueError(
            f"Unknown fit parameter(s) {invalid}; valid parameters are: {valid}"
        )

    if (
        "fits" in fit_data
        and int(fit_data.get("tone_index", tone_index)) != tone_index
    ):
        raise ValueError(
            "fit_data contains single-tone fits for tone "
            f"{int(fit_data['tone_index'])}, not tone {tone_index}."
        )

    fits_for_tone = _fits_for_tone(fit_data, tone_index)
    n_steps = max(
        len(run.get("sweeps", [])),
        len(run.get("powers_dbm", [])),
        len(run.get("readback_powers_dbm", [])),
        len(fits_for_tone),
    )
    if len(fits_for_tone) < n_steps:
        fits_for_tone = list(fits_for_tone) + [None] * (
            n_steps - len(fits_for_tone)
        )
    else:
        fits_for_tone = list(fits_for_tone[:n_steps])

    def _raw_power_for_tone(key, sweep_index):
        rows = run.get(key, [])
        if sweep_index >= len(rows):
            return np.nan
        row = np.asarray(rows[sweep_index], dtype=float).ravel()
        if tone_index < row.size:
            return float(row[tone_index])
        return np.nan

    requested_power = np.full(n_steps, np.nan, dtype=float)
    readback_power = np.full(n_steps, np.nan, dtype=float)
    selected_power = np.full(n_steps, np.nan, dtype=float)
    for sweep_index in range(n_steps):
        requested_power[sweep_index] = _raw_power_for_tone(
            "powers_dbm", sweep_index
        )
        readback_power[sweep_index] = _raw_power_for_tone(
            "readback_powers_dbm", sweep_index
        )
        selected_power[sweep_index] = _power_for_tone(
            run, sweep_index, tone_index, use_readback=use_readback
        )

    series = {
        "sweep_index": np.arange(n_steps, dtype=int),
        "tone_index": np.full(n_steps, tone_index, dtype=int),
        "fit_present": np.asarray(
            [fit is not None for fit in fits_for_tone], dtype=bool
        ),
        "power_dbm": selected_power,
        "requested_power_dbm": requested_power,
        "readback_power_dbm": readback_power,
    }

    for name in parameters:
        if name == "Qe":
            fill = complex(np.nan, np.nan)
        elif name == "noise_only":
            fill = False
        else:
            fill = np.nan
        values = [
            getattr(fit, name, fill) if fit is not None else fill
            for fit in fits_for_tone
        ]
        series[name] = np.asarray(values)
        if include_uncertainties and name in UNCERTAINTY_KEYS:
            sigmas = []
            for fit in fits_for_tone:
                unc = getattr(fit, "parameter_uncertainties", None) or {}
                sigmas.append(
                    float(unc.get(name, np.nan)) if fit is not None else np.nan
                )
            series[f"{name}_err"] = np.asarray(sigmas, dtype=float)

    if drop_missing:
        keep = np.asarray([fit is not None for fit in fits_for_tone], dtype=bool)
        for key, value in list(series.items()):
            if isinstance(value, np.ndarray) and value.shape == keep.shape:
                series[key] = value[keep]

    return series


def _fit_summary_rows(fit_data):
    """Flatten fit results into one CSV-friendly row per sweep/tone fit.

    Parameters
    ----------
    fit_data : dict
        Output of :py:func:`fit_power_sweep` (either per-power or per-tone
        mode).

    Returns
    -------
    rows : list of dict
        One row per fit.  Each row contains ``sweep_index``,
        ``tone_index``, the requested and readback ``power_dbm``,
        ``sweep_center_hz`` (the target centre frequency for the sweep,
        i.e. ``run['centers_by_step_hz'][sweep_index][tone_index]``),
        every key in :data:`SUMMARY_KEYS` (fitted parameters, empirical
        estimates, noise-only diagnostics, residual / chi-square statistics,
        and solver status), a ``<name>_err`` column for every name in
        :data:`UNCERTAINTY_KEYS` (the 1-σ measurement-noise-propagated
        uncertainty pulled from ``FitResult.parameter_uncertainties``),
        and ``message`` from the solver.  ``<name>_err`` columns appear
        immediately after their corresponding value.
    """
    run = fit_data["run"]
    rows = []

    def _center_hz(sweep_index, tone_index):
        centers_by_step = run.get("centers_by_step_hz")
        if not centers_by_step or sweep_index >= len(centers_by_step):
            return np.nan
        centers = np.asarray(centers_by_step[sweep_index], dtype=float).ravel()
        return float(centers[tone_index]) if tone_index < centers.size else np.nan

    # Each row records the power for the tone being fitted and the common
    # parameters exported by the resonator fitter.
    if "fits_by_power" in fit_data:
        for sweep_index, fits in enumerate(fit_data["fits_by_power"]):
            for fit_index, fit in enumerate(fits):
                tone_index = int(getattr(fit, "tone_index", fit_index))
                power = np.asarray(run["powers_dbm"][sweep_index], dtype=float).ravel()
                readback = np.asarray(run["readback_powers_dbm"][sweep_index], dtype=float).ravel()
                row = {
                    "sweep_index": sweep_index,
                    "tone_index": tone_index,
                    "power_dbm": power[tone_index] if tone_index < power.size else np.nan,
                    "readback_power_dbm": (
                        readback[tone_index] if tone_index < readback.size else np.nan
                    ),
                    "sweep_center_hz": _center_hz(sweep_index, tone_index),
                }
                row.update(fit_result_summary_row(fit))
                rows.append(row)
        return rows

    tone_index = int(fit_data["tone_index"])
    for sweep_index, fit in enumerate(fit_data["fits"]):
        power = np.asarray(run["powers_dbm"][sweep_index], dtype=float).ravel()
        readback = np.asarray(run["readback_powers_dbm"][sweep_index], dtype=float).ravel()
        row = {
            "sweep_index": sweep_index,
            "tone_index": tone_index,
            "power_dbm": power[tone_index] if tone_index < power.size else np.nan,
            "readback_power_dbm": readback[tone_index] if tone_index < readback.size else np.nan,
            "sweep_center_hz": _center_hz(sweep_index, tone_index),
        }
        row.update(fit_result_summary_row(fit))
        rows.append(row)
    return rows


_FIT_SUMMARY_INDEX_KEYS = {"sweep_index", "tone_index"}
_FIT_SUMMARY_BOOL_KEYS = {"success", "noise_only"}
_FIT_SUMMARY_STRING_KEYS = {"message"}


def _fit_summary_column_names():
    """Return the canonical fit-summary column order."""
    columns = [
        "sweep_index",
        "tone_index",
        "power_dbm",
        "readback_power_dbm",
        "sweep_center_hz",
    ]
    for key in SUMMARY_KEYS:
        columns.append(key)
        if key in UNCERTAINTY_KEYS:
            columns.append(f"{key}_err")
    columns.append("message")
    return tuple(columns)


def _fit_summary_array_dtype(rows, columns):
    """Infer a structured dtype for fit-summary rows."""
    dtype = []
    for name in columns:
        if name in _FIT_SUMMARY_INDEX_KEYS:
            dtype.append((name, np.int64))
        elif name in _FIT_SUMMARY_BOOL_KEYS:
            dtype.append((name, np.bool_))
        elif name in _FIT_SUMMARY_STRING_KEYS:
            width = max(
                [1]
                + [
                    len(str(row.get(name, "")))
                    for row in rows
                    if row.get(name, "") is not None
                ]
            )
            dtype.append((name, f"U{width}"))
        elif any(isinstance(row.get(name), complex) for row in rows):
            dtype.append((name, np.complex128))
        else:
            dtype.append((name, np.float64))
    return np.dtype(dtype)


def _coerce_summary_value(value, dtype):
    """Coerce one summary scalar into a structured-array field."""
    kind = dtype.kind
    if kind in {"U", "S"}:
        return "" if value is None else str(value)
    if kind == "b":
        if isinstance(value, str):
            return value.strip().lower() in {"true", "1", "yes"}
        return bool(value)
    if kind in {"i", "u"}:
        try:
            return int(value)
        except (TypeError, ValueError):
            return -1
    if kind == "c":
        try:
            return complex(value)
        except (TypeError, ValueError):
            return complex(np.nan, np.nan)
    try:
        return float(value)
    except (TypeError, ValueError):
        return np.nan


def fit_summary_array(fit_data):
    """Return fit-summary rows as a structured NumPy array.

    The field order matches the CSV written by :py:func:`write_fit_summary`,
    so interactive inspection can use the same names as the CSV header::

        table = fits["summary_array"]
        anl = table["anl"]
        powers = table["readback_power_dbm"]

    Integer index columns use ``int64``, boolean columns use ``bool``, the
    solver ``message`` column uses a Unicode string dtype, and fitted values
    / uncertainties use ``float64``.

    Parameters
    ----------
    fit_data : dict
        A :py:func:`fit_power_sweep` result (the same dict passed to
        :py:func:`write_fit_summary`).
    """
    return _rows_to_summary_array(_fit_summary_rows(fit_data))


def _rows_to_summary_array(rows):
    """Pack fit-summary rows (list of dicts) into a structured array."""
    columns = list(rows[0]) if rows else list(_fit_summary_column_names())
    array = np.empty(len(rows), dtype=_fit_summary_array_dtype(rows, columns))
    for row_index, row in enumerate(rows):
        for name in columns:
            array[name][row_index] = _coerce_summary_value(
                row.get(name, np.nan),
                array.dtype.fields[name][0],
            )
    return array


def _fit_summary_path(path):
    """Resolve a fit-summary path, accepting either a file or run directory."""
    path = Path(path)
    if path.exists() and path.is_dir():
        return path / FIT_SUMMARY_FILE
    if path.suffix:
        return path
    return path / FIT_SUMMARY_FILE


def write_fit_summary(fit_data, path=None):
    """Write the flat fit-summary table to a CSV file and return its path.

    One row per (power step, tone) fit, with the columns described in
    :py:func:`fit_summary_array`.  Read it back with
    :py:func:`load_fit_summary`.

    Parameters
    ----------
    fit_data : dict
        Output of :py:func:`fit_power_sweep`.
    path : str, Path, or None, optional
        Destination CSV file or power-sweep directory.  ``None`` (default)
        writes ``<fit_data['run']['root']>/analysis/fit_summary.csv``.
        Parent directories are created if missing.  If ``fit_data``
        produces no rows the file is not written.

    Returns
    -------
    Path
        The destination path.
    """
    if path is None:
        root = fit_data.get("run", {}).get("root")
        if root is None:
            raise ValueError(
                "path is required when fit_data['run']['root'] is missing."
            )
        path = Path(root) / FIT_SUMMARY_FILE
    else:
        path = _fit_summary_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = _fit_summary_rows(fit_data)
    if rows:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    return path


def load_fit_summary(path):
    """Load a fit-summary CSV back as a structured array.

    The inverse of :py:func:`write_fit_summary`.  Values are restored to
    float/bool/int/str per column, so the loaded table has the same fields
    as :py:func:`fit_summary_array` and can be fed straight to
    :py:func:`find_best_power` or indexed by column name.

    Parameters
    ----------
    path : str or Path
        The CSV file, or a power-sweep run directory containing
        ``analysis/fit_summary.csv``.

    Returns
    -------
    numpy.ndarray
        Structured array, one record per (power step, tone) fit.
    """
    source = _fit_summary_path(path)
    return _rows_to_summary_array(_coerce_summary_rows(source))


_FIT_ARCHIVE_KIND = "souk_readout_tools.power_sweep.fit_results"
_FIT_ARCHIVE_VERSION = 1
_FIT_RESULT_OPTIMISER_FIELDS = (
    "opt",
    "opt_linear",
    "opt_nonlinear",
    "optimizer_result",
    "optimizer_result_linear",
    "optimizer_result_nonlinear",
    "linear_fit_result",
)


def _fit_results_path(path):
    """Resolve a fit-results path, accepting either a file or run directory."""
    path = Path(path)
    if path.exists() and path.is_dir():
        return path / FIT_RESULTS_FILE
    if path.suffix:
        return path
    return path / FIT_RESULTS_FILE


def _fit_data_step_count(fit_data):
    """Return the number of power steps represented by fit_data."""
    if "fits_by_power" in fit_data:
        return len(fit_data["fits_by_power"])
    if "fits" in fit_data and not isinstance(fit_data.get("fits"), dict):
        return len(fit_data["fits"])
    run = fit_data.get("run", {})
    return len(run.get("sweeps", []))


def _archive_fit(fit, include_optimiser):
    """Return a fit object suitable for compact persistence."""
    if include_optimiser or not isinstance(fit, FitResult):
        return fit
    return replace(
        fit,
        **{name: None for name in _FIT_RESULT_OPTIMISER_FIELDS},
    )


def _archive_fit_data(fit_data, *, include_sweeps, include_optimiser):
    """Return a shallow, acyclic fit_data copy for persistence."""
    if not _is_fit_data(fit_data):
        raise TypeError("fit_data must be the dict returned by fit_power_sweep().")

    archived = {
        key: value
        for key, value in fit_data.items()
        if key not in {"run", "fits_by_power", "fits"}
    }
    if "fits_by_power" in fit_data:
        archived["fits_by_power"] = [
            [_archive_fit(fit, include_optimiser) for fit in fits]
            for fits in fit_data["fits_by_power"]
        ]
    else:
        archived["fits"] = [
            _archive_fit(fit, include_optimiser) for fit in fit_data["fits"]
        ]

    run = dict(fit_data["run"])
    run.pop("fits", None)
    if not include_sweeps:
        run["sweeps"] = [None] * _fit_data_step_count(fit_data)
    archived["run"] = run
    return archived


def write_fit_results(
    fit_data,
    path=None,
    *,
    include_sweeps=False,
    include_optimiser=False,
):
    """Write ``fit_power_sweep`` results to a pickle archive.

    Unlike :py:func:`write_fit_summary`, this preserves the
    :class:`~souk_readout_tools.fitting.FitResult` objects needed by
    :py:func:`plot_power_sweep`, :py:func:`parameter_series`,
    :py:func:`find_best_power`, and later summary export.  By default the
    embedded run metadata is compacted so the archive does not duplicate all
    raw sweeps already stored on disk; pass ``include_sweeps=True`` if you
    want a fully self-contained copy of ``fit_data['run']``.

    Parameters
    ----------
    fit_data : dict
        Output of :py:func:`fit_power_sweep`.
    path : str, Path, or None, optional
        Destination file or power-sweep directory.  ``None`` (default)
        writes ``<fit_data['run']['root']>/analysis/fit_results.pkl``.
    include_sweeps : bool, optional
        Include the raw sweep dictionaries from ``fit_data['run']`` in the
        archive.  Default ``False`` keeps only the metadata needed for
        plotting and summaries.
    include_optimiser : bool, optional
        Include raw SciPy optimiser objects and the nonlinear fit's stored
        linear seed result.  Default ``False`` keeps the archive smaller while
        preserving fitted parameters, uncertainties, traces, residuals, and
        status fields.

    Returns
    -------
    Path
        Resolved destination path.

    Notes
    -----
    Pickle files should only be loaded from trusted sources.
    """
    if path is None:
        root = fit_data.get("run", {}).get("root")
        if root is None:
            raise ValueError(
                "path is required when fit_data['run']['root'] is missing."
            )
        path = Path(root) / FIT_RESULTS_FILE
    else:
        path = _fit_results_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "kind": _FIT_ARCHIVE_KIND,
        "version": _FIT_ARCHIVE_VERSION,
        "created": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "include_sweeps": bool(include_sweeps),
        "include_optimiser": bool(include_optimiser),
        "fit_data": _archive_fit_data(
            fit_data,
            include_sweeps=include_sweeps,
            include_optimiser=include_optimiser,
        ),
    }
    with path.open("wb") as handle:
        pickle.dump(payload, handle, protocol=pickle.HIGHEST_PROTOCOL)
    return path.resolve()


def load_fit_results(path, run=None):
    """Load fit results written by :py:func:`write_fit_results`.

    Parameters
    ----------
    path : str or Path
        Fit-results pickle file, or a power-sweep directory containing
        :data:`FIT_RESULTS_FILE`.
    run : dict or str or Path or None, optional
        Optional fresh run dict, directory, or manifest to attach to the
        loaded fits.  When omitted, the compact run metadata stored in the
        archive is used.

    Returns
    -------
    fit_data : dict
        Restored ``fit_power_sweep``-style result.  The result is also stored
        on ``fit_data['run']['fits']`` so it can be passed straight to
        :py:func:`plot_power_sweep` or :py:func:`parameter_series`.
        ``fit_data['summary_array']`` is regenerated from the attached run
        metadata and loaded fits.

    Notes
    -----
    Pickle files should only be loaded from trusted sources.
    """
    filename = _fit_results_path(path)
    with filename.open("rb") as handle:
        payload = pickle.load(handle)

    if isinstance(payload, dict) and payload.get("kind") == _FIT_ARCHIVE_KIND:
        fit_data = payload["fit_data"]
    elif _is_fit_data(payload):
        # Backwards-compatible escape hatch for archives that directly stored
        # the fit_data dict before the small metadata wrapper existed.
        fit_data = payload
    else:
        raise ValueError(f"{filename} is not a power-sweep fit-results archive.")

    if run is not None:
        fit_data["run"] = _power_sweep_run_view(run)
    if "run" not in fit_data or not isinstance(fit_data["run"], dict):
        raise ValueError(
            "Loaded fit results do not include run metadata; pass run=..."
        )
    fit_data["summary_array"] = fit_summary_array(fit_data)
    fit_data["run"]["fits"] = fit_data
    return fit_data


def analyse_power_sweep(
    run,
    *,
    nonlinear=True,
    sweep_direction="up",
    n_jobs=-1,
    target_anl=0.01,
    fit=True,
    save=True,
    plot_fits=True,
    plot_best=True,
    best_power=True,
    show=False,
    verbose=True,
    fit_kwargs=None,
    plot_kwargs=None,
    best_power_kwargs=None,
    best_plot_kwargs=None,
):
    """Fit, summarise, plot, and choose best tone powers in one call.

    This is the notebook-friendly path for the usual post-run workflow.  It
    performs nonlinear ANL fitting by default, writes the compact fit archive
    and CSV summary, finds the best power per tone, and writes both fit plots
    and ANL-vs-power diagnostics.

    The stage switches make partial re-runs cheap.  The common case —
    re-choose the powers from the stored fits (say with a new
    ``target_anl``) and refresh only the ANL-vs-power plots, skipping the
    expensive per-tone fit plots::

        ps.analyse_power_sweep(run, fit=False, plot_fits=False,
                               target_anl=0.03)

    Parameters
    ----------
    run : dict or str or Path
        A run from :py:func:`run_power_sweep` / :py:func:`load_power_sweep`,
        or a path to a power-sweep directory or manifest.
    nonlinear : bool or {'auto'}, optional
        How to handle the Duffing ``anl`` nonlinearity (default ``True``, so
        ``anl`` is fitted at every power; see :func:`fit_power_sweep`).
        ``False`` fits the linear model only, and ``'auto'`` keeps low-power
        tones linear and only refines the rest (faster, but drops the
        small-``anl`` values).
    sweep_direction : {'up', 'down'}, optional
        Direction the saved sweeps were taken in (default ``'up'``).
    n_jobs : int, optional
        joblib worker count shared by fitting and both plot passes (``-1``
        all CPUs by default; ``1`` is serial).
    target_anl : float, optional
        Target nonlinearity for :py:func:`find_best_power` (default ``0.01``).
    fit : bool, optional
        Fit the sweeps (``True``) or load an existing fit archive (``False``).
    save : bool, optional
        Write the fit archive, CSV summary, and best-power JSON (default
        ``True``).
    plot_fits : bool, optional
        Write the per-tone fit and parameter plots via
        :py:func:`plot_power_sweep` (default ``True``).  These are the slow
        ones on a big array; disable them when only the power selection has
        changed.
    plot_best : bool, optional
        Write the ANL-vs-power selection plots via :py:func:`plot_best_power`
        (default ``True``; requires ``best_power``).
    best_power : bool, optional
        Run :py:func:`find_best_power` and write ``best_power.json`` (default
        ``True``).
    show : bool, optional
        Default ``show`` forwarded to the plotters (default ``False``).
    verbose : bool, optional
        Default ``verbose`` forwarded to fitting and plotting (default
        ``True``).
    fit_kwargs : dict, optional
        Extra keyword arguments forwarded to :py:func:`fit_power_sweep`
        (e.g. ``min_dip_depth_db``, ``tone_index``, ``initial_guess``,
        ``param_bounds``).  A ``'verbose'`` entry here overrides ``verbose``
        for fitting.
    plot_kwargs : dict, optional
        Extra keyword arguments forwarded to :py:func:`plot_power_sweep`
        (e.g. ``reference_plane``, ``deembed``, ``units``).  ``'show'`` ->
        ``show_overlay`` and ``'verbose'`` / ``'n_jobs'`` here override the
        top-level defaults for the fit plots.
    best_power_kwargs : dict, optional
        Extra keyword arguments forwarded to :py:func:`find_best_power`
        (e.g. ``param_valid_ranges``, ``use_readback_power``,
        ``min_points``).  A ``'target_anl'`` entry here overrides
        ``target_anl``.
    best_plot_kwargs : dict, optional
        Extra keyword arguments forwarded to :py:func:`plot_best_power`.
        ``'show'``, ``'verbose'``, and ``'n_jobs'`` here override the
        top-level defaults for the ANL-vs-power plots.

    Returns
    -------
    result : dict
        ``{'run', 'fits', 'fit_file', 'summary_csv', 'best_power',
        'best_power_file', 'balanced_power', 'balanced_power_file',
        'plots', 'best_power_plots'}``; entries for disabled stages are
        ``None``.  The ``balanced_power`` entries are always ``None`` here
        (balancing is a separate step, :py:func:`balance_tone_powers`);
        they exist so the dict is interchangeable with
        :py:func:`load_analysis` output.
    """

    view = _power_sweep_run_view(run)
    root = Path(view["root"])
    fit_kwargs = dict(fit_kwargs or {})
    plot_kwargs = dict(plot_kwargs or {})
    best_power_kwargs = dict(best_power_kwargs or {})
    best_plot_kwargs = dict(best_plot_kwargs or {})
    fit_verbose = fit_kwargs.pop("verbose", verbose)
    plot_show = plot_kwargs.pop("show", show)
    plot_verbose = plot_kwargs.pop("verbose", verbose)
    best_plot_show = best_plot_kwargs.pop("show", show)
    best_plot_verbose = best_plot_kwargs.pop("verbose", verbose)
    best_target_anl = best_power_kwargs.pop("target_anl", target_anl)

    result = {
        "run": view,
        "fits": None,
        "fit_file": None,
        "summary_csv": None,
        "best_power": None,
        "best_power_file": None,
        "balanced_power": None,
        "balanced_power_file": None,
        "plots": None,
        "best_power_plots": None,
    }

    if fit:
        fits = fit_power_sweep(
            view,
            nonlinear=nonlinear,
            sweep_direction=sweep_direction,
            n_jobs=n_jobs,
            verbose=fit_verbose,
            **fit_kwargs,
        )
    else:
        fits = load_fit_results(root, run=view)
    result["fits"] = fits

    if save:
        result["fit_file"] = str(write_fit_results(fits, root))
        result["summary_csv"] = str(write_fit_summary(fits, root))

    if best_power:
        best = find_best_power(
            fits,
            target_anl=best_target_anl,
            **best_power_kwargs,
        )
        result["best_power"] = best
        if save:
            result["best_power_file"] = str(write_best_power(best, root))
    else:
        best = None

    if plot_fits:
        # Per-tone plots are independent; reuse the run-level n_jobs so the
        # convenience path renders in parallel by default (override via
        # plot_kwargs={'n_jobs': ...}).
        plot_n_jobs = plot_kwargs.pop("n_jobs", n_jobs)
        result["plots"] = plot_power_sweep(
            view,
            fit_data=fits,
            show_overlay=plot_show,
            n_jobs=plot_n_jobs,
            verbose=plot_verbose,
            **plot_kwargs,
        )

    if plot_best and best_power:
        best_plot_n_jobs = best_plot_kwargs.pop("n_jobs", n_jobs)
        result["best_power_plots"] = plot_best_power(
            fits,
            best_power=best,
            target_anl=best_target_anl,
            show=best_plot_show,
            n_jobs=best_plot_n_jobs,
            verbose=best_plot_verbose,
            **best_plot_kwargs,
        )

    return result


def load_analysis(path):
    """Reload a previously analysed power sweep from disk.

    The read-only counterpart of :py:func:`analyse_power_sweep`: nothing is
    recomputed and nothing is written.  The run manifest is loaded, then
    every analysis artifact that exists under ``<run>/analysis/`` — the fit
    archive, the fit-summary CSV, the best-power JSON, and (if a balancing
    step was saved) the balanced-power JSON.

    To *recompute* the power selection or plots from the stored fits
    instead, use ``analyse_power_sweep(path, fit=False, ...)``.

    Parameters
    ----------
    path : str or Path
        Power-sweep run directory or its ``measurement.json`` manifest.

    Returns
    -------
    result : dict
        Same shape as the :py:func:`analyse_power_sweep` result.  Entries
        whose artifact file is missing are ``None``, as are ``plots`` /
        ``best_power_plots`` (plot paths are not recorded on disk).
    """
    view = load_power_sweep(path)
    root = Path(view["root"])
    result = {
        "run": view,
        "fits": None,
        "fit_file": None,
        "summary_csv": None,
        "best_power": None,
        "best_power_file": None,
        "balanced_power": None,
        "balanced_power_file": None,
        "plots": None,
        "best_power_plots": None,
    }

    fit_file = root / FIT_RESULTS_FILE
    if fit_file.exists():
        result["fits"] = load_fit_results(fit_file, run=view)
        result["fit_file"] = str(fit_file)
    summary_file = root / FIT_SUMMARY_FILE
    if summary_file.exists():
        result["summary_csv"] = str(summary_file)
    best_file = root / BEST_POWER_FILE
    if best_file.exists():
        result["best_power"] = load_best_power(best_file)
        result["best_power_file"] = str(best_file)
    balanced_file = root / BALANCED_POWER_FILE
    if balanced_file.exists():
        result["balanced_power"] = load_balanced_power(balanced_file)
        result["balanced_power_file"] = str(balanced_file)
    return result


def _normalise_tone_indices(tone_indices, tone_count):
    """Return validated tone indices as a list of ints."""
    if tone_indices is None:
        tones = list(range(int(tone_count)))
    elif np.isscalar(tone_indices):
        tones = [int(tone_indices)]
    else:
        tones = [int(tone) for tone in tone_indices]
    if not tones:
        raise ValueError("tone_indices must select at least one tone.")
    invalid = [tone for tone in tones if tone < 0 or tone >= int(tone_count)]
    if invalid:
        raise ValueError(
            f"tone_indices contains out-of-range tones {invalid}; "
            f"valid range is 0 to {int(tone_count) - 1}."
        )
    return tones


def _power_row_for_step(run, sweep_index, use_readback=True):
    """Return the requested/readback power row for one sweep step."""
    keys = (
        ("readback_powers_dbm", "powers_dbm")
        if use_readback
        else ("powers_dbm", "readback_powers_dbm")
    )
    for key in keys:
        rows = run.get(key, [])
        if sweep_index < len(rows):
            row = np.asarray(rows[sweep_index], dtype=float).ravel()
            if row.size:
                return row
    return np.array([], dtype=float)


def _power_for_tone(run, sweep_index, tone_index, use_readback=True):
    """Return the selected power for one sweep/tone pair, or NaN."""
    row = _power_row_for_step(run, sweep_index, use_readback=use_readback)
    if tone_index < row.size:
        return float(row[tone_index])
    return np.nan


def _run_power_reference_plane(run):
    """Return the TX power reference plane recorded for a power sweep."""
    manifest = run.get("manifest")
    if isinstance(manifest, dict) and manifest.get("reference_plane") is not None:
        return str(manifest["reference_plane"])
    for sweep in run.get("sweeps", []):
        if isinstance(sweep, dict) and sweep.get("tone_powers_reference_plane") is not None:
            return str(sweep["tone_powers_reference_plane"])
    return None


def _format_power_label(power, sweep_index):
    """Return a compact legend label for one power value."""
    if np.isfinite(power):
        return f"{power:.1f} dBm"
    return f"step {sweep_index}"


def _format_power_row_label(powers, sweep_index):
    """Return a compact legend label for one possibly-multitone power row."""
    powers = np.asarray(powers, dtype=float).ravel()
    finite = powers[np.isfinite(powers)]
    if finite.size == 0:
        return f"step {sweep_index}"
    if finite.size == 1 or np.allclose(finite, finite[0], atol=1e-9, rtol=0.0):
        return f"{finite[0]:.1f} dBm"
    return f"{np.nanmin(finite):.1f} to {np.nanmax(finite):.1f} dBm"


_EMPIRICAL_FALLBACK_PARAMETER_KEYS = {"fr", "Ql", "Qi", "Qc"}


def _fit_uses_empirical_fallback(fit):
    """Return True when nominal fit fields were filled from empirical values."""
    if fit is None:
        return False
    if bool(getattr(fit, "noise_only", False)):
        return True
    # Older persisted results may not have noise_only set but still carry the
    # unresolved-trace shape: no model trace, no optimiser evaluations, and a
    # failed status.
    if getattr(fit, "z_fit", None) is not None:
        return False
    nfev = getattr(fit, "nfev", np.nan)
    try:
        nfev = float(nfev)
    except (TypeError, ValueError):
        nfev = np.nan
    success = getattr(fit, "success", True)
    if isinstance(success, str):
        success = success.strip().lower() not in {"false", "0", "no"}
    failed = success is False or success == 0
    return bool(failed and (not np.isfinite(nfev) or nfev <= 0.0))


def _empirical_fallback_reason(fit):
    """Return a compact plotting reason for an empirical-only fit result."""
    if not _fit_uses_empirical_fallback(fit):
        return None
    text = str(
        getattr(fit, "noise_reason", "")
        or getattr(fit, "message", "")
        or ""
    ).strip()
    lower = text.lower()
    if "empirical dip depth" in lower or (
        "dip depth" in lower and "<" in lower
    ):
        return "min-dip-depth"
    if "too few" in lower:
        return "too-few-points"
    if "noise-only" in lower or bool(getattr(fit, "noise_only", False)):
        return "noise-only"
    if lower:
        return "fit-failed"
    return "no-optimiser-fit"


def _empirical_fallback_reason_summary(fits):
    """Summarise empirical-fallback reasons for concise plot labels."""
    reasons = []
    for fit in fits:
        reason = _empirical_fallback_reason(fit)
        if reason is not None:
            reasons.append(reason)
    if not reasons:
        return ""

    ordered = []
    counts = {}
    for reason in reasons:
        if reason not in counts:
            ordered.append(reason)
            counts[reason] = 0
        counts[reason] += 1
    if len(ordered) == 1:
        return ordered[0]
    return "mixed: " + ", ".join(
        f"{counts[reason]} {reason}" for reason in ordered
    )


def _append_fit_status_label(label, fit):
    """Mark legend labels for traces where only empirical diagnostics exist."""
    reason = _empirical_fallback_reason(fit)
    if reason is not None:
        return f"{label} (no fit: {reason})"
    return label


def _append_fit_row_status_label(label, fits):
    """Mark multitone row labels when any plotted trace used empirical fallback."""
    fits = list(fits)
    fallback_count = sum(
        _empirical_fallback_reason(fit) is not None for fit in fits
    )
    if fallback_count == 0:
        return label
    reason = _empirical_fallback_reason_summary(fits)
    if fallback_count == len(fits):
        return f"{label} (no fit: {reason})"
    return f"{label} ({fallback_count} no fit: {reason})"


def _unique_empirical_fallback_reasons(fits):
    """Return unique fallback reasons in first-seen order."""
    reasons = []
    for fit in fits:
        reason = _empirical_fallback_reason(fit)
        if reason is not None and reason not in reasons:
            reasons.append(reason)
    return reasons


def _plot_transform_title(title, deembed, phase_center, phase_rotate=False):
    """Append the plotted S21 coordinate frame to a figure title."""
    parts = []
    if deembed:
        parts.append("deembedded")
    if phase_center and phase_rotate:
        parts.append("centered + rotated")
    elif phase_center:
        parts.append("centered")
    elif phase_rotate:
        parts.append("rotated")
    state = ", ".join(parts) if parts else "raw"
    return f"{title} ({state})"


def _consolidate_legend_outside(fig):
    """Replace per-subplot legends with one figure-level legend on the right.

    ``plot_fits`` places ``ax.legend()`` on every subplot.  With many power
    steps that legend is large enough to crowd out the data inside the
    axes and is duplicated across mag/phase or grid subplots.  This
    helper collects the unique handle/label pairs from those legends,
    removes them, and adds one ``fig.legend`` on the right.  Callers save
    with ``bbox_inches='tight'`` to lay the figure out, so this helper does
    not call ``tight_layout`` — that would emit an "incompatible Axes"
    warning on the equal-aspect IQ panel of ``mag+phase+iq`` layouts.
    """
    handles, labels, seen = [], [], set()
    for ax in fig.axes:
        existing = ax.get_legend()
        ax_handles, ax_labels = ax.get_legend_handles_labels()
        for handle, label in zip(ax_handles, ax_labels):
            if label and label != "_nolegend_" and label not in seen:
                handles.append(handle)
                labels.append(label)
                seen.add(label)
        if existing is not None:
            existing.remove()
    if handles:
        # Reserve a right-side gutter so the figure legend does not overlap
        # the axes; bbox_inches='tight' at save time crops to content.
        fig.subplots_adjust(right=0.78)
        fig.legend(
            handles,
            labels,
            loc="center right",
            bbox_to_anchor=(0.995, 0.5),
            fontsize="small",
            frameon=True,
        )


def _fits_for_tone(fit_data, tone_index):
    """Return a per-power list of fits for one tone, with None for misses."""
    if fit_data is None:
        return []
    if "fits_by_power" in fit_data:
        fits_for_tone = []
        for fits_at_power in fit_data["fits_by_power"]:
            fit = None
            for fallback_index, candidate in enumerate(fits_at_power):
                if int(getattr(candidate, "tone_index", fallback_index)) == tone_index:
                    fit = candidate
                    break
            fits_for_tone.append(fit)
        return fits_for_tone
    if int(fit_data["tone_index"]) == int(tone_index):
        return list(fit_data["fits"])
    return []


def _fits_for_power(fit_data, sweep_index, tone_indices):
    """Return fits at one power step matching the selected tone indices."""
    if fit_data is None:
        return []
    tone_set = {int(tone) for tone in tone_indices}
    if "fits_by_power" in fit_data:
        if sweep_index >= len(fit_data["fits_by_power"]):
            return []
        fits = []
        for fallback_index, fit in enumerate(
            fit_data["fits_by_power"][sweep_index]
        ):
            tone_index = int(getattr(fit, "tone_index", fallback_index))
            if tone_index in tone_set:
                fits.append(fit)
        return fits
    if sweep_index >= len(fit_data.get("fits", [])):
        return []
    tone_index = int(fit_data["tone_index"])
    return [fit_data["fits"][sweep_index]] if tone_index in tone_set else []


def _make_power_palette(powers):
    """Build ``(colour, add_colorbar)`` helpers from a set of tone powers.

    ``colour(power)`` maps a power onto the rainbow colormap and
    ``add_colorbar(fig)`` adds a matching colorbar in a reserved right-side
    gutter.  Kept at module scope so the per-tone plotting body can run in a
    worker process.
    """
    from matplotlib import cm, colors

    power_cmap = cm.rainbow
    finite = np.asarray(powers, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size:
        pmin = float(np.nanmin(finite))
        pmax = float(np.nanmax(finite))
        if pmax > pmin:
            norm = colors.Normalize(vmin=pmin, vmax=pmax)
        else:
            norm = colors.Normalize(vmin=pmin - 0.5, vmax=pmax + 0.5)
    else:
        norm = None

    def colour(power):
        """Palette colour for a tone power (mid-colour when unscaled/non-finite)."""
        if norm is None or not np.isfinite(power):
            return power_cmap(0.5)
        return power_cmap(norm(float(power)))

    def add_colorbar(fig):
        """Attach a tone-power colorbar to ``fig`` (no-op when unscaled)."""
        if norm is None:
            return
        mappable = cm.ScalarMappable(norm=norm, cmap=power_cmap)
        mappable.set_array([])
        has_figure_legend = bool(getattr(fig, "legends", ()))
        axes_right = 0.64 if has_figure_legend else 0.82
        cbar_left = 0.68 if has_figure_legend else 0.86
        cbar_width = 0.018
        fig.subplots_adjust(right=axes_right)
        axes = [ax for ax in fig.axes if ax.get_visible()]
        positions = [ax.get_position() for ax in axes]
        bottom = min(pos.y0 for pos in positions)
        top = max(pos.y1 for pos in positions)
        cax = fig.add_axes([cbar_left, bottom, cbar_width, top - bottom])
        cbar = fig.colorbar(mappable, cax=cax)
        cbar.set_label("Tone power (dBm)")

    return colour, add_colorbar


def _plot_one_tone(
    run,
    fit_data,
    tone_index,
    *,
    format,
    use_readback,
    units,
    config,
    reference_plane,
    tx_power_reference_plane,
    deembed,
    phase_center,
    phase_rotate,
    mag_centered,
    phase_centered,
    mag_rotated,
    phase_rotated,
    group_delay_cal,
    unwrap_phase,
    fit_figsize,
    dpi,
    output_dir,
    parameters,
    parameter_figsize,
    ncol,
    parameter_show_errors,
    parameter_colour_points,
):
    """Render and save one tone's fit-overlay PNG and parameter-trend PNG.

    Returns ``{'fits': [...], 'parameters': [...]}`` of the PNG paths written
    for this tone.  Pulled out of :py:func:`plot_power_sweep` so the per-tone
    work can run either serially or in a worker process; both paths share this
    one body so they render identically.
    """
    import matplotlib.pyplot as plt

    output_dir = Path(output_dir)
    paths = {"fits": [], "parameters": []}
    n_sweeps = len(run["sweeps"])
    tone_index = int(tone_index)

    powers = np.asarray(
        [
            _power_for_tone(run, sweep_index, tone_index, use_readback=use_readback)
            for sweep_index in range(n_sweeps)
        ],
        dtype=float,
    )
    power_colour, add_power_colorbar = _make_power_palette(powers)

    def _sweep_info(sweep_index):
        """Return metadata for one loaded sweep, if available."""
        if sweep_index >= n_sweeps:
            return None
        sweep = run["sweeps"][sweep_index]
        return sweep.get("info") if isinstance(sweep, dict) else None

    fits_for_tone = _fits_for_tone(fit_data, tone_index)
    title = _plot_transform_title(
        f"Tone {tone_index} fitted power sweep",
        deembed,
        phase_center,
        phase_rotate,
    )

    fig = None
    for sweep_index, fit in enumerate(fits_for_tone):
        power = _power_for_tone(
            run, sweep_index, tone_index, use_readback=use_readback
        )
        if fit is not None:
            fig = plot_fits(
                fit,
                format=format,
                fig=fig,
                label="_nolegend_",
                show_errors=False,
                title=title,
                color=power_colour(power),
                deembed=deembed,
                phase_center=phase_center,
                phase_rotate=phase_rotate,
                mag_centered=mag_centered,
                phase_centered=phase_centered,
                mag_rotated=mag_rotated,
                phase_rotated=phase_rotated,
                group_delay_cal=group_delay_cal,
                unwrap_phase=unwrap_phase,
                units=units,
                info=_sweep_info(sweep_index),
                config=config,
                reference_plane=reference_plane,
                tx_power_dbm=power,
                tx_power_reference_plane=tx_power_reference_plane,
                figsize=fit_figsize,
                finalize=False,
            )
        elif sweep_index < n_sweeps and run["sweeps"][sweep_index] is not None:
            fig = plot_sweep(
                run["sweeps"][sweep_index],
                format=format,
                tones=[tone_index],
                fig=fig,
                label="_nolegend_",
                show_errors=False,
                title=title,
                color=power_colour(power),
                deembed=deembed,
                phase_center=phase_center,
                phase_rotate=phase_rotate,
                mag_centered=mag_centered,
                phase_centered=phase_centered,
                mag_rotated=mag_rotated,
                phase_rotated=phase_rotated,
                group_delay_cal=group_delay_cal,
                unwrap_phase=unwrap_phase,
                units=units,
                config=config,
                reference_plane=reference_plane,
                tx_power_dbm=power,
                tx_power_reference_plane=tx_power_reference_plane,
            )
    if fig is not None:
        full_title = title
        no_fit_count = sum(
            _empirical_fallback_reason(fit) is not None for fit in fits_for_tone
        )
        if no_fit_count:
            full_title += (
                f" ({no_fit_count} no fit: "
                f"{_empirical_fallback_reason_summary(fits_for_tone)})"
            )
        fig.suptitle(full_title)
        _consolidate_legend_outside(fig)
        add_power_colorbar(fig)
        fit_path = output_dir / f"tone_{tone_index:04d}_fits.png"
        # bbox_inches='tight' lays the figure out at save time without
        # tight_layout's "incompatible Axes" warning on the equal-aspect IQ
        # panel, and avoids clipping long calibrated axis labels.
        fig.savefig(fit_path, dpi=dpi, bbox_inches="tight")
        plt.close(fig)
        paths["fits"].append(str(fit_path))

    _plot_power_sweep_parameters(
        run,
        fit_data,
        tone_index,
        parameters,
        use_readback,
        output_dir,
        power_colour,
        False,
        paths,
        dpi=dpi,
        figsize=parameter_figsize,
        ncol=ncol,
        show_errors=parameter_show_errors,
        colour_points=parameter_colour_points,
    )
    return paths


# Worker-process state for parallel per-tone plotting. ``run`` and ``fit_data``
# are large, so they are shipped to each worker once via the pool initializer
# instead of being re-pickled with every tone task.
_PLOT_WORKER_STATE = {}


def _plot_worker_init(run, fit_data, common_kwargs):
    """Pool initializer: stash shared data and force a headless backend."""
    import matplotlib

    matplotlib.use("Agg")
    _PLOT_WORKER_STATE["run"] = run
    _PLOT_WORKER_STATE["fit_data"] = fit_data
    _PLOT_WORKER_STATE["common"] = common_kwargs


def _plot_one_tone_worker(tone_index):
    """Render one tone inside a worker using the stashed shared data."""
    state = _PLOT_WORKER_STATE
    return _plot_one_tone(
        state["run"], state["fit_data"], int(tone_index), **state["common"]
    )


def plot_power_sweep(
    run,
    fit_data=None,
    tone_indices=None,
    output_dir=None,
    use_readback=True,
    format="magphase",
    parameters=("fr", "Qi", "Qc", "anl"),
    show_overlay=False,
    save_overlay=True,
    dpi=80,
    fit_figsize=(8.0, 4.8),
    parameter_figsize=None,
    ncol=1,
    parameter_show_errors=True,
    parameter_colour_points=False,
    deembed=False,
    phase_center=False,
    phase_rotate=False,
    mag_centered=None,
    phase_centered=None,
    mag_rotated=None,
    phase_rotated=None,
    group_delay_cal=None,
    unwrap_phase=None,
    units=None,
    config=None,
    reference_plane="adc_input",
    n_jobs=1,
    verbose=True,
):
    """Save fitted overlays and parameter plots for selected tones.

    For every selected tone, two PNGs are written into ``output_dir``
    (created if missing):

    1. ``tone_<NNNN>_fits.png`` — measured trace and fitted model for that
       one tone, overlaid across all power steps.
    2. ``tone_<NNNN>_params.png`` — requested fit parameters vs. tone
       power, one compact subplot per parameter.  Error bars are drawn with
       one vectorized artist per subplot; per-power point colours are disabled
       by default because this helper is usually used in batch mode for many
       resonators.

    A single combined ``tones_fits_overlay.png`` is also written, with all
    selected tones overlaid on shared axes and coloured by tone power.  All
    saved figures are closed after writing, except the overlay when
    ``show_overlay=True``.

    Parameters
    ----------
    run : dict or str or Path
        Run dict from :py:func:`run_power_sweep` /
        :py:func:`load_power_sweep`, or a path to load on the fly.
    fit_data : dict or None, optional
        Output of :py:func:`fit_power_sweep`.  If ``None`` (default), the
        ``fits`` entry stored on ``run`` by ``fit_power_sweep`` is used.
        A ``ValueError`` is raised if no fit data is available.
    tone_indices : int, iterable of int, or None, optional
        Which tones to plot.  ``None`` (default) plots every tone.  Pass an
        ``int`` for one tone, or any iterable such as ``[0, 5, 9]``,
        ``range(0, tone_count, 10)``, or ``np.flatnonzero(mask)`` for a
        subset when a full per-tone PNG dump would be too large.
    output_dir : str, Path, or None, optional
        Directory for PNG output (created if missing).  ``None`` (default)
        uses ``<run['root']>/plots``.
    use_readback : bool, optional
        ``True`` (default) labels and positions each fit by the readback
        tone power (what the hardware actually delivered).  ``False`` uses
        the requested tone power instead.  Falls back to the other when
        the chosen series is empty for that sweep/tone.
    format : {'magphase', 'mag+phase+iq', 'iq', 'iq_vs_f'}, optional
        Trace format forwarded to :py:func:`plot_fits` and
        :py:func:`plot_sweep`. Default ``'magphase'``.
    parameters : iterable of str, optional
        Names of fit parameters to plot vs. tone power, one subplot each.
        Valid names are any field exposed by ``extract_parameters``,
        including ``'fr'``, ``'Ql'``, ``'Qi'``, ``'Qc'``, ``'Qc_abs'``,
        ``'phi'``, ``'a'``, ``'alpha'``, ``'tau'``, ``'anl'``,
        ``'nonlinear_detuning_hz'``, and the ``'empirical_*'`` estimates
        (e.g. ``'empirical_fr'``, ``'empirical_Qi'``).  All quantities
        are plotted in their raw units (``fr`` in Hz, Q's
        dimensionless).  Default ``('fr', 'Qi', 'Qc', 'anl')``.
    show_overlay : bool, optional
        If ``True``, leave the combined all-tones overlay figure open at
        the end (useful for interactive sessions; call ``plt.show()`` from
        a script).  Per-tone fit and parameter figures are always closed
        after saving.  Default ``False``.
    save_overlay : bool, optional
        If ``False``, skip the combined all-tones overlay PNG.  This is useful
        for large arrays where the overlay is visually crowded and expensive
        to render.  Default ``True`` preserves the historical output.
    dpi : int or float, optional
        Resolution for saved PNGs.  Default ``80`` keeps batch plot output
        light: ``savefig`` is the dominant per-figure cost and its raster
        time scales with ``dpi**2``.  Raise it for publication-style figures.
    fit_figsize : tuple or None, optional
        ``(width, height)`` in inches for fitted-trace figures.  ``None`` uses
        :py:func:`plot_fits` defaults.  Default ``(8.0, 4.8)``.
    parameter_figsize : tuple or None, optional
        ``(width, height)`` in inches for parameter-trend figures.  ``None``
        uses a compact size based on the number of plotted parameters.
    ncol : int, optional
        Number of columns in each per-tone parameter figure (default ``1``).
        Additional parameters fill rows across this grid before extending the
        figure downward.
    parameter_show_errors : bool, optional
        Draw vertical 1-σ parameter error bars on the parameter-trend figures.
        Default ``True``.  Stored uncertainties are inflated by
        ``sqrt(max(reduced_chi2, 1))``.
    parameter_colour_points : bool, optional
        Colour parameter-trend points by tone power, matching fit overlays.
        Default ``False`` uses one lightweight line+marker artist per subplot.
    deembed : bool, optional
        Apply RF deembedding to the traces before plotting (default
        ``False``).  Forwarded to :py:func:`plot_fits`.  For mag/phase plots
        with calibrated magnitude units (``'dbfs'``, ``'dbm'``, ``'volts'``,
        ``'watts'``, ``'s21'``), the magnitude axis stays as received power
        and deembedding is applied to the phase axis only.
    phase_center : bool, optional
        Apply the centering step before plotting (default ``False``).
        Forwarded to :py:func:`plot_fits`. Acts as the fallback for the
        per-axis centering knobs below.
    phase_rotate : bool, optional
        Apply the rotation step after centering (default ``False``).
    mag_centered : bool or None, optional
        Per-axis override for the magnitude subplot's centering step.
        ``None`` (default) inherits ``phase_center``. Set ``False`` together
        with ``phase_centered=True`` to keep the resonance dip on the
        magnitude plot while still flattening the phase plot.
    phase_centered : bool or None, optional
        Per-axis override for the phase subplot's centering step.
        ``None`` (default) inherits ``phase_center``. Ignored for non-
        ``magphase`` formats.
    mag_rotated : bool or None, optional
        Per-axis override for the magnitude subplot's rotation step.
        ``None`` (default) inherits ``phase_rotate``.
    phase_rotated : bool or None, optional
        Per-axis override for the phase subplot's rotation step.
        ``None`` (default) inherits ``phase_rotate``.
    group_delay_cal : optional
        Group-delay calibration object forwarded to :py:func:`plot_fits`.
        Default ``None`` (no group-delay correction).
    unwrap_phase : bool or None, optional
        Phase wrapping control forwarded to :py:func:`plot_fits` and
        :py:func:`plot_sweep`. ``None`` preserves the plotting default:
        unwrap unrotated phase traces, including center-only traces, but
        leave explicitly rotated phase wrapped.
    units : {'raw', 'peak', 'adc_units', 'adc', 'adc_fs', 'dbfs', 'dbm', 'volts', 'v', 'watts', 'w', 's21', 's21_log', 's21_linear'} or None, optional
        Trace amplitude units forwarded to :py:func:`plot_fits` and
        :py:func:`plot_sweep`.  Use ``'dbfs'`` with ``reference_plane`` to
        plot calibrated log magnitude referred to the ADC input, cryostat
        output, or detector without requiring an absolute dBm calibration.
        Use ``'adc_units'`` (or ``'adc'``) for linear ADC units with the
        firmware accumulator path undone and, for non-ADC reference planes,
        RX-chain gain removed.
        Use ``'dbm'`` for received power at ``reference_plane``; IQ panels use
        compact SI-prefixed RMS volts. Use ``'volts'``/``'v'`` for RMS voltage
        or ``'watts'``/``'w'`` for power at ``reference_plane``; IQ panels use
        compact SI-prefixed RMS volts for both of these units. Use
        ``'s21_log'`` (or the backwards-compatible alias
        ``'s21'``) for received power minus the TX power resolved at the same
        reference plane from each sweep's structured info/calibration, falling
        back to the programmed/readback power row for older data. Use
        ``'s21_linear'`` for the corresponding linear |S21| ratio.
        ``'raw'`` keeps accumulator units and is only valid with
        ``reference_plane='adc_input'``.
        ``None`` (default) preserves raw accumulator-unit plots for
        ``reference_plane='adc_input'`` and selects ``'dbfs'`` when a
        calibrated non-default reference plane is requested.
    config : dict or None, optional
        Config dict for calibration.  When ``None`` (default), plotting uses
        the config/calibration metadata embedded in each loaded sweep when
        available.
    reference_plane : {'adc_input', 'cryostat_output', 'detector'}, optional
        Reference plane forwarded to :py:func:`plot_fits` and
        :py:func:`plot_sweep`. ADC/RX-chain planes use calibrated magnitude
        units. ``units='raw'`` is accumulator units only; use
        ``units='adc_units'`` for a linear ADC-unit view referred to a
        detector/cryostat plane.
    n_jobs : int, optional
        Worker processes used for the per-tone fit/parameter PNGs, following
        the joblib convention: ``1`` (default) serial, ``-1`` all visible
        CPUs, ``-2`` all but one.  Each tone is independent, so saving them is
        embarrassingly parallel and ``savefig`` (the dominant cost) is
        CPU-bound; on a many-tone run this is the largest speedup.  The
        combined overlay PNG is always built serially.  Requires ``config``
        and ``group_delay_cal`` (when given) to be picklable.
    verbose : bool or int, optional
        Print compact plotting progress (default ``True``).  ``False``
        suppresses progress; values ``>=2`` print one line per tone and
        overlay sweep.

    Returns
    -------
    paths : dict
        ``{'fits': [...], 'parameters': [...]}`` of saved PNG paths as
        strings, in the order they were written.  The combined overlay
        PNG is appended to ``paths['fits']`` last.

    Raises
    ------
    ValueError
        If no fit data is available, or ``tone_indices`` includes an
        out-of-range tone.
    """
    import matplotlib.pyplot as plt
    from matplotlib import cm, colors
    verbose = _verbose_level(verbose)
    _validate_reference_plane(reference_plane)
    if units is None:
        raw_planes = {"adc_input"}
        units = "raw" if reference_plane in raw_planes else "dbfs"

    run = _power_sweep_run_view(run)
    if fit_data is None:
        fit_data = run.get("fits")
    if fit_data is None:
        raise ValueError(
            "fit_data is required for plot_power_sweep(); call "
            "fit_power_sweep() first or pass its result."
        )
    tx_power_reference_plane = _run_power_reference_plane(run)
    # For S21 units, each sweep's structured info records the live tone powers
    # and calibration state, so plot_fits()/plot_sweep() can resolve the TX
    # denominator at the requested RX reference plane.  The run-level plane is
    # kept as a fallback for older saved sweeps.
    tone_indices = _normalise_tone_indices(tone_indices, run["tone_count"])
    output_dir = (
        Path(output_dir)
        if output_dir is not None
        else Path(run["root"]) / "plots"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    if verbose:
        _progress(
            verbose,
            f"Plotting {len(tone_indices)} tone(s) to {output_dir}",
        )

    power_cmap = cm.rainbow

    tone_powers = {
        int(tone_index): np.asarray(
            [
                _power_for_tone(
                    run, sweep_index, tone_index, use_readback=use_readback
                )
                for sweep_index in range(len(run["sweeps"]))
            ],
            dtype=float,
        )
        for tone_index in tone_indices
    }

    def _sweep_info(sweep_index):
        """Return metadata for one loaded sweep, if available."""
        if sweep_index >= len(run.get("sweeps", [])):
            return None
        sweep = run["sweeps"][sweep_index]
        if not isinstance(sweep, dict):
            return None
        return sweep.get("info")

    def _powers_uniform_across_tones():
        if not tone_powers:
            return True
        stack = np.column_stack(list(tone_powers.values()))
        if stack.shape[1] < 2:
            return True
        for row in stack:
            finite = row[np.isfinite(row)]
            if finite.size > 1 and not np.allclose(finite, finite[0],
                                                   atol=1e-9, rtol=0.0):
                return False
        return True

    overlay_uniform = _powers_uniform_across_tones()
    if overlay_uniform:
        all_powers = (
            np.concatenate(list(tone_powers.values()))
            if tone_powers
            else np.array([], dtype=float)
        )
        overlay_power_palette, overlay_add_colorbar = _make_power_palette(
            all_powers
        )

        def overlay_colour(sweep_index, power):  # noqa: ARG001
            return overlay_power_palette(power)
    else:
        n_steps = len(run["sweeps"])
        if n_steps > 1:
            step_norm = colors.Normalize(vmin=0, vmax=n_steps - 1)
        elif n_steps == 1:
            step_norm = colors.Normalize(vmin=-0.5, vmax=0.5)
        else:
            step_norm = None

        def overlay_colour(sweep_index, power):  # noqa: ARG001
            if step_norm is None:
                return power_cmap(0.5)
            return power_cmap(step_norm(int(sweep_index)))

        def overlay_add_colorbar(fig):
            if step_norm is None:
                return
            mappable = cm.ScalarMappable(norm=step_norm, cmap=power_cmap)
            mappable.set_array([])
            has_figure_legend = bool(getattr(fig, "legends", ()))
            axes_right = 0.64 if has_figure_legend else 0.82
            cbar_left = 0.68 if has_figure_legend else 0.86
            cbar_width = 0.018
            fig.subplots_adjust(right=axes_right)
            axes = [ax for ax in fig.axes if ax.get_visible()]
            positions = [ax.get_position() for ax in axes]
            bottom = min(pos.y0 for pos in positions)
            top = max(pos.y1 for pos in positions)
            cax = fig.add_axes([cbar_left, bottom, cbar_width, top - bottom])
            cbar = fig.colorbar(mappable, cax=cax)
            cbar.set_label("Power step")

    paths = {"fits": [], "parameters": []}
    overlay_title = _plot_transform_title(
        "Fitted power sweep",
        deembed,
        phase_center,
        phase_rotate,
    )

    # Per-tone fits PNG + parameter-trend PNG, one of each per tone. The work
    # for each tone is independent, so it runs serially or, when n_jobs asks
    # for more than one worker, across processes. Both paths call
    # _plot_one_tone, so the saved figures are identical either way.
    common = dict(
        format=format,
        use_readback=use_readback,
        units=units,
        config=config,
        reference_plane=reference_plane,
        tx_power_reference_plane=tx_power_reference_plane,
        deembed=deembed,
        phase_center=phase_center,
        phase_rotate=phase_rotate,
        mag_centered=mag_centered,
        phase_centered=phase_centered,
        mag_rotated=mag_rotated,
        phase_rotated=phase_rotated,
        group_delay_cal=group_delay_cal,
        unwrap_phase=unwrap_phase,
        fit_figsize=fit_figsize,
        dpi=dpi,
        output_dir=str(output_dir),
        parameters=parameters,
        parameter_figsize=parameter_figsize,
        ncol=ncol,
        parameter_show_errors=parameter_show_errors,
        parameter_colour_points=parameter_colour_points,
    )
    workers = _resolve_n_jobs(n_jobs, len(tone_indices))
    plot_start = time.time()
    report_every = _progress_report_every(len(tone_indices))
    results_by_tone = {}
    running = {"fits": 0, "parameters": 0}

    def _on_tone_done(completed, tone_index, tone_paths):
        """Record one tone's saved plot paths and update progress counters."""
        results_by_tone[int(tone_index)] = tone_paths
        running["fits"] += len(tone_paths["fits"])
        running["parameters"] += len(tone_paths["parameters"])
        if verbose >= 2:
            _progress(
                verbose,
                f"  Tone {tone_index}: saved {len(tone_paths['fits'])} fit "
                f"plot(s), {len(tone_paths['parameters'])} parameter plot(s)",
            )
        elif verbose == 1 and _progress_should_report(
            completed, len(tone_indices), report_every
        ):
            _print_progress_line(
                _format_plot_progress(
                    "tones",
                    completed,
                    len(tone_indices),
                    time.time() - plot_start,
                    final=completed == len(tone_indices),
                    detail=(
                        f"fit_png={running['fits']}, "
                        f"param_png={running['parameters']}"
                    ),
                ),
                final=completed == len(tone_indices),
            )

    if workers == 1:
        for completed, tone_index in enumerate(tone_indices, start=1):
            tone_paths = _plot_one_tone(
                run, fit_data, int(tone_index), **common
            )
            _on_tone_done(completed, int(tone_index), tone_paths)
    else:
        _progress(verbose, f"  Rendering tones across {workers} processes")
        with ProcessPoolExecutor(
            max_workers=workers,
            initializer=_plot_worker_init,
            initargs=(run, fit_data, common),
        ) as pool:
            future_to_tone = {
                pool.submit(_plot_one_tone_worker, int(tone_index)): int(tone_index)
                for tone_index in tone_indices
            }
            for completed, future in enumerate(
                as_completed(future_to_tone), start=1
            ):
                tone_index = future_to_tone[future]
                _on_tone_done(completed, tone_index, future.result())

    # Merge in tone order so paths are stable regardless of completion order.
    for tone_index in tone_indices:
        tone_paths = results_by_tone[int(tone_index)]
        paths["fits"].extend(tone_paths["fits"])
        paths["parameters"].extend(tone_paths["parameters"])

    if verbose >= 2 and tone_indices:
        _progress(
            verbose,
            _format_plot_progress(
                "tones",
                len(tone_indices),
                len(tone_indices),
                time.time() - plot_start,
                final=True,
                detail=(
                    f"fit_png={len(paths['fits'])}, "
                    f"param_png={len(paths['parameters'])}"
                ),
            ),
        )

    # Combined all-tones overlay PNG. Left open when show_overlay=True.
    overlay_fig = None
    if save_overlay:
        overlay_start = time.time()
        overlay_total = len(run["sweeps"])
        overlay_report_every = _progress_report_every(overlay_total)
        overlay_trace_count = 0
        if verbose:
            _progress(
                verbose,
                f"Plotting combined overlay across {overlay_total} sweep(s)",
            )
        for completed, sweep_index in enumerate(range(overlay_total), start=1):
            fits_at_power = _fits_for_power(fit_data, sweep_index, tone_indices)
            fitted_tones = {
                int(getattr(fit, "tone_index", idx))
                for idx, fit in enumerate(fits_at_power)
            }
            sweep_at_power = (
                run["sweeps"][sweep_index]
                if sweep_index < len(run["sweeps"])
                else None
            )
            unfitted_tones = (
                [int(t) for t in tone_indices if int(t) not in fitted_tones]
                if sweep_at_power is not None
                else []
            )
            traces_at_power = len(fits_at_power) + len(unfitted_tones)
            overlay_trace_count += traces_at_power
            if not traces_at_power:
                if verbose >= 2:
                    _progress(verbose, f"  Overlay sweep {sweep_index}: 0 trace(s)")
                elif verbose == 1 and _progress_should_report(
                    completed, overlay_total, overlay_report_every
                ):
                    _print_progress_line(
                        _format_plot_progress(
                            "overlay sweeps",
                            completed,
                            overlay_total,
                            time.time() - overlay_start,
                            final=completed == overlay_total,
                            detail=f"traces={overlay_trace_count}",
                        ),
                        final=completed == overlay_total,
                    )
                continue
            for fallback_index, fit in enumerate(fits_at_power):
                tone_index = int(getattr(fit, "tone_index", fallback_index))
                power = _power_for_tone(
                    run,
                    sweep_index,
                    tone_index,
                    use_readback=use_readback,
                )
                overlay_fig = plot_fits(
                    fit,
                    format=format,
                    multi_tone="overlay",
                    fig=overlay_fig,
                    label="_nolegend_",
                    show_errors=False,
                    title=overlay_title,
                    color=overlay_colour(sweep_index, power),
                    deembed=deembed,
                    phase_center=phase_center,
                    phase_rotate=phase_rotate,
                    mag_centered=mag_centered,
                    phase_centered=phase_centered,
                    mag_rotated=mag_rotated,
                    phase_rotated=phase_rotated,
                    group_delay_cal=group_delay_cal,
                    unwrap_phase=unwrap_phase,
                    units=units,
                    info=_sweep_info(sweep_index),
                    config=config,
                    reference_plane=reference_plane,
                    tx_power_dbm=power,
                    tx_power_reference_plane=tx_power_reference_plane,
                    figsize=fit_figsize,
                    label_suffix=False,
                    finalize=False,
                )
            for tone_index in unfitted_tones:
                power = _power_for_tone(
                    run,
                    sweep_index,
                    tone_index,
                    use_readback=use_readback,
                )
                overlay_fig = plot_sweep(
                    sweep_at_power,
                    format=format,
                    tones=[int(tone_index)],
                    multi_tone="overlay",
                    fig=overlay_fig,
                    label="_nolegend_",
                    show_errors=False,
                    title=overlay_title,
                    color=overlay_colour(sweep_index, power),
                    deembed=deembed,
                    phase_center=phase_center,
                    phase_rotate=phase_rotate,
                    mag_centered=mag_centered,
                    phase_centered=phase_centered,
                    mag_rotated=mag_rotated,
                    phase_rotated=phase_rotated,
                    group_delay_cal=group_delay_cal,
                    unwrap_phase=unwrap_phase,
                    units=units,
                    config=config,
                    reference_plane=reference_plane,
                    tx_power_dbm=power,
                    tx_power_reference_plane=tx_power_reference_plane,
                )
            if verbose >= 2:
                _progress(
                    verbose,
                    f"  Overlay sweep {sweep_index}: {traces_at_power} trace(s)",
                )
            elif verbose == 1 and _progress_should_report(
                completed, overlay_total, overlay_report_every
            ):
                _print_progress_line(
                    _format_plot_progress(
                        "overlay sweeps",
                        completed,
                        overlay_total,
                        time.time() - overlay_start,
                        final=completed == overlay_total,
                        detail=f"traces={overlay_trace_count}",
                    ),
                    final=completed == overlay_total,
                )
        if verbose >= 2 and overlay_total:
            _progress(
                verbose,
                _format_plot_progress(
                    "overlay sweeps",
                    overlay_total,
                    overlay_total,
                    time.time() - overlay_start,
                    final=True,
                    detail=f"traces={overlay_trace_count}",
                ),
            )
        if overlay_fig is not None:
            overlay_fig.suptitle(overlay_title)
            _consolidate_legend_outside(overlay_fig)
            overlay_add_colorbar(overlay_fig)
            overlay_path = output_dir / "tones_fits_overlay.png"
            overlay_fig.savefig(overlay_path, dpi=dpi, bbox_inches="tight")
            if not show_overlay:
                plt.close(overlay_fig)
            paths["fits"].append(str(overlay_path))
            _progress(verbose, f"  Saved {overlay_path.name}")
        elif verbose:
            _progress(verbose, "  No overlay traces available to save")

    return paths


_PARAMETER_PLOT_LABELS = {
    "fr": "fr",
    "Ql": "Ql",
    "Qi": "Qi",
    "Qc": "Qc",
    "Qc_abs": "|Qc|",
    "phi": "phi",
    "a": "a",
    "alpha": "alpha",
    "tau": "tau",
    "anl": "ANL",
    "nonlinear_detuning_hz": "detuning",
    "empirical_fr": "empirical\nfr",
    "empirical_linewidth_hz": "empirical\nwidth",
    "empirical_Ql": "empirical\nQl",
    "empirical_Qc": "empirical\nQc",
    "empirical_Qi": "empirical\nQi",
    "empirical_dip_depth_db": "empirical\ndip depth",
    "empirical_skew": "empirical\nskew",
    "residual_rms": "resid RMS",
    "weighted_rms": "weighted RMS",
    "reduced_chi2": "red chi2",
}


_Q_PARAMETER_KEYS = {
    "Ql",
    "Qi",
    "Qc",
    "Qc_abs",
    "empirical_Ql",
    "empirical_Qc",
    "empirical_Qi",
}


def _apply_parameter_axis_ticks(ax, name, values):
    """Format parameter trend ticks with parameter-aware notation."""
    if str(name) in _Q_PARAMETER_KEYS:
        from matplotlib.ticker import FuncFormatter

        def _format_q(value, _pos):
            if not np.isfinite(value):
                return ""
            if abs(value) < np.finfo(float).tiny:
                return "0"
            if (
                abs(value) >= 100.0
                or abs(value - round(value)) < max(1e-9, abs(value) * 1e-9)
            ):
                return f"{value:,.0f}"
            return f"{value:,.3g}"

        ax.yaxis.set_major_formatter(FuncFormatter(_format_q))
        ax.yaxis.get_offset_text().set_visible(False)
        return

    _apply_compact_scientific_ticks(ax, axis="y", values=values)


def _plot_power_sweep_parameters(
    run,
    fit_data,
    tone_index,
    parameters,
    use_readback,
    output_dir,
    power_colour,
    show,
    paths,
    *,
    dpi=80,
    figsize=None,
    ncol=1,
    show_errors=True,
    colour_points=False,
):
    """Save one tone's fit-parameter trends, if fits are available."""
    import matplotlib.pyplot as plt

    if fit_data is None:
        return

    good = [
        (i, fit)
        for i, fit in enumerate(_fits_for_tone(fit_data, tone_index))
        if fit is not None
    ]
    if not good:
        return
    fit_params = extract_parameters([fit for _, fit in good])
    power_axis = []
    for sweep_index, _ in good:
        power_axis.append(
            _power_for_tone(run, sweep_index, tone_index, use_readback=use_readback)
        )

    parameters = tuple(parameters)
    ncol = max(1, int(ncol))
    ncol = min(ncol, max(len(parameters), 1))
    nrow = int(np.ceil(len(parameters) / float(ncol)))
    good_fits = [fit for _, fit in good]
    empirical_fallback = np.asarray(
        [_fit_uses_empirical_fallback(fit) for fit in good_fits],
        dtype=bool,
    )
    fallback_reasons = np.asarray(
        [_empirical_fallback_reason(fit) or "" for fit in good_fits],
        dtype=object,
    )
    has_fallback = bool(np.any(empirical_fallback))
    if show_errors:
        # Inflate the stored measurement-noise σ by sqrt(max(reduced χ², 1)) so
        # the displayed bars also reflect degeneracy / unmodelled systematics /
        # local-minimum jitter. No deflation when the fit is better than
        # expected. Matches scipy.optimize.curve_fit's default convention.
        chi2 = np.asarray(
            [
                float(getattr(fit, "reduced_chi2", np.nan))
                for fit in good_fits
            ],
            dtype=float,
        )
        chi2_scale = np.sqrt(
            np.where(np.isfinite(chi2) & (chi2 > 1.0), chi2, 1.0)
        )
    else:
        chi2_scale = None
    fig, axes = plt.subplots(
        nrow,
        ncol,
        sharex=True,
        figsize=figsize or (5.8 * ncol, max(2.2, 1.15 * nrow)),
        squeeze=False,
    )
    axes = np.asarray(axes).ravel()
    point_colours = (
        [power_colour(power) for power in power_axis]
        if colour_points else None
    )
    for ax, name in zip(axes, parameters):
        values = np.asarray(fit_params[name], dtype=float)
        ylabel = _PARAMETER_PLOT_LABELS.get(str(name), str(name))
        if colour_points:
            ax.plot(power_axis, values, color="0.55", linewidth=0.65, alpha=0.6)
            ax.scatter(
                power_axis,
                values,
                c=point_colours,
                marker=".",
                s=12,
                zorder=3,
            )
        else:
            ax.plot(
                power_axis,
                values,
                ".-",
                color="0.2",
                linewidth=0.75,
                markersize=3.0,
                zorder=3,
            )
        if show_errors:
            sigmas = np.asarray(
                [
                    (getattr(fit, "parameter_uncertainties", None) or {}).get(
                        name, np.nan
                    )
                    for fit in good_fits
                ],
                dtype=float,
            ) * chi2_scale
            err_mask = np.isfinite(sigmas) & (sigmas > 0.0)
            if np.any(err_mask):
                x_err = np.asarray(power_axis, dtype=float)[err_mask]
                y_err = values[err_mask]
                sigma_err = sigmas[err_mask]
                valid = np.isfinite(x_err) & np.isfinite(y_err)
                if np.any(valid):
                    ax.errorbar(
                        x_err[valid],
                        y_err[valid],
                        yerr=sigma_err[valid],
                        fmt="none",
                        ecolor="0.55",
                        elinewidth=0.7,
                        capsize=1.8,
                        alpha=0.65,
                        zorder=2,
                    )
        ax.set_ylabel(ylabel, fontsize=8)
        param_fallback = (
            empirical_fallback
            if name in _EMPIRICAL_FALLBACK_PARAMETER_KEYS
            else np.zeros_like(empirical_fallback)
        )
        if np.any(param_fallback):
            for reason in _unique_empirical_fallback_reasons(good_fits):
                reason_mask = param_fallback & (fallback_reasons == reason)
                fallback_x = np.asarray(power_axis, dtype=float)[reason_mask]
                fallback_y = values[reason_mask]
                valid = np.isfinite(fallback_x) & np.isfinite(fallback_y)
                if not np.any(valid):
                    continue
                ax.scatter(
                    fallback_x[valid],
                    fallback_y[valid],
                    marker="s",
                    facecolors="none",
                    edgecolors="tab:red",
                    linewidths=0.8,
                    s=24,
                    label=f"no fit: {reason}",
                    zorder=5,
                )
                ax.legend(fontsize=6.5, loc="best", frameon=True)
        ax.tick_params(axis="both", labelsize=7, length=2.5)
        ax.grid(True, alpha=0.18, linewidth=0.5)
        # Keep the y-range bound to the values, so optional large error bars
        # do not blow out the view. Bars that extend beyond are clipped.
        finite_vals = values[np.isfinite(values)]
        if finite_vals.size:
            vmin = float(np.min(finite_vals))
            vmax = float(np.max(finite_vals))
            span = vmax - vmin
            if span <= 0:
                # All values identical: give a tiny relative pad so the axis
                # is not zero-height.
                span = max(abs(vmin) * 1e-6, 1e-12)
            pad = 0.08 * span
            ax.set_ylim(vmin - pad, vmax + pad)
            _apply_parameter_axis_ticks(ax, name, finite_vals)

    for index, ax in enumerate(axes[: len(parameters)]):
        if index // ncol == nrow - 1:
            ax.set_xlabel("Tone power (dBm)", fontsize=8)
    for ax in axes[len(parameters) :]:
        ax.set_visible(False)

    title = f"Tone {tone_index} fit parameters"
    if has_fallback:
        title += (
            f" (no fit: {_empirical_fallback_reason_summary(good_fits)})"
        )
    fig.suptitle(title, fontsize=9)
    fig.subplots_adjust(
        left=0.10 if ncol > 1 else 0.16,
        right=0.98,
        bottom=0.12,
        top=0.90,
        hspace=0.16,
        wspace=0.22 if ncol > 1 else 0.12,
    )

    param_path = output_dir / f"tone_{tone_index:04d}_params.png"
    fig.savefig(param_path, dpi=dpi)
    if not show:
        plt.close(fig)
    paths["parameters"].append(str(param_path))


# -- find_best_power and helpers ---------------------------------------------

# 1.4826 = 1 / Phi^{-1}(0.75); scales MAD to the equivalent Gaussian σ so the
# σ-clip thresholds below are interpretable as "k standard deviations" even
# when the data are non-Gaussian.
_MAD_TO_SIGMA = 1.4826
DEFAULT_BIFURCATION_ANL = 4.0 / (3.0 * np.sqrt(3.0))
DEFAULT_BIFURCATION_BACKOFF_DB = 3.0


def _is_fit_data(summary):
    """Return True for dictionaries produced by ``fit_power_sweep``."""
    return (
        isinstance(summary, dict)
        and "run" in summary
        and ("fits_by_power" in summary or "fits" in summary)
    )


def _coerce_summary_rows(summary):
    """Return summary rows as a list of dicts with numeric columns floated.

    Accepts a ``fit_power_sweep`` result dict, a structured summary array
    (:py:func:`fit_summary_array` / :py:func:`load_fit_summary`), an iterable
    of dicts (as produced by :py:func:`_fit_summary_rows`), a single row
    dict, or a path to a CSV written by :py:func:`write_fit_summary`.
    String columns that look numeric are converted to ``float`` (with empty
    strings becoming ``NaN``); ``"true"``/``"false"`` become ``bool``; other
    strings are left alone so the ``message`` field still parses.
    """
    if _is_fit_data(summary):
        raw_rows = _fit_summary_rows(summary)
    elif isinstance(summary, (str, Path)):
        with Path(summary).open("r", newline="", encoding="utf-8") as handle:
            raw_rows = list(csv.DictReader(handle))
    elif isinstance(summary, dict):
        if "tone_index" in summary and "sweep_index" in summary:
            raw_rows = [summary]
        else:
            raise TypeError(
                "summary dict must be a fit_power_sweep result or one summary row"
            )
    elif isinstance(summary, np.ndarray) and summary.dtype.names is not None:
        raw_rows = []
        for record in summary:
            row = {}
            for name in summary.dtype.names:
                value = record[name]
                if isinstance(value, np.generic):
                    value = value.item()
                row[name] = value
            raw_rows.append(row)
    else:
        raw_rows = [dict(row) for row in summary]

    coerced = []
    for row in raw_rows:
        new_row = {}
        for key, value in row.items():
            if not isinstance(value, str):
                new_row[key] = value
                continue
            stripped = value.strip()
            if stripped == "":
                new_row[key] = np.nan
            elif stripped.lower() == "true":
                new_row[key] = True
            elif stripped.lower() == "false":
                new_row[key] = False
            else:
                try:
                    new_row[key] = float(stripped)
                except ValueError:
                    new_row[key] = stripped
        coerced.append(new_row)
    return coerced


def _robust_sigma(values):
    """Return a MAD-based Gaussian-equivalent sigma, with std fallback."""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return float("nan")
    if values.size == 1:
        return 0.0
    centre = float(np.median(values))
    sigma = _MAD_TO_SIGMA * float(np.median(np.abs(values - centre)))
    if not np.isfinite(sigma) or sigma <= 0.0:
        sigma = float(np.std(values, ddof=1))
    return sigma


def _line_fit(x, y, weights=None):
    """Least-squares fit of ``y = slope*x + intercept``."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if weights is None:
        good = np.isfinite(x) & np.isfinite(y)
        x = x[good]
        y = y[good]
        if x.size < 2 or np.nanmax(x) <= np.nanmin(x):
            return float("nan"), float("nan")
        design = np.column_stack([x, np.ones_like(x)])
        slope, intercept = np.linalg.lstsq(design, y, rcond=None)[0]
        return float(slope), float(intercept)

    weights = np.asarray(weights, dtype=float)
    good = np.isfinite(x) & np.isfinite(y) & np.isfinite(weights) & (weights > 0.0)
    x = x[good]
    y = y[good]
    weights = weights[good]
    if x.size < 2 or np.nanmax(x) <= np.nanmin(x):
        return float("nan"), float("nan")
    design = np.column_stack([x, np.ones_like(x)])
    scale = np.sqrt(weights / np.nanmedian(weights))
    slope, intercept = np.linalg.lstsq(
        design * scale[:, None],
        y * scale,
        rcond=None,
    )[0]
    return float(slope), float(intercept)


def _log_anl_sigma(anl, anl_err):
    """Propagate linear ANL uncertainty to log(ANL) uncertainty."""
    anl = np.asarray(anl, dtype=float)
    if anl_err is None:
        return np.full(anl.shape, np.nan, dtype=float)
    anl_err = np.asarray(anl_err, dtype=float)
    if anl_err.shape != anl.shape:
        try:
            anl_err = np.broadcast_to(anl_err, anl.shape)
        except ValueError:
            return np.full(anl.shape, np.nan, dtype=float)
    with np.errstate(divide="ignore", invalid="ignore"):
        sigma = anl_err / anl
    sigma = np.asarray(sigma, dtype=float)
    sigma[~np.isfinite(sigma) | (sigma <= 0.0)] = np.nan
    return sigma


def _log_anl_fit_weights(anl, anl_err, min_points):
    """Return inverse-variance weights for log(ANL), or ``None``."""
    sigma = _log_anl_sigma(anl, anl_err)
    finite = np.isfinite(sigma) & (sigma > 0.0)
    if np.count_nonzero(finite) < max(int(min_points), 2):
        return None, "unweighted"
    # Keep points whose uncertainty failed by assigning the median valid
    # sigma. That preserves the sweep support without pretending those rows
    # have exceptionally high or low leverage.
    if not np.all(finite):
        sigma = sigma.copy()
        sigma[~finite] = float(np.nanmedian(sigma[finite]))
    with np.errstate(divide="ignore", invalid="ignore"):
        weights = 1.0 / np.square(sigma)
    weights[~np.isfinite(weights) | (weights <= 0.0)] = np.nan
    if np.count_nonzero(np.isfinite(weights) & (weights > 0.0)) < max(
        int(min_points), 2
    ):
        return None, "unweighted"
    return weights, "log_anl_uncertainty"


def _solve_log_anl_power(slope, intercept, target_anl):
    """Return power where ``log(anl)`` fit reaches ``target_anl``, else NaN."""
    try:
        target_anl = float(target_anl)
    except (TypeError, ValueError):
        return float("nan")
    if not (np.isfinite(target_anl) and target_anl > 0.0):
        return float("nan")
    if not (np.isfinite(slope) and np.isfinite(intercept) and slope != 0.0):
        return float("nan")
    return float((np.log(target_anl) - intercept) / slope)


def _theil_sen_line(x, y):
    """Robust initial line estimate for small power-sweep series."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    slopes = []
    for i in range(x.size - 1):
        dx = x[i + 1 :] - x[i]
        good = dx != 0.0
        if np.any(good):
            slopes.append((y[i + 1 :][good] - y[i]) / dx[good])
    if not slopes:
        return _line_fit(x, y)
    slopes = np.concatenate(slopes)
    slopes = slopes[np.isfinite(slopes)]
    if slopes.size == 0:
        return _line_fit(x, y)
    slope = float(np.median(slopes))
    intercept = float(np.median(y - slope * x))
    return slope, intercept


def _fit_log_anl_power_law(
    powers,
    anl,
    valid,
    *,
    anl_err=None,
    target_anl,
    min_points,
    clip_sigma,
    min_slope,
    weight_by_uncertainty=True,
):
    """Fit ``log(anl) = slope * power_dbm + intercept``.

    Performs an initial Theil-Sen line on all eligible points, one pass of
    symmetric MAD-based outlier rejection in log space at ``clip_sigma``, and
    then a least-squares refit on the retained points.  When ``anl_err`` is
    available, the refit is weighted by propagated log-space uncertainty,
    ``sigma_log_anl ~= sigma_anl / anl``.  The fit is marked reliable when
    the refit has at least ``min_points`` retained points and a slope of at
    least ``min_slope`` (decades/dB-equivalent in log space).
    """
    powers = np.asarray(powers, dtype=float)
    anl = np.asarray(anl, dtype=float)
    valid = np.asarray(valid, dtype=bool)
    if anl_err is None:
        anl_err = np.full(anl.shape, np.nan, dtype=float)
    else:
        anl_err = np.asarray(anl_err, dtype=float)
        if anl_err.shape != anl.shape:
            try:
                anl_err = np.broadcast_to(anl_err, anl.shape)
            except ValueError:
                anl_err = np.full(anl.shape, np.nan, dtype=float)
    n = powers.size
    false_mask = np.zeros(n, dtype=bool)
    result = {
        "reliable": False,
        "reason": "",
        "reasons": [],
        "slope_log_anl_per_db": float("nan"),
        "intercept_log_anl": float("nan"),
        "initial_slope_log_anl_per_db": float("nan"),
        "initial_intercept_log_anl": float("nan"),
        "target_power_dbm": None,
        "range_position": "unavailable",
        "weighted": False,
        "weight_source": "unweighted",
        "eligible_mask": false_mask.copy(),
        "fit_mask": false_mask.copy(),
        "outlier_mask": false_mask.copy(),
    }

    try:
        target_anl = float(target_anl)
    except (TypeError, ValueError):
        target_anl = float("nan")
    if not (np.isfinite(target_anl) and target_anl > 0.0):
        result["reasons"].append("target_anl must be positive")
        result["reason"] = "; ".join(result["reasons"])
        return result

    eligible = valid & np.isfinite(powers) & np.isfinite(anl) & (anl > 0.0)
    result["eligible_mask"] = eligible.copy()

    min_points = max(int(min_points), 2)
    if np.count_nonzero(eligible) < min_points:
        result["reasons"].append(
            f"only {int(np.count_nonzero(eligible))} eligible ANL points"
        )
        result["reason"] = "; ".join(result["reasons"])
        return result

    eligible_indices = np.flatnonzero(eligible)
    order = np.argsort(powers[eligible_indices])
    eligible_indices = eligible_indices[order]
    x = powers[eligible_indices]
    y = np.log(anl[eligible_indices])
    yerr = anl_err[eligible_indices]

    if np.unique(x).size < 2:
        result["reasons"].append("ANL fit needs at least two distinct powers")
        result["reason"] = "; ".join(result["reasons"])
        return result

    weights = None
    weight_source = "unweighted"
    if weight_by_uncertainty:
        weights, weight_source = _log_anl_fit_weights(
            anl[eligible_indices], yerr, min_points
        )

    # Initial robust line: Theil-Sen, fall back to OLS.
    slope, intercept = _theil_sen_line(x, y)
    if not (np.isfinite(slope) and np.isfinite(intercept)):
        slope, intercept = _line_fit(x, y, weights=weights)
    if not (np.isfinite(slope) and np.isfinite(intercept)):
        result["reasons"].append("could not fit log(ANL) line")
        result["reason"] = "; ".join(result["reasons"])
        return result
    initial_slope = float(slope)
    initial_intercept = float(intercept)

    # One pass of MAD outlier rejection, then refit on the kept points.
    residual = y - (slope * x + intercept)
    sigma = _robust_sigma(residual)
    if np.isfinite(sigma) and sigma > 0.0:
        keep = np.abs(residual) <= float(clip_sigma) * sigma
        if np.count_nonzero(keep) < min_points:
            keep = np.ones_like(keep)
    else:
        keep = np.ones_like(residual, dtype=bool)

    fit_weights = weights[keep] if weights is not None else None
    refit_slope, refit_intercept = _line_fit(x[keep], y[keep], weights=fit_weights)
    if not (np.isfinite(refit_slope) and np.isfinite(refit_intercept)):
        keep = np.ones_like(keep)
        fit_weights = weights if weights is not None else None
        refit_slope, refit_intercept = _line_fit(x, y, weights=fit_weights)
    if np.isfinite(refit_slope) and np.isfinite(refit_intercept):
        slope, intercept = refit_slope, refit_intercept

    fit_mask = false_mask.copy()
    fit_mask[eligible_indices[keep]] = True
    outlier_mask = false_mask.copy()
    outlier_mask[eligible_indices[~keep]] = True

    target_power = _solve_log_anl_power(slope, intercept, target_anl)

    result.update({
        "slope_log_anl_per_db": float(slope),
        "intercept_log_anl": float(intercept),
        "initial_slope_log_anl_per_db": initial_slope,
        "initial_intercept_log_anl": initial_intercept,
        "target_power_dbm": float(target_power) if np.isfinite(target_power) else None,
        "weighted": bool(weights is not None),
        "weight_source": weight_source,
        "fit_mask": fit_mask,
        "outlier_mask": outlier_mask,
    })

    reasons = []
    n_fit = int(np.count_nonzero(fit_mask))
    if n_fit < min_points:
        reasons.append(f"only {n_fit} ANL fit points after clipping")
    if np.isfinite(slope) and slope <= float(min_slope):
        reasons.append(
            f"ANL slope {slope:.3g}/dB below min_slope={float(min_slope):g}"
        )
    if result["target_power_dbm"] is None:
        reasons.append("target ANL power is not finite")

    if result["target_power_dbm"] is not None and np.any(fit_mask):
        xmin = float(np.nanmin(powers[fit_mask]))
        xmax = float(np.nanmax(powers[fit_mask]))
        if result["target_power_dbm"] < xmin:
            result["range_position"] = "below_measured_range"
        elif result["target_power_dbm"] > xmax:
            result["range_position"] = "above_measured_range"
        else:
            result["range_position"] = "within_measured_range"

    result["reliable"] = not reasons
    result["reasons"] = reasons
    result["reason"] = "; ".join(reasons)
    return result


_MAD_OUTLIER_CHECK_PARAMS = ("Qi", "Qc", "phi")


def find_best_power(
    summary,
    *,
    target_anl=0.01,
    bifurcation_anl=DEFAULT_BIFURCATION_ANL,
    param_valid_ranges=None,
    param_outliers_mad_clip=5.0,
    param_uncertainty_outlier_mad_clip=None,
    require_success=True,
    use_readback_power=True,
    anl_fit_min_points=3,
    anl_fit_clip_sigma=3.0,
    anl_fit_min_slope=1e-2,
    anl_fit_weight_by_uncertainty=True,
):
    """Pick a robust readout power per resonator from a fit-summary table.

    Operates per ``tone_index`` on a :py:func:`fit_power_sweep` result, a
    structured summary array (:py:func:`fit_summary_array` /
    :py:func:`load_fit_summary`), or a CSV written by
    :py:func:`write_fit_summary`.  The single criterion is a least-squares
    fit to ``log(anl) = slope * power_dbm + intercept``, which is solved for
    ``target_anl`` (default ``0.01``) to return a precise chosen power rather
    than the nearest measured sweep point.  By default, the retained
    log-linear refit is weighted by the propagated log-ANL uncertainty
    ``sigma_log_anl ~= anl_err / anl`` when enough finite ``anl_err`` values
    are available.

    Row-level exclusions, applied in order:

    1. ``success=False`` (skipped if ``require_success=False``).
    2. Per-parameter validity-range check: a row is dropped when any entry of
       ``param_valid_ranges`` has the row's fitted value at or beyond a
       bound.  Defaults to :data:`DEFAULT_PARAM_VALID_RANGES`, which mirrors
       the static parts of the fitter's physical bounds so rows where a
       parameter was pinned at a fitter clamp are excluded.  The exception is
       a low-clamped ``anl`` value: it is excluded from the ANL-vs-power fit,
       but remains eligible for the measured ``anl_threshold`` fallback
       because it still proves the measured point is below the requested
       target.  Pass a dict like ``{"Qi": (1e3, 1e7), "anl": None}`` to
       override; ``None`` disables a single entry, ``param_valid_ranges={}``
       disables all.
    3. Per-parameter MAD outlier check: for each parameter expected to be
       near-constant across powers (``Qi``, ``Qc``, ``phi``), rows where the
       value deviates from the per-tone median by more than
       ``param_outliers_mad_clip`` MAD-equivalent sigmas are dropped.  Set
       ``param_outliers_mad_clip=None`` to disable.
    4. Per-parameter uncertainty MAD outlier check: same MAD criterion applied
       to the ``<name>_err`` columns instead of the values, so rows with a
       wildly inflated uncertainty on ``Qi``, ``Qc``, or ``phi`` are dropped.
       Disabled by default (``param_uncertainty_outlier_mad_clip=None``); pass
       a positive number to enable.

    The ANL fit then takes the non-excluded, non-low-clamped-ANL rows,
    performs one pass of symmetric MAD outlier rejection in log space at
    ``anl_fit_clip_sigma``, and is marked reliable when the retained set has
    at least ``anl_fit_min_points`` points and a slope of at least
    ``anl_fit_min_slope`` decades/dB (default 1e-2).  Set
    ``anl_fit_weight_by_uncertainty=False`` to restore an unweighted final
    refit.  The chosen power is the value solved from the log-linear fit at
    ``target_anl``, regardless of whether it falls inside or outside the
    measured sweep range; the result field ``range_position``
    (``within_measured_range`` / ``above_measured_range`` /
    ``below_measured_range``) is reported as a diagnostic.

    The top-level ``chosen_*`` fields use the ANL power-law pick when it is
    reliable, otherwise fall back in order to: the highest measured power
    with ``anl < target_anl`` (``anl_threshold``); the row whose measured
    ``anl`` is nearest to ``target_anl`` among those still below
    ``bifurcation_anl`` (``anl_nearest_target``, for sweeps where every
    point is safely below bifurcation but the slope is too flat to solve
    for ``target_anl``); and finally the lowest measured power
    (``lowest_power_fallback``).  The returned rows also include ``p_bif``,
    the power solved from the same ANL fit at ``bifurcation_anl`` (default
    ``4/(3*sqrt(3)) ~= 0.77``), and ``p_bif_sub_3db``. Both are ``NaN``
    when no reliable ANL fit is available.

    Each returned row also includes ``chosen_params``: a dict with each
    parameter-like fit-summary value (``fr``, ``Ql``, ``Qi``, ``Qc``,
    ``Qc_abs``, ``phi``, ``a``, ``alpha``, ``tau``, ``anl``,
    ``nonlinear_detuning_hz`` and the ``empirical_*`` fields) linearly
    interpolated against ``power_dbm`` on the Stage-1-valid rows and evaluated
    at ``chosen_power_dbm``.  Fit-quality diagnostics such as
    ``residual_rms``, ``weighted_rms`` and ``reduced_chi2`` are excluded
    because they are not meaningful to interpolate.  ``anl`` is taken from
    the log(anl) fit so it is consistent with the chosen power.  Linear
    extrapolation is used outside the measured range; ``extrapolated`` is set
    to ``True`` when that happens.  Most values are ``NaN`` when no valid rows
    are available.

    ``chosen_params['fr']`` is given a fallback chain so it is non-``NaN``
    whenever any row in the sweep supplies one of the inputs: when the
    linear interpolation produces a finite value it is used as-is and
    ``chosen_params['fr_source']`` is ``"interpolated"``; otherwise the
    nearest row by ``|power_dbm - chosen_power_dbm|`` is searched for a
    finite ``fr`` (``"nearest_fitted"``), then ``empirical_fr``
    (``"nearest_empirical"``), then ``sweep_center_hz``
    (``"nearest_sweep_center"``), with ``"unavailable"`` recorded only
    when none of those columns has a finite entry in any row.

    ``use_readback_power`` (default ``True``) selects the power axis: the
    measured ``readback_power_dbm`` per row when available, otherwise the
    requested ``power_dbm``.  Set ``False`` to always use the requested power.
    """
    rows = _coerce_summary_rows(summary)
    if not rows:
        return []
    tones = sorted({int(row["tone_index"]) for row in rows})
    param_valid_ranges = _normalise_param_valid_ranges(param_valid_ranges)
    best_rows = [
        _find_best_power_for_tone(
            tone,
            [r for r in rows if int(r["tone_index"]) == tone],
            target_anl=target_anl,
            bifurcation_anl=bifurcation_anl,
            param_valid_ranges=param_valid_ranges,
            param_outliers_mad_clip=param_outliers_mad_clip,
            param_uncertainty_outlier_mad_clip=param_uncertainty_outlier_mad_clip,
            require_success=require_success,
            use_readback_power=use_readback_power,
            anl_fit_min_points=anl_fit_min_points,
            anl_fit_clip_sigma=anl_fit_clip_sigma,
            anl_fit_min_slope=anl_fit_min_slope,
            anl_fit_weight_by_uncertainty=anl_fit_weight_by_uncertainty,
        )
        for tone in tones
    ]
    # Repair at the source: replace negative (extrapolated) chosen_params with the
    # cross-tone median so every consumer of these rows gets physical values.
    _repair_negative_chosen_params(best_rows)
    return best_rows


def _repair_negative_chosen_params(rows, *, label="find_best_power"):
    """Replace unphysical negative ``chosen_params`` values with a cross-tone median.

    ``find_best_power`` interpolates each fit-summary field against power and
    linearly extrapolates past the measured range (:func:`_linear_interp_extrap`),
    which can drive a positive-but-decreasing quantity (e.g. a linewidth or Q)
    negative at a chosen power outside the swept range. For each physically
    non-negative key (:data:`_NONNEGATIVE_PARAM_KEYS`) this replaces any negative
    entry with the median of the valid (finite, positive) values across all
    ``rows`` (or ``NaN`` if none exist), repairing ``chosen_params`` **in place**.
    Signed quantities (skew, nonlinear detuning, phases) are left untouched.

    Parameters
    ----------
    rows : list of dict
        Best-power rows, each with a ``chosen_params`` dict.
    label : str, optional
        Prefix for the warning emitted when any value is replaced.

    Returns
    -------
    dict
        ``{key: n_replaced}`` for the keys whose negatives were repaired.
    """
    # Cross-tone median of the valid (finite, positive) values for each key.
    medians = {}
    for key in _NONNEGATIVE_PARAM_KEYS:
        vals = []
        for row in rows:
            cp = row.get("chosen_params") or {}
            try:
                v = float(cp.get(key, np.nan))
            except (TypeError, ValueError):
                continue
            if np.isfinite(v) and v > 0.0:
                vals.append(v)
        medians[key] = float(np.median(vals)) if vals else np.nan

    repaired = {}
    for row in rows:
        cp = row.get("chosen_params")
        if not isinstance(cp, dict):
            continue
        for key in _NONNEGATIVE_PARAM_KEYS:
            try:
                v = float(cp.get(key, np.nan))
            except (TypeError, ValueError):
                continue
            if np.isfinite(v) and v < 0.0:
                cp[key] = medians[key]
                repaired[key] = repaired.get(key, 0) + 1
    if repaired:
        warnings.warn(
            f"{label}: replaced negative (extrapolated) chosen_params with the "
            f"cross-tone median for {repaired} (NaN where no valid median existed).",
            stacklevel=2)
    return repaired


def best_power_arrays(best_power, tone_count=None):
    """Pull per-tone power and interpolated-parameter arrays from best-power rows.

    Parameters
    ----------
    best_power : list of dict, dict, or path-like
        Output of :py:func:`find_best_power`, a single such row, or a path
        to a JSON file written by :py:func:`write_best_power` (the file or
        its parent directory).
    tone_count : int or None, optional
        Length of the returned arrays.  ``None`` (default) uses
        ``max(tone_index) + 1``.  Larger values pad with ``NaN``; smaller
        values raise ``ValueError``.

    Returns
    -------
    dict
        Arrays have one entry per tone, in tone-index order; tones missing
        from ``best_power`` are filled with ``NaN`` (or ``"unavailable"`` for
        ``fr_source``).  The legacy keys ``chosen_power_dbm``, ``p_bif``,
        ``p_bif_sub_3db``, ``fr_hz`` and ``fr_source`` are preserved.  Each
        entry from ``chosen_params`` is also exposed directly as an array:
        ``power_dbm``, every interpolated parameter-like fit-summary field
        such as ``fr``, ``Ql``, ``Qi``, ``Qc``, ``anl`` and empirical fields,
        plus ``extrapolated``, ``n_rows_used`` and ``fr_source``.

        ``fr_hz`` is a compatibility alias for ``fr``.  ``fr`` is the
        resonance frequency at the chosen power, with the fallback chain
        described in :py:func:`find_best_power` so it is non-``NaN`` whenever
        any row supplied a fitted ``fr``, ``empirical_fr``, or
        ``sweep_center_hz``.
    """
    if isinstance(best_power, (str, Path)):
        best_power = load_best_power(best_power)
    if isinstance(best_power, dict):
        best_power = [best_power]
    # Copy rows (and their chosen_params) so repairing negatives here never mutates
    # the caller's input. find_best_power already repairs at source; this also
    # covers rows loaded from older files written before that fix.
    rows = [dict(r) for r in best_power]
    for r in rows:
        if isinstance(r.get("chosen_params"), dict):
            r["chosen_params"] = dict(r["chosen_params"])
    _repair_negative_chosen_params(rows, label="best_power_arrays")
    indices = [int(row["tone_index"]) for row in rows]
    inferred = (max(indices) + 1) if indices else 0
    if tone_count is None:
        tone_count = inferred
    else:
        tone_count = int(tone_count)
        if tone_count < inferred:
            raise ValueError(
                f"tone_count={tone_count} is smaller than the highest "
                f"tone_index={inferred - 1} in best_power."
            )
    out = {
        "tone_index": np.arange(tone_count, dtype=int),
        "chosen_power_dbm": np.full(tone_count, np.nan, dtype=float),
        "p_bif": np.full(tone_count, np.nan, dtype=float),
        "p_bif_sub_3db": np.full(tone_count, np.nan, dtype=float),
    }
    param_arrays = {
        name: (
            np.full(tone_count, "unavailable", dtype=object)
            if name == "fr_source" else
            np.zeros(tone_count, dtype=bool)
            if name == "extrapolated" else
            np.full(tone_count, np.nan, dtype=float)
        )
        for name in _CHOSEN_PARAM_ARRAY_KEYS
    }
    for row in rows:
        i = int(row["tone_index"])
        for key, target in (
            ("chosen_power_dbm", out["chosen_power_dbm"]),
            ("p_bif", out["p_bif"]),
            ("p_bif_sub_3db", out["p_bif_sub_3db"]),
        ):
            try:
                target[i] = float(row.get(key, np.nan))
            except (TypeError, ValueError):
                pass
        chosen_params = row.get("chosen_params") or {}
        for key, target in param_arrays.items():
            value = chosen_params.get(key, np.nan)
            if key == "fr_source":
                if isinstance(value, str):
                    target[i] = value
            elif key == "extrapolated":
                if key in chosen_params:
                    target[i] = bool(value)
            else:
                try:
                    target[i] = float(value)
                except (TypeError, ValueError):
                    target[i] = np.nan

    out.update(param_arrays)
    out["fr_hz"] = out["fr"].copy()
    return out


def accumulator_level_db(
    parsed_samples,
    tones=None,
    *,
    statistic="median",
    ignore_packet_errors=True,
):
    """Return per-tone accumulated-I/Q levels from parsed samples in dB.

    ``parsed_samples`` should be the dict returned by
    :py:meth:`ReadoutClient.parse_samples`.  The returned values are relative
    accumulated-I/Q magnitudes, useful for comparing ADC/RX bit utilisation
    across tones.  They are not referred to an absolute RF plane.  Their
    tone-to-tone differences are the natural ``rx_offsets_db`` input for
    :py:func:`balance_tone_powers`.

    Parameters
    ----------
    parsed_samples : dict
        Parsed sample data from :py:meth:`ReadoutClient.parse_samples`.
    tones : int or iterable of int or None, optional
        Tone index/indices to report; ``None`` (default) does every tone.
    statistic : {'median', 'mean'}, optional
        How to reduce each tone's per-sample magnitude over time (default
        ``'median'``, which is robust to packet glitches).
    ignore_packet_errors : bool, optional
        If ``True`` (default), drop samples flagged as packet errors before
        reducing.
    """
    if not isinstance(parsed_samples, dict):
        raise TypeError("parsed_samples must be a parsed sample-data dict.")
    i_data = parsed_samples.get("i_data")
    q_data = parsed_samples.get("q_data")
    if not isinstance(i_data, dict) or not isinstance(q_data, dict):
        raise ValueError("parsed_samples must contain i_data and q_data dicts.")

    if tones is None:
        try:
            tone_count = int(parsed_samples.get("num_tones", len(i_data)))
        except (TypeError, ValueError):
            tone_count = len(i_data)
        tones = range(tone_count)
    elif np.isscalar(tones) and not isinstance(tones, str):
        tones = [int(tones)]
    else:
        tones = list(tones)

    first_key = f"{int(tones[0]):04d}" if tones else None
    if first_key is not None and first_key in i_data:
        n_samples = np.asarray(i_data[first_key]).size
    else:
        n_samples = 0
    good = np.ones(n_samples, dtype=bool)
    if ignore_packet_errors and "packet_error" in parsed_samples:
        packet_error = np.asarray(parsed_samples["packet_error"])
        if packet_error.size == n_samples:
            good &= packet_error == 0

    stat = str(statistic).lower()
    levels = np.full(len(tones), np.nan, dtype=float)
    for j, tone in enumerate(tones):
        key = tone if isinstance(tone, str) else f"{int(tone):04d}"
        if key not in i_data or key not in q_data:
            continue
        z = np.asarray(i_data[key], dtype=float) + 1j * np.asarray(
            q_data[key],
            dtype=float,
        )
        if z.size != good.size:
            mask = np.isfinite(z.real) & np.isfinite(z.imag)
        else:
            mask = good & np.isfinite(z.real) & np.isfinite(z.imag)
        mag = np.abs(z[mask])
        mag = mag[np.isfinite(mag)]
        if mag.size == 0:
            continue
        if stat == "median":
            value = float(np.median(mag))
        elif stat == "mean":
            value = float(np.mean(mag))
        elif stat == "rms":
            value = float(np.sqrt(np.mean(np.square(mag))))
        else:
            raise ValueError("statistic must be 'median', 'mean', or 'rms'.")
        levels[j] = 20.0 * np.log10(max(value, 1e-300))
    return levels


def _normalise_optional_range_db(value, name):
    """Return ``None`` or a finite non-negative dB range."""
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be None or a non-negative number.") from exc
    if not (np.isfinite(value) and value >= 0.0):
        raise ValueError(f"{name} must be None or a non-negative finite number.")
    return value


def _as_tone_array(value, tone_count, name, *, fill=np.nan):
    """Return a one-dimensional float array of length ``tone_count``."""
    if value is None:
        return np.full(tone_count, fill, dtype=float)
    arr = np.asarray(value, dtype=float).ravel()
    if arr.size == 1 and tone_count != 1:
        arr = np.full(tone_count, float(arr[0]), dtype=float)
    if arr.size != tone_count:
        raise ValueError(f"{name} must have one value per tone.")
    return arr.astype(float, copy=True)


def _normalise_best_power_rows(best_power):
    """Return ``best_power`` as a list of row dicts, loading paths if needed."""
    if best_power is None:
        return None
    if isinstance(best_power, (str, Path)):
        best_power = load_best_power(best_power)
    if isinstance(best_power, dict):
        return [best_power]
    return list(best_power)


def _row_power_for_balanced_params(row, use_readback_power):
    """Return the summary-row power used for balanced-parameter interpolation."""
    if use_readback_power:
        try:
            value = float(row.get("readback_power_dbm", np.nan))
        except (TypeError, ValueError):
            value = np.nan
        if np.isfinite(value):
            return value
    try:
        return float(row.get("power_dbm", np.nan))
    except (TypeError, ValueError):
        return np.nan


def _anl_fit_from_best_row(best_row):
    """Rebuild the small ANL-fit dict needed by ``_interpolated_params_at_power``."""
    if not isinstance(best_row, dict):
        return None
    pick = best_row.get("anl_power_law_pick") or {}
    if not isinstance(pick, dict):
        return None
    return {
        "reliable": bool(pick.get("reliable", False)),
        "slope_log_anl_per_db": pick.get("slope_log_anl_per_db", np.nan),
        "intercept_log_anl": pick.get("intercept_log_anl", np.nan),
        "fit_mask": [],
    }


def _balanced_param_arrays(summary, best_rows, tone_count, powers, *, use_readback_power):
    """Interpolate fit-summary parameters at one balanced-power vector."""
    names = _CHOSEN_PARAM_ARRAY_KEYS
    out = {
        name: (
            np.full(tone_count, "unavailable", dtype=object)
            if name == "fr_source" else
            np.zeros(tone_count, dtype=bool)
            if name == "extrapolated" else
            np.full(tone_count, np.nan, dtype=float)
        )
        for name in names
    }
    if summary is None:
        return out

    rows = _coerce_summary_rows(summary)
    rows_by_tone = {}
    for row in rows:
        try:
            tone = int(row["tone_index"])
        except (KeyError, TypeError, ValueError):
            continue
        rows_by_tone.setdefault(tone, []).append(row)

    best_by_tone = {}
    for row in best_rows or []:
        try:
            best_by_tone[int(row["tone_index"])] = row
        except (KeyError, TypeError, ValueError):
            continue

    for tone in range(tone_count):
        tone_rows = rows_by_tone.get(int(tone), [])
        if not tone_rows:
            continue
        tone_rows = sorted(
            tone_rows,
            key=lambda row: (
                not np.isfinite(_row_power_for_balanced_params(row, use_readback_power)),
                (
                    _row_power_for_balanced_params(row, use_readback_power)
                    if np.isfinite(_row_power_for_balanced_params(row, use_readback_power))
                    else 0.0
                ),
            ),
        )
        row_powers = np.asarray(
            [
                _row_power_for_balanced_params(row, use_readback_power)
                for row in tone_rows
            ],
            dtype=float,
        )
        best_row = best_by_tone.get(int(tone), {})
        excluded = {
            int(index)
            for index in (best_row.get("excluded_sweep_indices") or [])
        }
        valid = np.asarray(
            [
                int(row.get("sweep_index", -1)) not in excluded
                for row in tone_rows
            ],
            dtype=bool,
        )
        params = _interpolated_params_at_power(
            tone_rows,
            valid,
            row_powers,
            powers[tone],
            anl_fit=_anl_fit_from_best_row(best_row),
        )
        for name in names:
            if name not in params:
                continue
            if name == "fr_source":
                out[name][tone] = params.get(name, "unavailable")
            elif name == "extrapolated":
                out[name][tone] = bool(params.get(name, False))
            else:
                try:
                    out[name][tone] = float(params.get(name, np.nan))
                except (TypeError, ValueError):
                    out[name][tone] = np.nan
    return out


def _solve_balanced_power_lp(
    caps,
    *,
    targets=None,
    rx_offsets=None,
    tx_power_range_db=None,
    rx_level_range_db=None,
    objective="nearest_target",
):
    """Solve the balanced-power allocation for finite-cap tones."""
    from scipy.optimize import linprog

    caps = np.asarray(caps, dtype=float).ravel()
    n = caps.size
    targets = (
        None if targets is None
        else np.asarray(targets, dtype=float).ravel()
    )
    rx_offsets = (
        None if rx_offsets is None
        else np.asarray(rx_offsets, dtype=float).ravel()
    )
    objective = str(objective).lower()
    if objective not in {"nearest_target", "maximise_power"}:
        raise ValueError("objective must be 'nearest_target' or 'maximise_power'.")
    target_mask = (
        np.isfinite(targets)
        if objective == "nearest_target" and targets is not None
        else np.zeros(n, dtype=bool)
    )
    target_indices = np.flatnonzero(target_mask)

    n_vars = n
    dev_start = None
    if target_indices.size:
        dev_start = n_vars
        n_vars += target_indices.size
    tx_lo = tx_hi = None
    rx_lo = rx_hi = None
    if tx_power_range_db is not None:
        tx_lo = n_vars
        tx_hi = n_vars + 1
        n_vars += 2
    if rx_level_range_db is not None:
        rx_lo = n_vars
        rx_hi = n_vars + 1
        n_vars += 2

    c = np.zeros(n_vars, dtype=float)
    if target_indices.size:
        c[dev_start : dev_start + target_indices.size] = 1.0
        # Tiny tie-break: among equally target-close solutions, prefer the
        # louder comb.  Kept far below one dB of target error.
        c[:n] = -1e-6
    else:
        c[:n] = -1.0
    bounds = [(None, float(cap)) for cap in caps]
    bounds.extend([(0.0, None)] * target_indices.size)
    bounds.extend([(None, None)] * (n_vars - n - target_indices.size))
    a_ub = []
    b_ub = []

    def _row():
        """A zeroed constraint/objective coefficient row for the LP."""
        return np.zeros(n_vars, dtype=float)

    if target_indices.size:
        for j, i in enumerate(target_indices):
            dev = dev_start + j
            target = float(targets[i])

            row = _row()
            row[i] = 1.0
            row[dev] = -1.0
            a_ub.append(row)
            b_ub.append(target)

            row = _row()
            row[i] = -1.0
            row[dev] = -1.0
            a_ub.append(row)
            b_ub.append(-target)

    if tx_power_range_db is not None:
        for i in range(n):
            row = _row()
            row[i] = 1.0
            row[tx_hi] = -1.0
            a_ub.append(row)
            b_ub.append(0.0)

            row = _row()
            row[i] = -1.0
            row[tx_lo] = 1.0
            a_ub.append(row)
            b_ub.append(0.0)

        row = _row()
        row[tx_hi] = 1.0
        row[tx_lo] = -1.0
        a_ub.append(row)
        b_ub.append(float(tx_power_range_db))

    if rx_level_range_db is not None:
        for i in range(n):
            offset = float(rx_offsets[i])
            row = _row()
            row[i] = 1.0
            row[rx_hi] = -1.0
            a_ub.append(row)
            b_ub.append(-offset)

            row = _row()
            row[i] = -1.0
            row[rx_lo] = 1.0
            a_ub.append(row)
            b_ub.append(offset)

        row = _row()
        row[rx_hi] = 1.0
        row[rx_lo] = -1.0
        a_ub.append(row)
        b_ub.append(float(rx_level_range_db))

    result = linprog(
        c,
        A_ub=np.vstack(a_ub) if a_ub else None,
        b_ub=np.asarray(b_ub, dtype=float) if b_ub else None,
        bounds=bounds,
        method="highs",
    )
    if not result.success:
        return None, result
    return np.asarray(result.x[:n], dtype=float), result


def balance_tone_powers(
    best_power=None,
    *,
    summary=None,
    power_caps_dbm=None,
    power_targets_dbm=None,
    cap_key="p_bif_sub_3db",
    target_key="chosen_power_dbm",
    cap_margin_db=0.0,
    missing_cap_policy="target",
    tone_count=None,
    use_readback_power=True,
    rx_offsets_db=None,
    tx_power_range_db=6.0,
    rx_level_range_db=None,
    objective="nearest_target",
):
    """Allocate per-tone powers under bifurcation, DAC, and ADC constraints.

    This is the comb-balancing step after :py:func:`find_best_power`.
    ``power_caps_dbm`` should contain the hard upper power limit for each
    tone, usually ``p_bif_sub_3db``.  If ``power_caps_dbm`` is omitted,
    ``best_power`` is passed through :py:func:`best_power_arrays` and
    ``cap_key`` selects the cap array.  When a full ``best_power`` result is
    supplied, ``target_key`` defaults to ``"chosen_power_dbm"`` so the
    allocator treats the old best powers as preferred values while using the
    bifurcation-backed caps as hard safety limits.
    When a cap is missing, ``missing_cap_policy="target"`` uses the finite
    target power as a conservative cap: the tone can be lowered for balance,
    but not raised above its original best-power pick.  Pass
    ``missing_cap_policy="drop"`` to exclude tones without finite caps.
    If ``summary`` is supplied, it is used to interpolate the fit-summary
    parameters at the original, TX-only, and final allocated powers.

    With ``objective="nearest_target"`` (default), the optimiser minimises the
    absolute distance to ``power_targets_dbm`` / ``target_key`` with a tiny
    loudness tie-break.  With ``objective="maximise_power"``, it ignores
    targets and maximises the sum of allocated tone powers subject to:

    - ``allocated_power_dbm <= power_cap_dbm - cap_margin_db``
    - optional TX spread: ``max(P) - min(P) <= tx_power_range_db``
    - optional RX spread:
      ``max(P + rx_offsets_db) - min(P + rx_offsets_db)
      <= rx_level_range_db``
    There is intentionally no lower physical tone-power bound; finite target
    powers are preferences, not constraints.

    ``tone_count`` sets/validates the number of tones when ``power_caps_dbm``
    is given directly (inferred from ``best_power`` otherwise).
    ``use_readback_power`` (default ``True``) selects whether interpolated
    summary parameters use the measured readback power or the requested power
    (passed through to the summary interpolation, as in
    :py:func:`find_best_power`).
    """
    best_rows = _normalise_best_power_rows(best_power)
    arrays = None
    if best_rows is not None:
        arrays = best_power_arrays(best_rows, tone_count=tone_count)

    if power_caps_dbm is None:
        if arrays is None:
            raise ValueError("provide best_power or power_caps_dbm.")
        if cap_key not in arrays:
            raise ValueError(
                f"cap_key must be one of {sorted(arrays)}, got {cap_key!r}."
            )
        caps = np.asarray(arrays[cap_key], dtype=float).ravel()
        tone_count = caps.size
        tone_index = np.asarray(arrays["tone_index"], dtype=int)
        source = str(cap_key)
    else:
        caps = np.asarray(power_caps_dbm, dtype=float).ravel()
        if tone_count is None:
            tone_count = caps.size
        else:
            tone_count = int(tone_count)
            if caps.size != tone_count:
                raise ValueError("power_caps_dbm must have one value per tone.")
        tone_index = np.arange(tone_count, dtype=int)
        source = "power_caps_dbm"

    raw_caps = caps.copy()
    target_source = "none"
    if power_targets_dbm is None and arrays is not None and target_key is not None:
        if target_key not in arrays:
            raise ValueError(
                f"target_key must be one of {sorted(arrays)}, got {target_key!r}."
            )
        power_targets_dbm = arrays[target_key]
        target_source = str(target_key)
    elif power_targets_dbm is not None:
        target_source = "power_targets_dbm"

    try:
        cap_margin_db = float(cap_margin_db)
    except (TypeError, ValueError) as exc:
        raise ValueError("cap_margin_db must be a finite number.") from exc
    if not np.isfinite(cap_margin_db):
        raise ValueError("cap_margin_db must be finite.")
    caps = caps - cap_margin_db
    cap_source_per_tone = np.full(tone_count, source, dtype=object)

    if arrays is not None:
        initial_chosen = _as_tone_array(
            arrays.get("chosen_power_dbm"),
            tone_count,
            "chosen_power_dbm",
            fill=np.nan,
        )
        p_bif = _as_tone_array(
            arrays.get("p_bif"),
            tone_count,
            "p_bif",
            fill=np.nan,
        )
        p_bif_sub_3db = _as_tone_array(
            arrays.get("p_bif_sub_3db"),
            tone_count,
            "p_bif_sub_3db",
            fill=np.nan,
        )
    else:
        initial_chosen = _as_tone_array(
            power_targets_dbm,
            tone_count,
            "power_targets_dbm",
            fill=np.nan,
        )
        p_bif = np.full(tone_count, np.nan, dtype=float)
        p_bif_sub_3db = raw_caps.copy()

    tx_power_range_db = _normalise_optional_range_db(
        tx_power_range_db,
        "tx_power_range_db",
    )
    rx_level_range_db = _normalise_optional_range_db(
        rx_level_range_db,
        "rx_level_range_db",
    )
    rx_offsets = _as_tone_array(
        rx_offsets_db,
        tone_count,
        "rx_offsets_db",
        fill=np.nan,
    )
    targets = _as_tone_array(
        power_targets_dbm,
        tone_count,
        "power_targets_dbm",
        fill=np.nan,
    )
    objective = str(objective).lower()
    if objective not in {"nearest_target", "maximise_power"}:
        raise ValueError("objective must be 'nearest_target' or 'maximise_power'.")
    missing_cap_policy = str(missing_cap_policy).lower()
    if missing_cap_policy in {"chosen", "chosen_power", "target_power"}:
        missing_cap_policy = "target"
    if missing_cap_policy not in {"target", "drop"}:
        raise ValueError("missing_cap_policy must be 'target' or 'drop'.")

    missing_caps = ~np.isfinite(caps)
    if missing_cap_policy == "target":
        fallback = missing_caps & np.isfinite(targets)
        caps[fallback] = targets[fallback] - cap_margin_db
        cap_source_per_tone[fallback] = f"{target_source}_fallback"

    cap_valid = np.isfinite(caps)
    valid = cap_valid.copy()
    invalid_reasons = {}
    for i in np.flatnonzero(~valid):
        invalid_reasons[int(tone_index[i])] = (
            "power cap is not finite"
            if missing_cap_policy == "drop"
            else "power cap and fallback target are not finite"
        )

    if rx_level_range_db is not None:
        missing_rx = valid & ~np.isfinite(rx_offsets)
        for i in np.flatnonzero(missing_rx):
            invalid_reasons[int(tone_index[i])] = "rx offset is not finite"
        valid &= np.isfinite(rx_offsets)

    rx_offset_range = float("nan")
    min_rx_range_for_tx = float("nan")
    min_tx_range_for_rx = float("nan")
    rx_feasibility_note = ""
    rx_valid = valid & np.isfinite(rx_offsets)
    if np.any(rx_valid):
        rx_offset_range = float(
            np.nanmax(rx_offsets[rx_valid]) - np.nanmin(rx_offsets[rx_valid])
        )
        if tx_power_range_db is not None:
            min_rx_range_for_tx = max(0.0, rx_offset_range - tx_power_range_db)
        if rx_level_range_db is not None:
            min_tx_range_for_rx = max(0.0, rx_offset_range - rx_level_range_db)
        if (
            tx_power_range_db is not None
            and rx_level_range_db is not None
            and tx_power_range_db + rx_level_range_db + 1e-9 < rx_offset_range
        ):
            rx_feasibility_note = (
                "rx_offsets_db span exceeds tx_power_range_db + "
                "rx_level_range_db"
            )

    allocated = np.full(tone_count, np.nan, dtype=float)
    predicted_rx = np.full(tone_count, np.nan, dtype=float)
    initial_predicted_rx = np.full(tone_count, np.nan, dtype=float)
    tx_only_power = np.full(tone_count, np.nan, dtype=float)
    tx_only_predicted_rx = np.full(tone_count, np.nan, dtype=float)
    tx_only_solver_status = None
    tx_only_solver_message = ""
    tx_only_success = False
    tx_only_reason = ""
    solver_status = None
    solver_message = ""
    success = False
    reason = ""

    if np.any(np.isfinite(initial_chosen) & np.isfinite(rx_offsets)):
        initial_predicted_rx = initial_chosen + rx_offsets

    tx_only_idx = np.flatnonzero(cap_valid)
    if tx_only_idx.size == 0:
        tx_only_reason = "no tones with finite allocation constraints"
    else:
        tx_solution, tx_solver = _solve_balanced_power_lp(
            caps[tx_only_idx],
            targets=targets[tx_only_idx],
            tx_power_range_db=tx_power_range_db,
            objective=objective,
        )
        tx_only_solver_status = int(getattr(tx_solver, "status", -1))
        tx_only_solver_message = str(getattr(tx_solver, "message", ""))
        if tx_solution is None:
            tx_only_reason = tx_only_solver_message or "TX-only allocation failed"
        else:
            tx_only_power[tx_only_idx] = tx_solution
            tx_only_success = True

    if np.any(np.isfinite(tx_only_power) & np.isfinite(rx_offsets)):
        tx_only_predicted_rx = tx_only_power + rx_offsets

    solve_idx = np.flatnonzero(valid)
    if solve_idx.size == 0:
        reason = "no tones with finite allocation constraints"
    else:
        solution, solver = _solve_balanced_power_lp(
            caps[solve_idx],
            targets=targets[solve_idx],
            rx_offsets=(
                rx_offsets[solve_idx]
                if rx_level_range_db is not None else None
            ),
            tx_power_range_db=tx_power_range_db,
            rx_level_range_db=rx_level_range_db,
            objective=objective,
        )
        solver_status = int(getattr(solver, "status", -1))
        solver_message = str(getattr(solver, "message", ""))
        if solution is None:
            reason = solver_message or "balanced-power allocation failed"
        else:
            allocated[solve_idx] = solution
            success = True

    if np.any(np.isfinite(allocated) & np.isfinite(rx_offsets)):
        predicted_rx = allocated + rx_offsets

    with np.errstate(invalid="ignore"):
        initial_bif_margin = p_bif - initial_chosen
        tx_only_bif_margin = p_bif - tx_only_power
        final_bif_margin = p_bif - allocated
        initial_cap_margin = p_bif_sub_3db - initial_chosen
        tx_only_cap_margin = p_bif_sub_3db - tx_only_power
        final_cap_margin = p_bif_sub_3db - allocated

    finite_tx_only = np.isfinite(tx_only_power)
    if np.count_nonzero(finite_tx_only) >= 2:
        tx_only_tx_range = float(
            np.nanmax(tx_only_power[finite_tx_only])
            - np.nanmin(tx_only_power[finite_tx_only])
        )
    elif np.count_nonzero(finite_tx_only) == 1:
        tx_only_tx_range = 0.0
    else:
        tx_only_tx_range = float("nan")

    finite_tx_only_rx = np.isfinite(tx_only_predicted_rx)
    if np.count_nonzero(finite_tx_only_rx) >= 2:
        tx_only_rx_range = float(
            np.nanmax(tx_only_predicted_rx[finite_tx_only_rx])
            - np.nanmin(tx_only_predicted_rx[finite_tx_only_rx])
        )
    elif np.count_nonzero(finite_tx_only_rx) == 1:
        tx_only_rx_range = 0.0
    else:
        tx_only_rx_range = float("nan")

    finite_alloc = np.isfinite(allocated)
    if np.count_nonzero(finite_alloc) >= 2:
        actual_tx_range = float(
            np.nanmax(allocated[finite_alloc])
            - np.nanmin(allocated[finite_alloc])
        )
    elif np.count_nonzero(finite_alloc) == 1:
        actual_tx_range = 0.0
    else:
        actual_tx_range = float("nan")

    finite_rx = np.isfinite(predicted_rx)
    if np.count_nonzero(finite_rx) >= 2:
        actual_rx_range = float(
            np.nanmax(predicted_rx[finite_rx])
            - np.nanmin(predicted_rx[finite_rx])
        )
    elif np.count_nonzero(finite_rx) == 1:
        actual_rx_range = 0.0
    else:
        actual_rx_range = float("nan")

    if success and invalid_reasons:
        status = "partial"
        reason = f"{len(invalid_reasons)} tone(s) were not allocated"
    elif success:
        status = "success"
        reason = ""
    else:
        status = "failed"

    initial_params = _balanced_param_arrays(
        summary,
        best_rows,
        tone_count,
        initial_chosen,
        use_readback_power=use_readback_power,
    )
    tx_only_params = _balanced_param_arrays(
        summary,
        best_rows,
        tone_count,
        tx_only_power,
        use_readback_power=use_readback_power,
    )
    rx_constrained_params = _balanced_param_arrays(
        summary,
        best_rows,
        tone_count,
        allocated,
        use_readback_power=use_readback_power,
    )

    return {
        "tone_index": tone_index,
        "allocated_power_dbm": allocated,
        "initial_chosen_power_dbm": initial_chosen,
        "p_bif": p_bif,
        "p_bif_sub_3db": p_bif_sub_3db,
        "power_cap_dbm": caps,
        "power_cap_source": cap_source_per_tone.tolist(),
        "power_target_dbm": targets,
        "tx_only_power_dbm": tx_only_power,
        "rx_constrained_power_dbm": allocated,
        "initial_bifurcation_margin_db": initial_bif_margin,
        "tx_only_bifurcation_margin_db": tx_only_bif_margin,
        "bifurcation_margin_db": final_bif_margin,
        "rx_constrained_bifurcation_margin_db": final_bif_margin,
        "initial_cap_margin_db": initial_cap_margin,
        "tx_only_cap_margin_db": tx_only_cap_margin,
        "cap_margin_db": final_cap_margin,
        "rx_constrained_cap_margin_db": final_cap_margin,
        "rx_offsets_db": rx_offsets,
        "initial_predicted_rx_level_db": initial_predicted_rx,
        "tx_only_predicted_rx_level_db": tx_only_predicted_rx,
        "predicted_rx_level_db": predicted_rx,
        "initial_params": initial_params,
        "tx_only_params": tx_only_params,
        "rx_constrained_params": rx_constrained_params,
        "valid_mask": valid,
        "status": status,
        "success": bool(success),
        "reason": reason,
        "invalid_reasons": invalid_reasons,
        "constraints": {
            "cap_source": source,
            "target_source": target_source,
            "cap_margin_db": float(cap_margin_db),
            "missing_cap_policy": missing_cap_policy,
            "use_readback_power": bool(use_readback_power),
            "tx_power_range_db": tx_power_range_db,
            "rx_level_range_db": rx_level_range_db,
            "objective": objective,
        },
        "actual_tx_power_range_db": actual_tx_range,
        "actual_rx_level_range_db": actual_rx_range,
        "tx_only_tx_power_range_db": tx_only_tx_range,
        "tx_only_rx_level_range_db": tx_only_rx_range,
        "feasibility": {
            "rx_offset_range_db": rx_offset_range,
            "min_rx_level_range_db_for_tx_range": min_rx_range_for_tx,
            "min_tx_power_range_db_for_rx_range": min_tx_range_for_rx,
            "note": rx_feasibility_note,
        },
        "solver": {
            "name": "scipy.optimize.linprog",
            "status": solver_status,
            "message": solver_message,
        },
        "tx_only_solver": {
            "name": "scipy.optimize.linprog",
            "success": bool(tx_only_success),
            "reason": tx_only_reason,
            "status": tx_only_solver_status,
            "message": tx_only_solver_message,
        },
    }


_BALANCED_POWER_ARRAY_KEYS = (
    "tone_index",
    "allocated_power_dbm",
    "initial_chosen_power_dbm",
    "p_bif",
    "p_bif_sub_3db",
    "power_cap_dbm",
    "power_target_dbm",
    "tx_only_power_dbm",
    "rx_constrained_power_dbm",
    "initial_bifurcation_margin_db",
    "tx_only_bifurcation_margin_db",
    "bifurcation_margin_db",
    "rx_constrained_bifurcation_margin_db",
    "initial_cap_margin_db",
    "tx_only_cap_margin_db",
    "cap_margin_db",
    "rx_constrained_cap_margin_db",
    "rx_offsets_db",
    "initial_predicted_rx_level_db",
    "tx_only_predicted_rx_level_db",
    "predicted_rx_level_db",
    "valid_mask",
)


def _restore_balanced_array(values, *, dtype=float):
    """Rebuild a saved balanced-power column, mapping JSON ``null`` to NaN."""
    values = [] if values is None else values
    if dtype is bool:
        return np.asarray(values, dtype=bool)
    if dtype is int:
        return np.asarray(values, dtype=int)
    return np.asarray(
        [np.nan if value is None else value for value in values],
        dtype=float,
    )


def balanced_power_arrays(balanced_power, tone_count=None):
    """Return the main arrays from ``balance_tone_powers`` output.

    Parameters
    ----------
    balanced_power : dict or str or Path
        An :py:func:`balance_tone_powers` result, or a path to a
        JSON file written by :py:func:`write_balanced_power`.
    tone_count : int or None, optional
        Pad/validate the output to this many tones; ``None`` (default) infers
        it from the allocated-power array.
    """
    if isinstance(balanced_power, (str, Path)):
        balanced_power = load_balanced_power(balanced_power)
    if not isinstance(balanced_power, dict):
        raise TypeError("balanced_power must be a dict or path.")

    if "allocated_power_dbm" not in balanced_power:
        raise ValueError("balanced_power is missing allocated_power_dbm.")
    allocated = np.asarray(balanced_power["allocated_power_dbm"], dtype=float).ravel()
    inferred = allocated.size
    if tone_count is None:
        tone_count = inferred
    else:
        tone_count = int(tone_count)
        if tone_count < inferred:
            raise ValueError("tone_count is smaller than balanced_power arrays.")

    out = {
        "tone_index": np.arange(tone_count, dtype=int),
        "allocated_power_dbm": np.full(tone_count, np.nan, dtype=float),
        "initial_chosen_power_dbm": np.full(tone_count, np.nan, dtype=float),
        "p_bif": np.full(tone_count, np.nan, dtype=float),
        "p_bif_sub_3db": np.full(tone_count, np.nan, dtype=float),
        "power_cap_dbm": np.full(tone_count, np.nan, dtype=float),
        "power_target_dbm": np.full(tone_count, np.nan, dtype=float),
        "tx_only_power_dbm": np.full(tone_count, np.nan, dtype=float),
        "rx_constrained_power_dbm": np.full(tone_count, np.nan, dtype=float),
        "initial_bifurcation_margin_db": np.full(tone_count, np.nan, dtype=float),
        "tx_only_bifurcation_margin_db": np.full(tone_count, np.nan, dtype=float),
        "bifurcation_margin_db": np.full(tone_count, np.nan, dtype=float),
        "rx_constrained_bifurcation_margin_db": np.full(tone_count, np.nan, dtype=float),
        "initial_cap_margin_db": np.full(tone_count, np.nan, dtype=float),
        "tx_only_cap_margin_db": np.full(tone_count, np.nan, dtype=float),
        "cap_margin_db": np.full(tone_count, np.nan, dtype=float),
        "rx_constrained_cap_margin_db": np.full(tone_count, np.nan, dtype=float),
        "rx_offsets_db": np.full(tone_count, np.nan, dtype=float),
        "initial_predicted_rx_level_db": np.full(tone_count, np.nan, dtype=float),
        "tx_only_predicted_rx_level_db": np.full(tone_count, np.nan, dtype=float),
        "predicted_rx_level_db": np.full(tone_count, np.nan, dtype=float),
        "valid_mask": np.zeros(tone_count, dtype=bool),
    }
    for key in _BALANCED_POWER_ARRAY_KEYS:
        if key not in balanced_power:
            continue
        dtype = bool if key == "valid_mask" else int if key == "tone_index" else float
        arr = _restore_balanced_array(balanced_power[key], dtype=dtype)
        if arr.size > tone_count:
            raise ValueError(f"{key} is longer than tone_count.")
        out[key][: arr.size] = arr
    return out


def write_balanced_power(balanced_power, path):
    """Save balanced-power allocation output to JSON.

    Parameters
    ----------
    balanced_power : dict
        An :py:func:`balance_tone_powers` result.
    path : str or Path
        Output file, or a directory/extension-less path under which
        ``analysis/balanced_power.json`` is written.  Returns the file path.
    """
    target = Path(path)
    if target.is_dir() or target.suffix == "":
        target = target / BALANCED_POWER_FILE
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "kind": "souk_readout_tools.balanced_power",
        "created": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "result": _jsonify_best_row(balanced_power),
    }
    with target.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
    return target


def load_balanced_power(path):
    """Load a balanced-power allocation written by ``write_balanced_power``.

    Parameters
    ----------
    path : str or Path
        The JSON file, or a directory containing
        ``analysis/balanced_power.json``.
    """
    source = Path(path)
    if source.is_dir():
        source = source / BALANCED_POWER_FILE
    with source.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    result = payload.get("result", payload) if isinstance(payload, dict) else payload
    if not isinstance(result, dict):
        raise ValueError(f"{source} is not a balanced-power JSON file.")
    out = dict(result)
    reasons = out.get("invalid_reasons")
    if isinstance(reasons, dict):
        out["invalid_reasons"] = {int(k): v for k, v in reasons.items()}
    for key in _BALANCED_POWER_ARRAY_KEYS:
        if key not in out:
            continue
        dtype = bool if key == "valid_mask" else int if key == "tone_index" else float
        out[key] = _restore_balanced_array(out[key], dtype=dtype)
    return out


def write_best_power(best_power, path):
    """Save the list returned by :py:func:`find_best_power` to JSON.

    ``path`` may be a file name or a directory; for a directory, the file
    is written as ``analysis/best_power.json`` inside it.  The full per-tone
    dicts are preserved (every criterion pick, exclusion reason, and
    diagnostic field), so the load round-trip is lossless apart from NumPy
    scalar types being normalised to Python floats/ints.  Returns the path
    that was written.
    """
    if isinstance(best_power, dict):
        best_power = [best_power]
    target = Path(path)
    if target.is_dir() or target.suffix == "":
        target = target / BEST_POWER_FILE
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "kind": "souk_readout_tools.best_power",
        "created": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "tones": [_jsonify_best_row(row) for row in best_power],
    }
    with target.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
    return target


def load_best_power(path):
    """Load a best-power list previously written by :py:func:`write_best_power`.

    ``path`` may point at the JSON file directly or at the run directory that
    contains ``analysis/best_power.json``.  ``exclude_reasons`` keys are
    restored from strings to ``int`` so the loaded rows are interchangeable
    with an in-memory :py:func:`find_best_power` result.
    """
    source = Path(path)
    if source.is_dir():
        source = source / BEST_POWER_FILE
    with source.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    if isinstance(payload, list):
        rows = payload  # legacy format: bare list of rows
    else:
        rows = payload.get("tones", [])
    return [_restore_best_row(row) for row in rows]


def _jsonify_best_row(row):
    """Convert a best-power row to JSON-native types (recursive)."""
    if isinstance(row, dict):
        return {str(k): _jsonify_best_row(v) for k, v in row.items()}
    if isinstance(row, (list, tuple)):
        return [_jsonify_best_row(v) for v in row]
    if isinstance(row, (np.integer,)):
        return int(row)
    if isinstance(row, (np.floating,)):
        value = float(row)
        return value if np.isfinite(value) else None
    if isinstance(row, float):
        return row if np.isfinite(row) else None
    if isinstance(row, np.ndarray):
        return [_jsonify_best_row(v) for v in row.tolist()]
    return row


def _restore_best_row(row):
    """Restore int-keyed ``exclude_reasons`` and ``NaN`` floats after a JSON load."""
    if not isinstance(row, dict):
        return row
    out = dict(row)
    reasons = out.get("exclude_reasons")
    if isinstance(reasons, dict):
        out["exclude_reasons"] = {int(k): v for k, v in reasons.items()}
    for key in ("chosen_power_dbm", "p_bif", "p_bif_sub_3db", "bifurcation_anl"):
        if key in out and out[key] is None:
            out[key] = float("nan")
    chosen_params = out.get("chosen_params")
    if isinstance(chosen_params, dict):
        out["chosen_params"] = {
            k: (float("nan") if v is None else v)
            for k, v in chosen_params.items()
        }
    return out


def _linear_interp_extrap(xs, ys, target):
    """Linear interpolation in ``xs`` with linear extrapolation outside.

    Returns ``(value, extrapolated)``.  Outside the data range the line is
    extended through the two nearest endpoints.  With a single point the
    constant is returned and ``extrapolated`` reflects whether ``target``
    matches that point.
    """
    if xs.size == 0 or not np.isfinite(target):
        return float("nan"), False
    if xs.size == 1:
        return float(ys[0]), bool(target != xs[0])
    order = np.argsort(xs)
    xs = xs[order]
    ys = ys[order]
    if target < xs[0]:
        x0, x1, y0, y1 = xs[0], xs[1], ys[0], ys[1]
        if x1 == x0:
            return float(y0), True
        return float(y0 + (target - x0) * (y1 - y0) / (x1 - x0)), True
    if target > xs[-1]:
        x0, x1, y0, y1 = xs[-2], xs[-1], ys[-2], ys[-1]
        if x1 == x0:
            return float(y1), True
        return float(y0 + (target - x0) * (y1 - y0) / (x1 - x0)), True
    return float(np.interp(target, xs, ys)), False


def _nearest_finite_value(rows, powers, target_power, key):
    """Return the first finite ``rows[i][key]`` ordered by |power - target|.

    Returns ``NaN`` when no row has a finite value for ``key``.
    """
    if len(rows) == 0 or not np.isfinite(target_power):
        return float("nan")
    distances = np.abs(np.asarray(powers, dtype=float) - float(target_power))
    distances = np.where(np.isfinite(distances), distances, np.inf)
    for i in np.argsort(distances):
        try:
            v = float(rows[i].get(key, np.nan))
        except (TypeError, ValueError):
            continue
        if np.isfinite(v):
            return v
    return float("nan")


def _interpolated_params_at_power(
    rows, valid_mask, powers, target_power_dbm, *, anl_fit
):
    """Interpolate parameter-like fit-summary values at ``target_power_dbm``.

    Uses the Stage-1-valid rows (``valid_mask``) as the support for linear
    interpolation against ``power_dbm``, with linear extrapolation outside
    the measured range.  ``anl`` is evaluated from the existing log(anl)
    fit when it is reliable so the result is consistent with the
    ``anl_power_law`` pick.  Returns a dict with one entry per key in
    :data:`_INTERPOLATABLE_FIT_KEYS` plus:

    - ``power_dbm``: the target power echoed for convenience.
    - ``extrapolated``: ``True`` if any interpolated value used
      extrapolation outside the support range (or if ``anl`` was solved
      outside the ANL fit's clipped power range).
    - ``n_rows_used``: number of Stage-1-valid rows available.
    - ``fr_source``: how ``fr`` was obtained — ``"interpolated"`` when the
      linear interpolation produced a finite value, otherwise the fallback
      that supplied it: ``"nearest_fitted"`` (nearest row's fitted ``fr``),
      ``"nearest_empirical"`` (nearest row's ``empirical_fr``),
      ``"nearest_sweep_center"`` (nearest row's ``sweep_center_hz``), or
      ``"unavailable"`` if no fallback was usable.  When a fallback is
      used, ``fr`` is overwritten so it is never ``NaN`` as long as at
      least one row has any of those fields.
    """
    try:
        target = float(target_power_dbm)
    except (TypeError, ValueError):
        target = float("nan")
    n_rows_used = int(np.count_nonzero(valid_mask)) if valid_mask is not None else 0
    out = {
        "power_dbm": target if np.isfinite(target) else float("nan"),
        "extrapolated": False,
        "n_rows_used": n_rows_used,
        "fr_source": "unavailable",
    }
    for key in _INTERPOLATABLE_FIT_KEYS:
        out[key] = float("nan")

    valid_indices = (
        np.flatnonzero(valid_mask) if valid_mask is not None else np.arange(len(rows))
    )
    if np.isfinite(target) and valid_indices.size:
        valid_powers = powers[valid_indices]
        finite_pwr = np.isfinite(valid_powers)
        valid_indices = valid_indices[finite_pwr]
        valid_powers = valid_powers[finite_pwr]
    else:
        valid_indices = np.array([], dtype=int)
        valid_powers = np.array([], dtype=float)

    extrapolated_any = False
    if np.isfinite(target) and valid_indices.size:
        for key in _INTERPOLATABLE_FIT_KEYS:
            if (
                key == "anl"
                and anl_fit is not None
                and bool(anl_fit.get("reliable", False))
            ):
                slope = float(anl_fit.get("slope_log_anl_per_db", float("nan")))
                intercept = float(anl_fit.get("intercept_log_anl", float("nan")))
                if np.isfinite(slope) and np.isfinite(intercept):
                    out[key] = float(np.exp(slope * target + intercept))
                    fit_mask = np.asarray(anl_fit.get("fit_mask", []), dtype=bool)
                    if fit_mask.size and np.any(fit_mask):
                        fit_powers = powers[np.flatnonzero(fit_mask)]
                        fit_powers = fit_powers[np.isfinite(fit_powers)]
                        if fit_powers.size and (
                            target < fit_powers.min() or target > fit_powers.max()
                        ):
                            extrapolated_any = True
                    continue

            ys = np.empty(valid_indices.size, dtype=float)
            for j, i in enumerate(valid_indices):
                try:
                    ys[j] = float(rows[i].get(key, np.nan))
                except (TypeError, ValueError):
                    ys[j] = float("nan")
            finite = np.isfinite(ys)
            xs = valid_powers[finite]
            ys = ys[finite]
            value, was_extrap = _linear_interp_extrap(xs, ys, target)
            out[key] = value
            if was_extrap:
                extrapolated_any = True
    out["extrapolated"] = bool(extrapolated_any)

    if np.isfinite(out["fr"]):
        out["fr_source"] = "interpolated"
    else:
        for source_key, source_label in (
            ("fr", "nearest_fitted"),
            ("empirical_fr", "nearest_empirical"),
            ("sweep_center_hz", "nearest_sweep_center"),
        ):
            value = _nearest_finite_value(rows, powers, target, source_key)
            if np.isfinite(value):
                out["fr"] = value
                out["fr_source"] = source_label
                break

    return out


def _find_best_power_for_tone(
    tone_index,
    tone_rows,
    *,
    target_anl,
    bifurcation_anl,
    param_valid_ranges,
    param_outliers_mad_clip,
    param_uncertainty_outlier_mad_clip,
    require_success,
    use_readback_power,
    anl_fit_min_points,
    anl_fit_clip_sigma,
    anl_fit_min_slope,
    anl_fit_weight_by_uncertainty,
):
    """Per-tone implementation backing :py:func:`find_best_power`."""

    def _power(row):
        if use_readback_power:
            value = row.get("readback_power_dbm", np.nan)
            try:
                value = float(value)
            except (TypeError, ValueError):
                value = np.nan
            if np.isfinite(value):
                return value
        try:
            return float(row.get("power_dbm", np.nan))
        except (TypeError, ValueError):
            return np.nan

    def _power_sort_key(row):
        power = _power(row)
        return (not np.isfinite(power), power if np.isfinite(power) else 0.0)

    rows = sorted(tone_rows, key=_power_sort_key)
    n = len(rows)
    sweep_idx = np.array([int(r["sweep_index"]) for r in rows], dtype=int)
    powers = np.array([_power(r) for r in rows], dtype=float)

    def _column(name):
        out = np.empty(n, dtype=float)
        for i, row in enumerate(rows):
            try:
                out[i] = float(row.get(name, np.nan))
            except (TypeError, ValueError):
                out[i] = np.nan
        return out

    # Stage 1: per-row auto-exclusion.
    excluded = np.zeros(n, dtype=bool)
    exclude_reason = [""] * n
    anl_fit_excluded = np.zeros(n, dtype=bool)
    anl_fit_exclude_reason = [""] * n

    # 1a: solver-reported failure.
    if require_success:
        for i, row in enumerate(rows):
            success = row.get("success", True)
            if isinstance(success, str):
                success = success.strip().lower() not in {"false", "0", "no"}
            if success is False or success == 0:
                excluded[i] = True
                exclude_reason[i] = "success=False"

    # 1b: per-parameter validity-range exclusion (catches parameters pegged
    # at a fitter clamp). Low-clamped ANL values are useful for the measured
    # threshold fallback, but not for fitting the ANL-vs-power slope.
    for name, (lo, hi) in param_valid_ranges.items():
        values = _column(name)
        if name == "anl":
            low_pegged = (~excluded) & np.isfinite(values) & (values <= lo)
            for i in np.flatnonzero(low_pegged):
                anl_fit_excluded[i] = True
                anl_fit_exclude_reason[i] = f"{name} outside ({lo:g}, {hi:g})"
            high_pegged = (~excluded) & np.isfinite(values) & (values >= hi)
            for i in np.flatnonzero(high_pegged):
                excluded[i] = True
                exclude_reason[i] = f"{name} outside ({lo:g}, {hi:g})"
                anl_fit_excluded[i] = True
                anl_fit_exclude_reason[i] = exclude_reason[i]
            continue
        out_of_range = (
            (~excluded)
            & np.isfinite(values)
            & ((values <= lo) | (values >= hi))
        )
        for i in np.flatnonzero(out_of_range):
            excluded[i] = True
            exclude_reason[i] = f"{name} outside ({lo:g}, {hi:g})"

    # 1c: per-tone MAD outlier check on parameters expected to be
    # near-constant across powers.
    if param_outliers_mad_clip is not None:
        clip_k = float(param_outliers_mad_clip)
        for key in _MAD_OUTLIER_CHECK_PARAMS:
            values = _column(key)
            eligible = (~excluded) & np.isfinite(values)
            if np.count_nonzero(eligible) < 3:
                continue
            ref = values[eligible]
            centre = float(np.median(ref))
            scale = _MAD_TO_SIGMA * float(np.median(np.abs(ref - centre)))
            if not (np.isfinite(scale) and scale > 0.0):
                continue
            deviation = np.abs(values - centre)
            flag = eligible & (deviation > clip_k * scale)
            for i in np.flatnonzero(flag):
                excluded[i] = True
                exclude_reason[i] = (
                    f"{key} outside median ± {clip_k:g}*MAD"
                )

    # 1d: per-tone MAD outlier check on the uncertainty of those same
    # parameters, so rows with wildly inflated <name>_err are dropped.
    if param_uncertainty_outlier_mad_clip is not None:
        clip_k = float(param_uncertainty_outlier_mad_clip)
        for key in _MAD_OUTLIER_CHECK_PARAMS:
            err_key = f"{key}_err"
            values = _column(err_key)
            eligible = (~excluded) & np.isfinite(values)
            if np.count_nonzero(eligible) < 3:
                continue
            ref = values[eligible]
            centre = float(np.median(ref))
            scale = _MAD_TO_SIGMA * float(np.median(np.abs(ref - centre)))
            if not (np.isfinite(scale) and scale > 0.0):
                continue
            deviation = np.abs(values - centre)
            flag = eligible & (deviation > clip_k * scale)
            for i in np.flatnonzero(flag):
                excluded[i] = True
                exclude_reason[i] = (
                    f"{err_key} outside median ± {clip_k:g}*MAD"
                )

    # Stage 2: derived columns and validity mask.
    anl = _column("anl")
    anl_err = _column("anl_err")
    valid = ~excluded
    anl_fit_valid = valid & ~anl_fit_excluded
    anl_ok = (
        valid
        & np.isfinite(anl)
        & (anl > 0.0)
        & (anl < float(target_anl))
    )
    # Highest power index that still satisfies the ANL ceiling, or -1 if none.
    _anl_ok_indices = np.flatnonzero(anl_ok)
    anl_idx = int(_anl_ok_indices[-1]) if _anl_ok_indices.size else -1

    try:
        _bifurcation_anl_value = float(bifurcation_anl)
    except (TypeError, ValueError):
        _bifurcation_anl_value = float("nan")
    try:
        _target_anl_value = float(target_anl)
    except (TypeError, ValueError):
        _target_anl_value = float("nan")
    if (
        np.isfinite(_bifurcation_anl_value)
        and _bifurcation_anl_value > 0.0
        and np.isfinite(_target_anl_value)
    ):
        anl_safe = (
            valid
            & np.isfinite(anl)
            & np.isfinite(powers)
            & (anl > 0.0)
            & (anl < _bifurcation_anl_value)
        )
        safe_indices = np.flatnonzero(anl_safe)
        if safe_indices.size:
            nearest_target_idx = int(
                safe_indices[
                    np.argmin(np.abs(anl[safe_indices] - _target_anl_value))
                ]
            )
        else:
            nearest_target_idx = -1
    else:
        nearest_target_idx = -1

    # Stage 3: ANL power-law fit on the non-excluded rows, skipping
    # low-clamped ANL values that should not pull the fit slope down.
    anl_fit = _fit_log_anl_power_law(
        powers,
        anl,
        anl_fit_valid,
        anl_err=anl_err,
        target_anl=target_anl,
        min_points=anl_fit_min_points,
        clip_sigma=anl_fit_clip_sigma,
        min_slope=anl_fit_min_slope,
        weight_by_uncertainty=anl_fit_weight_by_uncertainty,
    )

    # Rows excluded from the ANL fit because anl pegged at a fitter bound;
    # tracked separately so plots can mark them distinctly from generic
    # excluded rows.
    anl_pegged_sweep_indices = [
        int(sweep_idx[i])
        for i in range(n)
        if (
            anl_fit_excluded[i]
            and anl_fit_exclude_reason[i].startswith("anl outside")
        )
    ]
    anl_pegged = bool(anl_pegged_sweep_indices) and not np.any(
        anl_fit_valid & np.isfinite(anl)
    )

    def _pack(i):
        if i < 0:
            return None
        return {
            "sweep_index": int(sweep_idx[i]),
            "power_dbm": float(powers[i]),
            "anl": float(anl[i]),
        }

    def _lowest_power_pick():
        finite_power_indices = np.flatnonzero(np.isfinite(powers))
        if finite_power_indices.size:
            i = int(finite_power_indices[np.argmin(powers[finite_power_indices])])
        elif n:
            i = 0
        else:
            return None
        return {
            "sweep_index": int(sweep_idx[i]),
            "power_dbm": float(powers[i]),
            "anl": float(anl[i]),
            "reason": (
                "no reliable fit-derived pick; using lowest stepped power"
            ),
        }

    def _pack_anl_power_law():
        fit_mask = np.asarray(anl_fit["fit_mask"], dtype=bool)
        outlier_mask = np.asarray(anl_fit["outlier_mask"], dtype=bool)
        target_power = anl_fit.get("target_power_dbm")
        range_position = anl_fit.get("range_position", "unavailable")
        reliable = bool(anl_fit.get("reliable", False))
        try:
            bifurcation_anl_value = float(bifurcation_anl)
        except (TypeError, ValueError):
            bifurcation_anl_value = float("nan")
        p_bif = (
            _solve_log_anl_power(
                float(anl_fit["slope_log_anl_per_db"]),
                float(anl_fit["intercept_log_anl"]),
                bifurcation_anl_value,
            )
            if reliable else float("nan")
        )
        p_bif_sub_3db = (
            float(p_bif - DEFAULT_BIFURCATION_BACKOFF_DB)
            if np.isfinite(p_bif) else float("nan")
        )
        reference_idx = -1
        if target_power is not None and np.any(fit_mask):
            fit_indices = np.flatnonzero(fit_mask)
            reference_idx = int(
                fit_indices[np.argmin(np.abs(powers[fit_indices] - target_power))]
            )

        # When the fit is reliable, always use the (extrapolated if needed)
        # target power solved from the log(anl) line. ``range_position`` is
        # kept as a diagnostic only.
        if reliable and target_power is not None:
            selected_power = float(target_power)
        else:
            selected_power = None

        return {
            "criterion": "anl_power_law",
            "reliable": reliable,
            "reason": anl_fit.get("reason", ""),
            "target_anl": float(target_anl),
            "bifurcation_anl": bifurcation_anl_value,
            "power_dbm": selected_power,
            "p_bif": p_bif,
            "p_bif_sub_3db": p_bif_sub_3db,
            "sweep_index": None,
            "reference_sweep_index": (
                int(sweep_idx[reference_idx]) if reference_idx >= 0 else None
            ),
            "target_power_dbm": target_power,
            "range_position": range_position,
            "slope_log_anl_per_db": float(anl_fit["slope_log_anl_per_db"]),
            "intercept_log_anl": float(anl_fit["intercept_log_anl"]),
            "initial_slope_log_anl_per_db": float(
                anl_fit["initial_slope_log_anl_per_db"]
            ),
            "initial_intercept_log_anl": float(
                anl_fit["initial_intercept_log_anl"]
            ),
            "weighted": bool(anl_fit.get("weighted", False)),
            "weight_source": str(anl_fit.get("weight_source", "unweighted")),
            "fit_sweep_indices": [
                int(sweep_idx[i]) for i in range(n) if fit_mask[i]
            ],
            "outlier_sweep_indices": [
                int(sweep_idx[i]) for i in range(n) if outlier_mask[i]
            ],
            "pegged_sweep_indices": list(anl_pegged_sweep_indices),
        }

    anl_power_law_pick = _pack_anl_power_law()
    anl_pick = _pack(anl_idx)
    anl_nearest_target_pick = _pack(nearest_target_idx)
    lowest_power_pick = _lowest_power_pick()

    # Stage 4: choose the most reliable available criterion.  The continuous
    # ANL power-law pick wins when it passed its diagnostics; otherwise fall
    # back to the highest-power row whose measured anl is below the target,
    # then to the row whose measured anl is nearest to the target among
    # those still below the bifurcation threshold, and finally to the
    # lowest measured power.
    chosen_power = None
    chosen_sweep = None
    chosen_reference = None
    criterion = "none"
    if (
        anl_power_law_pick["reliable"]
        and anl_power_law_pick["power_dbm"] is not None
    ):
        chosen_power = anl_power_law_pick["power_dbm"]
        chosen_sweep = anl_power_law_pick["sweep_index"]
        chosen_reference = anl_power_law_pick["reference_sweep_index"]
        criterion = anl_power_law_pick["criterion"]
    elif anl_pick is not None:
        chosen_power = anl_pick["power_dbm"]
        chosen_sweep = anl_pick["sweep_index"]
        chosen_reference = chosen_sweep
        criterion = "anl_threshold"
    elif anl_nearest_target_pick is not None:
        chosen_power = anl_nearest_target_pick["power_dbm"]
        chosen_sweep = anl_nearest_target_pick["sweep_index"]
        chosen_reference = chosen_sweep
        criterion = "anl_nearest_target"
    try:
        chosen_power_is_finite = np.isfinite(float(chosen_power))
    except (TypeError, ValueError):
        chosen_power_is_finite = False
    if not chosen_power_is_finite and lowest_power_pick is not None:
        chosen_power = lowest_power_pick["power_dbm"]
        chosen_sweep = lowest_power_pick["sweep_index"]
        chosen_reference = chosen_sweep
        criterion = "lowest_power_fallback"

    chosen_params = _interpolated_params_at_power(
        rows, valid, powers, chosen_power, anl_fit=anl_fit
    )

    return {
        "tone_index": int(tone_index),
        "chosen_sweep_index": chosen_sweep,
        "chosen_reference_sweep_index": chosen_reference,
        "chosen_power_dbm": chosen_power,
        "chosen_criterion": criterion,
        "chosen_params": chosen_params,
        "bifurcation_anl": anl_power_law_pick["bifurcation_anl"],
        "p_bif": anl_power_law_pick["p_bif"],
        "p_bif_sub_3db": anl_power_law_pick["p_bif_sub_3db"],
        "anl_power_law_pick": anl_power_law_pick,
        "anl_pick": anl_pick,
        "anl_nearest_target_pick": anl_nearest_target_pick,
        "lowest_power_pick": lowest_power_pick,
        "criteria": {
            "anl_power_law": anl_power_law_pick,
            "anl_threshold": anl_pick,
            "anl_nearest_target": anl_nearest_target_pick,
            "lowest_power_fallback": lowest_power_pick,
        },
        "anl_pegged": bool(anl_pegged),
        "excluded_sweep_indices": [
            int(sweep_idx[i]) for i in range(n) if excluded[i]
        ],
        "exclude_reasons": {
            int(sweep_idx[i]): exclude_reason[i]
            for i in range(n)
            if excluded[i]
        },
        "anl_outlier_sweep_indices": anl_power_law_pick["outlier_sweep_indices"],
        "anl_pegged_sweep_indices": anl_power_law_pick["pegged_sweep_indices"],
    }


def _fit_data_tone_indices(summary):
    """Return tone indices implied by fit_data when no rows are available."""
    if not _is_fit_data(summary):
        return []
    if "tone_index" in summary:
        try:
            return [int(summary["tone_index"])]
        except (TypeError, ValueError):
            return []
    run = summary.get("run", {})
    try:
        tone_count = int(run.get("tone_count", 0))
    except (TypeError, ValueError):
        tone_count = 0
    return list(range(max(tone_count, 0)))


def _save_empty_best_power_plot(
    plt,
    output_dir,
    tone_index,
    reason,
    *,
    show,
    dpi,
    figsize,
):
    """Write an empty best-power diagnostic figure with an explanatory note."""
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_title(
        "ANL power selection unavailable"
        if tone_index is None
        else f"Tone {tone_index} ANL power selection"
    )
    ax.set_xlabel("Tone power (dBm)")
    ax.set_ylabel("ANL")
    ax.text(
        0.5,
        0.5,
        reason,
        transform=ax.transAxes,
        ha="center",
        va="center",
        color="0.35",
        wrap=True,
    )
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
    fig.tight_layout()

    filename = (
        "best_power_unavailable.png"
        if tone_index is None
        else f"tone_{int(tone_index):04d}_best_power.png"
    )
    path = Path(output_dir) / filename
    fig.savefig(path, dpi=dpi)
    if not show:
        plt.close(fig)
    return str(path)


_BEST_POWER_CRITERION_LABELS = {
    "anl_power_law": "fitted target ANL",
    "anl_threshold": "highest measured power below target ANL",
    "anl_nearest_target": "measured ANL nearest target",
    "lowest_power_fallback": "lowest measured power fallback",
    "none": "no selection",
}


def _force_agg_worker_init():
    """Pool initializer: force a headless backend in best-power workers."""
    import matplotlib

    matplotlib.use("Agg")


def _plot_best_power_one_tone(
    tone_index,
    tone_rows,
    result,
    *,
    use_readback_power,
    anl_errorbar_scale,
    output_dir,
    dpi,
    figsize,
    show,
    show_anl_errors,
    show_weights,
    target_anl,
    bifurcation_anl,
):
    """Render and save one tone's ANL-power-selection PNG.

    Returns the list of PNG path(s) written for this tone (one entry; an
    explanatory placeholder when there is nothing to fit).  Pulled out of
    :py:func:`plot_best_power` so the per-tone work can run serially or in a
    worker process; both paths share this body.
    """
    import matplotlib.pyplot as plt
    from matplotlib import ticker as _mpl_ticker

    output_dir = Path(output_dir)
    tone_index = int(tone_index)

    def _as_float(value):
        """Coerce to float, returning NaN for missing/non-numeric values."""
        try:
            return float(value)
        except (TypeError, ValueError):
            return np.nan

    def _row_power(row):
        """The tone power for a summary row (readback if available, else requested)."""
        if use_readback_power:
            readback = _as_float(row.get("readback_power_dbm", np.nan))
            if np.isfinite(readback):
                return readback
        return _as_float(row.get("power_dbm", np.nan))

    if not tone_rows:
        return [_save_empty_best_power_plot(
            plt, output_dir, tone_index,
            f"No fit summary rows are available for tone {tone_index}.",
            show=show, dpi=dpi, figsize=figsize)]
    if result is None:
        return [_save_empty_best_power_plot(
            plt, output_dir, tone_index,
            f"No best-power result is available for tone {tone_index}.",
            show=show, dpi=dpi, figsize=figsize)]

    tone_rows = sorted(
        tone_rows,
        key=lambda row: (
            not np.isfinite(_row_power(row)),
            _row_power(row) if np.isfinite(_row_power(row)) else 0.0,
        ),
    )
    sweep_idx = np.asarray(
        [int(row["sweep_index"]) for row in tone_rows],
        dtype=int,
    )
    powers = np.asarray([_row_power(row) for row in tone_rows], dtype=float)
    anl = np.asarray(
        [_as_float(row.get("anl", np.nan)) for row in tone_rows],
        dtype=float,
    )
    anl_err = np.asarray(
        [_as_float(row.get("anl_err", np.nan)) for row in tone_rows],
        dtype=float,
    ) * anl_errorbar_scale
    plot_mask = np.isfinite(powers) & np.isfinite(anl) & (anl > 0.0)
    if not np.any(plot_mask):
        return [_save_empty_best_power_plot(
            plt, output_dir, tone_index,
            f"No finite positive ANL values are available for tone "
            f"{tone_index}.",
            show=show, dpi=dpi, figsize=figsize)]

    pick = result.get("anl_power_law_pick") or {}
    excluded_set = set(result.get("excluded_sweep_indices", ()))
    pegged_set = set(result.get("anl_pegged_sweep_indices", ()))
    outlier_set = set(result.get("anl_outlier_sweep_indices", ()))
    fit_set = set(pick.get("fit_sweep_indices", ()))

    excluded = np.asarray([idx in excluded_set for idx in sweep_idx]) & plot_mask
    pegged = np.asarray([idx in pegged_set for idx in sweep_idx]) & plot_mask
    outlier = np.asarray([idx in outlier_set for idx in sweep_idx]) & plot_mask
    fit_points = np.asarray([idx in fit_set for idx in sweep_idx]) & plot_mask
    other = plot_mask & ~(excluded | pegged | outlier | fit_points)

    if show_weights:
        fig, (ax, ax_weight) = plt.subplots(
            2,
            1,
            figsize=figsize,
            sharex=True,
            gridspec_kw={"height_ratios": [3, 1], "hspace": 0.08},
        )
    else:
        fig, ax = plt.subplots(figsize=figsize)
        ax_weight = None

    error_mask = plot_mask & np.isfinite(anl_err) & (anl_err > 0.0)
    error_lower = None
    error_upper = None
    if show_anl_errors:
        if np.any(error_mask):
            error_lower = np.minimum(
                anl_err[error_mask],
                0.9 * anl[error_mask],
            )
            error_upper = anl_err[error_mask]
            ax.errorbar(
                powers[error_mask],
                anl[error_mask],
                yerr=np.vstack([error_lower, error_upper]),
                fmt="none",
                ecolor="0.35",
                elinewidth=0.95,
                capsize=2.0,
                alpha=0.8,
                label=(
                    "ANL 1-sigma error"
                    if np.isclose(anl_errorbar_scale, 1.0)
                    else f"ANL {anl_errorbar_scale:g}-sigma error"
                ),
                zorder=0,
            )

    def _scatter(mask, **kwargs):
        """Scatter the ANL-vs-power points selected by ``mask`` (skip if empty)."""
        if np.any(mask):
            ax.scatter(powers[mask], anl[mask], **kwargs)

    _scatter(
        other,
        color="0.55",
        marker="o",
        label="other positive ANL rows",
        zorder=2,
    )
    _scatter(
        fit_points,
        color="tab:blue",
        marker="o",
        s=48,
        label="ANL fit points",
        zorder=4,
    )
    _scatter(
        pegged,
        facecolors="none",
        edgecolors="tab:orange",
        marker="s",
        label="ANL pegged; not fit",
        zorder=5,
    )
    _scatter(
        outlier,
        color="tab:red",
        marker="x",
        s=55,
        label="clipped from ANL fit",
        zorder=6,
    )
    _scatter(
        excluded,
        color="0.35",
        marker="x",
        s=45,
        label="excluded before ANL fit",
        zorder=3,
    )

    target_value = _as_float(pick.get("target_anl", target_anl))
    if np.isfinite(target_value) and target_value > 0.0:
        ax.axhline(
            target_value,
            color="tab:purple",
            linestyle="--",
            linewidth=1.1,
            label=f"target ANL {target_value:g}",
        )
    bifurcation_value = _as_float(
        pick.get(
            "bifurcation_anl",
            result.get("bifurcation_anl", bifurcation_anl),
        )
    )
    if np.isfinite(bifurcation_value) and bifurcation_value > 0.0:
        ax.axhline(
            bifurcation_value,
            color="tab:red",
            linestyle="--",
            linewidth=1.0,
            label=f"ANL_bif {bifurcation_value:g}",
        )

    slope = _as_float(pick.get("slope_log_anl_per_db", np.nan))
    intercept = _as_float(pick.get("intercept_log_anl", np.nan))
    initial_slope = _as_float(
        pick.get("initial_slope_log_anl_per_db", np.nan)
    )
    initial_intercept = _as_float(
        pick.get("initial_intercept_log_anl", np.nan)
    )
    target_power = _as_float(pick.get("target_power_dbm", np.nan))
    chosen_power = _as_float(result.get("chosen_power_dbm", np.nan))
    p_bif = _as_float(
        pick.get("p_bif", result.get("p_bif", np.nan))
    )
    p_bif_sub_3db = _as_float(
        pick.get("p_bif_sub_3db", result.get("p_bif_sub_3db", np.nan))
    )
    reliable_anl_fit = bool(pick.get("reliable", False))
    weighted_anl_fit = bool(pick.get("weighted", False))
    finite_x = powers[plot_mask]
    line_limits = list(finite_x)
    if reliable_anl_fit and np.isfinite(target_power):
        line_limits.append(target_power)
    if np.isfinite(chosen_power):
        line_limits.append(chosen_power)
    if reliable_anl_fit and np.isfinite(p_bif):
        line_limits.append(p_bif)
    if reliable_anl_fit and np.isfinite(p_bif_sub_3db):
        line_limits.append(p_bif_sub_3db)
    if len(line_limits) >= 2:
        xmin = float(np.nanmin(line_limits))
        xmax = float(np.nanmax(line_limits))
        pad = max(0.5, 0.05 * max(xmax - xmin, 1e-9))
        x_line = np.linspace(xmin - pad, xmax + pad, 200)
    else:
        x_line = None
    if (
        x_line is not None
        and np.any(outlier)
        and np.isfinite(initial_slope)
        and np.isfinite(initial_intercept)
    ):
        initial_y_line = np.exp(initial_slope * x_line + initial_intercept)
        ax.plot(
            x_line,
            initial_y_line,
            color="0.45",
            linewidth=1.0,
            linestyle=":",
            alpha=0.45,
            label="initial ANL fit before clipping",
            zorder=1,
        )
    if (
        x_line is not None
        and np.isfinite(slope)
        and np.isfinite(intercept)
    ):
        y_line = np.exp(slope * x_line + intercept)
        ax.plot(
            x_line,
            y_line,
            color="black" if reliable_anl_fit else "0.35",
            linewidth=1.4,
            linestyle="-" if reliable_anl_fit else "--",
            label=(
                (
                    "weighted final ANL fit"
                    if weighted_anl_fit
                    else "final ANL fit"
                )
                if reliable_anl_fit else "rejected final ANL fit"
            ),
            zorder=1,
        )

    if reliable_anl_fit and np.isfinite(target_power):
        ax.axvline(
            target_power,
            color="tab:purple",
            linestyle=":",
            linewidth=1.4,
            label="fitted target power",
        )
    if reliable_anl_fit and np.isfinite(p_bif):
        ax.axvline(
            p_bif,
            color="tab:red",
            linestyle=":",
            linewidth=1.25,
            label="p_bif",
        )
    if reliable_anl_fit and np.isfinite(p_bif_sub_3db):
        ax.axvline(
            p_bif_sub_3db,
            color="tab:orange",
            linestyle=":",
            linewidth=1.15,
            label="p_bif_sub_3db",
        )
    if np.isfinite(chosen_power):
        # Map the internal criterion key to a plot-friendly label.
        criterion = str(result.get("chosen_criterion") or "none")
        criterion_label = _BEST_POWER_CRITERION_LABELS.get(
            criterion, criterion.replace("_", " "))
        ax.axvline(
            chosen_power,
            color="tab:green",
            linestyle="-",
            linewidth=1.6,
            label=f"chosen: {criterion_label}",
        )

    fallback_lines = (
        (
            "anl_pick",
            "fallback: highest measured power below target ANL",
            "tab:cyan",
            "-.",
        ),
    )
    drawn = []
    for key, label, colour, linestyle in fallback_lines:
        item = result.get(key)
        if not item:
            continue
        power = _as_float(item.get("power_dbm", np.nan))
        if not np.isfinite(power):
            continue
        if any(abs(power - existing) < 1e-9 for existing in drawn):
            continue
        drawn.append(power)
        ax.axvline(
            power,
            color=colour,
            linestyle=linestyle,
            linewidth=0.9,
            alpha=0.75,
            label=label,
        )

    rejection_reason = str(pick.get("reason", "") or "").strip()
    if not reliable_anl_fit and rejection_reason:
        ax.text(
            0.02,
            0.98,
            "ANL fit rejected:\n"
            + textwrap.fill(rejection_reason, width=48),
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize="small",
            color="0.2",
            bbox={
                "boxstyle": "round,pad=0.35",
                "facecolor": "white",
                "edgecolor": "0.65",
                "alpha": 0.88,
            },
            zorder=10,
        )

    positive = list(anl[plot_mask])
    if (
        show_anl_errors
        and np.any(error_mask)
        and error_lower is not None
        and error_upper is not None
    ):
        positive.extend(
            np.maximum(
                anl[error_mask] - error_lower,
                np.finfo(float).tiny,
            )
        )
        positive.extend(anl[error_mask] + error_upper)
    if np.isfinite(target_value) and target_value > 0.0:
        positive.append(target_value)
    if np.isfinite(bifurcation_value) and bifurcation_value > 0.0:
        positive.append(bifurcation_value)
    positive = np.asarray(positive, dtype=float)
    positive = positive[np.isfinite(positive) & (positive > 0.0)]
    if positive.size:
        ymin = float(np.nanmin(positive))
        ymax = float(np.nanmax(positive))
        if ymax <= ymin:
            ymin, ymax = ymin / 2.0, ymax * 2.0
        ax.set_ylim(ymin / 1.6, ymax * 1.6)
    ax.set_yscale("log")
    if ax_weight is None:
        ax.set_xlabel("Tone power (dBm)")
    ax.set_ylabel("ANL")
    ax.set_title(f"Tone {tone_index} ANL power selection")
    ax.grid(True, which="both", alpha=0.25)
    ax.legend(
        fontsize="small",
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
        borderaxespad=0.0,
        frameon=True,
    )

    if ax_weight is not None:
        anl_err_raw = np.asarray(
            [_as_float(row.get("anl_err", np.nan)) for row in tone_rows],
            dtype=float,
        )
        fit_idx = np.flatnonzero(fit_points)
        weight_frac = np.full(anl.shape, np.nan, dtype=float)
        msg = None
        if fit_idx.size >= 1:
            with np.errstate(divide="ignore", invalid="ignore"):
                sigmas = np.where(
                    np.isfinite(anl_err_raw[fit_idx])
                    & (anl_err_raw[fit_idx] > 0.0)
                    & (anl[fit_idx] > 0.0),
                    anl_err_raw[fit_idx]
                    / np.where(anl[fit_idx] > 0.0, anl[fit_idx], np.nan),
                    np.nan,
                )
            finite_s = np.isfinite(sigmas) & (sigmas > 0.0)
            if weighted_anl_fit and np.count_nonzero(finite_s) >= 2:
                if not np.all(finite_s):
                    sigmas = sigmas.copy()
                    sigmas[~finite_s] = float(
                        np.nanmedian(sigmas[finite_s])
                    )
                with np.errstate(divide="ignore", invalid="ignore"):
                    weights = 1.0 / np.square(sigmas)
                total = float(np.nansum(weights))
                if np.isfinite(total) and total > 0.0:
                    weight_frac[fit_idx] = weights / total
                else:
                    msg = "no usable weights"
            elif fit_idx.size >= 1:
                weight_frac[fit_idx] = 1.0 / fit_idx.size
                if not weighted_anl_fit:
                    msg = "fit is unweighted; equal contribution shown"
        else:
            msg = "no fit points"

        plot_w = (
            fit_points
            & np.isfinite(weight_frac)
            & (weight_frac > 0.0)
        )
        if np.any(plot_w):
            values_pct = weight_frac[plot_w] * 100.0
            ax_weight.scatter(
                powers[plot_w],
                values_pct,
                color="tab:blue",
                marker="o",
                s=32,
                zorder=4,
                label="weight fraction",
            )
            uniform_pct = 100.0 / int(np.count_nonzero(plot_w))
            ax_weight.axhline(
                uniform_pct,
                color="0.45",
                linestyle="--",
                linewidth=0.9,
                alpha=0.7,
                label=f"equal weight = {uniform_pct:.2g}%",
            )
            ax_weight.set_yscale("log")
            vmax = float(np.nanmax(values_pct))
            vmin = float(np.nanmin(values_pct))
            lo = min(vmin, uniform_pct) / 3.0
            hi = max(vmax, uniform_pct) * 3.0
            if not (np.isfinite(lo) and lo > 0.0):
                lo = max(vmin * 0.3, 1e-6)
            ax_weight.set_ylim(lo, hi)
            ax_weight.legend(fontsize="x-small", loc="best")
        else:
            ax_weight.set_ylim(0.1, 100.0)
            ax_weight.set_yscale("log")
            ax_weight.text(
                0.5,
                0.5,
                msg or "no weight info",
                transform=ax_weight.transAxes,
                ha="center",
                va="center",
                fontsize="small",
                color="0.4",
            )

        ax_weight.set_xlabel("Tone power (dBm)")
        ax_weight.set_ylabel("Fit weight (%)")
        ax_weight.grid(True, which="both", alpha=0.25)
        ax_weight.yaxis.set_major_locator(
            _mpl_ticker.LogLocator(base=10.0, subs=(1.0, 3.0), numticks=20)
        )
        ax_weight.yaxis.set_major_formatter(
            _mpl_ticker.FormatStrFormatter("%g")
        )
        ax_weight.yaxis.set_minor_locator(
            _mpl_ticker.LogLocator(
                base=10.0, subs=(2.0, 5.0, 7.0), numticks=40
            )
        )
        ax_weight.yaxis.set_minor_formatter(_mpl_ticker.NullFormatter())


    # bbox_inches='tight' lays the figure out at save time (including the
    # outside legend) without emitting tight_layout's "incompatible Axes"
    # warning on the equal-aspect IQ-adjacent layout.
    path = output_dir / f"tone_{tone_index:04d}_best_power.png"
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    if not show:
        plt.close(fig)
    return [str(path)]


def plot_best_power(
    summary,
    best_power=None,
    *,
    tone_indices=None,
    output_dir=None,
    target_anl=0.01,
    bifurcation_anl=DEFAULT_BIFURCATION_ANL,
    use_readback_power=True,
    show_anl_errors=True,
    anl_errorbar_scale=1.0,
    show_weights=True,
    show=False,
    dpi=80,
    figsize=(6.8, 4.2),
    n_jobs=1,
    verbose=True,
    **find_best_power_kwargs,
):
    """Plot ANL-vs-power fits and selected powers for each tone.

    ``summary`` may be an in-memory result from :py:func:`fit_power_sweep`, a
    structured summary array, a single summary row, or a CSV path.  When ``best_power`` is omitted this function calls
    :py:func:`find_best_power` with the same ``target_anl`` and
    ``use_readback_power`` settings plus any extra ``find_best_power_kwargs``.

    Each saved figure shows the measured ``anl`` values, scaled ``anl_err``
    error bars when available, rows excluded before selection,
    pegged ANL values, the clipped ANL outliers, the retained log-linear ANL
    fit, the target ANL line, the ``ANL_bif`` line, the fitted target power,
    ``p_bif`` / ``p_bif_sub_3db``, the chosen power, and the measured-ANL
    fallback pick when present.  ``anl_errorbar_scale`` is display-only; it
    does not change the ANL fit weights used by :py:func:`find_best_power`.

    When ``show_weights`` is true a second panel beneath the main plot shows
    each fit point's fractional contribution to the weighted log-ANL fit,
    ``w_i / sum(w_j)`` with ``w_i = (anl_i / anl_err_i)**2`` (mirroring the
    weights used by :py:func:`find_best_power`).  A dashed line marks the
    equal-weight reference ``1/N`` so points that dominate or contribute
    little are obvious.  This is the visualisation of how each point's
    ``anl_err`` actually feeds into the fit, which the raw log-y error bars
    cannot show.

    ``dpi`` and ``figsize`` control the saved PNG pixel count.  The defaults
    are intentionally compact because this helper usually writes one figure per
    resonator.
    ``tone_indices`` limits the per-tone PNGs that are written; pass an
    ``int`` or iterable such as ``range(0, tone_count, 10)`` for quick-look
    subsets of large arrays.  ``verbose`` controls compact plotting progress
    and can be set to ``False`` to silence it, or ``2`` for one line per tone.
    ``n_jobs`` renders the per-tone PNGs across worker processes (joblib
    convention: ``1`` serial default, ``-1`` all CPUs, ``-2`` all but one);
    it is the largest speedup on many-tone arrays.  ``n_jobs`` is ignored
    (forced serial) when ``show=True``, since open figures only display in
    this process.

    ``output_dir`` is where the per-tone PNGs are written; ``None`` (default)
    falls back to the summary's run directory / CSV location.
    ``bifurcation_anl`` is the ANL value drawn as the ``ANL_bif`` reference
    line and forwarded to :py:func:`find_best_power`.  ``show_anl_errors``
    (default ``True``) toggles the ``anl_err`` error bars.

    Returns
    -------
    result : dict
        ``{'anl': [paths...], 'best_power': best_power}``.
    """
    import matplotlib.pyplot as plt
    from matplotlib import ticker as _mpl_ticker
    verbose = _verbose_level(verbose)
    try:
        anl_errorbar_scale = float(anl_errorbar_scale)
    except (TypeError, ValueError):
        anl_errorbar_scale = 1.0
    if not (np.isfinite(anl_errorbar_scale) and anl_errorbar_scale > 0.0):
        anl_errorbar_scale = 1.0

    source_path = Path(summary).resolve() if isinstance(summary, (str, Path)) else None
    rows = _coerce_summary_rows(summary)
    if best_power is None:
        best_power = find_best_power(
            rows,
            target_anl=target_anl,
            bifurcation_anl=bifurcation_anl,
            use_readback_power=use_readback_power,
            **find_best_power_kwargs,
        )
    elif isinstance(best_power, dict):
        best_power = [best_power]
    else:
        best_power = list(best_power)

    best_by_tone = {int(item["tone_index"]): item for item in best_power}
    rows_by_tone = {}
    for row in rows:
        rows_by_tone.setdefault(int(row["tone_index"]), []).append(row)
    row_tones = sorted(rows_by_tone)
    if tone_indices is None:
        tones = row_tones or sorted(best_by_tone) or _fit_data_tone_indices(summary)
    elif np.isscalar(tone_indices):
        tones = [int(tone_indices)]
    else:
        tones = [int(tone) for tone in tone_indices]

    if output_dir is None:
        if _is_fit_data(summary):
            root = summary.get("run", {}).get("root")
            output_dir = Path(root) / "plots" if root else Path.cwd()
        elif source_path is not None:
            output_dir = source_path.parent / "plots"
        else:
            output_dir = Path.cwd() / "plots"
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    if verbose:
        _progress(
            verbose,
            f"Plotting best-power diagnostics for {len(tones)} tone(s) "
            f"to {output_dir}",
        )

    def _as_float(value):
        """Coerce to float, returning NaN for missing/non-numeric values."""
        try:
            return float(value)
        except (TypeError, ValueError):
            return np.nan

    def _row_power(row):
        """The tone power for a summary row (readback if available, else requested)."""
        if use_readback_power:
            readback = _as_float(row.get("readback_power_dbm", np.nan))
            if np.isfinite(readback):
                return readback
        return _as_float(row.get("power_dbm", np.nan))

    paths = []
    if not tones:
        paths.append(
            _save_empty_best_power_plot(
                plt,
                output_dir,
                None,
                "No fit summary rows are available; best-power selection "
                "needs fit data with finite ANL values.",
                show=show,
                dpi=dpi,
                figsize=figsize,
            )
        )
        _progress(verbose, "  Saved best-power unavailable placeholder")
        return {"anl": paths, "best_power": best_power}

    plot_start = time.time()
    report_every = _progress_report_every(len(tones))
    running_png = [0]

    def _report_best_power_plot_progress(completed, tone_index, saved_delta):
        running_png[0] += saved_delta
        detail = f"png={running_png[0]}"
        if verbose >= 2:
            _progress(
                verbose,
                f"  Tone {tone_index}: saved {saved_delta} ANL plot(s)",
            )
        elif verbose == 1 and _progress_should_report(
            completed, len(tones), report_every
        ):
            _print_progress_line(
                _format_plot_progress(
                    "best-power tones",
                    completed,
                    len(tones),
                    time.time() - plot_start,
                    final=completed == len(tones),
                    detail=detail,
                ),
                final=completed == len(tones),
            )

    common = dict(
        use_readback_power=use_readback_power,
        anl_errorbar_scale=anl_errorbar_scale,
        output_dir=str(output_dir),
        dpi=dpi,
        figsize=figsize,
        show=show,
        show_anl_errors=show_anl_errors,
        show_weights=show_weights,
        target_anl=target_anl,
        bifurcation_anl=bifurcation_anl,
    )
    tasks = [
        (tone_index, rows_by_tone.get(tone_index, []),
         best_by_tone.get(tone_index))
        for tone_index in tones
    ]
    # show=True keeps figures open for interactive display, which only works
    # in this process, so it forces serial rendering.
    workers = 1 if show else _resolve_n_jobs(n_jobs, len(tasks))
    results_by_tone = {}
    if workers == 1:
        for completed, (tone_index, tone_rows, result) in enumerate(
            tasks, start=1
        ):
            tone_paths = _plot_best_power_one_tone(
                tone_index, tone_rows, result, **common
            )
            results_by_tone[tone_index] = tone_paths
            _report_best_power_plot_progress(
                completed, tone_index, len(tone_paths)
            )
    else:
        _progress(
            verbose,
            f"  Rendering best-power tones across {workers} processes",
        )
        with ProcessPoolExecutor(
            max_workers=workers, initializer=_force_agg_worker_init
        ) as pool:
            future_to_tone = {
                pool.submit(
                    _plot_best_power_one_tone,
                    tone_index,
                    tone_rows,
                    result,
                    **common,
                ): tone_index
                for tone_index, tone_rows, result in tasks
            }
            for completed, future in enumerate(
                as_completed(future_to_tone), start=1
            ):
                tone_index = future_to_tone[future]
                results_by_tone[tone_index] = future.result()
                _report_best_power_plot_progress(
                    completed, tone_index, len(results_by_tone[tone_index])
                )

    # Merge in tone order so paths are stable regardless of completion order.
    for tone_index in tones:
        paths.extend(results_by_tone[tone_index])

    if verbose >= 2 and tones:
        _progress(
            verbose,
            _format_plot_progress(
                "best-power tones",
                len(tones),
                len(tones),
                time.time() - plot_start,
                final=True,
                detail=f"png={len(paths)}",
            ),
        )

    return {"anl": paths, "best_power": best_power}
