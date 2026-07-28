"""
Match resonance lists between two sweeps.

Give it two lists of resonances and it works out which entry in one is which
physical device in the other, even when the lists have different lengths, the
frequencies have all shifted, and some devices were found in one sweep but not
the other.

The short version::

    from souk_readout_tools import resonance_matching as rm

    idx = rm.match_index('cold.resonances', 'warm.resonances')
    # idx[i] is the entry in the second list matching entry i of the first,
    # or -1 if there isn't one.

The longer version keeps the diagnostics::

    m = rm.match_resonances('cold.resonances', 'warm.resonances')
    m.summary()
    qi_cold, qi_warm = m.compare('Qi')

Both accept the same inputs: bare frequency lists, dicts, ``.resonances``
files, KIDLAB toneslists, fit-summary CSVs, power-sweep run directories,
parsed sweep dicts (resonances are found in them automatically), or the
results of :py:func:`~souk_readout_tools.peak_finder.find_mkid_resonances`.

How it works, in three stages:

1. **Transform.**  Resonators rarely stay put.  A global model of the shift is
   estimated first -- usually a constant fractional shift ``df/f``, which is
   what ageing, a loading change or a thermal cycle produce -- so that the
   matching sees only the per-device scatter about it.
2. **Cost.**  Every plausible pairing within ``tolerance`` is scored on how far
   apart the two entries are once that transform is applied.
3. **Assignment.**  The scores are solved globally, so a device is never
   claimed by two neighbours at once.  Entries with no counterpart inside the
   tolerance are left unmatched rather than forced onto something.

Defaults are meant to work without being told what happened to the array.  The
controls exist to fix the leftovers and to encode physics you already know; see
``doc/resonance_matching.md``.
"""

import csv
import json
import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# 1.4826 = 1 / Phi^{-1}(0.75); scales a median absolute deviation to the
# equivalent Gaussian sigma, so "5 sigma" means the same thing on the
# heavy-tailed residual distributions these lists actually have.
_MAD_TO_SIGMA = 1.4826

# Frequency scatter between two sweeps has a tight core plus a sparse tail of
# devices that moved a long way (or that have no counterpart at all, and whose
# nearest neighbour is therefore a different device entirely).  Tolerances are
# always set from the core, never from the tail.
DEFAULT_TOLERANCE_SIGMA = 5.0
# A tolerance wider than about half the spacing between resonances cannot
# distinguish "it moved" from "that is its neighbour", whatever the algorithm.
DEFAULT_SPACING_FRACTION = 0.4
# ...and one narrower than a linewidth is asking for more frequency precision
# than a resonance has. Also stops the tolerance collapsing to zero when the
# two lists are near-identical.
DEFAULT_LINEWIDTH_FLOOR = 1.0
# Matched pairs beyond this many sigma of the global model are reported as
# outliers: still matched, but worth a look.
DEFAULT_OUTLIER_SIGMA = 5.0
# A match whose runner-up costs almost as much is a coin flip. Ratio of best
# cost to next-best; 0 is unambiguous, 1 is a tie.
DEFAULT_AMBIGUITY = 0.5

TRANSFORM_MODELS = ('none', 'constant', 'proportional', 'affine', 'linear_frac')


# --- input handling ---------------------------------------------------------

# Column names that mean "this is the resonance frequency", and the canonical
# names everything else is mapped onto so that a .resonances file and a fit
# summary can be compared without the caller renaming anything.  Keys are
# compared lower-cased; anything not listed keeps its original name.
_FREQUENCY_ALIASES = {
    'frequency', 'frequency(hz)', 'frequency_hz', 'freq', 'freqs', 'f',
    'fr', 'fr_hz', 'center', 'center_hz', 'resonance_frequency',
}
_PARAM_ALIASES = {
    'linewidth(hz)': 'fwhm', 'linewidth_hz': 'fwhm', 'linewidth': 'fwhm',
    'fwhm': 'fwhm',
    'qfactor(qr)': 'Ql', 'qfactor': 'Ql', 'qr': 'Ql', 'ql': 'Ql',
    'qcoupling': 'Qc', 'qc': 'Qc',
    'qinternal': 'Qi', 'qi': 'Qi',
    'dipdepth(db)': 'dip_depth', 'dip_depth_db': 'dip_depth',
    'dip_depth': 'dip_depth', 'dipdepth': 'dip_depth',
    'phi': 'phi', 'anl': 'anl', 'skew': 'skew',
    'power_dbm': 'power_dbm',
}


@dataclass
class ResonanceList:
    """One sweep's worth of resonances, in the form the matcher works with.

    Attributes
    ----------
    frequency : numpy.ndarray
        Resonance frequencies in Hz, sorted ascending.
    params : dict
        Per-resonance quantities (``Qi``, ``Qc``, ``fwhm``, ...), each an
        array in the same order as ``frequency``.  Whatever the input
        happened to carry; may be empty.
    labels : numpy.ndarray
        Per-resonance names, for reporting.
    order : numpy.ndarray
        ``order[k]`` is the row of the original input that became sorted
        entry ``k``.  Results are reported in the caller's input order, not
        this internal sorted order.
    source : str
        Where it came from, for plot titles and summaries.
    coverage : tuple
        ``(f_min, f_max)`` actually searched, in Hz.  Used to tell "this
        device was not found" apart from "this device was never looked for".
    sweep : dict or None
        The parsed sweep this came from, when there was one, so plots can
        show the underlying S21.
    """

    frequency: np.ndarray
    params: dict = field(default_factory=dict)
    labels: np.ndarray = None
    order: np.ndarray = None
    source: str = ''
    coverage: tuple = None
    sweep: dict = None

    def __post_init__(self):
        self.frequency = np.asarray(self.frequency, dtype=float)
        if self.labels is None:
            self.labels = np.array(['K%03d' % i for i in range(len(self.frequency))])
        if self.order is None:
            self.order = np.arange(len(self.frequency))
        if self.coverage is None and len(self.frequency):
            self.coverage = (float(self.frequency.min()), float(self.frequency.max()))

    def __len__(self):
        return len(self.frequency)

    @property
    def spacing(self):
        """Median gap between neighbouring resonances (Hz)."""
        if len(self.frequency) < 2:
            return np.nan
        return float(np.median(np.diff(self.frequency)))

    @property
    def linewidth(self):
        """Median linewidth (Hz), or NaN when the input carried none."""
        for key in ('fwhm', 'empirical_linewidth_hz'):
            if key in self.params:
                values = np.asarray(self.params[key], dtype=float)
                values = values[np.isfinite(values) & (values > 0)]
                if values.size:
                    return float(np.median(values))
        for key in ('Ql', 'Qi'):
            if key in self.params:
                q = np.asarray(self.params[key], dtype=float)
                ok = np.isfinite(q) & (q > 0)
                if ok.any():
                    return float(np.median(self.frequency[ok] / q[ok]))
        return np.nan

    def sorted_by_frequency(self):
        """Return a copy sorted ascending in frequency, remembering the order."""
        k = np.argsort(self.frequency, kind='stable')
        return ResonanceList(
            frequency=self.frequency[k],
            params={name: np.asarray(v)[k] for name, v in self.params.items()},
            labels=np.asarray(self.labels)[k],
            order=np.asarray(self.order)[k],
            source=self.source,
            coverage=self.coverage,
            sweep=self.sweep,
        )


def _canonical_params(columns):
    """Map raw column names onto canonical parameter names.

    Unrecognised names are kept as they are, and a canonical name is never
    overwritten once claimed -- so a fit summary carrying both ``Qc`` and
    ``empirical_Qc`` keeps them separate.
    """
    out = {}
    for raw, values in columns.items():
        key = _PARAM_ALIASES.get(str(raw).strip().lower(), str(raw))
        if key in out:
            key = str(raw)
        out[key] = values
    return out


def _frequency_scale(frequency, units):
    """Return the factor converting ``frequency`` to Hz."""
    if units in (None, 'auto'):
        finite = np.asarray(frequency, dtype=float)
        finite = finite[np.isfinite(finite) & (finite > 0)]
        if not finite.size:
            return 1.0, 'hz'
        median = float(np.median(finite))
        # Readout bands run from roughly 0.1 to 10 GHz, so the three unit
        # choices are orders of magnitude apart and cannot be confused:
        # ~1.2 in GHz, ~1200 in MHz, ~1.2e9 in Hz.
        if median < 100:
            return 1e9, 'ghz'
        if median < 1e6:
            return 1e6, 'mhz'
        return 1.0, 'hz'
    units = str(units).lower()
    return {'hz': 1.0, 'mhz': 1e6, 'ghz': 1e9}[units], units


def _table_to_list(columns, source, units='auto', coverage=None, sweep=None):
    """Build a :class:`ResonanceList` from a mapping of column name -> array."""
    columns = dict(columns)
    freq_key = None
    for raw in columns:
        if str(raw).strip().lower() in _FREQUENCY_ALIASES:
            freq_key = raw
            break
    if freq_key is None:
        raise ValueError(
            f"{source}: no frequency column found (looked for "
            f"{sorted(_FREQUENCY_ALIASES)}); pass a dict with a "
            f"'frequency' key, or a bare array of frequencies."
        )
    frequency = np.asarray(columns.pop(freq_key), dtype=float)
    ragged = {k: len(np.atleast_1d(v)) for k, v in columns.items()
              if len(np.atleast_1d(v)) != len(frequency)}
    if ragged:
        raise ValueError(
            f"{source}: columns {ragged} do not have one entry per resonance "
            f"({len(frequency)}).")
    scale, units_used = _frequency_scale(frequency, units)
    if scale != 1.0:
        print(f"{source}: frequencies read as {units_used.upper()}"
              f" (median {np.median(frequency):.4g}); pass units= to override.")
    frequency = frequency * scale

    labels = None
    for raw in list(columns):
        if str(raw).strip().lower() in ('name', 'names', 'id', '#id', 'label'):
            labels = np.asarray([str(v) for v in columns.pop(raw)])
            break

    params = _canonical_params(columns)
    # Drop entries with no usable frequency; keep everything else, including
    # failed fits, so the caller can see them in the table.
    keep = np.isfinite(frequency)
    if not keep.all():
        frequency = frequency[keep]
        params = {k: np.asarray(v)[keep] for k, v in params.items()}
        if labels is not None:
            labels = labels[keep]
    order = np.arange(len(frequency))
    out = ResonanceList(frequency=frequency, params=params, labels=labels,
                        order=order, source=source, coverage=coverage,
                        sweep=sweep)
    return out.sorted_by_frequency()


def _structured_to_columns(array):
    """Split a structured numpy array into a plain {name: array} dict."""
    return {name: array[name] for name in array.dtype.names}


def _from_resonances_file(path):
    """Read a ``.resonances`` table written by ``peak_finder.write_resonances``."""
    data = np.genfromtxt(path, names=True, delimiter='\t', dtype=None,
                         encoding=None, deletechars='')
    data = np.atleast_1d(data)
    return _table_to_list(_structured_to_columns(data), os.path.basename(path))


def _from_toneslist(path):
    """Read a KIDLAB toneslist (``Name  Freq  Offset att  All  None``)."""
    data = np.genfromtxt(path, names=True, delimiter='\t', dtype=None,
                         encoding=None)
    data = np.atleast_1d(data)
    return _table_to_list(_structured_to_columns(data), os.path.basename(path))


def _from_csv(path):
    """Read a CSV: a fit summary, a simple ``#ID,Frequency(Hz)`` export, ..."""
    data = np.genfromtxt(path, names=True, delimiter=',', dtype=None,
                         encoding=None, deletechars='')
    data = np.atleast_1d(data)
    columns = _structured_to_columns(data)
    # A power-sweep fit summary has one row per (power step, tone). Reduce it
    # to one row per tone before matching, else every device appears N times.
    if 'sweep_index' in columns and 'tone_index' in columns:
        columns = _reduce_power_sweep_rows(columns, os.path.basename(path))
    if 'success' in columns:
        ok = np.asarray(columns['success'], dtype=float) > 0
        if ok.any() and not ok.all():
            print(f"{os.path.basename(path)}: dropping {int((~ok).sum())} "
                  f"failed fits (success=0).")
            columns = {k: np.asarray(v)[ok] for k, v in columns.items()}
    return _table_to_list(columns, os.path.basename(path))


def _reduce_power_sweep_rows(columns, source):
    """Keep one row per tone from a multi-power fit summary.

    Picks each tone's middle successful power step: the lowest steps are
    noise-limited and the highest are bifurcated, so neither end is a fair
    description of the device.  Prefer a run directory as input, which uses
    the run's own chosen best power instead of this fallback.
    """
    tone = np.asarray(columns['tone_index'], dtype=int)
    step = np.asarray(columns['sweep_index'], dtype=int)
    ok = np.ones(len(tone), dtype=bool)
    if 'success' in columns:
        ok = np.asarray(columns['success'], dtype=float) > 0
    keep = []
    for t in np.unique(tone):
        rows = np.where((tone == t) & ok)[0]
        if not rows.size:
            rows = np.where(tone == t)[0]
        if rows.size:
            keep.append(rows[np.argsort(step[rows])[rows.size // 2]])
    print(f"{source}: {len(np.unique(tone))} tones over "
          f"{len(np.unique(step))} power steps; using each tone's middle "
          f"successful step. Pass the run directory to use its best power.")
    keep = np.array(sorted(keep), dtype=int)
    return {k: np.asarray(v)[keep] for k, v in columns.items()}


def _from_best_power(path):
    """Read a power-sweep ``best_power.json``: fitted params at the tuned power."""
    with open(path) as handle:
        data = json.load(handle)
    tones = data.get('tones') or []
    names = sorted({key for tone in tones
                    for key in (tone.get('chosen_params') or {})})
    columns = {'tone_index': [], 'chosen_power_dbm': []}
    columns.update({key: [] for key in names})
    for tone in tones:
        chosen = tone.get('chosen_params') or {}
        columns['tone_index'].append(tone.get('tone_index', -1))
        columns['chosen_power_dbm'].append(tone.get('chosen_power_dbm', np.nan))
        for key in names:
            columns[key].append(chosen.get(key, np.nan))
    columns = {k: np.asarray(v) for k, v in columns.items()}
    source = os.path.basename(os.path.dirname(os.path.dirname(path))) or path
    return _table_to_list(columns, source)


def _from_run_directory(path):
    """Read a power-sweep run: best-power fits if analysed, else the summary."""
    root = Path(path)
    if root.name == 'measurement.json':
        root = root.parent
    best = root / 'analysis' / 'best_power.json'
    if best.exists():
        return _from_best_power(str(best))
    summary = root / 'analysis' / 'fit_summary.csv'
    if summary.exists():
        return _from_csv(str(summary))
    raise ValueError(
        f"{root}: no analysis/best_power.json or analysis/fit_summary.csv. "
        f"Run power_sweep.analyse_power_sweep on it first, or pass a "
        f".resonances file instead."
    )


def _from_sweep(sweep, source='sweep', **finder_kwargs):
    """Find resonances in a parsed sweep dict and return them as a list."""
    from . import peak_finder

    f = np.atleast_2d(np.asarray(sweep['sweep_f'], dtype=float))
    z = (np.atleast_2d(np.asarray(sweep['sweep_i'], dtype=float))
         + 1j * np.atleast_2d(np.asarray(sweep['sweep_q'], dtype=float)))
    # parse_sweep_data gives (points, tones); parse_wideband_sweep gives a
    # single concatenated row.
    if sweep.get('wideband_sweep', False) or f.shape[0] == 1:
        traces = [(f[0], z[0])]
    else:
        traces = [(f[:, j], z[:, j]) for j in range(f.shape[1])]

    found = []
    for trace_f, trace_z in traces:
        ok = np.isfinite(trace_f) & np.isfinite(trace_z)
        if ok.sum() < 8:
            continue
        found.extend(peak_finder.find_mkid_resonances(
            trace_f[ok], trace_z[ok], **finder_kwargs))
    print(f"{source}: found {len(found)} resonances in the sweep.")
    coverage = (float(np.nanmin(f)), float(np.nanmax(f)))
    return _from_resonance_results(found, source, coverage=coverage, sweep=sweep)


def _from_resonance_results(results, source, coverage=None, sweep=None):
    """Build a list from ``peak_finder.ResonanceResult`` objects."""
    columns = {
        'frequency': [r.frequency for r in results],
        'fwhm': [r.fwhm for r in results],
        'Ql': [r.q_factor for r in results],
        'Qc': [r.qc for r in results],
        'Qi': [r.qi for r in results],
        'dip_depth': [r.dip_depth for r in results],
        'skew': [r.skew for r in results],
    }
    columns = {k: np.array([np.nan if v is None else float(v) for v in vals])
               for k, vals in columns.items()}
    return _table_to_list(columns, source, units='hz', coverage=coverage,
                          sweep=sweep)


def as_resonance_list(obj, units='auto', name=None, coverage=None):
    """Coerce anything resonance-list-shaped into a :class:`ResonanceList`.

    Accepts, in order of how it is recognised:

    - an existing :class:`ResonanceList` (returned unchanged);
    - a path to a ``.resonances`` file, a KIDLAB ``.txt`` toneslist, a CSV
      (fit summary or ``#ID,Frequency(Hz)`` export), a ``.npy``/``.npz``, a
      power-sweep run directory, or its ``measurement.json``;
    - a parsed sweep dict (``sweep_f``/``sweep_i``/``sweep_q``) -- resonances
      are found in it with
      :py:func:`~souk_readout_tools.peak_finder.find_mkid_resonances`;
    - a dict of columns with a frequency-like key;
    - a structured numpy array (a fit summary, a resonator catalogue, ...);
    - a ``ResonanceSearchResult`` or list of ``ResonanceResult``;
    - a plain list or 1-D array of frequencies.

    Parameters
    ----------
    obj : object
        Any of the above.
    units : {'auto', 'hz', 'mhz', 'ghz'}, optional
        Units of the input frequencies.  ``'auto'`` guesses from their
        magnitude and prints what it assumed.
    name : str or None, optional
        Overrides the recorded source name, used in summaries and plots.
    coverage : tuple or None, optional
        ``(f_min, f_max)`` searched, in Hz.  Defaults to the span of the
        list itself; give it explicitly when the list is a subset of a wider
        sweep, so devices outside the other list's reach are reported as
        out-of-range rather than missing.

    Returns
    -------
    ResonanceList
    """
    if isinstance(obj, ResonanceList):
        return obj

    if isinstance(obj, (str, os.PathLike)):
        path = str(obj)
        if os.path.isdir(path) or os.path.basename(path) == 'measurement.json':
            out = _from_run_directory(path)
        else:
            ext = os.path.splitext(path)[1].lower()
            if ext == '.resonances':
                out = _from_resonances_file(path)
            elif ext == '.txt':
                out = _from_toneslist(path)
            elif ext == '.csv':
                out = _from_csv(path)
            elif ext in ('.npy', '.npz'):
                loaded = np.load(path, allow_pickle=True)
                if isinstance(loaded, np.ndarray) and loaded.dtype == object:
                    loaded = loaded.item()
                elif hasattr(loaded, 'files'):
                    loaded = {k: loaded[k] for k in loaded.files}
                out = as_resonance_list(loaded, units=units,
                                        name=name or os.path.basename(path))
            elif ext == '.json':
                out = _from_best_power(path)
            else:
                raise ValueError(f"{path}: unrecognised file type {ext!r}.")
        if name:
            out.source = name
        if coverage is not None:
            out.coverage = coverage
        return out

    if isinstance(obj, dict):
        if 'sweep_f' in obj and 'sweep_i' in obj:
            return _from_sweep(obj, source=name or 'sweep')
        if 'run' in obj and 'best_power' in obj:      # load_analysis result
            best = obj.get('best_power')
            if best is None:
                raise ValueError(
                    "analysis result has no best_power; run "
                    "power_sweep.analyse_power_sweep or pass the run directory.")
            tmp = {'tones': best.get('tones', best)}
            columns = {}
            for tone in tmp['tones']:
                for key, value in (tone.get('chosen_params') or {}).items():
                    columns.setdefault(key, []).append(value)
            columns = {k: np.asarray(v, dtype=float) for k, v in columns.items()}
            return _table_to_list(columns, name or 'power sweep', units='hz',
                                  coverage=coverage)
        return _table_to_list(obj, name or 'dict', units=units,
                              coverage=coverage)

    if hasattr(obj, 'all_resonances'):                 # ResonanceSearchResult
        return _from_resonance_results(list(obj.all_resonances),
                                       name or 'resonance search',
                                       coverage=coverage)

    array = np.asarray(obj)
    if array.dtype.names:
        return _table_to_list(_structured_to_columns(array),
                              name or 'structured array', units=units,
                              coverage=coverage)
    if array.ndim == 1 and array.size and not np.issubdtype(array.dtype, np.number):
        # A list of ResonanceResult objects.
        if hasattr(array[0], 'frequency'):
            return _from_resonance_results(list(array),
                                           name or 'resonance results',
                                           coverage=coverage)
    if array.ndim != 1:
        raise ValueError(
            f"expected a 1-D array of frequencies, got shape {array.shape}.")
    return _table_to_list({'frequency': array.astype(float)},
                          name or 'frequency list', units=units,
                          coverage=coverage)


# --- the global frequency transform -----------------------------------------

def _robust_sigma(values):
    """MAD-based standard deviation estimate, immune to the outlier tail."""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size < 2:
        return np.nan
    return float(_MAD_TO_SIGMA * np.median(np.abs(values - np.median(values))))


def _scatter_about_zero(values):
    """How far residuals sit from zero, robustly.

    Not the same as :py:func:`_robust_sigma`, which measures spread about
    the median and so is blind to a systematic offset.  Models are compared
    on this, otherwise "no shift at all" scores just as well as the correct
    shift: both leave the same scatter, one of them centred a long way from
    zero.  For a model that does fit, the two agree.
    """
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size < 2:
        return np.nan
    return float(_MAD_TO_SIGMA * np.median(np.abs(values)))


def _nearest_index(sorted_values, targets):
    """Index of the nearest entry of ``sorted_values`` for each target."""
    if len(sorted_values) == 0:
        return np.zeros(len(targets), dtype=int)
    j = np.searchsorted(sorted_values, targets)
    j = np.clip(j, 1, max(len(sorted_values) - 1, 1))
    left = np.abs(targets - sorted_values[j - 1])
    right = np.abs(sorted_values[np.minimum(j, len(sorted_values) - 1)] - targets)
    return np.where(left <= right, j - 1, np.minimum(j, len(sorted_values) - 1))


def apply_transform(transform, frequency):
    """Map frequencies from list A onto list B's frequency axis."""
    f = np.asarray(frequency, dtype=float)
    model = transform['model']
    p = transform['params']
    if model == 'none':
        return f
    if model == 'constant':
        return f + p[0]
    if model == 'proportional':
        return f * (1.0 + p[0])
    if model == 'affine':
        return p[0] * f + p[1]
    if model == 'linear_frac':
        return f * (1.0 + p[0] + p[1] * f)
    raise ValueError(f"unknown transform model {model!r}")


def _fit_transform(fa, fb, model):
    """Fit one shift model to a set of provisionally matched pairs."""
    if model == 'none' or len(fa) < 2:
        return {'model': 'none', 'params': ()}
    if model == 'constant':
        return {'model': 'constant', 'params': (float(np.median(fb - fa)),)}
    if model == 'proportional':
        return {'model': 'proportional',
                'params': (float(np.median((fb - fa) / fa)),)}
    if model in ('affine', 'linear_frac'):
        if model == 'affine':
            x, y = fa, fb
        else:
            x, y = fa, (fb - fa) / fa
        # Two rounds of clipped least squares: enough to shrug off the tail
        # without needing a full robust regression.
        keep = np.ones(len(x), dtype=bool)
        coeffs = np.polyfit(x[keep], y[keep], 1)
        for _ in range(2):
            resid = y - np.polyval(coeffs, x)
            sigma = _robust_sigma(resid)
            if not np.isfinite(sigma) or sigma <= 0:
                break
            keep = np.abs(resid - np.median(resid)) < 3 * sigma
            if keep.sum() < 3:
                break
            coeffs = np.polyfit(x[keep], y[keep], 1)
        if model == 'linear_frac':
            # polyfit returns (slope, intercept); apply_transform wants
            # (intercept, slope) so that df/f = p0 + p1*f.
            return {'model': model, 'params': (float(coeffs[1]), float(coeffs[0]))}
        return {'model': model, 'params': (float(coeffs[0]), float(coeffs[1]))}
    raise ValueError(f"unknown transform model {model!r}")


def _coarse_fractional_shift(fa, fb, window, max_shift=0.01):
    """Find the fractional shift that lines up the most resonances.

    A brute scan, which is the honest way to bootstrap this: there is no
    match yet to fit to, and the shift can be far larger than the spacing
    between resonances.  Cheap -- a few hundred searchsorted calls.
    """
    if len(fa) == 0 or len(fb) == 0:
        return 0.0
    median_f = float(np.median(fa))
    spacing = float(np.median(np.diff(fa))) if len(fa) > 1 else window
    step = max(0.05 * spacing / median_f, 1e-7)
    grid = np.arange(-max_shift, max_shift + step, step)
    best, best_count = 0.0, -1
    for s in grid:
        shifted = fa * (1.0 + s)
        j = _nearest_index(fb, shifted)
        count = int(np.sum(np.abs(fb[j] - shifted) < window))
        if count > best_count:
            best, best_count = float(s), count
    return best


def _match_within(fa_t, fb, window):
    """Provisional nearest-neighbour pairing, used while fitting the model."""
    if len(fa_t) == 0 or len(fb) == 0:
        return np.zeros(0, dtype=int), np.zeros(0, dtype=int)
    j = _nearest_index(fb, fa_t)
    ok = np.abs(fb[j] - fa_t) < window
    return np.where(ok)[0], j[ok]


def estimate_transform(a, b, model='auto', window=None, iterations=3,
                       verbose=True):
    """Estimate the global frequency shift between two resonance lists.

    Parameters
    ----------
    a, b : ResonanceList
        Frequency-sorted lists.
    model : str or float, optional
        One of ``'none'``, ``'constant'``, ``'proportional'``, ``'affine'``,
        ``'linear_frac'``, ``'auto'``, or a number taken as a fixed
        fractional shift ``df/f``.  ``'auto'`` (default) fits them all and
        keeps the simplest that is not meaningfully beaten.
    window : float or None, optional
        Pairing window in Hz used while fitting.  Defaults to a quarter of
        the median resonance spacing.
    iterations : int, optional
        Match/refit cycles per model (default 3).
    verbose : bool, optional
        Print the model comparison.

    Returns
    -------
    dict
        ``{'model', 'params', 'sigma', 'n_matched', 'trace'}``.  ``sigma`` is
        the robust fractional scatter left over after the fit.
    """
    fa, fb = a.frequency, b.frequency
    if window is None:
        spacing = a.spacing if np.isfinite(a.spacing) else np.nan
        window = 0.25 * spacing if np.isfinite(spacing) else np.inf

    if isinstance(model, (int, float)) and not isinstance(model, bool):
        transform = {'model': 'proportional', 'params': (float(model),)}
        ia, ib = _match_within(apply_transform(transform, fa), fb, window)
        resid = ((fb[ib] - apply_transform(transform, fa)[ia])
                 / np.maximum(fa[ia], 1.0)) if len(ia) else np.zeros(0)
        transform.update(sigma=_scatter_about_zero(resid), n_matched=len(ia),
                         trace=['df/f fixed by caller'])
        return transform

    candidates = TRANSFORM_MODELS if model == 'auto' else (model,)
    s0 = 0.0
    if any(m in ('proportional', 'affine', 'linear_frac') for m in candidates):
        s0 = _coarse_fractional_shift(fa, fb, window)

    results, trace = [], []
    for name in candidates:
        start = {'model': 'proportional', 'params': (s0,)} \
            if name in ('proportional', 'affine', 'linear_frac') \
            else {'model': 'none', 'params': ()}
        transform = start
        for _ in range(max(1, iterations)):
            ia, ib = _match_within(apply_transform(transform, fa), fb, window)
            if len(ia) < 3:
                break
            transform = _fit_transform(fa[ia], fb[ib], name)
        fa_t = apply_transform(transform, fa)
        ia, ib = _match_within(fa_t, fb, window)
        resid = (fb[ib] - fa_t[ia]) / np.maximum(fa[ia], 1.0) if len(ia) else np.zeros(0)
        sigma = _scatter_about_zero(resid)
        results.append({'model': transform['model'], 'params': transform['params'],
                        'sigma': sigma, 'n_matched': len(ia)})
        trace.append(f"    {name:13s} matched {len(ia):4d}  sigma {sigma:.2e}")

    # Prefer the simplest model; step up only for a real improvement, so a
    # trim that genuinely has no global shift is not fitted a spurious one.
    best = results[0]
    n_total = max(len(fa), 1)
    for candidate in results[1:]:
        better_sigma = (np.isfinite(candidate['sigma']) and np.isfinite(best['sigma'])
                        and candidate['sigma'] < 0.8 * best['sigma'])
        better_count = candidate['n_matched'] > best['n_matched'] + max(2, 0.01 * n_total)
        if better_sigma or better_count:
            best = candidate
    if verbose and len(results) > 1:
        print("  transform model selection:")
        for line in trace:
            print(line)
        print(f"    -> {best['model']}")
    best['trace'] = trace
    return best


def describe_transform(transform, frequency=None):
    """One-line human description of a fitted transform."""
    model, p = transform['model'], transform['params']
    f = float(np.median(frequency)) if frequency is not None and len(frequency) else np.nan
    if model == 'none':
        return 'no global shift'
    if model == 'constant':
        return f"df = {p[0] / 1e3:+.2f} kHz"
    if model == 'proportional':
        extra = f" ({p[0] * f / 1e3:+.2f} kHz at {f / 1e6:.0f} MHz)" if np.isfinite(f) else ''
        return f"df/f = {p[0]:+.3e}{extra}"
    if model == 'affine':
        return f"f_b = {p[0]:.9f} * f_a {p[1] / 1e3:+.2f} kHz"
    if model == 'linear_frac':
        return f"df/f = {p[0]:+.3e} {p[1]:+.3e} * f"
    return model


# --- tolerance --------------------------------------------------------------

def choose_tolerance(a, b, transform, sigma, tolerance='auto',
                     sigma_multiple=DEFAULT_TOLERANCE_SIGMA):
    """Decide how far apart two entries may be and still be the same device.

    The scale that matters is physical, not a fixed number of Hz, so the
    default is derived from the data: a multiple of the measured scatter
    about the global shift, floored at a linewidth (you cannot localise a
    resonance better than that) and capped at a fraction of the spacing
    between resonances (beyond which "it moved" and "that is its neighbour"
    are indistinguishable).

    Returns
    -------
    tuple
        ``(tolerance_hz, explanation)``.
    """
    median_f = float(np.median(a.frequency)) if len(a) else np.nan
    if tolerance not in (None, 'auto'):
        return float(tolerance), 'set by caller'

    def _best(values, pick):
        values = [v for v in values if np.isfinite(v)]
        return pick(values) if values else np.nan

    from_sigma = sigma_multiple * sigma * median_f if np.isfinite(sigma) else np.nan
    spacing = _best([a.spacing, b.spacing], min)
    cap = DEFAULT_SPACING_FRACTION * spacing if np.isfinite(spacing) else np.inf
    # No linewidths in the input (a bare frequency list) means no physical
    # floor to apply -- the scatter term is then the only thing setting it.
    linewidth = _best([a.linewidth, b.linewidth], max)
    floor = DEFAULT_LINEWIDTH_FLOOR * linewidth if np.isfinite(linewidth) else 0.0

    value = from_sigma if np.isfinite(from_sigma) else floor
    reason = f"{sigma_multiple:.0f}*sigma"
    if value < floor:
        value, reason = floor, 'linewidth floor'
    if value > cap:
        value, reason = cap, 'spacing cap'
    if not np.isfinite(value) or value <= 0:
        value, reason = (spacing * 0.1 if np.isfinite(spacing) else 1e5), 'fallback'
    detail = (f"{value / 1e3:.1f} kHz ({reason}; "
              f"{sigma_multiple:.0f}*sigma = "
              f"{from_sigma / 1e3:.1f} kHz, "
              f"linewidth floor = {floor / 1e3:.1f} kHz, "
              f"spacing cap = {cap / 1e3:.0f} kHz)")
    return float(value), detail


# --- assignment -------------------------------------------------------------

# Cost given to a pairing that is outside the tolerance: large enough that the
# solver never prefers it, finite so the assignment problem stays feasible.
_FORBIDDEN = 1e6


def _cost_matrix(fa_t, fb, tolerance):
    """Normalised |df| cost for every pairing, _FORBIDDEN beyond tolerance."""
    cost = np.abs(fb[None, :] - fa_t[:, None]) / tolerance
    cost[cost > 1.0] = _FORBIDDEN
    return cost


def _assign_free(cost):
    """Globally optimal one-to-one assignment (allows devices to reorder)."""
    from scipy.optimize import linear_sum_assignment

    rows, cols = linear_sum_assignment(cost)
    keep = cost[rows, cols] < 1.0
    return rows[keep], cols[keep]


def _assign_monotonic(cost, gap=1.0):
    """Best order-preserving alignment, with gaps for unmatched entries.

    Ordinary sequence alignment: useful when you know the devices cannot
    have reordered, and the tie-breaker when two neighbours are equally
    plausible partners.
    """
    n, m = cost.shape
    dp = np.zeros((n + 1, m + 1))
    dp[0, :] = np.arange(m + 1) * gap
    dp[:, 0] = np.arange(n + 1) * gap
    back = np.zeros((n + 1, m + 1), dtype=np.int8)
    back[0, 1:], back[1:, 0] = 2, 1
    for i in range(1, n + 1):
        row, prev = dp[i], dp[i - 1]
        for j in range(1, m + 1):
            diag = prev[j - 1] + cost[i - 1, j - 1]
            up = prev[j] + gap
            left = row[j - 1] + gap
            if diag <= up and diag <= left:
                row[j], back[i, j] = diag, 0
            elif up <= left:
                row[j], back[i, j] = up, 1
            else:
                row[j], back[i, j] = left, 2
    rows, cols = [], []
    i, j = n, m
    while i > 0 or j > 0:
        move = back[i, j]
        if move == 0:
            if cost[i - 1, j - 1] < 1.0:
                rows.append(i - 1)
                cols.append(j - 1)
            i, j = i - 1, j - 1
        elif move == 1:
            i -= 1
        else:
            j -= 1
    return np.array(rows[::-1], dtype=int), np.array(cols[::-1], dtype=int)


def _assign_nearest(cost):
    """Mutual nearest neighbours: the simplest thing that can work."""
    rows, cols = [], []
    best_for_row = np.argmin(cost, axis=1)
    best_for_col = np.argmin(cost, axis=0)
    for i, j in enumerate(best_for_row):
        if cost[i, j] < 1.0 and best_for_col[j] == i:
            rows.append(i)
            cols.append(j)
    return np.array(rows, dtype=int), np.array(cols, dtype=int)


_ASSIGNERS = {'free': _assign_free, 'monotonic': _assign_monotonic,
              'nearest': _assign_nearest}


def _ambiguity(cost, rows, cols):
    """How close the runner-up was, per matched pair. 0 clean, 1 a tie."""
    out = np.zeros(len(rows))
    for k, (i, j) in enumerate(zip(rows, cols)):
        best = cost[i, j]
        row = np.delete(cost[i], j)
        col = np.delete(cost[:, j], i)
        runner = min(row.min() if row.size else np.inf,
                     col.min() if col.size else np.inf)
        out[k] = 0.0 if not np.isfinite(runner) or runner >= _FORBIDDEN \
            else float(np.clip(best / runner, 0.0, 1.0))
    return out


def _longest_increasing(values):
    """Mask of the longest increasing subsequence -- the un-crossed pairs."""
    n = len(values)
    if n == 0:
        return np.zeros(0, dtype=bool)
    tails, tail_index, prev = [], [], [-1] * n
    for i, v in enumerate(values):
        pos = np.searchsorted(tails, v)
        if pos == len(tails):
            tails.append(v)
            tail_index.append(i)
        else:
            tails[pos] = v
            tail_index[pos] = i
        prev[i] = tail_index[pos - 1] if pos > 0 else -1
    mask = np.zeros(n, dtype=bool)
    k = tail_index[-1]
    while k != -1:
        mask[k] = True
        k = prev[k]
    return mask


# --- fingerprints -----------------------------------------------------------

# Resonator properties that can help say which device is which, and how to
# compare two values of each.  'log' for anything positive and spanning
# decades, 'linear' for quantities already in dB or dimensionless, 'angle'
# for phases, whose differences wrap.
#
# Add your own with, e.g.::
#
#     rm.FEATURES['responsivity'] = {'transform': 'log'}
#
# Nothing here is assumed to survive any particular perturbation -- which
# ones actually did is measured per dataset by fingerprint='auto'.
FEATURES = {
    'Qc': {'transform': 'log'},
    'Qi': {'transform': 'log'},
    'Ql': {'transform': 'log'},
    'fwhm': {'transform': 'log'},
    'dip_depth': {'transform': 'linear'},
    'phi': {'transform': 'angle'},
    'skew': {'transform': 'linear'},
    'anl': {'transform': 'log'},
}

# A feature is worth using when its scatter between matched pairs is small
# compared with its spread across the array: that ratio is how much it can
# tell one device from another.  0.5 keeps anything better than a factor two.
DEFAULT_FINGERPRINT_MAX_RATIO = 0.5
# Fingerprints break ties inside the frequency tolerance; they never widen
# it. This caps their say in the main assignment.
DEFAULT_FINGERPRINT_WEIGHT = 0.1
# Rescue (matching with frequency ignored) is only trustworthy when the
# fingerprints demonstrably identify devices on their own.
DEFAULT_RESCUE_MIN_ACCURACY = 0.8
DEFAULT_RESCUE_MAX_DISTANCE = 3.0
DEFAULT_RESCUE_MIN_RATIO = 2.0


def _feature_transform(name):
    """The comparison rule for a feature, defaulting to log for unknown ones."""
    return FEATURES.get(name, {}).get('transform', 'log')


def _feature_values(source, name):
    """A feature's values from a list, mapped ready for differencing."""
    if name not in source.params:
        return None
    values = np.asarray(source.params[name], dtype=float)
    kind = _feature_transform(name)
    if kind == 'log':
        out = np.full(values.shape, np.nan)
        good = np.isfinite(values) & (values > 0)
        out[good] = np.log(values[good])
        return out
    return np.where(np.isfinite(values), values, np.nan)


def _feature_difference(name, values_a, values_b):
    """``b - a`` for a feature, wrapping if it is an angle."""
    diff = values_b - values_a
    if _feature_transform(name) == 'angle':
        diff = (diff + np.pi) % (2 * np.pi) - np.pi
    return diff


def calibrate_fingerprint(a, b, ia, ib, features='auto',
                          max_ratio=DEFAULT_FINGERPRINT_MAX_RATIO):
    """Measure which resonator properties survived, using confident pairs.

    Each feature gets a global offset and a residual scatter, exactly as the
    frequencies do: a property that changed the same way for every device
    (all the Qi values halving under load, say) still identifies devices
    perfectly well, because the change is common-mode and subtracted.  What
    disqualifies a feature is *scatter*, not change.

    Parameters
    ----------
    a, b : ResonanceList
        The two lists, frequency-sorted.
    ia, ib : array-like
        Indices of pairs confident enough to calibrate on.
    features : 'auto', list, dict, or None
        ``'auto'`` tests everything both lists carry and keeps what
        discriminates.  A list forces a selection, a dict gives weights, and
        ``None`` disables fingerprinting.
    max_ratio : float, optional
        Keep features whose pair scatter is below this fraction of their
        spread across the array (default 0.5).

    Returns
    -------
    dict
        ``{feature: {'offset', 'scatter', 'spread', 'ratio', 'weight',
        'used'}}``.
    """
    if features is None or features is False or not len(ia):
        return {}
    weights = features if isinstance(features, dict) else {}
    if features == 'auto':
        names = [n for n in FEATURES if n in a.params and n in b.params]
    else:
        names = list(weights) if weights else list(features)

    calibration = {}
    for name in names:
        va, vb = _feature_values(a, name), _feature_values(b, name)
        if va is None or vb is None:
            continue
        pair_a, pair_b = va[np.asarray(ia)], vb[np.asarray(ib)]
        diff = _feature_difference(name, pair_a, pair_b)
        diff = diff[np.isfinite(diff)]
        if diff.size < 8:
            continue
        offset = float(np.median(diff))
        scatter = _robust_sigma(diff)
        spread = _robust_sigma(va[np.isfinite(va)])
        # A feature that is the same for every device (a fit pinned to a
        # target, say) has nothing to say about which device is which.
        if not (np.isfinite(scatter) and np.isfinite(spread)) \
                or scatter <= 0 or spread <= 0:
            continue
        ratio = scatter / spread
        used = ratio < max_ratio if features == 'auto' else True
        calibration[name] = {
            'offset': offset, 'scatter': scatter, 'spread': spread,
            'ratio': ratio, 'weight': float(weights.get(name, 1.0)),
            'used': bool(used),
        }
    return calibration


def describe_fingerprint(calibration):
    """Lines describing what each feature did, and whether it was used."""
    lines = []
    for name, c in sorted(calibration.items(), key=lambda kv: kv[1]['ratio']):
        verdict = 'USED  ' if c['used'] else 'dropped'
        lines.append(
            f"    {name:12s} scatter {c['scatter']:7.3f} vs spread "
            f"{c['spread']:7.3f}  ratio {c['ratio']:6.2f}  "
            f"offset {c['offset']:+7.3f}  -> {verdict}")
    return lines


def fingerprint_cost(a, b, calibration):
    """Fingerprint distance between every pair, in scatters.

    Returns ``None`` when no feature survived, so callers can tell "the
    fingerprints say these are different devices" from "there are no
    fingerprints".
    """
    used = {n: c for n, c in calibration.items() if c['used']}
    if not used:
        return None
    total = np.zeros((len(a), len(b)))
    weight = np.zeros((len(a), len(b)))
    for name, c in used.items():
        va, vb = _feature_values(a, name), _feature_values(b, name)
        diff = _feature_difference(name, va[:, None], vb[None, :])
        z = np.abs(diff - c['offset']) / c['scatter']
        ok = np.isfinite(z)
        total[ok] += c['weight'] * z[ok]
        weight[ok] += c['weight']
    return np.where(weight > 0, total / np.maximum(weight, 1e-12), np.nan)


def fingerprint_power(cost, ia, ib):
    """How often fingerprints alone name the right device, out of everything.

    A leave-one-out check on the pairs the frequencies already agreed about:
    hide the frequency, rank every candidate by fingerprint distance, and
    see where the true partner lands.  This is what decides whether a rescue
    (matching with no frequency information at all) can be trusted.

    Returns
    -------
    dict
        ``{'rank1', 'top5', 'median_rank', 'n'}``; ``rank1`` is the fraction
        of devices whose true partner was the single closest candidate.
    """
    if cost is None or not len(ia):
        return {'rank1': 0.0, 'top5': 0.0, 'median_rank': np.nan, 'n': 0}
    ranks = []
    for i, j in zip(np.asarray(ia), np.asarray(ib)):
        row = cost[i]
        if not np.isfinite(row[j]):
            continue
        ranks.append(int(np.sum(row < row[j])))
    if not ranks:
        return {'rank1': 0.0, 'top5': 0.0, 'median_rank': np.nan, 'n': 0}
    ranks = np.array(ranks)
    return {'rank1': float(np.mean(ranks == 0)),
            'top5': float(np.mean(ranks < 5)),
            'median_rank': float(np.median(ranks)),
            'n': int(len(ranks))}


def _rescue(rows, cols, a, b, fp_cost, power, mode, log, held=(),
            min_accuracy=DEFAULT_RESCUE_MIN_ACCURACY,
            max_distance=DEFAULT_RESCUE_MAX_DISTANCE,
            min_ratio=DEFAULT_RESCUE_MIN_RATIO):
    """Pair up the leftovers, ignoring frequency entirely.

    A device that moved a long way -- far enough to land outside any sensible
    tolerance, perhaps below the bottom of the band -- is invisible to the
    windowed assignment.  What is still true is that it is missing from one
    list and unaccounted for in the other, and that its fingerprint has not
    changed.  Both lines of evidence are used, and neither is trusted
    further than it has been shown to go.

    Everything here works in the lists' internal frequency-sorted order, the
    same as ``rows``/``cols``; the caller maps back to input order once.

    Returns ``(rows, cols, rescued, suggested)``.
    """
    if mode is False or mode == 'off':
        return rows, cols, [], []
    matched_a, matched_b = set(rows.tolist()), set(cols.tolist())
    # An entry the caller deliberately unmatched is not a leftover looking
    # for a home; leaving it out is the whole point.
    matched_a |= set(held)
    left_a = [i for i in range(len(a)) if i not in matched_a]
    left_b = [j for j in range(len(b)) if j not in matched_b]
    # Only leftovers the other sweep actually looked for can be rescued.
    cover_a, cover_b = a.coverage or (-np.inf, np.inf), b.coverage or (-np.inf, np.inf)
    left_a = [i for i in left_a if cover_b[0] <= a.frequency[i] <= cover_b[1]]
    left_b = [j for j in left_b if cover_a[0] <= b.frequency[j] <= cover_a[1]]
    if not left_a or not left_b:
        return rows, cols, [], []

    accuracy = power.get('rank1', 0.0)
    usable = fp_cost is not None and accuracy >= min_accuracy and mode != 'count_only'
    log(f"  rescue: {len(left_a)} leftover in A, {len(left_b)} in B"
        + (f"; fingerprints name the right device {100 * accuracy:.0f}% of "
           f"the time" if fp_cost is not None else '; no fingerprints available'))

    rescued, suggested = [], []
    if usable:
        sub = fp_cost[np.ix_(left_a, left_b)]
        for k, i in enumerate(left_a):
            row = sub[k]
            if not np.any(np.isfinite(row)):
                continue
            best = int(np.nanargmin(row))
            best_cost = float(row[best])
            others = np.delete(row, best)
            runner = float(np.nanmin(others)) if others.size and np.any(np.isfinite(others)) \
                else np.inf
            ratio = runner / best_cost if best_cost > 0 else np.inf
            entry = {'a_index': i, 'b_index': left_b[best], 'distance': best_cost,
                     'ratio': ratio}
            if best_cost <= max_distance and ratio >= min_ratio:
                rescued.append(entry)
            else:
                suggested.append(entry)
        # Never let two rescues claim the same device.
        taken = set()
        keep = []
        for entry in sorted(rescued, key=lambda e: e['distance']):
            if entry['b_index'] in taken:
                suggested.append(entry)
                continue
            taken.add(entry['b_index'])
            keep.append(entry)
        rescued = keep
    elif len(left_a) == 1 and len(left_b) == 1:
        # Bookkeeping alone: one device missing here, one unaccounted for
        # there, and nowhere else for either to have gone.
        rescued = [{'a_index': left_a[0], 'b_index': left_b[0],
                    'distance': np.nan, 'ratio': np.nan, 'by_count': True}]
        log("    one leftover each side: paired by elimination")

    if rescued:
        rows = np.concatenate([rows, [e['a_index'] for e in rescued]]).astype(int)
        cols = np.concatenate([cols, [e['b_index'] for e in rescued]]).astype(int)
    if rescued or suggested:
        log(f"    {len(rescued)} rescued, {len(suggested)} suggested but not applied")
        for entry in rescued:
            log(f"      A {a.frequency[entry['a_index']] / 1e6:10.4f} MHz -> "
                f"B {b.frequency[entry['b_index']] / 1e6:10.4f} MHz   "
                f"df/f {(b.frequency[entry['b_index']] / a.frequency[entry['a_index']] - 1):+.3f}"
                + ('  by elimination' if entry.get('by_count') else
                   f"  fingerprint {entry['distance']:.2f}, "
                   f"runner-up {entry['ratio']:.1f}x worse"))
    return rows, cols, rescued, suggested


# --- the match --------------------------------------------------------------

_PAIR_DTYPE = np.dtype([
    ('a_index', np.int64), ('b_index', np.int64),
    ('f_a', np.float64), ('f_b', np.float64),
    ('df_hz', np.float64), ('df_frac', np.float64),
    ('residual_hz', np.float64), ('residual_sigma', np.float64),
    ('ambiguity', np.float64), ('flag', 'U16'),
])


@dataclass
class ResonanceMatch:
    """The mapping between two resonance lists, plus how it was arrived at.

    The plain answer is :py:attr:`index`; everything else is there to let you
    check it and fix it.
    """

    a: ResonanceList
    b: ResonanceList
    pairs: np.ndarray
    transform: dict
    tolerance: float
    settings: dict = field(default_factory=dict)
    trace: list = field(default_factory=list)
    fingerprint: dict = field(default_factory=dict)
    fingerprint_power: dict = field(default_factory=dict)
    rescued: list = field(default_factory=list)
    suggested: list = field(default_factory=list)

    # -- the answer ----------------------------------------------------------

    @property
    def index(self):
        """``index[i]`` = entry of B matching entry ``i`` of A, or -1.

        Both are indices into the lists **as you passed them in**, not into
        any internally sorted order.
        """
        out = np.full(len(self.a), -1, dtype=np.int64)
        matched = self.pairs[(self.pairs['a_index'] >= 0)
                             & (self.pairs['b_index'] >= 0)]
        out[matched['a_index']] = matched['b_index']
        return out

    @property
    def index_b(self):
        """``index_b[j]`` = entry of A matching entry ``j`` of B, or -1."""
        out = np.full(len(self.b), -1, dtype=np.int64)
        matched = self.pairs[(self.pairs['a_index'] >= 0)
                             & (self.pairs['b_index'] >= 0)]
        out[matched['b_index']] = matched['a_index']
        return out

    @property
    def matched(self):
        """The matched pairs only."""
        return self.pairs[(self.pairs['a_index'] >= 0)
                          & (self.pairs['b_index'] >= 0)]

    @property
    def unmatched_a(self):
        """Entries of A with no counterpart (excluding out-of-range ones)."""
        return self.pairs[self.pairs['flag'] == 'unmatched_a']

    @property
    def unmatched_b(self):
        """Entries of B with no counterpart (excluding out-of-range ones)."""
        return self.pairs[self.pairs['flag'] == 'unmatched_b']

    def flagged(self, *flags):
        """Pairs carrying any of the given flags (default: everything odd)."""
        if not flags:
            flags = ('ambiguous', 'outlier', 'crossed', 'rescued')
        return self.pairs[np.isin(self.pairs['flag'], flags)]

    def candidates(self, a_index, n=5, by='fingerprint'):
        """Rank the plausible partners for one entry of A.

        For checking a match by hand, and for the cases the matcher declined
        to decide: it will not guess, but it will show you its shortlist.

        Parameters
        ----------
        a_index : int
            Entry of A, in the order you passed it in.
        n : int, optional
            How many candidates to return (default 5).
        by : {'fingerprint', 'frequency'}, optional
            What to rank on.  ``'fingerprint'`` ignores frequency entirely,
            which is the point when a device has moved a long way.

        Returns
        -------
        numpy.ndarray
            Structured array of ``b_index``, ``f_b``, ``df_hz``, ``cost``,
            best first.
        """
        sorted_a = int(np.where(self.a.order == a_index)[0][0])
        if by == 'fingerprint':
            cost = fingerprint_cost(self.a, self.b, self.fingerprint)
            if cost is None:
                raise ValueError(
                    "no usable fingerprints; rank by='frequency' instead, or "
                    "supply lists carrying fitted parameters.")
            costs = cost[sorted_a]
        else:
            f_a = apply_transform(self.transform, self.a.frequency[sorted_a])
            costs = np.abs(self.b.frequency - f_a) / self.tolerance
        keep = np.argsort(np.where(np.isfinite(costs), costs, np.inf))[:n]
        out = np.empty(len(keep), dtype=[
            ('b_index', np.int64), ('f_b', np.float64),
            ('df_hz', np.float64), ('cost', np.float64)])
        for k, j in enumerate(keep):
            out[k] = (self.b.order[j], self.b.frequency[j],
                      self.b.frequency[j] - self.a.frequency[sorted_a],
                      costs[j])
        return out

    def groups(self, merge_window=None):
        """Devices grouped into clusters, so N-to-M relationships are visible.

        A blended pair that resolved into two, or two that merged into one,
        is not an error and not a simple pairing.  Entries close enough to
        be confusable are collected into clusters labelled ``n_a:n_b``; the
        one-to-one map in :py:attr:`index` still holds inside each.

        Parameters
        ----------
        merge_window : float or None, optional
            How close (Hz) two entries must be to belong to the same
            cluster.  Defaults to three linewidths.

        Returns
        -------
        list of dict
            ``{'a_indices', 'b_indices', 'kind'}``, ordered by frequency.
        """
        if merge_window is None:
            widths = [w for w in (self.a.linewidth, self.b.linewidth)
                      if np.isfinite(w)]
            merge_window = 3.0 * max(widths) if widths else self.tolerance
        na, nb = len(self.a), len(self.b)
        parent = list(range(na + nb))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def union(x, y):
            rx, ry = find(x), find(y)
            if rx != ry:
                parent[ry] = rx

        # Everything is placed on B's frequency axis so the two lists are
        # directly comparable.
        fa = apply_transform(self.transform, self.a.frequency)
        fb = self.b.frequency
        nodes = sorted([(f, k) for k, f in enumerate(fa)]
                       + [(f, na + k) for k, f in enumerate(fb)])
        for (f1, n1), (f2, n2) in zip(nodes, nodes[1:]):
            if f2 - f1 < merge_window:
                union(n1, n2)
        index = self.index
        for i in range(na):
            if index[i] >= 0:
                sorted_a = int(np.where(self.a.order == i)[0][0])
                sorted_b = int(np.where(self.b.order == index[i])[0][0])
                union(sorted_a, na + sorted_b)

        clusters = {}
        for node in range(na + nb):
            clusters.setdefault(find(node), []).append(node)
        out = []
        for members in clusters.values():
            a_sorted = [m for m in members if m < na]
            b_sorted = [m - na for m in members if m >= na]
            out.append({
                'a_indices': np.sort(self.a.order[a_sorted]),
                'b_indices': np.sort(self.b.order[b_sorted]),
                'kind': f"{len(a_sorted)}:{len(b_sorted)}",
                'frequency': float(np.min([fa[m] for m in a_sorted] or
                                          [fb[m - na] for m in members])),
            })
        return sorted(out, key=lambda g: g['frequency'])

    # -- getting values out --------------------------------------------------

    def aligned(self, on='matched'):
        """Index arrays lining the two lists up.

        Parameters
        ----------
        on : {'matched', 'a', 'b', 'union'}
            ``'matched'`` keeps only devices found in both.  ``'a'`` keeps
            A's length and order, ``'b'`` keeps B's, and ``'union'`` keeps
            everything from both.  Missing entries are ``-1``.

        Returns
        -------
        tuple of numpy.ndarray
            ``(ia, ib)``, equal length, indices into A and B as passed in.
        """
        index, index_b = self.index, self.index_b
        if on == 'matched':
            ia = np.where(index >= 0)[0]
            return ia, index[ia]
        if on == 'a':
            return np.arange(len(self.a)), index
        if on == 'b':
            return index_b, np.arange(len(self.b))
        if on == 'union':
            ia = np.arange(len(self.a))
            ib = index
            extra = np.where(index_b < 0)[0]
            return (np.concatenate([ia, np.full(len(extra), -1, dtype=np.int64)]),
                    np.concatenate([ib, extra]))
        raise ValueError(f"on must be 'matched', 'a', 'b' or 'union', got {on!r}")

    def _values(self, spec, which):
        """Resolve a name or an array into values in that list's input order."""
        source = self.a if which == 'a' else self.b
        if isinstance(spec, str):
            if spec not in source.params:
                raise KeyError(
                    f"{spec!r} is not in list {which.upper()} "
                    f"(has: {sorted(source.params)}).")
            values = np.asarray(source.params[spec])
            if values.dtype.kind not in 'fiub':
                raise TypeError(
                    f"{spec!r} is not numeric in list {which.upper()} "
                    f"(dtype {values.dtype}); it cannot be compared.")
            values = values.astype(float)
        else:
            values = np.asarray(spec, dtype=float)
            if len(values) != len(source):
                raise ValueError(
                    f"expected {len(source)} values for list {which.upper()} "
                    f"(its input length), got {len(values)}.")
            return values
        # params are held in sorted order; put them back in input order.
        out = np.empty(len(source), dtype=float)
        out[source.order] = values
        return out

    def compare(self, a_values, b_values=None, on='matched'):
        """Line up any per-resonance quantity across the two lists.

        Parameters
        ----------
        a_values, b_values : str, array-like, or dict
            A parameter name present in the list, or an array in that list's
            **input order**.  A dict of either gives a dict back.  If
            ``b_values`` is omitted, ``a_values`` must be a name and is
            looked up in both.
        on : {'matched', 'a', 'b', 'union'}, optional
            Layout, as in :py:meth:`aligned`.  Anything but ``'matched'``
            pads with NaN where there is no counterpart.

        Returns
        -------
        tuple
            ``(values_a, values_b)``, equal length.

        Examples
        --------
        >>> qi_cold, qi_warm = m.compare('Qi')
        >>> nep_cold, nep_warm = m.compare(nep_a, nep_b)
        """
        if b_values is None:
            if not isinstance(a_values, (str, dict)):
                raise ValueError(
                    "give b_values as well, or pass a parameter name present "
                    "in both lists.")
            b_values = a_values
        if isinstance(a_values, dict) or isinstance(b_values, dict):
            keys = list(a_values) if isinstance(a_values, dict) else list(b_values)
            out_a, out_b = {}, {}
            for key in keys:
                spec_a = a_values[key] if isinstance(a_values, dict) else a_values
                spec_b = b_values[key] if isinstance(b_values, dict) else b_values
                out_a[key], out_b[key] = self.compare(spec_a, spec_b, on=on)
            return out_a, out_b

        va = self._values(a_values, 'a')
        vb = self._values(b_values, 'b')
        ia, ib = self.aligned(on=on)
        take_a = np.where(ia >= 0, va[np.maximum(ia, 0)], np.nan)
        take_b = np.where(ib >= 0, vb[np.maximum(ib, 0)], np.nan)
        return take_a, take_b

    def transfer(self, values_a, fill=np.nan):
        """Carry per-device information from A onto B's ordering.

        For keeping a beam map, device names, or tone powers across a change
        that renumbered everything.

        Parameters
        ----------
        values_a : array-like
            One entry per resonance in A, in A's input order.
        fill : object, optional
            Value for devices in B that were not matched (default NaN; use
            ``None`` for object arrays such as names).

        Returns
        -------
        numpy.ndarray
            One entry per resonance in B, in B's input order.
        """
        values_a = np.asarray(values_a)
        if len(values_a) != len(self.a):
            raise ValueError(
                f"expected {len(self.a)} values (list A's input length), "
                f"got {len(values_a)}.")
        index_b = self.index_b
        dtype = values_a.dtype
        if fill is None or dtype.kind in 'USO':
            out = np.full(len(self.b), fill, dtype=object)
        else:
            if dtype.kind in 'iub' and isinstance(fill, float) \
                    and not np.isfinite(fill):
                dtype = np.float64      # an int array cannot hold the NaN fill
            out = np.full(len(self.b), fill, dtype=dtype)
        found = index_b >= 0
        out[found] = values_a[index_b[found]]
        return out

    def table(self):
        """One row per pair, with both lists' parameters side by side.

        Returns
        -------
        numpy.ndarray
            Structured array: the pair columns, then each parameter present
            in either list suffixed ``_a`` / ``_b``.
        """
        columns = [(name, self.pairs[name]) for name in self.pairs.dtype.names]
        for which, source in (('a', self.a), ('b', self.b)):
            idx = self.pairs[f'{which}_index']
            for name in sorted(source.params):
                # Non-numeric columns (a fit's provenance string, say) are
                # carried in the list but have no place in a comparison table.
                if np.asarray(source.params[name]).dtype.kind not in 'fiub':
                    continue
                values = self._values(name, which)
                taken = np.where(idx >= 0, values[np.maximum(idx, 0)], np.nan)
                columns.append((f'{name}_{which}', taken))
        dtype = np.dtype([(name, values.dtype if values.dtype.kind == 'U'
                           else np.float64 if values.dtype.kind == 'f'
                           else values.dtype)
                          for name, values in columns])
        out = np.empty(len(self.pairs), dtype=dtype)
        for name, values in columns:
            out[name] = values
        return out

    # -- reporting -----------------------------------------------------------

    def summary(self, show=10):
        """Print what happened, and what is worth looking at.

        Parameters
        ----------
        show : int, optional
            How many flagged pairs to list (default 10; 0 for none).
        """
        for line in self.trace:
            print(line)
        matched = self.matched
        counts = {flag: int(np.sum(self.pairs['flag'] == flag))
                  for flag in np.unique(self.pairs['flag'])}
        n_common = len(matched) + counts.get('unmatched_a', 0)
        print(f"  matched {len(matched)} of {len(self.a)} (A) / {len(self.b)} (B)"
              + (f"  [{100.0 * len(matched) / n_common:.1f}% of A in the common range]"
                 if n_common else ''))
        if len(matched):
            resid = matched['residual_hz']
            print(f"  residuals: median {np.median(resid) / 1e3:+.2f} kHz, "
                  f"robust sigma {_robust_sigma(resid) / 1e3:.2f} kHz, "
                  f"max {np.max(np.abs(resid)) / 1e3:.1f} kHz")
        for flag in ('ok', 'crossed', 'ambiguous', 'outlier',
                     'unmatched_a', 'unmatched_b', 'out_of_range', 'manual'):
            if counts.get(flag):
                print(f"    {flag:14s} {counts[flag]:5d}")
        odd = self.flagged()
        if show and len(odd):
            print(f"  worth a look (highest ambiguity first, {min(show, len(odd))} "
                  f"of {len(odd)}):")
            for row in odd[np.argsort(-odd['ambiguity'])][:show]:
                print(f"    A[{row['a_index']:4d}] {row['f_a'] / 1e6:10.4f} MHz -> "
                      f"B[{row['b_index']:4d}] {row['f_b'] / 1e6:10.4f} MHz   "
                      f"resid {row['residual_hz'] / 1e3:+7.2f} kHz "
                      f"({row['residual_sigma']:+.1f} sigma)  "
                      f"ambiguity {row['ambiguity']:.2f}  {row['flag']}")

    # -- adjusting it by hand ------------------------------------------------

    def _resolve(self, key, which):
        """An index into a list, from either an index or a frequency.

        Integers select by position, floats by nearest frequency, so both
        "the one that was number 12" and "the resonator at 1.2345 GHz" work
        without having to know which is which.
        """
        source = self.a if which == 'a' else self.b
        if key is None:
            return None
        if isinstance(key, (int, np.integer)):
            if not -len(source) <= int(key) < len(source):
                raise IndexError(
                    f"list {which.upper()} has {len(source)} entries; "
                    f"{key} is out of range.")
            return int(key) % len(source)
        frequency = float(key)
        sorted_index = int(np.argmin(np.abs(source.frequency - frequency)))
        offset = abs(source.frequency[sorted_index] - frequency)
        if offset > 10 * self.tolerance:
            raise ValueError(
                f"nothing in list {which.upper()} within "
                f"{10 * self.tolerance / 1e3:.0f} kHz of "
                f"{frequency / 1e6:.4f} MHz (nearest is "
                f"{source.frequency[sorted_index] / 1e6:.4f} MHz).")
        return int(source.order[sorted_index])

    @property
    def locked(self):
        """Pairings fixed by hand, as ``{a_index: b_index}``.

        Survive :py:meth:`rematch`, which re-solves everything else around
        them.
        """
        return dict(self.settings.get('locked') or {})

    def set(self, a, b):
        """Pair these two by hand, and keep them paired.

        Whatever either of them was matched to is released.  Both arguments
        take an index or a frequency in Hz.

        Examples
        --------
        >>> m.set(a=17, b=22)
        >>> m.set(a=1.23456e9, b=1.23012e9)
        """
        ai, bi = self._resolve(a, 'a'), self._resolve(b, 'b')
        locked = {held_a: held_b for held_a, held_b in self.locked.items()
                  if held_a != ai and held_b != bi}
        locked[ai] = bi
        index = self.index
        index[index == bi] = -1
        index[ai] = bi
        return self._replace(index, manual=set(locked), locked=locked)

    def unmatch(self, a=None, b=None):
        """Break a pairing, and stop it being remade.

        Give either side; the other follows.
        """
        ai, bi = self._resolve(a, 'a'), self._resolve(b, 'b')
        index = self.index
        if ai is None and bi is not None:
            ai = int(self.index_b[bi]) if self.index_b[bi] >= 0 else None
        if ai is None:
            raise ValueError("give a= or b= to say which pairing to break.")
        locked = dict(self.locked)
        # Remember the refusal as a lock to nothing, so a rematch -- and the
        # rescue pass, which is otherwise glad to pair up two leftovers --
        # does not simply undo it.
        locked[ai] = -1
        index[ai] = -1
        return self._replace(index, manual=(), locked=locked)

    def lock(self, *a_indices):
        """Fix the current pairings of these entries of A against a rematch."""
        locked = dict(self.locked)
        index = self.index
        for key in a_indices:
            ai = self._resolve(key, 'a')
            locked[ai] = int(index[ai])
        return self._replace(index, manual=(), locked=locked)

    def rematch(self, **kwargs):
        """Solve again from scratch, holding the locked pairings fixed.

        Everything else is free to move around them, which is the point:
        pin the two you are sure of and let the rest re-solve.  Any keyword
        overrides the setting used the first time.
        """
        settings = {k: v for k, v in self.settings.items() if k != 'locked'}
        settings.update(kwargs)
        settings.setdefault('verbose', False)
        return match_resonances(self.a, self.b, locked=self.locked, **settings)

    def _replace(self, index, manual=(), locked=None):
        """A copy of this match carrying a different index array.

        Settings are copied, not shared, so editing a match never reaches
        back and changes the one it came from.
        """
        pairs = _build_pairs(self.a, self.b, index, self.transform,
                             self.tolerance)
        for row in pairs:
            if row['a_index'] in manual and row['b_index'] >= 0:
                row['flag'] = 'manual'
        settings = dict(self.settings)
        if locked is not None:
            settings['locked'] = dict(locked)
        return ResonanceMatch(
            a=self.a, b=self.b, pairs=pairs, transform=self.transform,
            tolerance=self.tolerance, settings=settings, trace=self.trace,
            fingerprint=self.fingerprint, fingerprint_power=self.fingerprint_power,
            rescued=self.rescued, suggested=self.suggested)

    # -- plots ---------------------------------------------------------------

    def plot(self, **kwargs):
        """The overlay plot; see :py:func:`plot_match`."""
        return plot_match(self, **kwargs)

    def plot_shift(self, **kwargs):
        """Fractional shift against frequency; see :py:func:`plot_shift`."""
        return plot_shift(self, **kwargs)

    def plot_quality(self, **kwargs):
        """Residuals and ambiguity; see :py:func:`plot_quality`."""
        return plot_quality(self, **kwargs)

    def plot_compare(self, *args, **kwargs):
        """Compare a quantity across the two; see :py:func:`plot_compare`."""
        return plot_compare(self, *args, **kwargs)

    def plot_pair(self, a_index, **kwargs):
        """One device's two raw traces; see :py:func:`plot_pair`."""
        return plot_pair(self, a_index, **kwargs)

    def save(self, path):
        """Write the match to CSV, for the record or for editing by hand.

        Edit the ``b_index`` column and read it back with
        :py:func:`load_match` to override any pairing.
        """
        table = self.table()
        with open(path, 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(table.dtype.names)
            for row in table:
                writer.writerow(['' if isinstance(v, float) and not np.isfinite(v)
                                 else v for v in row])
        return path


def load_match(path, a, b, **kwargs):
    """Read back a match CSV, honouring any ``b_index`` edited by hand.

    Parameters
    ----------
    path : str
        A CSV written by :py:meth:`ResonanceMatch.save`.
    a, b : any
        The same two inputs the match was made from.
    **kwargs
        Passed to :py:func:`match_resonances` when recomputing the
        diagnostics for the edited pairing.

    Returns
    -------
    ResonanceMatch
    """
    data = np.genfromtxt(path, names=True, delimiter=',', dtype=None,
                         encoding=None, deletechars='')
    data = np.atleast_1d(data)
    overrides = {}
    for row in data:
        ai, bi = int(row['a_index']), int(row['b_index'])
        if ai >= 0 and bi >= 0:
            overrides[ai] = bi
    match = match_resonances(a, b, verbose=False, **kwargs)
    return _apply_overrides(match, overrides)


def _apply_overrides(match, overrides):
    """Force the given a->b pairings, dropping whatever they displace."""
    if not overrides:
        return match
    index = match.index
    for ai, bi in overrides.items():
        index[index == bi] = -1
        index[ai] = bi
    return _rebuild(match, index, flag_manual=set(overrides))


def _rebuild(match, index, flag_manual=()):
    """Rebuild the pair table from an a->b index array."""
    pairs = _build_pairs(match.a, match.b, index, match.transform,
                         match.tolerance, ambiguity=None)
    for row in pairs:
        if row['a_index'] in flag_manual and row['b_index'] >= 0:
            row['flag'] = 'manual'
    return ResonanceMatch(a=match.a, b=match.b, pairs=pairs,
                          transform=match.transform, tolerance=match.tolerance,
                          settings=match.settings, trace=match.trace,
                          fingerprint=match.fingerprint,
                          fingerprint_power=match.fingerprint_power,
                          rescued=match.rescued, suggested=match.suggested)


def _build_pairs(a, b, index, transform, tolerance, ambiguity=None,
                 outlier_sigma=DEFAULT_OUTLIER_SIGMA,
                 ambiguity_threshold=DEFAULT_AMBIGUITY):
    """Assemble the pair table, with residuals and flags, in input order."""
    # a/b are frequency-sorted internally; index is in input order.
    fa_input = np.empty(len(a))
    fa_input[a.order] = a.frequency
    fb_input = np.empty(len(b))
    fb_input[b.order] = b.frequency
    fa_t_input = apply_transform(transform, fa_input)

    matched_a = np.where(index >= 0)[0]
    matched_b = index[matched_a]
    resid = (fb_input[matched_b] - fa_t_input[matched_a]) if len(matched_a) \
        else np.zeros(0)
    frac = resid / np.maximum(fa_input[matched_a], 1.0) if len(matched_a) else np.zeros(0)
    sigma = _robust_sigma(frac)
    resid_sigma = (frac - np.median(frac)) / sigma if (len(frac) and sigma > 0) \
        else np.zeros(len(frac))

    # Order matched pairs by frequency so crossings can be spotted.
    keep = np.argsort(fa_input[matched_a])
    matched_a, matched_b = matched_a[keep], matched_b[keep]
    resid, frac, resid_sigma = resid[keep], frac[keep], resid_sigma[keep]
    crossed = ~_longest_increasing(fb_input[matched_b]) if len(matched_b) \
        else np.zeros(0, dtype=bool)
    amb = np.zeros(len(matched_a)) if ambiguity is None else np.asarray(ambiguity)[keep]

    rows = []
    for k, (ai, bi) in enumerate(zip(matched_a, matched_b)):
        if abs(resid_sigma[k]) > outlier_sigma:
            flag = 'outlier'
        elif amb[k] > ambiguity_threshold:
            flag = 'ambiguous'
        elif crossed[k]:
            flag = 'crossed'
        else:
            flag = 'ok'
        rows.append((ai, bi, fa_input[ai], fb_input[bi],
                     fb_input[bi] - fa_input[ai],
                     (fb_input[bi] - fa_input[ai]) / fa_input[ai],
                     resid[k], resid_sigma[k], amb[k], flag))

    cover_b = b.coverage or (-np.inf, np.inf)
    cover_a = a.coverage or (-np.inf, np.inf)
    for ai in np.where(index < 0)[0]:
        inside = cover_b[0] <= fa_t_input[ai] <= cover_b[1]
        rows.append((ai, -1, fa_input[ai], np.nan, np.nan, np.nan, np.nan,
                     np.nan, np.nan, 'unmatched_a' if inside else 'out_of_range'))
    claimed = set(int(v) for v in index[index >= 0])
    for bi in range(len(b)):
        if bi in claimed:
            continue
        inside = cover_a[0] <= fb_input[bi] <= cover_a[1]
        rows.append((-1, bi, np.nan, fb_input[bi], np.nan, np.nan, np.nan,
                     np.nan, np.nan, 'unmatched_b' if inside else 'out_of_range'))

    pairs = np.empty(len(rows), dtype=_PAIR_DTYPE)
    for i, row in enumerate(rows):
        pairs[i] = row
    return pairs


def match_resonances(a, b, shift_model='auto', tolerance='auto', order='free',
                     fingerprint='auto', rescue='auto', units='auto',
                     coverage_a=None, coverage_b=None, iterations=3,
                     outlier_sigma=DEFAULT_OUTLIER_SIGMA,
                     ambiguity_threshold=DEFAULT_AMBIGUITY,
                     fingerprint_max_ratio=DEFAULT_FINGERPRINT_MAX_RATIO,
                     fingerprint_weight=DEFAULT_FINGERPRINT_WEIGHT,
                     locked=None, verbose=True):
    """Match two resonance lists, working out which entry is which device.

    Parameters
    ----------
    a, b : any
        The two lists, in any form :py:func:`as_resonance_list` accepts.
    shift_model : str or float, optional
        How the frequencies are allowed to have moved as a whole:
        ``'auto'`` (default) picks between ``'none'``, ``'constant'``,
        ``'proportional'``, ``'affine'`` and ``'linear_frac'``, preferring
        the simplest that fits.  A number is taken as a fixed ``df/f``.
    tolerance : float or 'auto', optional
        How far apart (Hz) two entries may be, after the shift, and still be
        the same device.  ``'auto'`` derives it from the data; see
        :py:func:`choose_tolerance`.
    order : {'free', 'monotonic', 'nearest'}, optional
        ``'free'`` (default) solves the assignment globally and lets devices
        reorder.  ``'monotonic'`` forbids reordering, which helps in dense
        arrays when you know the order held.  ``'nearest'`` keeps only
        mutual nearest neighbours.
    fingerprint : 'auto', list, dict, or None, optional
        Resonator properties used to tell devices apart when frequency
        cannot.  ``'auto'`` (default) measures which ones survived and keeps
        those; a list or ``{name: weight}`` dict forces a choice; ``None``
        matches on frequency alone.  Fingerprints only break ties inside
        the tolerance -- they never pull in a pair frequency excluded.

        Note that matching on a property and then comparing it biases the
        comparison towards "nothing changed".  Exclude whatever you are
        measuring, or check :py:attr:`ResonanceMatch.fingerprint` to see
        what was used.
    rescue : {'auto', 'count_only', False}, optional
        Whether to pair up leftovers with frequency ignored, for devices
        that moved too far to be found any other way.  ``'auto'`` (default)
        does so only when the fingerprints are measurably good enough;
        ``'count_only'`` pairs a lone leftover on each side and nothing
        else.  Rescued pairs are always flagged, never silently absorbed.
    units : {'auto', 'hz', 'mhz', 'ghz'}, optional
        Units of the inputs, when they are bare numbers.
    coverage_a, coverage_b : tuple or None, optional
        ``(f_min, f_max)`` searched for each list.  Set these when one sweep
        covered less band than the other, so devices outside the overlap are
        reported as out-of-range rather than lost.
    iterations : int, optional
        Match/refit cycles when estimating the shift (default 3).
    outlier_sigma : float, optional
        Matched pairs further than this from the global model are flagged
        ``outlier`` (default 5).
    ambiguity_threshold : float, optional
        Pairs whose runner-up costs more than this fraction of the best are
        flagged ``ambiguous`` (default 0.5).
    locked : dict or None, optional
        ``{a_index: b_index}`` pairings to hold fixed, imposed on the cost
        matrix so everything else is solved optimally around them.  A
        ``b_index`` of ``-1`` holds that entry deliberately unmatched.
        Usually set for you by :py:meth:`ResonanceMatch.set` and applied by
        :py:meth:`ResonanceMatch.rematch`.
    verbose : bool, optional
        Print the decisions as they are made (default True).

    Returns
    -------
    ResonanceMatch

    Examples
    --------
    >>> m = match_resonances('mirror.resonances', 'room.resonances')
    >>> m.index[:5]
    array([0, 1, 2, 3, 4])
    """
    a = as_resonance_list(a, units=units, coverage=coverage_a)
    b = as_resonance_list(b, units=units, coverage=coverage_b)
    trace = []

    def log(message):
        trace.append(message)
        if verbose:
            print(message)

    log(f"matching {a.source or 'A'} ({len(a)}) -> {b.source or 'B'} ({len(b)})")
    if not len(a) or not len(b):
        pairs = _build_pairs(a, b, np.full(len(a), -1, dtype=np.int64),
                             {'model': 'none', 'params': ()}, 1.0)
        return ResonanceMatch(a=a, b=b, pairs=pairs,
                              transform={'model': 'none', 'params': ()},
                              tolerance=np.nan, trace=trace)

    transform = estimate_transform(a, b, model=shift_model,
                                   iterations=iterations, verbose=verbose)
    log(f"  shift: {describe_transform(transform, a.frequency)}"
        f"   (scatter {transform.get('sigma', np.nan):.2e})")

    tol, detail = choose_tolerance(a, b, transform, transform.get('sigma', np.nan),
                                   tolerance=tolerance)
    log(f"  tolerance: {detail}")

    fa_t = apply_transform(transform, a.frequency)
    cost = _cost_matrix(fa_t, b.frequency, tol)
    # Pinned pairings are imposed on the cost matrix rather than forced on
    # afterwards, so the rest of the array is solved optimally around them.
    for pin_a, pin_b in (locked or {}).items():
        row = int(np.where(a.order == pin_a)[0][0])
        cost[row, :] = _FORBIDDEN
        if pin_b is None or pin_b < 0:
            continue                       # held deliberately unmatched
        col = int(np.where(b.order == pin_b)[0][0])
        cost[:, col] = _FORBIDDEN
        cost[row, col] = 0.0
    if locked:
        log(f"  holding {len(locked)} pairing(s) fixed by hand")
    if order not in _ASSIGNERS:
        raise ValueError(f"order must be one of {sorted(_ASSIGNERS)}, got {order!r}")
    rows, cols = _ASSIGNERS[order](cost)
    log(f"  assignment ({order}): {len(rows)} pairs")

    # Cross-check against the order-preserving solution: where the two
    # disagree is exactly where the frequencies alone cannot decide.
    if order == 'free' and len(rows):
        alt_rows, alt_cols = _assign_monotonic(cost)
        alt = dict(zip(alt_rows.tolist(), alt_cols.tolist()))
        disagree = sum(1 for i, j in zip(rows.tolist(), cols.tolist())
                       if alt.get(i, j) != j)
        log(f"  monotonic cross-check: agrees on {len(rows) - disagree}"
            f"/{len(rows)} pairs")

    # Second pass: work out which resonator properties survived whatever
    # happened between the two sweeps, and use the ones that did to settle
    # pairings the frequencies alone cannot.
    amb = _ambiguity(cost, rows, cols)
    calibration, power, fp_cost = {}, {}, None
    if fingerprint is not None and fingerprint is not False and len(rows):
        confident = amb < ambiguity_threshold
        calibration = calibrate_fingerprint(
            a, b, rows[confident], cols[confident], features=fingerprint,
            max_ratio=fingerprint_max_ratio)
        fp_cost = fingerprint_cost(a, b, calibration)
        if calibration:
            log(f"  fingerprint ({fingerprint if isinstance(fingerprint, str) else 'given'}"
                f", from {int(confident.sum())} confident pairs):")
            for line in describe_fingerprint(calibration):
                log(line)
        if fp_cost is not None:
            power = fingerprint_power(fp_cost, rows[confident], cols[confident])
            log(f"    identifies the right device on its own "
                f"{100 * power['rank1']:.0f}% of the time "
                f"(top-5 {100 * power['top5']:.0f}%)")
            # Fingerprints only break ties. Blending as a weighted average of
            # two costs that both run 0..1 keeps every feasible pair feasible:
            # a bad fingerprint can lose a pair to a better rival, but it can
            # never push one outside the frequency tolerance on its own.
            fp_scaled = np.clip(np.nan_to_num(fp_cost, nan=1.0) / 5.0, 0.0, 1.0)
            blended = ((1.0 - fingerprint_weight) * cost
                       + fingerprint_weight * fp_scaled)
            blended[cost >= _FORBIDDEN] = _FORBIDDEN
            new_rows, new_cols = _ASSIGNERS[order](blended)
            changed = len(set(zip(rows.tolist(), cols.tolist()))
                          - set(zip(new_rows.tolist(), new_cols.tolist())))
            if changed:
                log(f"    re-assigned with fingerprints: {changed} pairs changed")
            rows, cols = new_rows, new_cols
            amb = _ambiguity(blended, rows, cols)

    n_windowed = len(rows)
    held = [int(np.where(a.order == pin_a)[0][0])
            for pin_a, pin_b in (locked or {}).items()
            if pin_b is None or pin_b < 0]
    rows, cols, rescued, suggested = _rescue(rows, cols, a, b, fp_cost, power,
                                             rescue, log, held=held)
    # A rescue was only tested against the other leftovers, not the whole
    # array, so carry its own runner-up margin across as its ambiguity.
    if len(rows) > n_windowed:
        amb = np.concatenate([amb, [min(1.0, 1.0 / e['ratio'])
                                    if np.isfinite(e['ratio']) and e['ratio'] > 0
                                    else 0.0 for e in rescued]])

    # Translate sorted-order results back into the caller's input order.
    index = np.full(len(a), -1, dtype=np.int64)
    index[a.order[rows]] = b.order[cols]
    rescued_a = {int(a.order[e['a_index']]) for e in rescued}
    for entry in rescued + suggested:
        entry['a_index'] = int(a.order[entry['a_index']])
        entry['b_index'] = int(b.order[entry['b_index']])

    amb_by_a = dict(zip(a.order[rows].tolist(), amb.tolist()))
    ordered_amb = np.array([amb_by_a.get(int(i), 0.0)
                            for i in np.where(index >= 0)[0]])

    pairs = _build_pairs(a, b, index, transform, tol, ambiguity=ordered_amb,
                         outlier_sigma=outlier_sigma,
                         ambiguity_threshold=ambiguity_threshold)
    for row in pairs:
        if row['a_index'] in rescued_a and row['b_index'] >= 0:
            row['flag'] = 'rescued'
    settings = {'shift_model': shift_model, 'tolerance': tolerance,
                'order': order, 'iterations': iterations,
                'outlier_sigma': outlier_sigma, 'fingerprint': fingerprint,
                'ambiguity_threshold': ambiguity_threshold, 'rescue': rescue,
                'locked': dict(locked or {})}
    return ResonanceMatch(a=a, b=b, pairs=pairs, transform=transform,
                          tolerance=tol, settings=settings, trace=trace,
                          fingerprint=calibration, fingerprint_power=power,
                          rescued=rescued, suggested=suggested)


# --- plots ------------------------------------------------------------------

# Kept in this module rather than in the plotting package: these are only
# useful with a match in hand, and the point of the tool is that one file
# can be read and adjusted end to end.

_FLAG_COLOURS = {
    'ok': '#4c72b0', 'crossed': '#dd8452', 'ambiguous': '#c44e52',
    'outlier': '#8172b3', 'rescued': '#55a868', 'manual': '#000000',
}


def _source_labels(match):
    """Names for the two lists, kept distinguishable when they collide.

    Two sweeps loaded the same way arrive with the same source name, which
    would otherwise label both axes of a comparison identically.
    """
    name_a = match.a.source or 'A'
    name_b = match.b.source or 'B'
    if name_a == name_b:
        return f'{name_a} (A)', f'{name_b} (B)'
    return name_a, name_b


def _new_axes(ax, **kwargs):
    """Return ``(figure, axes)``, making them only if the caller did not."""
    import matplotlib.pyplot as plt

    if ax is None:
        fig, ax = plt.subplots(**kwargs)
        return fig, ax
    return ax.get_figure(), ax


def _sweep_traces(source):
    """``(frequency, |S21|)`` traces from a list's sweep, if it has one."""
    sweep = source.sweep
    if not sweep:
        return []
    f = np.atleast_2d(np.asarray(sweep['sweep_f'], dtype=float))
    z = (np.atleast_2d(np.asarray(sweep['sweep_i'], dtype=float))
         + 1j * np.atleast_2d(np.asarray(sweep['sweep_q'], dtype=float)))
    if sweep.get('wideband_sweep', False) or f.shape[0] == 1:
        return [(f[0], np.abs(z[0]))]
    return [(f[:, j], np.abs(z[:, j])) for j in range(f.shape[1])]


def _draw_list(ax, source, traces, invert=False):
    """One list's S21 (or bare ticks) with a marker at every resonance."""
    if traces:
        for trace_f, trace_mag in traces:
            good = np.isfinite(trace_mag) & (trace_mag > 0)
            ax.plot(trace_f[good] / 1e6, 20 * np.log10(trace_mag[good]),
                    lw=0.5, color='0.4')
        ax.set_ylabel('|S21| (dB)')
    else:
        ax.set_yticks([])
        ax.set_ylim(0, 1)
    low, high = ax.get_ylim()
    ax.vlines(source.frequency / 1e6, low, low + 0.12 * (high - low),
              color='#4c72b0', lw=0.8)
    if invert:
        ax.invert_yaxis()


def plot_match(match, f_range=None, ax=None, figsize=(12, 6)):
    """The two lists, one above the other, with matched devices joined up.

    The plot to look at first.  Crossed links, devices with no counterpart,
    and any rescue that claims a device moved a long way are all visible as
    shapes rather than numbers.  Zoom in (or pass ``f_range``) to read
    individual pairings.

    Parameters
    ----------
    match : ResonanceMatch
    f_range : tuple or None, optional
        ``(f_min, f_max)`` in Hz to show.  Default shows everything.
    ax : matplotlib.axes.Axes or None, optional
        Ignored -- this plot builds its own three-panel layout.
    figsize : tuple, optional

    Returns
    -------
    matplotlib.figure.Figure
    """
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(
        3, 1, figsize=figsize, sharex=True,
        gridspec_kw={'height_ratios': [3, 1, 3], 'hspace': 0.05})
    top, link, bottom = axes
    _draw_list(top, match.a, _sweep_traces(match.a))
    _draw_list(bottom, match.b, _sweep_traces(match.b), invert=True)

    link.set_ylim(0, 1)
    link.set_yticks([])
    for spine in link.spines.values():
        spine.set_visible(False)
    for row in match.pairs:
        if row['a_index'] < 0 or row['b_index'] < 0:
            lonely = row['f_a'] if row['a_index'] >= 0 else row['f_b']
            y = 1.0 if row['a_index'] >= 0 else 0.0
            link.plot([lonely / 1e6], [y], marker='x', ms=5,
                      color='#c44e52' if row['flag'] != 'out_of_range' else '0.7')
            continue
        colour = _FLAG_COLOURS.get(row['flag'], '#4c72b0')
        link.plot([row['f_a'] / 1e6, row['f_b'] / 1e6], [1, 0],
                  color=colour, lw=1.4 if row['flag'] != 'ok' else 0.6,
                  alpha=0.9 if row['flag'] != 'ok' else 0.5)

    seen = [f for f in np.unique(match.pairs['flag']) if f in _FLAG_COLOURS]
    if seen:
        link.legend(handles=[plt.Line2D([], [], color=_FLAG_COLOURS[f], label=f)
                             for f in seen],
                    loc='center left', bbox_to_anchor=(1.0, 0.5),
                    frameon=False, fontsize='small')
    name_a, name_b = _source_labels(match)
    top.set_title(f"{name_a} ({len(match.a)})  ->  {name_b} ({len(match.b)}):  "
                  f"{len(match.matched)} matched")
    bottom.set_xlabel('frequency (MHz)')
    if f_range is not None:
        top.set_xlim(f_range[0] / 1e6, f_range[1] / 1e6)
    return fig


def plot_shift(match, ax=None, figsize=(9, 5)):
    """Fractional frequency shift against frequency, with the fitted model.

    This is where you check the physics rather than the bookkeeping.  A flat
    band of points means the whole array moved together, as a loading change
    or ageing would do.  Structure means the global model is the wrong
    shape.  A tight core with a sparse tail is the signature of a subset of
    devices being disturbed on their own -- trapped flux through the
    transition, say -- rather than of a bad match.

    Returns
    -------
    matplotlib.figure.Figure
    """
    fig, ax = _new_axes(ax, figsize=figsize)
    matched = match.matched
    if len(matched):
        for flag in np.unique(matched['flag']):
            rows = matched[matched['flag'] == flag]
            ax.plot(rows['f_a'] / 1e6, rows['df_frac'], 'o', ms=3.5,
                    color=_FLAG_COLOURS.get(flag, '0.5'), label=flag,
                    alpha=0.8)
        grid = np.linspace(matched['f_a'].min(), matched['f_a'].max(), 300)
        model = (apply_transform(match.transform, grid) - grid) / grid
        ax.plot(grid / 1e6, model, '-', color='k', lw=1.2,
                label=describe_transform(match.transform, match.a.frequency))
        ax.legend(frameon=False, fontsize='small')
    ax.axhline(0.0, color='0.8', lw=0.8, zorder=0)
    ax.set_xlabel('frequency in A (MHz)')
    ax.set_ylabel(r'$\Delta f / f$')
    ax.set_title('frequency shift between the two sweeps')
    return fig


def plot_quality(match, ax=None, figsize=(11, 4.5)):
    """Residuals and ambiguity, against the tolerance that was used.

    Left: how far matched pairs sit from the global model, with the
    tolerance drawn on.  A histogram that fills the tolerance means it is
    too tight to trust; one confined to the middle means there was room to
    spare.  Right: how close each match's runner-up was, so a handful of
    coin-flips in a dense patch can be told from a systematic problem.

    Returns
    -------
    matplotlib.figure.Figure
    """
    import matplotlib.pyplot as plt

    if ax is None:
        fig, axes = plt.subplots(1, 2, figsize=figsize)
    else:
        fig, axes = ax.get_figure(), np.atleast_1d(ax)
    matched = match.matched
    left = axes[0]
    if len(matched):
        left.hist(matched['residual_hz'] / 1e3, bins=40, color='#4c72b0')
    for sign in (-1, 1):
        left.axvline(sign * match.tolerance / 1e3, color='#c44e52', ls='--',
                     lw=1.0, label='tolerance' if sign > 0 else None)
    left.set_xlabel('residual (kHz)')
    left.set_ylabel('devices')
    left.legend(frameon=False, fontsize='small')
    left.set_title('residual about the global shift')

    if len(axes) > 1:
        right = axes[1]
        if len(matched):
            for flag in np.unique(matched['flag']):
                rows = matched[matched['flag'] == flag]
                right.plot(rows['f_a'] / 1e6, rows['ambiguity'], 'o', ms=3.5,
                           color=_FLAG_COLOURS.get(flag, '0.5'), label=flag,
                           alpha=0.8)
            right.legend(frameon=False, fontsize='small')
        right.axhline(match.settings.get('ambiguity_threshold', DEFAULT_AMBIGUITY),
                      color='#c44e52', ls='--', lw=1.0)
        right.set_ylim(-0.05, 1.05)
        right.set_xlabel('frequency in A (MHz)')
        right.set_ylabel('ambiguity (0 clean, 1 a tie)')
        right.set_title('how close the runner-up was')
    fig.tight_layout()
    return fig


def plot_compare(match, a_values, b_values=None, label=None, mode='ratio',
                 log=None, ax=None, figsize=(11, 4.5)):
    """Compare any per-resonance quantity between the two sweeps.

    Takes the same arguments as :py:meth:`ResonanceMatch.compare`: a
    parameter name present in both lists, or two arrays in each list's own
    input order.

    Parameters
    ----------
    match : ResonanceMatch
    a_values, b_values : str or array-like
        What to compare.  See :py:meth:`ResonanceMatch.compare`.
    label : str or None, optional
        Axis label; defaults to the parameter name.
    mode : {'ratio', 'delta'}, optional
        Whether the right-hand panel shows B/A or B-A.
    log : bool or None, optional
        Log axes.  Default decides from the spread of the values.
    ax : matplotlib.axes.Axes or None, optional

    Returns
    -------
    matplotlib.figure.Figure
    """
    import matplotlib.pyplot as plt

    if ax is None:
        fig, axes = plt.subplots(1, 2, figsize=figsize)
    else:
        fig, axes = ax.get_figure(), np.atleast_1d(ax)
    if label is None:
        label = a_values if isinstance(a_values, str) else 'value'
    va, vb = match.compare(a_values, b_values, on='matched')
    good = np.isfinite(va) & np.isfinite(vb)
    if log is None:
        positive = good & (va > 0) & (vb > 0)
        log = bool(positive.sum() > 0.9 * good.sum() and positive.sum()
                   and np.nanmax(va[positive]) / np.nanmin(va[positive]) > 20)

    left = axes[0]
    left.plot(va[good], vb[good], 'o', ms=3.5, color='#4c72b0', alpha=0.7)
    if good.any():
        lim = [np.nanmin([va[good].min(), vb[good].min()]),
               np.nanmax([va[good].max(), vb[good].max()])]
        left.plot(lim, lim, '-', color='0.6', lw=1.0, label='unchanged')
        left.legend(frameon=False, fontsize='small')
    if log:
        left.set_xscale('log')
        left.set_yscale('log')
    name_a, name_b = _source_labels(match)
    left.set_xlabel(f'{label} in {name_a}')
    left.set_ylabel(f'{label} in {name_b}')
    left.set_title(f'{label}, device by device')

    if len(axes) > 1:
        right = axes[1]
        f_a = match.matched['f_a']
        change = vb / va if mode == 'ratio' else vb - va
        right.plot(f_a[good] / 1e6, change[good], 'o', ms=3.5,
                   color='#4c72b0', alpha=0.7)
        reference = 1.0 if mode == 'ratio' else 0.0
        right.axhline(reference, color='0.6', lw=1.0)
        if good.sum() > 2:
            right.axhline(float(np.nanmedian(change[good])), color='#c44e52',
                          ls='--', lw=1.0,
                          label=f'median {np.nanmedian(change[good]):.3g}')
            right.legend(frameon=False, fontsize='small')
        if log and mode == 'ratio':
            right.set_yscale('log')
        right.set_xlabel('frequency (MHz)')
        right.set_ylabel(f'{label} {"B/A" if mode == "ratio" else "B - A"}')
        right.set_title(f'change in {label} across the band')
    fig.tight_layout()
    return fig


def plot_pair(match, a_index, span=None, ax=None, figsize=(9, 5)):
    """The two raw sweep traces around one matched device, overlaid.

    The check to make before believing a surprising match -- a rescue, or
    anything flagged ambiguous.  Needs both lists to have come from sweeps.

    Parameters
    ----------
    match : ResonanceMatch
    a_index : int
        Entry of A, in the order you passed it in.
    span : float or None, optional
        Width in Hz to show.  Defaults to forty linewidths.

    Returns
    -------
    matplotlib.figure.Figure
    """
    traces_a, traces_b = _sweep_traces(match.a), _sweep_traces(match.b)
    if not traces_a or not traces_b:
        raise ValueError(
            "plot_pair needs the underlying sweeps; build the match from "
            "parsed sweeps, or pass sweep= when constructing the lists.")
    fig, ax = _new_axes(ax, figsize=figsize)
    b_index = match.index[a_index]
    f_a = float(match.a.frequency[np.where(match.a.order == a_index)[0][0]])
    f_b = float(match.b.frequency[np.where(match.b.order == b_index)[0][0]]) \
        if b_index >= 0 else np.nan
    if span is None:
        widths = [w for w in (match.a.linewidth, match.b.linewidth)
                  if np.isfinite(w)]
        span = 40 * max(widths) if widths else 40 * match.tolerance

    name_a, name_b = _source_labels(match)
    for traces, centre, name, colour in (
            (traces_a, f_a, name_a, '#4c72b0'),
            (traces_b, f_b, name_b, '#dd8452')):
        if not np.isfinite(centre):
            continue
        for trace_f, trace_mag in traces:
            window = np.abs(trace_f - centre) < span / 2
            if window.sum() < 3:
                continue
            mag = trace_mag[window]
            good = np.isfinite(mag) & (mag > 0)
            # Plotted against detuning so the two sit on top of each other
            # however far the device moved.
            ax.plot((trace_f[window][good] - centre) / 1e3,
                    20 * np.log10(mag[good]) - np.nanmedian(20 * np.log10(mag[good])),
                    lw=1.0, color=colour,
                    label=f'{name} @ {centre / 1e6:.4f} MHz')
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), frameon=False, fontsize='small')
    ax.set_xlabel('detuning from each fitted centre (kHz)')
    ax.set_ylabel('|S21| (dB, baseline removed)')
    ax.set_title(f"A[{a_index}] -> B[{b_index}]"
                 + (f"   df/f = {(f_b / f_a - 1):+.4g}" if b_index >= 0 else
                    "   (unmatched)"))
    return fig


def match_index(a, b, **kwargs):
    """The mapping from list A to list B, and nothing else.

    Parameters
    ----------
    a, b : any
        Two resonance lists, in any form :py:func:`as_resonance_list` takes.
    **kwargs
        Passed to :py:func:`match_resonances`.

    Returns
    -------
    numpy.ndarray
        ``index[i]`` is the entry of B matching entry ``i`` of A, or ``-1``
        where there is none.  Indices refer to the lists as passed in.

    Examples
    --------
    >>> idx = match_index('cold.resonances', 'warm.resonances')
    >>> qi_warm_like_cold = np.where(idx >= 0, qi_warm[idx], np.nan)
    """
    kwargs.setdefault('verbose', False)
    return match_resonances(a, b, **kwargs).index
