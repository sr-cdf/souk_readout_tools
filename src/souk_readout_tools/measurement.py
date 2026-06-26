"""Parameter-series measurement tools.

Repeat a readout measurement across an external parameter and save the results
as they complete.  The external parameter is abstract: you supply small
callbacks to set and/or read it (a cryostat temperature, an attenuator, a bias
voltage, ...), and a ``measure_func(client)`` that acquires the data at each
point.  All callbacks receive the active ``client`` as their first argument so
the same functions work for any board; ignore it when the parameter is an
external instrument.  ``run(..., client=...)`` overrides the client per run,
so one tool can be re-pointed at different boards/pipelines.

Four tools, one shared engine:

- :class:`ParameterSeries`      step through a list of values you control.
- :class:`ParameterGrid`        nest several axes into a cartesian grid.
- :class:`TimedMeasurement`     measure on a clock (every N seconds / for a time).
- :class:`ConditionalMeasurement`  measure when a monitored value meets a target.

Each run writes a JSON manifest (rewritten after every step, so it is always
inspectable and survives a crash) plus, optionally, one data file per step and
a live ``summary.csv`` digest.  Read a finished run back with :func:`load_run`
and plot its summary with :func:`plot_run_summary`.

The ``measure_func`` return value decides what is stored per step:

- a ``dict``  -> saved as a ``.npz`` (or via your ``save_func``),
- ``None``    -> nothing is saved (the step is still logged).

This module is also the home of the small JSON/npz manifest helpers shared with
:mod:`souk_readout_tools.power_sweep`.
"""

from __future__ import annotations

import csv
import json
import os
import shutil
import sys
import time
import traceback
import warnings
from dataclasses import dataclass, field
from itertools import product
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np


MANIFEST_FILE = "measurement.json"
SUMMARY_FILE = "summary.csv"


__all__ = [
    "MANIFEST_FILE",
    "SUMMARY_FILE",
    "MeasurementPoint",
    "SeriesAxis",
    "ParameterSeries",
    "ParameterGrid",
    "TimedMeasurement",
    "ConditionalMeasurement",
    "LoadedRun",
    "load_run",
    "plot_run_summary",
]


# ---------------------------------------------------------------------------
# Shared serialisation / IO helpers (also imported by power_sweep).
# ---------------------------------------------------------------------------


def _timestamp():
    return time.strftime("%Y-%m-%d %H:%M:%S %z")


def _format_duration(seconds):
    """Format a duration compactly for progress output."""
    if seconds is None or not np.isfinite(seconds):
        return "n/a"
    seconds = float(seconds)
    if seconds < 1.0:
        return f"{seconds * 1e3:.0f} ms"
    if seconds < 60.0:
        return f"{seconds:.1f} s"
    minutes, rem = divmod(seconds, 60.0)
    return f"{int(minutes)}m{rem:04.1f}s"


def _jsonify(value):
    """Make numpy/Path values JSON-serialisable (recursively); non-finite
    floats become ``None`` so the manifest is always valid JSON."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonify(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonify(item) for item in value]
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    return value


def _write_manifest(manifest, root, filename=MANIFEST_FILE):
    """Write the run ``manifest`` dict to ``<root>/<filename>``."""
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    with (root / filename).open("w", encoding="utf-8") as handle:
        json.dump(_jsonify(manifest), handle, indent=2)


def _load_manifest(path, filename=MANIFEST_FILE):
    """Load a run manifest from a run directory or a manifest file ``path``.

    ``root`` is rewritten to the manifest's own directory so a run that has
    been moved still resolves its data files correctly.
    """
    path = Path(path).resolve()
    manifest_path = path / filename if path.is_dir() else path
    with manifest_path.open("r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    manifest["root"] = str(manifest_path.parent)
    return manifest


def _save_npz(path, data):
    """Save a ``data`` mapping to ``.npz``: array values are stored directly,
    everything else is JSON-encoded into a single ``_metadata_json`` entry.
    Returns ``path`` as a string."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays, metadata = {}, {}
    for key, value in data.items():
        if isinstance(value, np.ndarray):
            arrays[str(key)] = value
        elif isinstance(value, np.generic):
            metadata[str(key)] = value.item()
        else:
            metadata[str(key)] = _jsonify(value)
    arrays["_metadata_json"] = np.array(json.dumps(metadata))
    np.savez_compressed(path, **arrays)
    return str(path)


def _load_npz(path):
    """Load an ``.npz`` written by :func:`_save_npz`, merging the arrays and the
    decoded ``_metadata_json`` back into one dict."""
    with np.load(path, allow_pickle=False) as archive:
        data = {}
        if "_metadata_json" in archive:
            data.update(json.loads(str(archive["_metadata_json"].item())))
        for key in archive.files:
            if key != "_metadata_json":
                data[key] = archive[key].copy()
    return data


# ---------------------------------------------------------------------------
# Resource estimation (best-effort; never blocks a run).
# ---------------------------------------------------------------------------

# Default per-run memory budget: a run may keep step data in RAM up to this
# fraction of *total* system memory before it starts offloading to disk.  Lower
# it (per tool, via ``memory_fraction=``) when several runs share a machine, so
# the budgets add up to something safe - e.g. ``memory_fraction=0.05`` each.
_DEFAULT_MEMORY_FRACTION = 0.8


def _validate_memory_fraction(fraction):
    if not 0 < fraction <= 1:
        raise ValueError("memory_fraction must be in (0, 1]")
    return fraction


def _available_memory_bytes():
    """Best-effort available system memory in bytes, or ``None`` if unknown."""
    try:
        import psutil
        return int(psutil.virtual_memory().available)
    except Exception:                              # noqa: BLE001 - optional dependency
        pass
    try:
        return os.sysconf("SC_AVPHYS_PAGES") * os.sysconf("SC_PAGE_SIZE")
    except (ValueError, AttributeError, OSError):  # not POSIX / unsupported
        return None


def _total_memory_bytes():
    """Best-effort total system memory in bytes, or ``None`` if unknown."""
    try:
        import psutil
        return int(psutil.virtual_memory().total)
    except Exception:                              # noqa: BLE001 - optional dependency
        pass
    try:
        return os.sysconf("SC_PHYS_PAGES") * os.sysconf("SC_PAGE_SIZE")
    except (ValueError, AttributeError, OSError):  # not POSIX / unsupported
        return None


def _free_disk_bytes(path):
    """Free disk space in bytes at ``path``, or ``None`` if it can't be read."""
    try:
        return shutil.disk_usage(path).free
    except OSError:
        return None


def _path_size(path):
    """Size of a file in bytes, or 0 if it can't be read."""
    try:
        return Path(path).stat().st_size
    except OSError:
        return 0


def _data_nbytes(data):
    """Rough in-memory size of a ``measure_func`` result (arrays dominate)."""
    if not isinstance(data, dict):
        return sys.getsizeof(data) if data is not None else 0
    total = 0
    for value in data.values():
        if isinstance(value, np.ndarray):
            total += value.nbytes
        else:
            total += sys.getsizeof(value)
    return total


def _format_bytes(n):
    if n is None:
        return "unknown"
    size = float(n)
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if size < 1024 or unit == "TB":
            return f"{size:.1f} {unit}"
        size /= 1024


def _fits_in_memory(data, headroom=_DEFAULT_MEMORY_FRACTION):
    """True if ``data`` fits within ``headroom`` of currently free memory.
    Returns False when free memory can't be read, so callers don't risk it."""
    avail = _available_memory_bytes()
    if avail is None:
        return False
    return _data_nbytes(data) <= avail * headroom


# ---------------------------------------------------------------------------
# Data model.
# ---------------------------------------------------------------------------


@dataclass
class MeasurementPoint:
    """One point in a run: the coordinate, the data, and where it was saved.

    ``axis`` is the coordinate ``{name: value}`` (the value you set, or the
    value observed for monitor-only tools).  ``readback`` holds any values read
    back from the hardware.  ``data`` is whatever ``measure_func`` returned.
    """

    index: int
    axis: Dict[str, Any]
    readback: Dict[str, Any]
    timestamp: float
    data: Optional[dict]
    data_file: Optional[str] = None
    plot_file: Optional[str] = None
    summary: Optional[dict] = None
    attempts: int = 1
    status: str = "success"
    error: Optional[str] = None

    # Convenience accessors for the common single-axis case.
    @property
    def parameter_name(self) -> Optional[str]:
        return next(iter(self.axis), None)

    @property
    def value_set(self) -> Any:
        return next(iter(self.axis.values()), None)

    @property
    def parameter_value(self) -> Any:
        """Readback of the first axis if available, else the value set."""
        name = self.parameter_name
        if name is not None and name in self.readback:
            return self.readback[name]
        return self.axis.get(name)


@dataclass
class SeriesAxis:
    """One axis of a :class:`ParameterGrid`.

    ``set_parameter(client, value)`` applies a value (optional for monitor-only
    axes); ``get_parameter(client) -> value`` reads it back (optional);
    ``settle_time`` is the pause in seconds after this axis changes before
    measuring.  The ``client`` is passed in so the same axis works for any
    board; ignore it for an external instrument.
    """

    name: str
    values: Sequence[Any]
    set_parameter: Optional[Callable[[Any], Any]] = None
    get_parameter: Optional[Callable[[], Any]] = None
    settle_time: float = 0.0


# ---------------------------------------------------------------------------
# Run context: the shared per-step engine (save + manifest + summary + print).
# ---------------------------------------------------------------------------


_KIND_PREFIX = {
    "parameter_series": "series",
    "parameter_grid": "grid",
    "timed_measurement": "timed",
    "conditional_measurement": "cond",
}


def _auto_run_name(kind, names):
    prefix = _KIND_PREFIX.get(kind, "run")
    stamp = time.strftime("%Y%m%d-%H%M%S")
    tag = "_".join(str(n) for n in names) if names else ""
    return "_".join(part for part in (prefix, tag, stamp) if part)


def _unique_dir(target: Path) -> Path:
    """Return ``target`` if free (missing or empty), else ``target_002`` etc."""
    if not target.exists() or not any(target.iterdir()):
        return target
    i = 2
    while True:
        candidate = target.parent / f"{target.name}_{i:03d}"
        if not candidate.exists() or not any(candidate.iterdir()):
            return candidate
        i += 1


def _clear_dir(target: Path) -> None:
    for child in target.iterdir():
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()


def _short_error(error: Optional[str]) -> str:
    if not error:
        return ""
    lines = [line for line in error.strip().splitlines() if line.strip()]
    return lines[-1] if lines else ""


def _fmt_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


class _RunContext:
    """Owns the on-disk state for one run and the shared per-step machinery."""

    def __init__(self, client, kind, root, run_name, manifest_name,
                 save_func, plot_func, summarise_func,
                 retries, on_error, verbose,
                 expected_steps=None, estimated_step_bytes=None,
                 memory_fraction=_DEFAULT_MEMORY_FRACTION):
        if on_error not in ("raise", "skip"):
            raise ValueError("on_error must be 'raise' or 'skip'")
        self.client = client
        self.kind = kind
        self.root = root                      # Path, or None for in-memory mode
        self.run_name = run_name
        self.manifest_name = manifest_name
        self.save_func = save_func
        self.plot_func = plot_func
        self.summarise_func = summarise_func
        self.retries = int(retries)
        self.on_error = on_error
        self.verbose = verbose

        self.points: List[MeasurementPoint] = []
        self.manifest: Optional[dict] = None
        self.completed: Dict[int, dict] = {}  # index -> step dict (from resume)
        self.t_start = time.time()
        self.step_times: List[float] = []

        # Resource guard state.
        self.expected_steps = expected_steps
        self.estimated_step_bytes = estimated_step_bytes
        self.memory_fraction = _validate_memory_fraction(memory_fraction)
        self.free_memory = False              # drop in-RAM data after saving
        self._max_step_mem = 0
        self._max_step_disk = 0
        self._memory_assessed = False
        self._run_ram = 0                     # bytes this run currently holds in RAM
        self._run_disk = 0                    # bytes this run has written to disk

    # -- lifecycle ------------------------------------------------------

    def start(self, parameters, extra):
        self.manifest = {
            "kind": self.kind,
            "run_name": self.run_name,
            "root": str(self.root) if self.root else None,
            "created": _timestamp(),
            "finished": None,
            "status": "running",
            "parameters": _jsonify(parameters),
            "steps": [],
        }
        self.manifest.update(extra or {})
        self._flush()

    def adopt_resumed(self, steps):
        """Pre-load already-completed (successful) steps from a prior run."""
        for step in steps:
            if step.get("status") == "success":
                index = int(step["index"])
                self.completed[index] = step
                self.points.append(_point_from_step(step))

    def completed_indices(self):
        return set(self.completed)

    def completed_count(self):
        return len(self.completed)

    def note_skip(self, index, axis):
        prior = self.completed.get(index)
        if prior is not None and prior.get("axis") not in (None, axis):
            warnings.warn(
                f"resume: step {index} was recorded at {prior.get('axis')} but "
                f"the plan now asks for {axis}; keeping the recorded result."
            )
        if self.verbose:
            coord = _format_coord(axis)
            print(f"[{index + 1}] {coord}  (already done - skipping)")

    # -- the per-step engine -------------------------------------------

    def record_step(self, index, axis, readback, measure_func,
                    total=None, remaining=None, eta_seconds=None):
        t0 = time.time()
        data, attempts, status, error, exc = self._measure(measure_func)
        dt = time.time() - t0
        self.step_times.append(dt)

        point = MeasurementPoint(
            index=index, axis=dict(axis), readback=dict(readback),
            timestamp=time.time(), data=data, attempts=attempts,
            status=status, error=error,
        )
        step_mem = _data_nbytes(data)

        if status == "success" and data is not None:
            point.data_file = self._save_data(index, data)
        if status == "success":
            point.plot_file = self._make_plot(index, point)
            point.summary = self._summarise(point)

        self.points.append(point)
        self._run_ram += step_mem             # this point now holds its data in RAM
        if eta_seconds is None and remaining is not None and self.step_times:
            eta_seconds = float(np.mean(self.step_times)) * remaining
        self._flush()
        self._print_step(point, total, eta_seconds, dt)
        self._check_resources(point)          # may free all retained data (-> _run_ram 0)
        if self.free_memory and point.data_file and point.data is not None:
            point.data = None                 # offloaded to disk; free the RAM
            self._run_ram -= step_mem

        if status == "failed" and self.on_error == "raise":
            self._finalise("failed")
            raise exc
        return point

    def _measure(self, measure_func):
        """Call ``measure_func`` with retries.  Returns
        ``(data, attempts, status, error, exception)``."""
        last_exc = None
        last_tb = ""
        for attempt in range(1, self.retries + 2):
            try:
                return measure_func(self.client), attempt, "success", None, None
            except KeyboardInterrupt:
                raise
            except Exception as exc:           # noqa: BLE001 - reported, not hidden
                last_exc = exc
                last_tb = traceback.format_exc()
                if attempt <= self.retries and self.verbose:
                    print(f"      attempt {attempt} failed: {exc!r} - retrying...")
        return None, self.retries + 1, "failed", last_tb.strip(), last_exc

    def _save_data(self, index, data):
        if self.root is None:
            return None
        rel = f"data/{self.run_name}_step{index:03d}.npz"
        path = self.root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        if self.save_func is not None:
            self.save_func(data, path)
        else:
            _save_npz(path, data)
        self._run_disk += _path_size(path)
        return rel

    def _make_plot(self, index, point):
        if self.plot_func is None:
            return None
        if self.root is None:
            warnings.warn("plot_func is ignored when save_dir is None")
            return None
        rel = f"plots/{self.run_name}_step{index:03d}.png"
        path = self.root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self.plot_func(point, path)
        except Exception as exc:               # noqa: BLE001 - a bad plot must not kill a run
            warnings.warn(f"plot_func failed at step {index}: {exc!r}")
            return None
        self._run_disk += _path_size(path)
        return rel

    def _summarise(self, point):
        if self.summarise_func is None:
            return None
        try:
            summary = dict(self.summarise_func(point))
        except Exception as exc:               # noqa: BLE001
            warnings.warn(f"summarise_func failed at step {point.index}: {exc!r}")
            return None
        return summary

    # -- resource guard -------------------------------------------------

    def _memory_limit(self):
        """This run's RAM budget in bytes: the smaller of its share of *total*
        system memory (``memory_fraction``) and what is actually free now.  The
        fraction keeps concurrent runs on many boards from adding up to an
        out-of-memory; the live-free floor keeps a single run safe on a busy
        machine.  ``None`` if memory can't be read."""
        candidates = []
        total = _total_memory_bytes()
        if total is not None:
            candidates.append(self.memory_fraction * total)
        avail = _available_memory_bytes()
        if avail is not None:
            candidates.append(avail)
        return min(candidates) if candidates else None

    def preflight(self):
        """Before any step: if the caller gave a per-step size estimate, warn
        about a run that looks too big for memory/disk.  Always continues."""
        per_step = self.estimated_step_bytes
        if not per_step or not self.expected_steps:
            return
        projected = per_step * self.expected_steps
        if self.root is None:
            limit = self._memory_limit()
            if limit is not None and projected > limit:
                warnings.warn(
                    f"estimated ~{_format_bytes(projected)} for "
                    f"{self.expected_steps} steps may exceed this run's "
                    f"~{_format_bytes(limit)} memory budget, and save_dir is None "
                    f"so nothing can be offloaded; continuing anyway.")
            return
        free = _free_disk_bytes(self.root)
        if free is not None and projected > free:
            warnings.warn(
                f"estimated ~{_format_bytes(projected)} for {self.expected_steps} "
                f"steps may exceed ~{_format_bytes(free)} free disk; continuing "
                f"anyway.")

    def _check_resources(self, point):
        """After a step: assess memory once and disk every step from the
        observed per-step sizes.  Never blocks; warns and adapts instead."""
        if point.status != "success":
            return
        self._max_step_mem = max(self._max_step_mem, _data_nbytes(point.data))
        if point.data_file and self.root is not None:
            self._max_step_disk = max(
                self._max_step_disk, _path_size(self.root / point.data_file))
        if not self._memory_assessed and self._max_step_mem > 0:
            self._memory_assessed = True
            self._assess_memory()
        if self.root is not None and self._max_step_disk > 0:
            self._check_disk()

    def _completed(self):
        return sum(1 for p in self.points if p.status == "success")

    def _assess_memory(self):
        """Once, after the first sized step: if keeping every step in RAM would
        exceed this run's budget, switch to freeing each step to disk (or warn if
        we can't, because nothing is being saved)."""
        total = self.expected_steps
        limit = self._memory_limit()
        if not total or limit is None:
            return
        projected = self._max_step_mem * total
        if projected <= limit:
            return
        budget = _format_bytes(limit)
        if self.root is not None:
            self.free_memory = True
            for p in self.points:             # release the steps already held
                if p.data_file:
                    p.data = None
            self._run_ram = 0
            if self.verbose:
                print(f"  [memory] projected ~{_format_bytes(projected)} for "
                      f"{total} steps exceeds this run's ~{budget} budget "
                      f"(memory_fraction={self.memory_fraction:g}); freeing each "
                      f"step after saving (reload with load_run).")
        else:
            warnings.warn(
                f"projected memory ~{_format_bytes(projected)} for {total} steps "
                f"may exceed this run's ~{budget} budget (memory_fraction="
                f"{self.memory_fraction:g}), and save_dir is None so nothing can "
                f"be freed; continuing anyway.")

    def _check_disk(self):
        """Every step: if the remaining steps won't fit on disk, notify (so the
        warning repeats each step until either the run ends or the disk fills)."""
        free = _free_disk_bytes(self.root)
        if free is None:
            return
        remaining = max((self.expected_steps or 0) - self._completed(), 0)
        needed = self._max_step_disk * remaining
        if needed > free:
            print(f"  [disk] WARNING: ~{_format_bytes(needed)} needed for "
                  f"{remaining} remaining step(s) but only ~{_format_bytes(free)} "
                  f"free - the run may not finish.")

    def _resource_line(self, point):
        """A per-step line showing what *this run* is using - RAM currently held
        and disk written so far (with the projected run total) - plus system free
        space.  Printed every step so usage is always visible."""
        size = 0
        if point.data_file and self.root is not None:
            size = _path_size(self.root / point.data_file)
        elif point.data is not None:
            size = _data_nbytes(point.data)
        parts = []
        if size:
            parts.append(f"step {_format_bytes(size)}")

        usage = [f"{_format_bytes(self._run_ram)} RAM"]
        if self.root is not None:
            disk_use = _format_bytes(self._run_disk)
            if self._max_step_disk and self.expected_steps:
                projected = self._max_step_disk * self.expected_steps
                disk_use += f" of ~{_format_bytes(projected)}"
            usage.append(f"{disk_use} disk")
        parts.append("run using " + ", ".join(usage))

        free = []
        mem = _available_memory_bytes()
        if mem is not None:
            free.append(f"{_format_bytes(mem)} mem")
        disk = _free_disk_bytes(self.root if self.root is not None else ".")
        if disk is not None:
            free.append(f"{_format_bytes(disk)} disk")
        if free:
            parts.append("free " + ", ".join(free))
        return "      " + " · ".join(parts)

    # -- writing --------------------------------------------------------

    def _flush(self):
        if self.root is None or self.manifest is None:
            return
        ordered = sorted(self.points, key=lambda p: p.index)
        self.manifest["steps"] = [_step_dict(p) for p in ordered]
        _write_manifest(self.manifest, self.root, self.manifest_name)
        self._write_summary_csv(ordered)

    def _write_summary_csv(self, ordered):
        rows = [p for p in ordered if p.summary]
        if not rows:
            return
        axis_keys: List[str] = []
        for p in ordered:
            for key in p.axis:
                if key not in axis_keys:
                    axis_keys.append(key)
        sum_keys: List[str] = []
        for p in rows:
            for key in p.summary:
                if key not in sum_keys:
                    sum_keys.append(key)
        header = ["index"] + axis_keys + sum_keys
        with (self.root / SUMMARY_FILE).open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(header)
            for p in rows:
                row = [p.index]
                row += [_jsonify(p.axis.get(k)) for k in axis_keys]
                row += [_jsonify(p.summary.get(k)) for k in sum_keys]
                writer.writerow(row)

    def _finalise(self, status):
        if self.manifest is None:
            return
        self.manifest["status"] = status
        self.manifest["finished"] = _timestamp()
        self._flush()

    # -- console output -------------------------------------------------

    def _print_step(self, point, total, eta_seconds, dt):
        if not self.verbose:
            return
        head = f"[{point.index + 1}" + (f"/{total}" if total else "") + "]"
        coord = _format_coord(point.axis, point.readback)
        if point.status == "failed":
            print(f"{head} {coord}  FAILED after {point.attempts} "
                  f"attempt(s): {_short_error(point.error)}")
        else:
            print(f"{head} {coord}  done in {_format_duration(dt)}")
        elapsed = time.time() - self.t_start
        tail = f"      elapsed {_format_duration(elapsed)}"
        if eta_seconds is not None and np.isfinite(eta_seconds) and eta_seconds > 0:
            finish = time.strftime("%H:%M", time.localtime(time.time() + eta_seconds))
            tail += f" · ETA ~{_format_duration(eta_seconds)} · finish ~{finish}"
        print(tail)
        resources = self._resource_line(point)
        if resources:
            print(resources)
        if point.summary:
            summary = " · ".join(
                f"{k}={_fmt_value(v)}" for k, v in point.summary.items()
            )
            print(f"      {summary}")

    def finish(self, status="success"):
        self._finalise(status)
        if not self.verbose:
            return self.points
        total = _format_duration(time.time() - self.t_start)
        n = len(self.points)
        where = f" -> {self.root.resolve()}" if self.root else " (not saved)"
        label = {"success": "Done", "timeout": "Timed out",
                 "interrupted": "Interrupted"}.get(status, status.capitalize())
        print(f"{label}: {n} point(s) in {total}{where}")
        return self.points

    def interrupted(self):
        return self.finish(status="interrupted")


def _format_coord(axis, readback=None):
    parts = []
    for name, value in axis.items():
        text = f"{name}={_fmt_value(value)}"
        if readback and name in readback and readback[name] != value:
            text += f" (readback {_fmt_value(readback[name])})"
        parts.append(text)
    return ", ".join(parts)


def _step_dict(point: MeasurementPoint) -> dict:
    return {
        "index": int(point.index),
        "axis": _jsonify(point.axis),
        "readback": _jsonify(point.readback),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S",
                                   time.localtime(point.timestamp)),
        "epoch": float(point.timestamp),
        "status": point.status,
        "attempts": int(point.attempts),
        "data_file": point.data_file,
        "plot_file": point.plot_file,
        "summary": _jsonify(point.summary) if point.summary else None,
        "error": point.error,
    }


def _point_from_step(step: dict) -> MeasurementPoint:
    return MeasurementPoint(
        index=int(step["index"]),
        axis=dict(step.get("axis") or {}),
        readback=dict(step.get("readback") or {}),
        timestamp=float(step.get("epoch") or 0.0),
        data=None,
        data_file=step.get("data_file"),
        plot_file=step.get("plot_file"),
        summary=step.get("summary"),
        attempts=int(step.get("attempts", 1)),
        status=step.get("status", "success"),
        error=step.get("error"),
    )


def _start_run(client, kind, parameter_names, *, save_dir, run_name, manifest_name,
               save_func, plot_func, summarise_func, retries, on_error, verbose,
               resume, overwrite, parameters, extra,
               expected_steps=None, estimated_step_bytes=None,
               memory_fraction=_DEFAULT_MEMORY_FRACTION):
    """Resolve the destination folder (honouring resume/overwrite/auto-suffix),
    build the run context, and write the initial manifest."""
    if resume and overwrite:
        raise ValueError("resume and overwrite are mutually exclusive")

    root = None
    resumed_steps: List[dict] = []
    if save_dir is not None:
        run_name = run_name or _auto_run_name(kind, parameter_names)
        base = Path(save_dir).expanduser()
        requested = base / run_name
        target = requested
        if resume:
            if (target / manifest_name).exists():
                resumed_steps = _load_manifest(target, manifest_name).get("steps", [])
        elif overwrite:
            if target.exists():
                _clear_dir(target)
        else:
            target = _unique_dir(target)
        root = target
        run_name = root.name                  # file prefix follows the final folder
        (root / "data").mkdir(parents=True, exist_ok=True)
        if verbose:
            print(f"Saving to: {root.resolve()}")
            if root.resolve() != requested.resolve():
                print(f"  (requested '{requested}' was taken - using '{root.name}')")

    ctx = _RunContext(client, kind, root, run_name, manifest_name, save_func,
                      plot_func, summarise_func, retries, on_error, verbose,
                      expected_steps=expected_steps,
                      estimated_step_bytes=estimated_step_bytes,
                      memory_fraction=memory_fraction)
    ctx.start(parameters, extra)
    if resumed_steps:
        ctx.adopt_resumed(resumed_steps)
        if verbose and ctx.completed:
            print(f"Resuming: {len(ctx.completed)} step(s) already complete.")
    ctx.preflight()
    return ctx


# ---------------------------------------------------------------------------
# The four tools.
# ---------------------------------------------------------------------------


class ParameterSeries:
    """Step an external parameter through a list of values, measuring at each.

    Args:
        client: the default client passed to your callbacks and ``measure_func``
            (override per run with ``run(..., client=...)``).
        parameter_name: Human-readable name (e.g. ``'tx_attenuation_db'``).
        set_parameter: ``set_parameter(client, value)`` applies a value.
            Optional for a monitor-only parameter.  The ``client`` is passed in
            so the same callback works for any board; ignore it (``lambda c, v:
            instrument.set(v)``) if the parameter is an external instrument.
        get_parameter: ``get_parameter(client) -> value`` reads the value back.
            Optional if ``set_parameter`` is given.
        settle_time: Seconds to wait after setting before measuring.
    """

    def __init__(self, client, parameter_name,
                 set_parameter=None, get_parameter=None, settle_time=0.0,
                 memory_fraction=_DEFAULT_MEMORY_FRACTION):
        if set_parameter is None and get_parameter is None:
            raise ValueError(
                "provide at least one of set_parameter or get_parameter")
        self.client = client
        self.parameter_name = parameter_name
        self.set_parameter = set_parameter
        self.get_parameter = get_parameter
        self.settle_time = settle_time
        self.memory_fraction = _validate_memory_fraction(memory_fraction)

    def run(self, values, measure_func, *, client=None,
            save_dir=None, run_name=None, manifest_name=MANIFEST_FILE,
            save_func=None, plot_func=None, summarise_func=None,
            overwrite=False, resume=False, retries=0, on_error="raise",
            estimated_step_bytes=None, verbose=True):
        """Run the series.  ``client`` overrides the constructor's client, so
        one series can be re-pointed at another board.  See the module docstring
        for the saving contract.

        Returns a list of :class:`MeasurementPoint`.
        """
        client = self.client if client is None else client
        values = list(values)
        ctx = _start_run(
            client, "parameter_series", [self.parameter_name],
            save_dir=save_dir, run_name=run_name, manifest_name=manifest_name,
            save_func=save_func, plot_func=plot_func, summarise_func=summarise_func,
            retries=retries, on_error=on_error, verbose=verbose,
            resume=resume, overwrite=overwrite,
            parameters={self.parameter_name: values},
            extra={"parameter_name": self.parameter_name},
            expected_steps=len(values), estimated_step_bytes=estimated_step_bytes,
            memory_fraction=self.memory_fraction,
        )
        done = ctx.completed_indices()
        total = len(values)
        try:
            for i, value in enumerate(values):
                axis = {self.parameter_name: value}
                if i in done:
                    ctx.note_skip(i, axis)
                    continue
                readback = self._apply(client, value)
                ctx.record_step(i, axis, readback, measure_func,
                                total=total, remaining=total - (i + 1))
        except KeyboardInterrupt:
            return ctx.interrupted()
        return ctx.finish()

    def _apply(self, client, value):
        readback = {}
        if self.set_parameter is not None:
            self.set_parameter(client, value)
            if self.settle_time > 0:
                time.sleep(self.settle_time)
        if self.get_parameter is not None:
            readback[self.parameter_name] = self.get_parameter(client)
        return readback


class ParameterGrid:
    """Nest several :class:`SeriesAxis` into a cartesian grid.

    The first axis is the outermost (slowest) loop.  Only the axes whose value
    changes between adjacent points are re-applied, so a slow outer parameter
    (e.g. temperature) is not needlessly re-set on every inner step.
    """

    def __init__(self, client, axes, memory_fraction=_DEFAULT_MEMORY_FRACTION):
        self.client = client
        self.axes = list(axes)
        if not self.axes:
            raise ValueError("provide at least one SeriesAxis")
        self.memory_fraction = _validate_memory_fraction(memory_fraction)

    def run(self, measure_func, *, client=None,
            save_dir=None, run_name=None, manifest_name=MANIFEST_FILE,
            save_func=None, plot_func=None, summarise_func=None,
            overwrite=False, resume=False, retries=0, on_error="raise",
            estimated_step_bytes=None, verbose=True):
        """Run the grid.  ``client`` overrides the constructor's client.
        Returns a list of :class:`MeasurementPoint`."""
        client = self.client if client is None else client
        names = [axis.name for axis in self.axes]
        combos = list(product(*[list(axis.values) for axis in self.axes]))
        total = len(combos)
        ctx = _start_run(
            client, "parameter_grid", names,
            save_dir=save_dir, run_name=run_name, manifest_name=manifest_name,
            save_func=save_func, plot_func=plot_func, summarise_func=summarise_func,
            retries=retries, on_error=on_error, verbose=verbose,
            resume=resume, overwrite=overwrite,
            parameters={axis.name: list(axis.values) for axis in self.axes},
            extra={"axes": names},
            expected_steps=total, estimated_step_bytes=estimated_step_bytes,
            memory_fraction=self.memory_fraction,
        )
        done = ctx.completed_indices()
        prev: Dict[str, Any] = {}
        first_real = True
        try:
            for i, combo in enumerate(combos):
                axis = {axis.name: value for axis, value in zip(self.axes, combo)}
                if i in done:
                    ctx.note_skip(i, axis)
                    prev = axis
                    continue
                readback = self._apply(client, axis, prev, force=first_real)
                first_real = False
                prev = axis
                ctx.record_step(i, axis, readback, measure_func,
                                total=total, remaining=total - (i + 1))
        except KeyboardInterrupt:
            return ctx.interrupted()
        return ctx.finish()

    def _apply(self, client, axis, prev, force):
        # Set only the axes whose value changed (or all of them on the first
        # real step, when the hardware state is unknown).
        for ax in self.axes:
            value = axis[ax.name]
            if (force or prev.get(ax.name) != value) and ax.set_parameter is not None:
                ax.set_parameter(client, value)
                if ax.settle_time > 0:
                    time.sleep(ax.settle_time)
        readback = {}
        for ax in self.axes:
            if ax.get_parameter is not None:
                readback[ax.name] = ax.get_parameter(client)
        return readback


class TimedMeasurement:
    """Take measurements on a clock.

    Args:
        client: the default client passed to ``measure_func`` and
            ``get_parameter`` (override per run with ``run(..., client=...)``).
        parameter_name: name of the monitored value (default ``'time'``).
        get_parameter: ``get_parameter(client) -> value`` read at each point.
            Optional; when absent the elapsed time is logged as the coordinate.
        interval_s: seconds between the start of successive measurements.
    """

    def __init__(self, client, parameter_name="time",
                 get_parameter=None, interval_s=60.0,
                 memory_fraction=_DEFAULT_MEMORY_FRACTION):
        self.client = client
        self.parameter_name = parameter_name
        self.get_parameter = get_parameter
        self.interval_s = interval_s
        self.memory_fraction = _validate_memory_fraction(memory_fraction)

    def run(self, measure_func, *, n_points=None, duration_s=None, client=None,
            save_dir=None, run_name=None, manifest_name=MANIFEST_FILE,
            save_func=None, plot_func=None, summarise_func=None,
            overwrite=False, resume=False, retries=0, on_error="raise",
            estimated_step_bytes=None, verbose=True):
        """Run timed measurements.  Specify ``n_points`` or ``duration_s`` (or
        both - stops at whichever comes first).  ``client`` overrides the
        constructor's client."""
        if n_points is None and duration_s is None:
            raise ValueError("specify n_points or duration_s (or both)")
        client = self.client if client is None else client
        if n_points is not None:
            expected = n_points
        elif self.interval_s > 0:
            expected = int(np.ceil(duration_s / self.interval_s))
        else:
            expected = None
        ctx = _start_run(
            client, "timed_measurement", [self.parameter_name],
            save_dir=save_dir, run_name=run_name, manifest_name=manifest_name,
            save_func=save_func, plot_func=plot_func, summarise_func=summarise_func,
            retries=retries, on_error=on_error, verbose=verbose,
            resume=resume, overwrite=overwrite,
            parameters={"interval_s": self.interval_s, "n_points": n_points,
                        "duration_s": duration_s},
            extra={"parameter_name": self.parameter_name},
            expected_steps=expected, estimated_step_bytes=estimated_step_bytes,
            memory_fraction=self.memory_fraction,
        )
        start_index = ctx.completed_count()
        t_start = time.time()
        i = start_index
        try:
            while True:
                if n_points is not None and i >= n_points:
                    break
                elapsed = time.time() - t_start
                if duration_s is not None and elapsed >= duration_s:
                    break
                value = self.get_parameter(client) if self.get_parameter else round(elapsed, 3)
                readback = {self.parameter_name: value} if self.get_parameter else {}
                axis = {self.parameter_name: value}
                if n_points is not None:
                    total = n_points
                    eta = (n_points - (i + 1)) * self.interval_s
                else:
                    total = None
                    eta = duration_s - elapsed
                ctx.record_step(i, axis, readback, measure_func,
                                total=total, eta_seconds=eta)
                i += 1
                next_t = t_start + (i - start_index) * self.interval_s
                wait = next_t - time.time()
                if wait > 0:
                    time.sleep(wait)
        except KeyboardInterrupt:
            return ctx.interrupted()
        return ctx.finish()


class ConditionalMeasurement:
    """Measure when a monitored value meets a condition.

    Args:
        client: the default client passed to ``measure_func`` and
            ``get_parameter`` (override per run with ``run(..., client=...)``).
        parameter_name: name of the monitored value.
        get_parameter: ``get_parameter(client) -> value``.
        condition: ``condition(value, target) -> bool``; True triggers a
            measurement.  ``target`` is the target being tested in
            ``target_values`` mode, or ``None`` in ``n_points`` mode -
            e.g. ``lambda v, target: abs(v - target) < 5``.
        poll_interval_s: how often to check the condition (seconds).
    """

    def __init__(self, client, parameter_name, get_parameter, condition,
                 poll_interval_s=1.0, memory_fraction=_DEFAULT_MEMORY_FRACTION):
        self.client = client
        self.parameter_name = parameter_name
        self.get_parameter = get_parameter
        self.condition = condition
        self.poll_interval_s = poll_interval_s
        self.memory_fraction = _validate_memory_fraction(memory_fraction)

    def run(self, measure_func, *, target_values=None, n_points=None,
            timeout_s=None, client=None, save_dir=None, run_name=None,
            manifest_name=MANIFEST_FILE, save_func=None, plot_func=None,
            summarise_func=None, overwrite=False, resume=False, retries=0,
            on_error="raise", estimated_step_bytes=None, verbose=True):
        """Poll the parameter and measure when ``condition`` is met.

        Specify ``target_values`` (measure once per target) or ``n_points``
        (measure whenever the condition is met, that many times).  In
        ``target_values`` mode the targets are matched **in any order** -
        whichever target the value satisfies first is captured first - so a
        drifting quantity (e.g. temperature) need not pass them in order.
        ``client`` overrides the constructor's client.
        """
        if (target_values is None) == (n_points is None):
            raise ValueError("specify exactly one of target_values or n_points")
        client = self.client if client is None else client
        expected = len(target_values) if target_values is not None else n_points
        ctx = _start_run(
            client, "conditional_measurement", [self.parameter_name],
            save_dir=save_dir, run_name=run_name, manifest_name=manifest_name,
            save_func=save_func, plot_func=plot_func, summarise_func=summarise_func,
            retries=retries, on_error=on_error, verbose=verbose,
            resume=resume, overwrite=overwrite,
            parameters={"target_values": target_values, "n_points": n_points,
                        "timeout_s": timeout_s},
            extra={"parameter_name": self.parameter_name},
            expected_steps=expected, estimated_step_bytes=estimated_step_bytes,
            memory_fraction=self.memory_fraction,
        )
        t_start = time.time()
        try:
            if target_values is not None:
                ok = self._run_targets(ctx, client, measure_func,
                                       list(target_values), t_start, timeout_s,
                                       verbose)
            else:
                ok = self._run_n(ctx, client, measure_func, n_points, t_start,
                                 timeout_s, verbose)
        except KeyboardInterrupt:
            return ctx.interrupted()
        return ctx.finish(status="success" if ok else "timeout")

    def _run_targets(self, ctx, client, measure_func, targets, t_start,
                     timeout_s, verbose):
        # Match targets in any order: each poll captures whichever not-yet-met
        # target the current value satisfies.  Already-captured targets (from a
        # resume) are dropped from the remaining set.
        remaining = list(targets)
        for point in ctx.points:
            captured = point.axis.get(self.parameter_name)
            if captured in remaining:
                remaining.remove(captured)
        index = len(ctx.points)
        self._announce(remaining, verbose)
        while remaining:
            if self._timed_out(t_start, timeout_s, verbose):
                return False
            value = self.get_parameter(client)
            hit = next((t for t in remaining if self.condition(value, t)), None)
            if hit is not None:
                remaining.remove(hit)
                ctx.record_step(index, {self.parameter_name: hit},
                                {self.parameter_name: value}, measure_func,
                                total=len(targets))
                index += 1
                self._announce(remaining, verbose)
                continue
            self._poll_status(value, f"{len(remaining)} target(s) left",
                              t_start, verbose)
            time.sleep(self.poll_interval_s)
        return True

    def _run_n(self, ctx, client, measure_func, n_points, t_start, timeout_s,
               verbose):
        index = len(ctx.points)
        if verbose and index < n_points:
            print(f"Waiting for the condition on {self.parameter_name} ...")
        while index < n_points:
            if self._timed_out(t_start, timeout_s, verbose):
                return False
            value = self.get_parameter(client)
            if self.condition(value, None):
                ctx.record_step(index, {self.parameter_name: value},
                                {self.parameter_name: value}, measure_func,
                                total=n_points)
                index += 1
            else:
                self._poll_status(value, f"{index}/{n_points} captured",
                                  t_start, verbose)
            if index < n_points:
                time.sleep(self.poll_interval_s)
        return True

    def _announce(self, remaining, verbose):
        if verbose and remaining:
            goals = ", ".join(_fmt_value(t) for t in remaining)
            print(f"Waiting for {self.parameter_name} to reach any of "
                  f"[{goals}] (any order) ...")

    def _poll_status(self, value, detail, t_start, verbose):
        if verbose:
            elapsed = _format_duration(time.time() - t_start)
            print(f"  current {_fmt_value(value)} · {detail} · elapsed {elapsed}    ",
                  end="\r", flush=True)

    def _timed_out(self, t_start, timeout_s, verbose):
        if timeout_s is not None and (time.time() - t_start) > timeout_s:
            if verbose:
                print("\n  timeout reached.")
            return True
        return False


# ---------------------------------------------------------------------------
# Reading a finished run back.
# ---------------------------------------------------------------------------


class _LoadedStep:
    """A step from a loaded run, with its data file loaded on access."""

    def __init__(self, run, step):
        self._run = run
        self._step = step
        self.index = int(step["index"])
        self.axis = step.get("axis") or {}
        self.readback = step.get("readback") or {}
        self.status = step.get("status")
        self.summary = step.get("summary")
        self.data_file = step.get("data_file")
        self.plot_file = step.get("plot_file")

    @property
    def data(self):
        return self._run.data(self.index)


class LoadedRun:
    """A finished run loaded from disk: the manifest plus lazily-loaded data.

    Loaded step data is cached so repeated access is cheap, but **only while the
    cache fits in free memory** - once a step would not fit, it (and larger ones)
    are loaded without being cached.  So caching helps for normal runs yet can
    never grow a run larger than memory back into RAM.  Pass ``cache=False`` to
    disable it entirely, or use :meth:`iter_data` to stream without caching.
    """

    def __init__(self, path, manifest_name=MANIFEST_FILE, cache=True):
        path = Path(path)
        self.root = path if path.is_dir() else path.parent
        self.manifest = _load_manifest(path, manifest_name)
        self.steps = self.manifest.get("steps", [])
        self._caching = bool(cache)
        self._cache: Dict[int, dict] = {}

    def _step(self, index):
        for step in self.steps:
            if int(step["index"]) == index:
                return step
        raise KeyError(f"no step with index {index}")

    def _load(self, index):
        rel = self._step(index).get("data_file")
        return _load_npz(self.root / rel) if rel else None

    def data(self, index):
        """The saved data dict for ``index`` (``None`` if the step saved none).

        Returns the cached copy if present; otherwise loads from disk and caches
        it **only if it fits in free memory** (so reading back a run larger than
        memory never blows up - those steps simply aren't cached).
        """
        if index in self._cache:
            return self._cache[index]
        data = self._load(index)
        if data is not None and self._caching and _fits_in_memory(data):
            self._cache[index] = data
        return data

    def clear_cache(self):
        """Drop all cached step data."""
        self._cache.clear()

    def iter_data(self):
        """Yield ``(step, data)`` one step at a time, loading each step's data as
        it is yielded and **not** caching it - so a run that is too big to hold in
        memory can be processed point by point regardless of the cache setting::

            for step, data in load_run('runs/long').iter_data():
                process(step.axis, data)   # only one step's data is in memory
        """
        for step in self.steps:
            yield _LoadedStep(self, step), self._load(int(step["index"]))

    def values(self):
        """The list of per-step ``axis`` coordinates."""
        return [step.get("axis") for step in self.steps]

    def summary(self):
        """Structured per-step summary: ``[{index, axis, summary}, ...]`` for
        the steps that have a summary."""
        return [
            {"index": int(s["index"]),
             "axis": s.get("axis") or {},
             "summary": s.get("summary") or {}}
            for s in self.steps if s.get("summary")
        ]

    def __len__(self):
        return len(self.steps)

    def __iter__(self):
        for step in self.steps:
            yield _LoadedStep(self, step)


def load_run(path, manifest_name=MANIFEST_FILE, cache=True):
    """Load a finished (or in-progress) run from a directory or manifest path.

    Step data is cached while it fits in free memory; pass ``cache=False`` to
    never cache (see :class:`LoadedRun`)."""
    return LoadedRun(path, manifest_name=manifest_name, cache=cache)


def plot_run_summary(run, x=None, y=None, ncols=1, savefig=None):
    """Plot a run's ``value -> scalar`` summary as one figure.

    Args:
        run: a :class:`LoadedRun`, or a path to a run directory/manifest.
        x: axis name to use for the horizontal axis (default: the first axis).
        y: summary key, or list of keys, to plot (default: all summary keys),
            one subplot each.
        ncols: number of subplot columns to arrange them in (default 1, i.e.
            a single stacked column).
        savefig: optional path to save the figure to.

    Returns ``(figure, axes)`` where ``axes`` is the list of plotted axes in
    ``y`` order.
    """
    import matplotlib.pyplot as plt

    if not isinstance(run, LoadedRun):
        run = load_run(run)
    rows = run.summary()
    if not rows:
        raise ValueError("run has no summary data (was summarise_func used?)")

    axis_names = list(rows[0]["axis"].keys())
    if not axis_names:
        raise ValueError("run steps have no axis coordinate to plot against")
    xname = x or axis_names[0]
    ykeys = list(rows[0]["summary"].keys()) if y is None else (
        [y] if isinstance(y, str) else list(y))

    xs = [r["axis"].get(xname) for r in rows]
    ncols = max(1, int(ncols))
    nrows = int(np.ceil(len(ykeys) / ncols))
    fig, axes = plt.subplots(nrows, ncols, sharex=True,
                             figsize=(1 + 5 * ncols, 2.2 * nrows + 0.8),
                             squeeze=False)
    flat = axes.ravel()
    used = list(flat[:len(ykeys)])
    for ax, key in zip(used, ykeys):
        ax.plot(xs, [r["summary"].get(key) for r in rows], "o-")
        ax.set_ylabel(key)
        ax.grid(True, alpha=0.3)
    for ax in flat[len(ykeys):]:           # hide any empty cells
        ax.set_visible(False)
    for col in range(ncols):               # x-label on the lowest used cell per column
        col_used = [flat[r * ncols + col] for r in range(nrows)
                    if r * ncols + col < len(ykeys)]
        if col_used:
            col_used[-1].set_xlabel(xname)
    fig.suptitle(run.manifest.get("run_name") or run.manifest.get("kind", "run"))
    fig.tight_layout()
    if savefig is not None:
        fig.savefig(savefig, dpi=120)
    return fig, used
