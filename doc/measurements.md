# Parameter-Series Measurements

Tools for repeating a readout measurement across an external parameter and
saving the results as they complete. Use these when you want to take the *same*
measurement (a sweep, a timestream, a snapshot, ...) at many settings of
something you control or monitor — attenuation, temperature, bias voltage, time.

There are four tools, all in `souk_readout_tools.measurement`:

| Tool | Use it when you want to ... |
|---|---|
| [`ParameterSeries`](#parameterseries) | step a parameter through a **list of values** you choose |
| [`ParameterGrid`](#parametergrid) | sweep **several parameters** as a nested grid |
| [`TimedMeasurement`](#timedmeasurement) | measure **on a clock** (every N seconds, or for a duration) |
| [`ConditionalMeasurement`](#conditionalmeasurement) | measure **when a monitored value reaches a target** |

> These are a separate idea from a resonator **frequency sweep** (`wideband_sweep`,
> `perform_sweep`). A parameter *series* repeats *some* measurement — often a
> frequency sweep — at each value of an *external* parameter.

---

## The shared idea

Every tool follows the same recipe:

1. You describe the parameter with small callbacks (set it, read it, or both).
2. You provide a `measure_func(client)` that takes one measurement and returns
   its data.
3. The tool walks the parameter, calls `measure_func` at each point, and writes
   the results to disk **as it goes**.

```python
from souk_readout_tools.measurement import ParameterSeries

series = ParameterSeries(client, "tx_attenuation_db",
                         set_parameter=lambda c, v: c.set_tx_attenuation(v))

results = series.run(
    values=[0, 5, 10, 15, 20],
    measure_func=lambda c: c.wideband_sweep(verbose=False),
    save_dir="runs",
)
```

> **Always keep the return value** — `results = series.run(...)`. In a notebook
> an un-assigned call dumps the whole results list to the output; in a script it
> is simply lost.

### Callbacks all receive the client

All of your callbacks — `set_parameter`, `get_parameter` and `measure_func` —
receive the active `client` as their first argument (here named `c`). This lets
you write each one *generically* and reuse it against any board, rather than
hard-wiring a particular client inside the function.

If the parameter is an **external instrument** (a thermometer, a cryostat, a
separate attenuator) just ignore the `client` argument:

```python
def set_temperature(client, mk):   # client unused — it's an external instrument
    cryostat.set_setpoint_mk(mk)
```

### Several boards / pipelines

A setup may have several client instances, one per readout server (pipeline /
board). `run(..., client=...)` overrides the client for that run, so you can
build **one** tool and re-point it at each board — no need to rebuild it or
capture clients in closures:

```python
def set_atten(client, v): client.set_tx_attenuation(v)
def measure(client):      return client.wideband_sweep(verbose=False)

series = ParameterSeries(clients[0], "tx_attenuation_db", set_parameter=set_atten)

for board, client in clients.items():          # e.g. {"board0": c0, "board1": c1}
    results = series.run([0, 5, 10, 15, 20], measure, client=client,
                         save_dir="runs", run_name=f"atten_{board}")
```

### `measure_func` — what gets saved

`measure_func(client)` is called once per point. Its **return value decides what
is stored**:

- return a **`dict`** → it is saved as a compressed `.npz` file for that step
  (arrays stored directly; scalars/metadata folded into the file). This is the
  normal case.
- return **`None`** → nothing is saved as a product, but the step is still
  recorded in the manifest. Use this when your `measure_func` already saves its
  own files, or when there is nothing to store.

You can override the saver with `save_func(data, path)` if you want a format
other than `.npz`.

### Where data goes

When you pass `save_dir`, a run is written to `<save_dir>/<run_name>/`:

```
runs/atten_series/
├── measurement.json        ← the manifest (rewritten after every step)
├── summary.csv             ← live digest (only if you pass summarise_func)
├── data/
│   ├── atten_series_step000.npz
│   └── atten_series_step001.npz
└── plots/                  ← only if you pass plot_func
    └── atten_series_step000.png
```

- `save_dir` is relative to where you are working unless you give an absolute
  path. Folders are **created for you**.
- The **resolved absolute path is printed** when the run starts and again when it
  finishes — so you always know exactly where your data landed.
- **Nothing is ever overwritten by accident.** If `<save_dir>/<run_name>/`
  already exists, a numbered suffix is used instead (`atten_series_002`,
  `atten_series_003`, ...). Pass `overwrite=True` to deliberately reuse and clear
  a name, or `resume=True` to continue an interrupted run (see
  [Resuming](#resuming-an-interrupted-run)).
- If you omit `run_name`, a timestamped one is generated
  (e.g. `series_tx_attenuation_db_20260625-143001`).
- If you omit `save_dir` entirely, the run stays **in memory** — nothing is
  written, and you just get the results list back. Good for quick interactive
  use.

### What you get back

Every `run(...)` returns a list of `MeasurementPoint` objects:

| Attribute | Meaning |
|---|---|
| `index` | step number, starting at 0 |
| `axis` | the coordinate, `{name: value}` |
| `readback` | values read back from the hardware, `{name: value}` |
| `parameter_value` | readback of the first axis if available, else the set value |
| `value_set` | the value that was requested |
| `data` | whatever `measure_func` returned (kept in memory) |
| `data_file` | path to the saved `.npz`, relative to the run folder (or `None`) |
| `plot_file` | path to the saved plot (or `None`) |
| `summary` | the dict from `summarise_func` (or `None`) |
| `attempts` | how many tries the measurement took (see [retries](#retries-and-skipping-failures)) |
| `status` | `"success"` or `"failed"` |
| `error` | error text if the step failed |

---

## Common arguments

Every tool's `run(...)` accepts the same saving/control arguments:

| Argument | Default | Meaning |
|---|---|---|
| `measure_func` | *(required)* | `measure_func(client)` returning a dict or `None`; called at each point |
| `client` | *(constructor's)* | Override the client passed to all callbacks for this run |
| `save_dir` | `None` | Folder to save into. `None` = in-memory only |
| `run_name` | `None` | Name of the run folder and file prefix. `None` = auto, timestamped |
| `manifest_name` | `"measurement.json"` | Name of the manifest file |
| `save_func` | `None` | `save_func(data, path)` to save a step yourself (instead of `.npz`) |
| `plot_func` | `None` | `plot_func(point, path)` to draw a plot per step (see [Auto-plot](#auto-plotting-each-step)) |
| `summarise_func` | `None` | `summarise_func(point) -> dict` of scalars for the live summary |
| `overwrite` | `False` | Reuse and clear an existing run folder |
| `resume` | `False` | Continue an interrupted run in the same folder |
| `retries` | `0` | Re-attempt a failing measurement this many times |
| `on_error` | `"raise"` | After retries: `"raise"` (stop) or `"skip"` (log and continue) |
| `estimated_step_bytes` | `None` | Optional per-step size hint for a pre-run memory/disk check (see [Resource safety](#resource-safety-memory-and-disk)) |
| `verbose` | `True` | Print progress and ETA |

---

## ParameterSeries

Step one parameter through a list of values.

**Constructor**

| Argument | Default | Meaning |
|---|---|---|
| `client` | *(required)* | Default client for the callbacks (override per run with `client=`) |
| `parameter_name` | *(required)* | Human-readable name, e.g. `"tx_attenuation_db"` |
| `set_parameter` | `None` | `set_parameter(client, value)` applies a value. Omit for a monitor-only parameter |
| `get_parameter` | `None` | `get_parameter(client) -> value` reads it back. Omit if you only set it |
| `settle_time` | `0.0` | Seconds to wait after setting before measuring |
| `memory_fraction` | `0.8` | Fraction of total system memory this run may hold before offloading (see [Resource safety](#resource-safety-memory-and-disk)) |

You must provide at least one of `set_parameter` / `get_parameter`.

### Example — with named functions (recommended)

This is the readable style for a real measurement. The functions are where you
plug in your own instrument code (and they ignore `client` if they don't need
it).

```python
from souk_readout_tools.measurement import ParameterSeries

def set_attenuation(client, value):
    client.set_tx_attenuation(value)

def read_attenuation(client):
    return client.get_rf_peripheral_status()["result"]["tx_attenuation_db"]

def measure(client):
    # any measurement that returns a dict of results
    return client.wideband_sweep(verbose=False)

series = ParameterSeries(
    client,
    parameter_name="tx_attenuation_db",
    set_parameter=set_attenuation,
    get_parameter=read_attenuation,   # verifies what the hardware actually did
    settle_time=1.0,                  # let the attenuator settle
)

results = series.run(
    values=[0, 5, 10, 15, 20],
    measure_func=measure,
    save_dir="runs",
    run_name="atten_series",
)
```

### Example — as a one-liner with lambdas (quick interactive use)

Equivalent, compact, good for a notebook when the callbacks are trivial:

```python
series = ParameterSeries(
    client, "tx_attenuation_db",
    set_parameter=lambda c, v: c.set_tx_attenuation(v),
    get_parameter=lambda c: c.get_rf_peripheral_status()["result"]["tx_attenuation_db"],
    settle_time=1.0,
)
results = series.run([0, 5, 10, 15, 20],
                     lambda c: c.wideband_sweep(verbose=False),
                     save_dir="runs")
```

Use named functions when the logic is more than a single expression or you will
reuse it; use lambdas for throwaway one-liners.

---

## ParameterGrid

Sweep several parameters as a nested cartesian grid. Each axis is a `SeriesAxis`.
The **first axis is the outermost (slowest) loop**, and only the axes whose value
actually changes between adjacent points are re-applied — so a slow outer
parameter (like temperature) is not needlessly re-set on every inner step.

**`SeriesAxis`**

| Field | Default | Meaning |
|---|---|---|
| `name` | *(required)* | Axis name |
| `values` | *(required)* | The values to step through |
| `set_parameter` | `None` | `set_parameter(client, value)` to apply a value |
| `get_parameter` | `None` | `get_parameter(client) -> value` to read it back |
| `settle_time` | `0.0` | Seconds to wait after *this axis* changes |

### Example

```python
from souk_readout_tools.measurement import ParameterGrid, SeriesAxis

def set_temperature(client, mk): cryostat.set_setpoint_mk(mk)   # external instrument
def read_temperature(client):    return cryostat.temperature_mk()
def measure(client):             return client.wideband_sweep(verbose=False)

grid = ParameterGrid(client, axes=[
    SeriesAxis("temperature_mk", [100, 200, 300],
               set_parameter=set_temperature, get_parameter=read_temperature,
               settle_time=60.0),                      # outer: slow, settles 60 s
    SeriesAxis("tx_attenuation_db", [0, 5, 10],
               set_parameter=lambda c, v: c.set_tx_attenuation(v)),  # inner: fast
])

results = grid.run(measure, save_dir="runs", run_name="temp_atten_grid")
# 3 x 3 = 9 points; temperature is set 3 times, attenuation 9 times.
```

Each manifest step records the full coordinate, e.g.
`{"temperature_mk": 200, "tx_attenuation_db": 5}`.

---

## TimedMeasurement

Take measurements on a clock.

**Constructor**

| Argument | Default | Meaning |
|---|---|---|
| `client` | *(required)* | Default client for `measure_func`/`get_parameter` (override per run with `client=`) |
| `parameter_name` | `"time"` | Name of the value logged at each point |
| `get_parameter` | `None` | `get_parameter(client) -> value` read at each point. If omitted, elapsed seconds are logged |
| `interval_s` | `60.0` | Seconds between the *start* of successive measurements |
| `memory_fraction` | `0.8` | Fraction of total system memory this run may hold before offloading (see [Resource safety](#resource-safety-memory-and-disk)) |

**`run(...)`** also takes `n_points` and/or `duration_s` (give at least one; if
both, it stops at whichever comes first).

### Example

```python
from souk_readout_tools.measurement import TimedMeasurement

def read_temperature(client): return thermometer.read_mk()
def measure(client):          return client.get_samples(2048)

timed = TimedMeasurement(client, "temperature_mk",
                         get_parameter=read_temperature, interval_s=60.0)

# one measurement per minute for an hour
results = timed.run(measure, duration_s=3600, save_dir="runs",
                    run_name="overnight_drift")
```

The interval is honoured from the *start* of each measurement, so a measurement
that takes 8 s followed by `interval_s=60` waits ~52 s before the next one.

---

## ConditionalMeasurement

Measure when a monitored value meets a condition.

**Constructor**

| Argument | Default | Meaning |
|---|---|---|
| `client` | *(required)* | Default client for `measure_func`/`get_parameter` (override per run with `client=`) |
| `parameter_name` | *(required)* | Name of the monitored value |
| `get_parameter` | *(required)* | `get_parameter(client) -> value` |
| `condition` | *(required)* | `condition(value, target) -> bool`; `True` triggers a measurement |
| `poll_interval_s` | `1.0` | How often to check the condition |
| `memory_fraction` | `0.8` | Fraction of total system memory this run may hold before offloading (see [Resource safety](#resource-safety-memory-and-disk)) |

**`run(...)`** takes either `target_values` (measure once per target) or
`n_points` (measure whenever the condition is met, that many times), plus an
optional `timeout_s`.

The `condition` receives both the current `value` and the `target` being tested
(`target` is `None` in `n_points` mode). **Targets are matched in any order** —
each poll captures whichever not-yet-met target the value satisfies first — so a
drifting quantity like temperature need not pass them in the order you listed.

### Example

```python
from souk_readout_tools.measurement import ConditionalMeasurement

def read_temperature(client): return thermometer.read_mk()
def measure(client):          return client.wideband_sweep(verbose=False)

# Measure once each time the cryostat settles within 5 mK of any listed target.
cond = ConditionalMeasurement(
    client, "temperature_mk",
    get_parameter=read_temperature,
    condition=lambda value, target: abs(value - target) < 5,
    poll_interval_s=5.0,
)
results = cond.run(measure, target_values=[100, 200, 300, 400], timeout_s=3600,
                   save_dir="runs", run_name="temp_steps")
```

Whether the cryostat warms or cools — passing 300 before 200, say — each target
is captured when it is reached. The step's `axis` records the target it captured;
its `readback` records the actual value at that moment. Completion is genuinely
unpredictable, so it reports elapsed time and which targets remain rather than a
(fake) ETA.

---

## Progress and ETA

With `verbose=True` (the default) each step prints a line, and an estimated time
to completion once timing is known:

```
Saving to: /home/you/project/runs/atten_series
[2/5] tx_attenuation_db=5 (readback 5.0)  done in 8.1 s
      elapsed 16.4 s · ETA ~24.3 s · finish ~14:32
      step 131 MB · run using 262 MB RAM, 84 MB of ~210 MB disk · free 12 GB mem, 270 GB disk
      f0_MHz=612.4 · Qr=18500
...
Done: 5 point(s) in 41.0 s -> /home/you/project/runs/atten_series
```

The third line is the **resource line**, printed every step: this step's size, what
**this run** is currently using (RAM held + disk written, with the projected run
total), and the system free space. See [Resource safety](#resource-safety-memory-and-disk).

ETA is exact for `TimedMeasurement`, estimated from the mean step time for
`ParameterSeries`/`ParameterGrid`, and omitted for `ConditionalMeasurement`. Pass
`verbose=False` to silence all output.

---

## Auto-plotting each step

Pass `plot_func(point, path)` to draw and save a figure after every step. You
draw the figure and save it to `path`; the run records the location.

```python
import matplotlib.pyplot as plt

def plot_step(point, path):
    d = point.data
    fig, ax = plt.subplots()
    ax.plot(d["freqs"] / 1e6, 20 * np.log10(abs(d["s21"])))
    ax.set_title(f"{point.parameter_name} = {point.value_set}")
    ax.set_xlabel("MHz"); ax.set_ylabel("dB")
    fig.savefig(path, dpi=100)
    plt.close(fig)

results = series.run(values, measure, save_dir="runs", plot_func=plot_step)
```

A failure inside `plot_func` is reported as a warning and never aborts the run.

---

## Live summary table

Pass `summarise_func(point) -> dict` of scalars to build a running digest. After
each step the scalars are printed, stored in the manifest, and written to a live
**`summary.csv`** in the run folder (so you can watch a long run from another
terminal):

```python
def summarise(point):
    d = point.data
    return {"f0_MHz": fit_centre(d) / 1e6, "depth_dB": fit_depth(d)}

results = series.run(values, measure, save_dir="runs", summarise_func=summarise)
```

`summary.csv`:

```
index,tx_attenuation_db,f0_MHz,depth_dB
0,0,612.41,-18.2
1,5,612.40,-15.7
```

(Only the *summary* is written as CSV — measurement **data** is always saved as
`.npz`, never CSV.)

---

## Retries and skipping failures

By default a failing measurement stops the run (and the partial results are still
on disk). To make a long unattended run robust:

```python
results = series.run(values, measure, save_dir="runs",
                     retries=2,          # try each point up to 3 times total
                     on_error="skip")    # if it still fails, log it and move on
```

Skipped points are recorded with `status="failed"` and their error text, so you
can see exactly what happened. With the default `on_error="raise"`, the manifest
is finalized as `"failed"` before the error is re-raised.

---

## Resource safety (memory and disk)

Long timestreams, batches of snapshots, or a multi-axis grid with thousands of
tones can produce more data than will fit in RAM or on disk. The tools guard
against this automatically — they never block or abort a run, they **adapt or
warn and continue**.

**Memory.** After the first step that produces data, the tool measures that
step's size, projects it over the whole run, and compares it against this run's
**memory budget** (see below):

- if keeping every step in RAM would exceed the budget **and you are saving**
  (`save_dir` set), it switches to **freeing each step after it is saved**. The
  returned `MeasurementPoint.data` becomes `None` for freed steps, but the data
  is safe on disk — read it back **one step at a time** with
  [`run.iter_data()`](#reading-a-run-back) (or `run.data(i)`, which only caches
  steps small enough to fit in free memory). You'll see:
  ```
  [memory] projected ~48.0 GB for 200 steps exceeds this run's ~11.5 GB budget
           (memory_fraction=0.8); freeing each step after saving (reload with load_run).
  ```
- if you are **not** saving (`save_dir=None`), there is nowhere to offload, so it
  warns once and continues — capture the results incrementally or pass a
  `save_dir` if you expect a lot of data.

The budget is `memory_fraction` × **total system memory** (a constructor
argument, default `0.8`), but never more than what is actually free at the time.
The fraction matters when you run **several measurements at once** — e.g. one per
board/pipeline — because each run sizes itself against total memory, so the
budgets add up predictably instead of each grabbing all free RAM. Give each
concurrent run a small slice:
```python
series = ParameterSeries(client, "tx_attenuation_db",
                         set_parameter=set_atten, memory_fraction=0.05)  # ~5% each
```
(`memory_fraction` is on every tool's constructor and must be in `(0, 1]`.)

**Disk.** Every step, the tool compares the disk needed for the remaining steps
against the free space and, if it won't fit, prints a warning **on each step**
until the run finishes or the disk fills:
```
[disk] WARNING: ~120.0 GB needed for 150 remaining step(s) but only ~80.0 GB free
       - the run may not finish.
```

**Before the run.** If you know the per-step size, pass `estimated_step_bytes`
(e.g. `n_tones * n_samples * 16` for complex128) and the same checks run *before*
the first step, so you're warned of a likely problem up front. As everywhere
here, it only warns — the run always proceeds:

```python
results = timed.run(measure, n_points=5000, save_dir="runs",
                    estimated_step_bytes=2000 * 4096 * 16)   # ~131 MB/step
```

The checks are best-effort: available memory is read via `psutil` if installed,
otherwise the OS (`os.sysconf`); if memory can't be determined that check is
skipped. Disk space uses `shutil.disk_usage`.

---

## Resuming an interrupted run

If a run is stopped (a crash, or you pressed Ctrl-C), re-run it with
`resume=True` and the **same `save_dir` and `run_name`**:

```python
results = series.run(values, measure, save_dir="runs", run_name="atten_series",
                     resume=True)
```

It reads the existing manifest, keeps the steps that already completed
successfully, and only does the remaining ones. (For `ParameterSeries`/
`ParameterGrid` it matches by step position and warns if the values no longer
line up; for `ConditionalMeasurement` it drops targets that were already
captured.)

### Graceful Ctrl-C

Pressing **Ctrl-C** during a run stops it cleanly: the current manifest is
finalized with `status="interrupted"`, the location is printed, and the results
collected so far are returned. Nothing already saved is lost, and you can pick up
later with `resume=True`.

---

## Reading a run back

Load a finished (or in-progress) run with `load_run`:

```python
from souk_readout_tools.measurement import load_run

run = load_run("runs/atten_series")     # a folder or a manifest path

run.manifest         # the full manifest dict
run.values()         # the list of per-step coordinates
run.summary()        # [{index, axis, summary}, ...] for steps that have a summary
run.data(0)          # step 0's saved data dict, read from disk on each call
```

`run.data(i)` caches each step it loads **while the cache fits in free memory**,
so repeated access is cheap for normal runs, yet reading back a run larger than
memory never blows up — steps that wouldn't fit simply aren't cached. Pass
`load_run(..., cache=False)` to disable caching, or `run.clear_cache()` to drop
it.

To process a run that is too big to hold at once, use `iter_data()` — it loads
one step at a time and **never** caches:

```python
for step, data in run.iter_data():       # only one step's data in memory
    process(step.axis, data)             # the saved .npz contents
```

Iterating the run directly yields the steps; `.data` loads (and bounded-caches)
on access:

```python
for step in run:
    use(step.data)
```

### Plotting a run's summary

`plot_run_summary` renders the whole `value → scalar` summary as one figure
(one subplot per scalar; arrange them with `ncols`):

```python
from souk_readout_tools.measurement import plot_run_summary

# plots every summary key vs the first axis (single stacked column)
fig, axes = plot_run_summary("runs/atten_series")

# choose the x axis, which scalars to plot, and the column count
fig, axes = plot_run_summary("runs/temp_atten_grid",
                             x="temperature_mk", y=["f0_MHz", "Qr", "depth_dB"],
                             ncols=2, savefig="summary.png")
```

This needs a run that was taken with a `summarise_func`. It reads only the
manifest summaries, not the per-step data files.

---

## The manifest

`measurement.json` is plain JSON, rewritten after every step. A run looks like:

```json
{
  "kind": "parameter_series",
  "run_name": "atten_series",
  "created": "2026-06-25 14:30:01 +0000",
  "finished": "2026-06-25 14:30:42 +0000",
  "status": "success",
  "parameters": {"tx_attenuation_db": [0, 5, 10, 15, 20]},
  "steps": [
    {
      "index": 0,
      "axis": {"tx_attenuation_db": 0},
      "readback": {"tx_attenuation_db": 0.0},
      "timestamp": "2026-06-25 14:30:09",
      "status": "success",
      "attempts": 1,
      "data_file": "data/atten_series_step000.npz",
      "plot_file": null,
      "summary": {"f0_MHz": 612.41},
      "error": null
    }
  ]
}
```

`status` is `"running"` while in progress, then `"success"`, `"failed"`,
`"interrupted"`, or `"timeout"`.

---

## See also

- [Power sweeps](getting_started.md#parameter-space-measurements) —
  `run_power_sweep`, a worked stepped acquisition that uses the same manifest/npz
  format under the hood.
- [Resonator Drive Tuning and Noise Measurements](resonator_noise_workflow.md).
