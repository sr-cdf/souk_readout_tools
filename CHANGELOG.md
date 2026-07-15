# Changelog & Feature List

## v1.6.3

**Fix: stale mixer dwell breaking software modulation after firmware-slot runs**
- `disable_fw_modulation` left the mixer `dwell` register at the fw
  `n_dwell`. The dwell gates *all* allowed LO switches — including the
  ping-pong buffer flips driven by software modulation, sweeps and retunes —
  so after any fw-mod run, sw modulation showed skipped points and ragged
  dwells and sweeps averaged stale-frequency accumulations, long after all
  modulation was reported off (confirmed on hardware: `dwell` read back 11).
  The dwell is now restored to 1 on fw disable and asserted at sw arm.

**Filterbank compensation: applied where you look at the data**
- The software readout-flattening factors (the RX half the inert fabric
  scale word cannot apply) are now applied at **parse time** by default:
  `parse_samples(apply_readout_correction=True)` (per sample via the
  modulation point tag) and `parse_sweep_data(apply_readout_correction=True)`
  (per sweep point; the server ships the factors with `get_sweep_data`).
  Output carries `readout_correction_applied`; `group_cycles` skips its own
  application when marked. Pass `False` for raw accumulator values.
- Sweeps/retunes now resolve the standard path group-delay calibration for
  the image-phase term (previously override-only) and take a per-request
  `compensate_filterbank` override (`perform_sweep`, `perform_retune`,
  `wideband_sweep`), matching `enable_modulation`.
- Plot/fit-time toggle: `plot_sweep`, `plot_timestream`,
  `plot_timestream_psd`, `plot_timestream_on_resonance` and `batch_fit`
  take `apply_readout_correction` (default on) and convert already-parsed
  data either way (parsed sweeps keep the factors; modulated captures
  re-derive them from the info snapshot).
- Power optimisers (`maximise_tx_power`, `set_tone_powers(...,
  optimise_dynamic_range=True)`, `maximise_tx_dsp_gain`) reserve
  `FILTERBANK_TX_HEADROOM_DB` (1 dB) of amplitude-word headroom for the TX
  drive-restoring boost when compensation is enabled — the psb_scale ramp
  recovers the output level, so no power is lost. Explicit `set_tone_powers`
  requests that eat into the reserve warn instead of failing.

## v1.6.2

**Power-sweep API tidy (breaking)**
- `power_sweep` now reads as the workflow: `run_power_sweep` /
  `load_power_sweep` acquire, `analyse_power_sweep` / `load_analysis` (new)
  analyse, then optional comb balancing — with the pieces grouped per
  artifact, each with a symmetric `write_*`/`load_*` pair. The module
  docstring tells the story and the file is ordered to match.
- Removed the `analyze_power_sweep` alias; the spelling is
  `analyse_power_sweep`.
- Renamed `fit_parameter_series` → `parameter_series` and
  `allocate_balanced_tone_powers` → `balance_tone_powers`; the
  `include_optimizer` kwarg is now `include_optimiser` and the balancing
  objective `"maximize_power"` is `"maximise_power"` (matching
  `rx_policy='maximise'`). `fit_summary_rows` is private —
  `fit_summary_array` and the CSV are the public views of the table.
- New `load_analysis(path)`: read-only reload of everything
  `analyse_power_sweep` wrote (fits, summary CSV, best power, and balanced
  power if saved).
- New `load_fit_summary(path)`: the fit-summary CSV back as a structured
  array, accepted directly by `find_best_power`.
- `analyse_power_sweep`'s `plot` flag split into `plot_fits` / `plot_best`,
  so the power selection can be redone from stored fits without
  re-rendering every per-tone fit plot:
  `analyse_power_sweep(run, fit=False, plot_fits=False, target_anl=...)`.
- Writers accept a destination file or run directory, default into
  `<run>/analysis/` (`write_fit_summary(fit_data)` included), and return
  `Path`.

**Sweeps**
- Overlapping sweep spans are clipped so neighbouring segments do not
  overlap (`clip_overlapping_spans`, on by default in `perform_sweep` /
  `perform_retune`); the actually-swept centers/spans are recorded in the
  sweep metadata.

## v1.6.1

**Filterbank channel-response compensation**
- Tones operating away from their filterbank bin centre are now gain- (and
  optionally phase-) compensated automatically wherever mixer control words
  are prepared: tone setting, sweeps, and — the motivating case — frequency
  modulation riding the channel overlap, where the combined TX+RX response
  reaches −1 dB at 0.70 bin spacings (±211 kHz) and −6 dB at the channel edge.
- Model: 8-tap DPSS+sinc prototype (single-bank gain `G`) plus the coherent
  synthesis-image recombination in the analysis bin
  (`H(δ) = Σₘ G(δ−2m)²·e^{4πjmBτ}`), hardware-validated in digital loopback
  (≤0.04 dB across the full channel, zero phase) and RF loopback (0.05 dB /
  2.7 mrad with the fitted 150 ns bench group delay), frequency-independent
  across 0.6–2.1 GHz. See [filterbank_compensation.md](doc/filterbank_compensation.md).
- Corrections: TX scaling `× 1/G` (hardware, per point) restores the physical
  drive on the detector (the drive rolls off with the single bank even though
  the readout only sags to |H|); the readout-flattening `× G/|H|` part is
  applied in **software** — the RX LO scale word turned out to be inert in
  the v7.11 fabric (hardware-verified; raised in souk-firmware#117), so the
  server reports per-(tone, point) `readout_correction` factors in the
  modulation state and `modulation.group_cycles` applies them (flip
  `firmware_lib.FILTERBANK_RX_SCALE_IN_FABRIC` when a firmware connects the
  multiplier). The RX phase offsets (which do work in hardware) remove the
  near-edge image phase turn-over per modulation point. A clipped TX boost
  is reported in the modulation state warnings.
- Fixed the server `psb_scale` setter truncating fractional scales to int
  (the block value is UFix16.8; restoring a read-back 2.77 silently wrote 2).
- Fixed an intermittent `RuntimeError: delayed modulation preload was not
  serviced…` killing sw-modulated finite captures near their end: when the
  read cadence and the dwell grid are phase-shifted (the buffer_id tag lags
  the commanded flip), the final short dwell advanced the scheduler before
  the previous preload's reuse delay had elapsed. The scheduler now services
  a stale preload immediately and logs it (data was never mislabelled —
  frames are tagged from the firmware buffer_id), and captures no longer
  advance after their final dwell.
- The phase term's group delay comes from the standard path-delay calibration
  (`rf_frontend.path_group_delay_ns`, scalar or frequency-dependent — measure
  once with `client.measure_path_group_delay(save_to_config=True)`),
  evaluated per tone; `filterbank_group_delay_override_ns` (firmware
  defaults) overrides it for testing/loopback work. Master switch:
  `filterbank_compensation` (default **on**);
  `enable_modulation(..., compensate_filterbank=False)` gives a per-request
  A/B override. Keep base tone amplitudes ~1 dB below full scale so the
  modulation TX boost has headroom (the preparer warns when it clips).
- Standalone measurement/verification scripts in `scripts/filterbank/`:
  channel-response measurement, a compensation acceptance test, and an
  LO-stepping diagnostic. They configure the system from scratch (single
  test tone — avoids intermod/alias contamination), guard the digital-gain
  state around internal-loopback switches, and restore what they change.

**Other**
- rfsoc4x2: krc-utils availability checks in the clock status handling.
- `so3g` is no longer a hard dependency — without it, SO 3G streaming is
  unavailable (e.g. on Windows) but everything else works.
- Docs/profiling: README and profiling scripts detail the accumulator
  read-back geometry and timing metrics.

## v1.6.0

**v7.11 multi-LO firmware support (firmware-slot frequency modulation)**
- Adapted to the v7.11 register map: `acc_len` moves from the accumulator to the
  mixer (get/set, readiness checks and sample-rate helpers; changing it now
  needs an mrst sync). `sync_delay` is deprecated — the mixer owns the RX/TX LO
  delay and sets it during init; config/parameter writes warn and no-op, and
  `info_pipeline` reports the live mixer `sync_delay`, `tx_rx_skew` and
  `buffer_switch_skew`.
- Mixer LO control ping-pong buffers are now separate registers
  (`{lo}_lo{i}_control0/1`), each holding `n_slots` LO slots selected by
  in-register offset. All slow/fast control-buffer read/write paths gain a
  `slot` argument (`slot=0` reproduces the pre-v7.11 single-LO behaviour).
  `accumulator.get_new_spectra` now returns a 5-tuple.

**Firmware-slot modulation engine (`engine='fw'`, hardware-validated)**
- New modulation engine using the v7.11 mixer's four LO slots: load up to four
  full combs (centre + per-slot offset) once and let the firmware switch
  between them — auto round-robin every `n_dwell` accumulations, or manual
  slot selection — with no per-sample software writes and no software-timing
  jitter. Switches land on accumulation edges; the residual transient is well
  under one accumulation, so `n_settle` of 0 or 1 suffices.
- Each accumulation is tagged from the firmware `buffer_id` register (live LO
  slot for fw, ping-pong buffer → point for sw) in the same devmem call as the
  IQ data, removing the one-sample tag/data lag of the v7.11 edge-latched
  buffer switch. `get_samples` aligns its first returned sample to the start
  of a point-1 dwell; settling is edge-detected from tag changes so it
  self-corrects across dropped samples.
- Seamless `update_modulation` rides the inactive ping-pong buffer and flips
  in one step; `recenter_modulation` reloads fresh armed bins. The two engines
  share the `flag5` tag word and are mutually exclusive; tone/sweep/retune
  writes invalidate both.

**Unified modulation API**
- One client surface for both engines: `enable_modulation(center, offsets, ...,
  engine='fw')` defaults to the firmware-slot engine;
  `update/recenter/disable_modulation` and `get_modulation_state` auto-detect
  the active engine (`engine=None`), with `set_modulation_slot` for fw manual
  mode. The sw path is unchanged underneath.
- `get_info`: the `tone_modulation` section becomes `modulation` — a single
  payload for whichever engine is active, tagged with an `engine` field and
  carrying `num_points` / `samples_per_point` / `offsets_hz` aliases so it is
  drop-in for `group_cycles` / `demodulate`.
  `modulation.active_modulation_state()` picks the state out of a `get_info`
  payload for the analysis/plotting helpers.
- [frequency_modulation.md](doc/frequency_modulation.md) rewritten around the
  unified API with firmware-slot as the primary engine.

**Snapshots (v7.11)**
- Snapshot blocks latch the telescope time of the first sample;
  `get_adc/dac_snapshot` requests return it as `timestamp`. New
  `batch_adc_snapshots` / `batch_dac_snapshots` stream raw timestamped
  snapshots to the client over one TCP connection via the fast devmem path.

**Server robustness**
- `set_config` now verifies pipeline readiness after a config push and fails
  loudly instead of persisting a dead config; a pushed config that changes the
  firmware image drives the full deprogram/reprogram path instead of a live
  parameter tweak.

**Misc**
- Added `resonator_fitter.py` for compatibility with the ukkid_controller OCS
  agent.

## v1.5.0

**v7.10-timed-sync firmware support (timed sync & multi-board alignment)**
- Added support for the v7.10-timed-sync firmware, including TT-targeted syncs so
  independent RFSoC boards can align timestamps and accumulations to the same epoch.
- New client controls `timed_sync_needed()` / `timed_sync_ready()` /
  `timed_sync_arm()` / `timed_sync_check()` and `set_telescope_time()`, backed by
  firmware alignment/drift helpers. `set_telescope_time()` loads the current Linux
  time into the firmware TT counter, latched on the next PPS edge so the TT lands on
  the second boundary; `timed_sync_arm()` then schedules a future PPS-latched resync
  and only reloads TT when it is missing or has drifted by about 1 s.
  `timed_sync_check()` / `timed_sync_needed()` verify TT alignment against the
  Linux clock. These flows require PTP lock and a healthy on-board PPS strobe;
  see [tsu_strobe_and_timed_sync.md](doc/tsu_strobe_and_timed_sync.md).
- Board PPS generation now comes from the TSU strobe path (`tsu_strobe.py`,
  `souk-tsu-strobe`) rather than an external PPS input, with test and
  verification scripts in `scripts/timed_sync/`.
- `get_info('sync')` now reports timed-sync state, readiness, alignment, last
  sync, last TT load and drift; `get_info('timing')` adds `sync_readiness`.
  The server caches the PPS-aligned TT load second so `timed_sync_check()` is
  instant by default and only falls back to slow resampling when needed.
- Timed-sync results now include UTC strings for absolute UNIX times, clearer
  key names (`last_pps_*`, `pps_boundary_offset_s`, `target_tt_*`, `armed_at_*`),
  and a shared default alignment tolerance of 0.1 ms
  (`timing.DEFAULT_ALIGN_TOL_S`, overridable via `align_tol_s`).
- Added instructions for standing up a lab PTP grandmaster on a Linux workstation
  in [timing.md](doc/timing.md): checking NIC PHC/hardware-timestamping support,
  installing the tools, and the chrony/`phc2sys`/`ptp4l` hardware-GM flow (with the
  deployed `ptp4l-gm`/`phc2sys-gm` services), plus the finding that software-only
  timestamping did not work with the RFSoCs.

**TX/RX phase offset fix (firmware)**
- The v7.10 firmware fixes the TX/RX phase offset seen after mixer/tone
  control-buffer switches, so post-switch firmware syncs are no longer required.
  The earlier software workaround is now disabled by default:
  `compensate_rx_ticks=0`.

**Sweep & modulation stepping without per-step syncs (v7.10)**
- With the phase-offset bug fixed, sweeps, retunes and modulation now default to
  a single start-of-run sync (`setup_sync=True`, `setup_mrst=False`) with no
  per-step syncs (`autosync=False`). Instead they drop a few accumulations around
  each step; `settle_accumulations=4` / `chanmap_settle_accumulations=4` are now
  the defaults for `perform_sweep`, `perform_retune` and `wideband_sweep`.
- Standalone tone setters (`set_tone_frequencies`, `set_tone_amplitudes`,
  `set_tone_phases`, `apply_tone_frequency_settings`, `set_tone_powers`) also
  default to `autosync=False`.
- `enable_modulation` now defaults `buffer_reuse_delay_accs=3` with
  `samples_per_point=4` to avoid rewriting the inactive mixer-control buffer too
  early or straddling a buffer switch. Both issues are expected to disappear in
  a future firmware.

**Clock-source safety**
- Clock health is now checked at initialise/restart and via `get_info`; there is
 no forced clock reconfiguration on every config push now. If a PLL is unlocked 
 or the live source differs from config, the server refuses to touch the clock 
 and reports `reset_required`, because we found that changing clock configuration 
 against a loaded pipeline can stall or crash the board. The fix is to deprogram 
 the firmware before any clock change, so seamless runtime switching between 
 internal and external clock sources is no longer supported. Clock changes now go 
 through a reset-to-base -> deprogram -> clock-init -> reprogram (`hard_reset`) 
 cycle, with new helpers `classify_clock_status`, `apply_clock_config`, 
 `deprogram_fpga`, and `select_clock_source`.

**Modulation / demodulation**
- Added `modulation.demodulate_timestream()`, which performs sample-aligned
  demodulation without averaging away retained modulation samples. It supports
  calibrated Mobius inversion or the model-free basis plus `raw` /
  `interpolate` / `none` settling fill modes.
- `group_cycles()` now supports `include_settling` and emits blind / regular /
  modulated tone-role metadata; `params_from_sweep()` adds mutually exclusive
  `point_sequence` and `offset_linewidths` inputs.
- `recenter_modulation()` gains RX-tick phase compensation, client-visible
  modulation-state warnings, and `purge_modulation_revisions()` for stale
  revision bookkeeping.
- `enable_modulation(..., force=True)` now arms even when some tones fall
  beyond fixed-bin coverage; the response reports
  `result.tones_beyond_coverage` and warns that those tones will read back
  wrapped to the other end of the bin.

**DAC inverse sync filter**
- Added config-driven DAC inverse-sync-filter control, enabled by default unless
  explicitly set `False`, directly controllable from the client and reported in
  `get_info` / config sync.

**Plotting & analysis**
- Added `plotting.log_bin_psd()` for log-spaced PSD binning, timestream
  plotting for modulated-probe frequencies and precomputed
  frequency/dissipation columns, plus resonator-fitting uncertainty/model
  refinements.

**Resonator fitting — automatic nonlinearity estimation (`fitting.py`)**
- Added `nonlinear='auto'` for `fit_resonance`, `batch_fit` and
  `fit_sweep_stack`: the fitter tries the fast linear model first and only
  escalates to the slower Duffing `anl` fit when needed. Diagnostics are stored
  in `linear_sufficiency`; linear-only results report `anl = 0`.
- This mode is intentionally not the default and is not suitable for power
  sweeps or `anl`-versus-power studies, where small but real nonlinearities
  would be suppressed. `nonlinear` therefore remains `True` by default,
  including in `fit_power_sweep` / `analyse_power_sweep`.
- Linear fitting is also faster via a closed-form Jacobian, and
  `store_optimizer=False` can shrink large parallel-fit results by dropping
  bulky optimiser objects.

**Mock server sample rate**
- The mock server now derives sample rate from `acc_len`, matching real
  hardware in both directions (`sample_rate_hz` updates `acc_len`, and vice
  versa).
- Mock acquisition is now throttled to approximately real-time using the live
  sample rate, so G3 streaming, `receive_stream`, and non-burst `get_samples`
  take realistic wall-clock time and avoid false "streaming finished early"
  warnings.

**`get_sweep_data` and `get_info` speedup**
- `get_sweep_data` and `get_sweep_txt` now serve a cached sweep-time system-info
  snapshot instead of re-polling hardware on each fetch, cutting fetch time from
  about 6 s to about 5 ms.
- `get_info('all')` is also much faster (about 6 s to about 1 s) thanks to fast
  control-buffer/channel-map reads and narrower LNA I2C access.

**Parameter-series measurement tools**
- Reintroduced `measurement.py` in a deliberately simple form: four tools for
  repeating any measurement across an external parameter —
  `ParameterSeries` (a list of values), `ParameterGrid` (nested axes),
  `TimedMeasurement` (on a clock) and `ConditionalMeasurement` (on a condition).
  You supply set/read callbacks and a `measure_func(client)`; the tools walk the
  parameter and save each result plus a `measurement.json` manifest as they go.
  All callbacks receive the active `client` (ignore it for external
  instruments), and `run(..., client=...)` re-points one tool at another
  board/pipeline. `ConditionalMeasurement` matches `target_values` in any order,
  so a drifting quantity (e.g. temperature) need not reach them in sequence.
- The `measure_func` return value drives saving: a dict is stored as `.npz`,
  `None` stores nothing (the step is still logged); `save_func` overrides the
  saver. Saving is opt-in via `save_dir` (relative paths, auto-created folders,
  resolved path printed); existing run folders are never overwritten (numbered
  suffix) unless `overwrite=True`.
- Robust long runs: `resume=True` continues an interrupted run, `retries` /
  `on_error="skip"` tolerate flaky points, and Ctrl-C finalizes the manifest
  cleanly. Optional `plot_func` saves a plot per step and `summarise_func`
  builds a live `summary.csv` digest. Read runs back with `load_run` and plot
  their summary with `plot_run_summary`. See
  [measurements.md](doc/measurements.md).
- Resource guard for large runs (long timestreams, batch snapshots, many-tone
  grids): after the first step the run projects per-step memory and disk over
  the whole run. If memory would be exhausted it frees each step after saving
  (data stays on disk) or warns when nothing can be offloaded; if disk would
  fill it warns every step. Each step prints what the run is using (RAM held +
  disk written, with the projected total). The per-run memory budget is a
  `memory_fraction` constructor argument (default 0.8 of total system memory),
  so several runs across boards can each be capped (e.g. 0.05). Pass
  `estimated_step_bytes` for a pre-run check. The guard only adapts/warns - it
  never aborts a run.
- `load_run` caches step data only while it fits in free memory (oversized steps
  are read without caching), so repeated access is cheap yet a run larger than
  memory never blows up; `run.iter_data()` streams one step at a time without
  caching, `load_run(cache=False)` / `run.clear_cache()` control it.
  `plot_run_summary` gained an `ncols` argument to arrange the summary subplots
  in a grid.
- The shared manifest/npz helpers now live in `measurement.py` and are imported
  by `power_sweep.py` (the per-step saver is renamed `_save_npz`/`_load_npz`);
  the power-sweep run format is unchanged.

**Removals & housekeeping**
- Replaced the earlier over-complicated measurement-run framework (`MeasurementRun`
  / `MeasurementStore` / artifact dataclasses) with the simpler tools above;
  power-sweep directories remain the on-disk record (`run_power_sweep` /
  `load_power_sweep`).
- Also tightened config-push validation, added Windows guards for POSIX-only
  imports, and introduced `config_utils.get_user_dir()` for resolving the target
  user's data directory under `sudo`.

## v1.4.0

**Fast Frequency Modulation (real-time IQ conversion & resonator tracking)**
- Modulation is now a mode of the single continuous streamer: each tone is
  stepped through a small set of probe frequencies (typically 2-3 points) every
  accumulation, and the samples are returned as ordinary frames tagged per
  sample with the modulation step and a settling marker (packed into the spare
  `flag5` word; decode as unsigned). Normal streaming is the modulation-off case.
- New client controls: `enable_modulation()` / `update_modulation()` /
  `recenter_modulation()` / `disable_modulation()`, plus `get_modulation_state()`
  and a `get_info('tone_modulation')` section. Arm is independent of output —
  start it with `enable_stream()` (continuous) or `get_samples(N)` (finite); both
  return tagged frames while armed.
- On/off at any time, and **seamless live updates** of per-tone centres/offsets
  with no intentionally dropped frames (heavy preparation off the hot path, a
  single latest-wins command applied by the streamer at a cycle boundary). Tones
  ride the ~2x filterbank overlap; a bounded `recenter_modulation()` reloads the
  channel maps when a tone drifts beyond coverage. Per-(tone,point) bin occupancy
  is reported.
- Correct double-buffer "ping-pong" scheduler (valid for any N, including odd N
  across cycles) with an N=2 zero-rewrite fast path; per-point mixer words are
  computed relative to the armed bins so the phase stays continuous as tones
  dither/drift.
- New pure `souk_readout_tools.modulation` toolkit (setup + demodulation):
  `params_from_sweep` (build a config from your own resonator fits),
  `group_cycles` (group by step/cycle; `on_missing='notify'|'fill'` handles
  packet gaps), and `demodulate`. The recommended de-embedded/phase-centred basis
  (via a per-tone `resonator.ResonatorCalibration`) yields the frequency shift
  and dissipation directly through the exact Möbius inversion; a model-free
  raw-phase fallback is also provided.
- Mock mode emulates modulation (resonator phase model + bin grid) for
  hardware-free testing; `client_scripts/test_frequency_modulation.py` exercises
  the whole path. See [Fast Frequency Modulation](doc/frequency_modulation.md).

**Power Sweep**
- `find_best_power` now repairs unphysical negative `chosen_params` (an artifact
  of linearly extrapolating a fit-summary field past the measured power range) by
  substituting the cross-tone median of the valid values for that key, so every
  consumer (including `best_power_arrays`) gets physical values. Signed
  quantities are left untouched; affected tones remain flagged `extrapolated`.

## v1.3.0

Forward-ported the `jl_ocs_devel` branch (PR #10) plus follow-on hardening.

**G3 Streaming Output (so3g)**
- Added `client.receive_stream_g3()` to record a continuous data stream as
  an so3g/spt3g G3 file. The output contains an `Observation` frame, a
  `Wiring` frame describing tone metadata, and `Scan` frames carrying
  `G3SuperTimestream` payloads for I/Q data, packet counters, and PTP
  telescope timestamps. Mock mode is also supported.
- Added the `receive_stream_g3.py` client script for command-line G3
  capture (connect via config file or address/port). The script is shipped
  as a module rather than an entry point, matching the existing
  `receive_stream.py` convention.
- Client install now requires `so3g` (which pulls in `spt3g`).

**Mock Server Mode**
- Added `ReadoutClient(..., mock=True)` for local emulation of the readout
  server without RFSoC hardware, intended for OCS / controller integration
  testing. Mock mode swaps socket traffic for an in-process
  `MockReadoutServer` (`souk_readout_tools.client.mock_readout`) and
  defaults connection details to `127.0.0.1:10000` when no config or
  address is supplied.
- The mock server generates a synthetic resonator catalogue and emulates
  sweeps, snapshots, streams, the full `get_info()` surface, and the new
  G3 stream writer.

**OCS Compatibility Fitting Helpers (`client/res_fns.py`)**
- Forward-ported `souk_readout_tools.client.res_fns` for `UKKIDController`
  compatibility. Provides simple resonator-fitting helpers (`s21_model`,
  etc.). Slated for removal once the core fitting tools in
  `souk_readout_tools` are fully validated in `UKKIDController`.

**Core Resonator Fitting (`fitting.py`)**
- Reworked the linear and nonlinear resonator fitters around a clearer
  parameter convention (`fr, Qi, Qc, phi, A, alpha, tau[, anl]`). `Qc` is the
  fitted real coupling parameter; `Qe` and `Qc_abs` are derived reporting
  fields. Fit setup now uses explicit `initial_guess`, `param_bounds`, and
  `param_fixed` dictionaries.
- Improved nonlinear/Duffing fitting for highly driven resonances. The new
  seed strategy, high-drive handling, optional subsampling, and fit diagnostics
  give much better and faster convergence on distorted resonance shapes.
- Added process-based parallel fitting for `fit_sweep_stack()` and
  `batch_fit()` via `n_jobs`, with compact verbose progress showing throughput,
  fit timing, and cumulative function evaluations.
- Added conservative wideband peak-finder defaults for
  `client.find_resonances(mode='wideband')` and
  `batch_fit(find_resonances=True)`: inner 10-90% frequency trim, dip finding,
  1-100 dB prominence, 1 kHz-10 MHz width, 100 kHz minimum spacing, and
  lowpass 0.5.

**Resonator Transforms and Plotting**
- Deembedding, phase-centering, and sweep plotting now propagate optional S21
  uncertainties (`real=sigma_I`, `imag=sigma_Q`) through the same transforms
  applied to the data.
- Updated the phase-centering convention and calibration construction so
  fitted resonator geometry is handled consistently across fitting, plotting,
  and frequency/dissipation conversion.

**Noise Analysis**
- Added `noise.remove_common_modes_svd()` for NumPy-only common-mode
  subtraction across slow accumulated detector timestreams, plus
  `noise.fractional_frequency_and_dissipation_timestreams()` for converting
  multi-tone I/Q captures with a matching calibration sweep.
- Added `noise.remove_blind_tone_common_modes()` to fit temporal reference
  modes from simultaneous blind-tone amplitude/phase variations and regress
  those modes out of regular-tone I/Q before frequency/dissipation
  conversion. Blind sweep traces are not used.
- `plot_timestream_psd(..., format='freq_diss')` can overlay SVD-cleaned
  and blind-tone-cleaned spectra or plot them in place of the raw spectra.
  Legends record the number of removed modes.

**CSV Import Hardening**
- `import_sweep` and the equivalent stream/snapshot CSV importers now use
  `ast.literal_eval` instead of `eval` for metadata parsing. This removes
  an arbitrary-code-execution sink and fixes a latent crash on values that
  are valid strings but not valid Python expressions (e.g. `fpg_file`
  paths, ISO-8601 date strings, free-form text). All three importers now
  share an identical metadata-parse loop.

**Client API**
- `ReadoutClient.perform_sweep()` and `perform_retune()` now accept a
  `wait=False` keyword. Setting `wait=True` blocks until the sweep
  finishes by calling `wait_for_sweep()` internally after a successful
  dispatch.

**Power Sweep Workflow Polish**
- `power_sweep.run_power_sweep()` now defaults to pre-centering before the
  first saved sweep and following dips between power steps. The initial
  unsaved center-search sweep uses `search_span_factor=2.0` by default so
  edge-clipped dips can be pulled back into the saved sweep span. Pass
  `follow_dips=False` for fixed centers or `search_for_center=False` to skip
  only the initial search.
- `power_sweep.run_power_sweep()` now defaults `follow_min_depth_db=0.5`
  (matching `fit_power_sweep()`'s `min_dip_depth_db` default), so
  `follow_dips=True` no longer chases shallow noise minima between power
  steps. Pass `follow_min_depth_db=None` to restore the previous accept-
  any-depth behaviour.
- Added `power_sweep.best_power_arrays()`, `write_best_power()`, and
  `load_best_power()` for extracting the chosen power, `p_bif`, and
  `p_bif_sub_3db` per tone as 1-D NumPy arrays, and round-tripping the
  full `find_best_power()` result to `best_power.json`.
- `plot_fits()` now disables the shared axis offset annotation
  (`+1.5e9`-style text) on every magnitude/phase/IQ panel it draws.
- `plot_power_sweep()` and `plot_best_power()` are faster on many-tone runs.
  Both gained an `n_jobs` argument (joblib convention: `1` serial default,
  `-1` all CPUs) that renders the independent per-tone PNGs across worker
  processes; `savefig` is the dominant, CPU-bound cost so this scales toward
  the core count (`plot_best_power()` is forced serial when `show=True`).
  `analyse_power_sweep()` forwards its `n_jobs` to both, so the convenience
  path is parallel by default. The default `dpi` dropped from `100` to `80`
  (savefig raster time scales with `dpi**2`); raise it for publication
  figures. Per-tone output is byte-identical between serial and parallel runs.
- Removed the `UserWarning: ... Axes that are not compatible with
  tight_layout` emitted (once per tone) by `plot_power_sweep()` and
  `plot_best_power()` on `mag+phase+iq` layouts. Those figures now lay out via
  `savefig(bbox_inches='tight')` instead of `tight_layout()`, which also
  prevents long calibrated axis labels from being clipped.
- Fixed `analyse_power_sweep(plot=True)` passing an invalid `show=` argument
  to `plot_power_sweep()` (now `show_overlay=`), which raised `TypeError`.

## v1.2.0

**Get Info API**
- `get_info()` now accepts a single section name, such as
  `get_info("timing")`, and returns that section dictionary directly rather
  than requiring callers to pass a one-item list. List requests still work and
  `get_info(["section_a", "section_b"])` returns a list in the requested order.


**RFSoC Timing and PTP**
- Added a packaged `souk-timing-monitor` daemon that polls `ptp4l` via `pmc`
  and chrony via `chronyc`, tracks timing state, and exposes JSON status on
  `/run/timing-monitor.sock`.
- Added packaged timing templates for `ptp4l.service`, `timing-monitor.service`,
  `ptp4l.conf`, and the chrony PHC refclock drop-in.
- Added `souk-enable-timing` to install and start the RFSoC timing services
  from the packaged templates.
- Added `souk-test-timing-monitor` as a small lab client for `status`, `ping`,
  and streaming checks against the live timing-monitor socket.
- Added `client.get_timing_status()` and a `timing` section in `get_info()`.
  `health_check()` now includes `timing_state` and `timing_ready`.
- `get_info("timing")` now presents a compact grouped user-facing view with
  `summary`, `monitor`, `ptp`, `phc`, `ntp`, and `chrony` sections. The
  summary is intended for regular health logs, while the detailed sections keep
  concise parsed values plus raw `pmc`/`chronyc` command output for debugging.
- Timing summaries now expose `ready_for_firmware_sync` as the gate for
  firmware timestamp syncs.
- Timing summaries now include the current and last-seen PTP grandmaster
  identities, compact NTP source error values, and a rough
  PTP-holdover-vs-NTP comparison for extended holdover decisions. The summary
  and monitor sections also expose the holdover drift-rate and policy constants
  used by that estimate.
- `summary.system_synced` now means actively synchronised to a current external
  source. It is false during PTP holdover; `chrony.synced` still reports
  chrony's local `Leap status == Normal` view.
- NTP source entries now include both the configured source name and the
  resolved address where chrony exposes both, plus adjusted/measured offsets
  and the measurement error bound.
- Timing status now includes `ptp_data_fresh`, `ptp_ingress_time_ns`,
  `ptp_seconds_since_ingress`, and `ptp_seconds_since_ingress_update`; stale
  `gmPresent` and `master_offset` values are masked when
  `TIME_STATUS_NP.ingress_time` shows no recent PTP ingress.
- Timing status now stores raw `pmc` outputs and raw `chronyc tracking` /
  `chronyc sources -v` outputs, while keeping parsed values only where the
  monitor or summary needs them.
- PTP holdover policy is configurable via `souk-timing-monitor` CLI options and
  is reported in status as `ptp_holdover_window_s` and
  `ptp_holdover_error_rate_ppm`. When this monitor has observed PTP lock and
  chrony remains selected on `PHC0`, the monitor stays in `ptp_holdover` beyond
  the nominal window and reports `ptp_holdover_expired`.
- Revised monitor startup and stale-ingress handling: starting with no GM but
  chrony selected on `PHC0` is now reported as `phc_free_run` until this monitor
  has observed a PTP lock, and a non-zero `ingress_time` must keep advancing to
  remain fresh.
- If the monitor starts and neither PTP, PHC, nor NTP is selected yet, it now
  reports `initializing` during a configurable startup grace period before
  falling back to `free_run`.
- The packaged chrony PHC refclock uses `prefer` but not `trust`, so NTP can
  reject a free-running PHC if the board starts without a grandmaster and the
  PHC epoch is wrong.
- Added timing documentation and a site commissioning checklist. The packaged
  chrony PHC refclock now defaults to `offset 0`; the lab-only `offset -37`
  workaround can be installed with `souk-enable-timing --offset=-37`.

## v1.1.1

**Removed Deprecated APIs**
- `get_system_information()` (client/server/`firmware_lib`) and
  `get_server_status()` (client/server) have been removed. Use
  `get_info(sections=...)` for structured system state and
  `health_check()` for compact polling.
- Sweep, snapshot, stream, and parsed-data payloads now carry the
  structured info under the `info` key (replacing the legacy flat
  `system_information` key). Plotting and parsing helpers read from
  `data['info']` accordingly.

**LNA Bias Setting Failure Reporting**
- `set_lna_bias_voltage()` and `set_lna_bias_voltage_all()` now propagate
  hardware-side rejections from the upstream `souk-peripherals-control`
  algorithm as proper `status: 'error'` responses (with the original
  diagnostic message and full result still attached) rather than burying
  the failure inside a `status: 'success'` payload.
- Each per-channel result now carries a `success: bool` flag so callers
  can branch on it directly.
- LNA bias config now includes an explicit backend plus `bias_voltage_v`
  (default 1.5 V), `method`, and `blind`; config application sets the
  configured LNA channel when the `i2c` backend is enabled.
- Added explicit `fixed` backends for LNA bias and RF attenuators so
  non-controllable values are represented deliberately instead of by
  blank backend leaves.
- Added `soft_off_lna_bias()` / `soft_off_lna_bias_all()` and the
  `souk-find-lnas --status` discovery CLI.

**RF Peripheral Configuration & Discovery**
- RF frontend config now separates `mixerless_module`, `attenuator`, and
  `bypass_amps` settings. Attenuator values live at
  `rf_frontend.attenuator.tx_value_db` / `rx_value_db`; measured bypass-amp
  S21 values live under `rf_frontend.mixerless_module`.
- The controller no longer uses a software mimic after hardware init
  failures. Failed hardware remains unavailable and subsequent set/get calls
  raise clear errors instead of returning fake-success state.
- Added `souk-find-bypass-amps` and `souk-rf-peripherals-status`, and
  extended `souk-find-attenuators` with `--status`.
- Power-management helpers now distinguish controllable attenuators from
  bypass-amplifier support, so RUDAT/fixed attenuator setups are not asked to
  toggle mixerless-module amps.

**Config / Runtime State Split**
- `pull_config()` (and the `config` section of `get_info()`) now return the
  active config file only. Runtime hardware changes are reported through the
  live status/info APIs instead of being patched into config.
- `sync_config_from_system()` explicitly captures the live state into
  `client.config` and returns the updated config dict. Use
  `sync_config_to_local(save_as=...)` as a clearer alias when creating a new
  local config from the running system.
- When blind tone metadata is active, `sync_config_from_system()` preserves the
  regular/blind split and writes `blind_frequencies`, `blind_amplitudes`,
  `blind_phases`, and `blind_spans` into `firmware.defaults`.

**Sweep Plotter Speed-up**
- `plot_sweep()` `show_errors` now defaults to `None`, which auto-disables
  error fills for wideband sweeps where `fill_between` over hundreds of
  thousands of points dominates render time.  Per-tone sweeps are
  unchanged.  Pass `show_errors=True`/`False` to override.

**Sweep & Tone Update Robustness**
- Frequency-only tone updates now preserve existing tone amplitudes and phases
  when the tone count is unchanged.
- Fast sweep setup can carry amplitudes/phases through every sweep point, and
  protects the VACC from shared-bin amplitude overflow by scaling sweep
  amplitudes and temporarily compensating `psb_scale` when possible.
- Fast sweep/retune paths use the fast tone-frequency writer and direct
  channel-map updates for lower overhead.

**Blind Tone Management**
- Added blind tone support for fixed gain/phase monitor tones.
  Configs can now set `firmware.defaults.blind_frequencies`,
  `blind_amplitudes`, `blind_phases`, and `blind_spans` alongside the
  regular tone list.
- The firmware is programmed with one combined tone list, so blind tones are
  included in Newman phase generation, VACC/shared-bin protection, calibrated
  power setting, dynamic-range optimisation, timestreams, and sweeps.
- Retune and tracking paths keep blind tones fixed at their configured centers
  while only updating regular tones.
- Added interactive client/server helpers:
  `suggest_blind_frequencies()`, `set_blind_tones()`, `get_blind_tones()`,
  `remove_blind_tones()`, `get_tone_metadata()`,
  `get_blind_tone_indices()`, and `get_regular_tone_indices()`.
- `suggest_blind_frequencies()` now jitters the initially even target positions
  by default, reducing regular-grid intermodulation spur alignment while still
  respecting resonance/blind-tone spacing constraints.

**Peak Finder Edge Trim**
- `PeakFinderParams` accepts new `f_low` / `f_high` (Hz) fields to drop
  peaks outside a chosen band.  Useful for excluding band edges where
  filtering artefacts can produce spurious peaks.  Filtering happens
  after `find_peaks` so prominence/width context still uses the full
  band.

**Other Fixes & Improvements**
- `maximise_tx_power()` now accepts `compression_headroom_db` to optionally cap
  total TX frontend input power below the RF frontend input-referred 1 dB
  compression point. The RF peripheral wrapper also corrects the mixerless
  module amplifier P1dB arithmetic without modifying the upstream submodule.
- Mixerless-module measured calibration overrides can now be supplied under
  `rf_frontend.mixerless_module`: per-state TX/RX amp S21, bypass delta S21,
  input-referred P1dB, and module group-delay metadata. S21/group-delay keys
  accept scalars, inline frequency tables, or calibration filenames.
- Wideband sweep with `tone_powers=None` no longer reapplies tone powers
  (preserves whatever the pipeline currently has set).
- Fix late-import bug in `firmware_lib`.
- `perform_sweep()` and `perform_retune()` can refresh the ADC calibration
  before sweeping for improved S21 stability.
- Group-delay estimation and removal: improved cable-delay fit;
  `remove_group_delay()` honours frequency-dependent calibrations.
- Fixed `generate_newman_phases` to return phases in the original frequency 
  order, not the sorted order. 

**Structured Info System**
- `get_info(sections)` provides structured system state in 15 named sections: `server`, `versions`, `clock`, `fpga`, `rfdc`, `pipeline`, `tones`, `rf_frontend`, `lna`, `rfsoc_sensors`, `diagnostics`, `config`, `calibrations`, `resonators`, `registers`.
- `rfsoc_sensors` section reports on-chip PS/PL SYSMON readings via IIO sysfs: die temperatures (C) and supply voltages (V), keyed by the raw sensor names so PS/PL rails with duplicate short names (e.g. `vccams`, `vccint`) stay distinct.
- Each section includes a `ready` flag indicating whether its data could be read from hardware.
- Default call excludes expensive sections (`diagnostics`, `config`, `calibrations`, `resonators`, `registers`); use `'all'` for everything.
- `health_check()` for compact intermittent polling — returns pass/fail bools for clock lock, ADC/DAC saturation, DSP overflow, RTS events, plus key state indicators.
- `rf_frontend` section now includes full signal chain description: hardware identity, attenuator backend details (I2C bus/channel or RUDAT serial numbers), live attenuator/amp state, derived gain/compression, and updownconverter characterisation (LO frequency, sideband, mixer/combiner losses, IF/RF S21).
- `lna` section includes controller status and bias readings (voltage, current) for all 14 channels.
- `config` section returns the active config file as YAML text; mutable hardware
  state is reported by the relevant live info sections instead of being patched
  into the config.  Use `sync_config_from_system()` / `sync_config_to_local()`
  to explicitly capture live settings into a new local config.
- `calibrations` section returns resolved per-tone calibration values (after frequency-dependent interpolation).
- `resonators` section placeholder for future resonator detuning tracking module.
- `registers` section placeholder for future full firmware register dump.
- `tone_indices` renamed to `firmware_indices` in `info['tones']`.
- Server activity flags (`streaming`, `triggered_streaming`, `sweeping`) now exposed in `server` section.
- `firmware_interface_ready` / `firmware_fast_interface_ready` checks added (verifies blocks are present, not just that the object exists).

**Pipeline Parameter Access**
- New client methods: `set_sync_delay()` / `get_sync_delay()`, `set_acc_len()` / `get_acc_len()`, `set_internal_loopback()` / `get_internal_loopback()`, `set_psb_scale()` / `get_psb_scale()`, `set_psb_fftshift()` / `get_psb_fftshift()`, `set_pfb_fftshift()` / `get_pfb_fftshift()`.
- Previously these could only be set via `apply_config()`.

## v1.1.0

**Telescope Time (PTP Timestamps)**
- PTP timestamps from the firmware are now included in every sample frame, stream packet, and sweep point.
- `get_telescope_time()` client method for on-demand timestamp reads.
- `read_tt_fast()` in `firmware_lib` reads the accumulator's `acc_tt_msb`/`acc_tt_lsb` registers via the fast local memory transport.
- Frame slots previously used for unused flags 6 and 7 now carry the 64-bit timestamp (split as `tt_msb` / `tt_lsb`).
- Parsed data dictionaries (`parse_samples`, `parse_stream`, `parse_sweep_data`) include a `telescope_time` field.
- Sweep results include per-point telescope time (timestamp of the first sample at each sweep point).
- CSV, JSON, and NPY exports include telescope time data.

**RF Peripheral Controller**
- `RFPeripheralController` with two backends: mixerless (I2C) and rudat (USB attenuators).
- `sync_config_from_system()` and `get_rf_peripheral_status()` for reading hardware state.
- Global rename: `tx_amp_s21_db` → `tx_bypass_amp_s21_db` (and rx) across the calibration chain.

**Batch Snapshots**
- `batch_snapshots()` method to acquire pre-accumulator snapshots across multiple tones.
- Exports to `.npz` with metadata including firmware channel indices.
- CLI tool: `souk-batch-snapshots`.

**Plotting Library (`souk_readout_tools.plotting`)**
- `plot_sweep()` — S21 magnitude/phase, I/Q vs frequency, or complex plane, with deembedding, phase centering, and error bars.
- `plot_timestream()` — I/Q, magnitude/phase, or frequency/dissipation vs time.
- `plot_timestream_psd()` — power spectral density of timestream data.
- `plot_timestream_on_resonance()` — overlay timestream points on sweep resonance circle.
- `plot_snapshots()`, `plot_snapshots_psd()`, `plot_batch_snapshots()` — snapshot visualization with per-repetition, averaged, and concatenated modes.
- All formats support optional deembedding and/or phase centering via the `resonator` module.
- PSD utilities: `compute_psd()`, `compute_psd_averaged()`, `compute_psd_concatenated()`.

**Resonator Analysis (`souk_readout_tools.resonator`)**
- `remove_cable_delay()` — auto-estimate and remove electrical delay.
- `deembed()` — true RF deembedding: cable delay + baseline normalisation (off-resonance → (1, 0)).
- `apply_deembed_params()` — apply deembed to timestream data.
- `center_circle()` — Kasa algebraic circle fit.
- `rotate_to_real_axis()` — rotate the off-resonance point to the negative real axis.
- `phase_center()` — circle centering + rotation (may act on raw or deembedded data).
- `apply_phase_center_params()` — apply phase centering to timestream data.
- Transform helpers accept optional S21 errors and propagate independent I/Q
  uncertainties through delay removal, baseline division, and rotations.

**Resonance Finding Enhancements**
- `find_resonances(mode='targeted')` — per-tone resonance search with double/triple flagging.
- `find_resonances()` now returns one `ResonanceSearchResult` shape for both
  wideband and targeted inputs, with `all_resonances`, `per_tone`,
  `flagged_tones`, and `mode` available in all cases.
- Fixed `find_resonances(mode='auto', sweep_data=None)` so it performs the
  default wideband sweep instead of returning the string `"wideband"`.
- `flagged_tones` output for tones containing multiple resonances.

**Resonator Fitting (`souk_readout_tools.fitting`)**
- Notch/Duffing resonator model with fitted parameters
  `fr, Qi, Qc, phi, a, alpha, tau[, anl]`; `Qe` and `Qc_abs` are derived.
- `fit_resonance()` / `fit_resonance_nonlinear()` — single-resonance
  least-squares fit with named `initial_guess`, `param_bounds`, and
  `param_fixed` controls.
- `fit_sweep_stack()` — process-parallel fitting of already-windowed sweep
  stacks.
- `batch_fit()` — automatic detection/windowing and process-parallel fitting
  of all resonances.
- `extract_parameters()` — extract fitted and diagnostic parameters into arrays.
- CLI tool: `souk-find-resonances` (with `--fit` option).

**Parameter Space Measurements (`souk_readout_tools.measurement`)**
- `ParameterSweep` — step through external parameter values with set/get callbacks.
- `TimedMeasurement` — periodic measurements at fixed time intervals.
- `ConditionalMeasurement` — measure when a monitored parameter meets a condition.
- Abstract `measure_func(client) → dict` pattern works with any readout measurement.
- `save_measurement()` for exporting results.

**Clock Source Control**
- `get_clock_source()` / `set_clock_source()` for reading and setting the PL reference clock (internal 12.8 MHz or external 10 MHz).
- `get_clock_status()` for querying PLL lock status of all clock chips (LMK04208 + LMX2594s).
- Clock source and lock status included in `get_info()` (under the `clock` section).
- `apply_config()` enforces `firmware.clock_source` on every config push.
- See [clock_source.md](doc/clock_source.md) for full details and manual procedures.

**Path Group Delay Calibration**
- `measure_path_group_delay()` client method measures the full TX+RX round-trip group delay (cable delay) across the band from a wideband sweep.
- Three-stage filtering: resonance masking (using known MKID frequencies and Q-factors to exclude ±N×HWHM around each resonance), median filter on the phase-gradient spectrum to suppress Lorentzian tails, and a low-order polynomial fit to produce a smooth, continuously defined group-delay vs frequency model.
- New config field `rf_frontend.path_group_delay_ns` accepts a scalar (ns), inline `[[freq_hz, tau_ns], …]` list, or path to a two-column CSV calibration file.
- Results can be saved inline to the config or as a CSV calibration file via `save_to_config` / `save_to_csv` arguments; CSV files are registered and transferred automatically with `push_config()` / `pull_config()`.
- `path_group_delay_ns` added to `CAL_FILE_KEYS` so file-backed calibrations are handled consistently with all other RF path calibrations.

**souk-restart-daemon**
- New `souk-restart-daemon` server command to restart the readout server systemd service(s) without a full disable/re-enable cycle.
- Supports `-p 0`, `-p 1`, and `-p 0 1` flags (default: pipeline 0).
- Backed by `restart_systemd_service.sh`, deployed alongside the existing install/remove scripts on first server start.

**Crest Factor Calculator**
- `estimate_papr_db()` in `firmware_lib` for computing the peak-to-average power ratio (dB) of a multitone waveform. Simulates the time-domain composite signal to verify phase/amplitude choices before applying to hardware.

**System Information**
- `get_info()` reports software versions, git info, and RFDC RTS events.
- `check_rfdc_rts_events()` for DAC/ADC overvoltage sticky flag checking.

**Sweep Progress**
- `wait_for_sweep(progress_bar=True)` with ASCII progress bar.
- `wideband_sweep()` prints progress when `verbose=True`.

## v1.0.1

**Dual-Pipeline Support**
- Two independent readout pipelines on a single RFSoC board.
- Per-pipeline config directories, server instances, and client connections.
- 3-level initialisation state machine (programming / shared / pipeline) to avoid disrupting the other pipeline during init.
- `ensure_ready(level=...)` for safe, minimal-disruption initialisation.
- `hard_reset()` for explicit full FPGA reprogram.

**VACC Multitone (v7.9 Firmware)**
- Support for multiple tones per FFT bin via the Vector Accumulator (VACC).
- Inmap semantics for PSB channel mapping (`inmap[lo_index] = fft_bin`).
- Non-contiguous tone index allocation with minimum separation of 6 (dual-port RAM constraint).
- `compute_vacc_tone_indices()` for optimal LO index assignment.

**Wideband Sweep**
- `wideband_sweep()` method for surveying the full RF bandwidth with parallel multi-tone sweeps.
- Automatic saturation checks before sweeping.
- Optional linear phase slope removal.
- CLI tool: `souk-wideband_sweep`.

**Resonance Finding**
- `find_resonances()` and `find_resonance_frequencies()` integrated into the client.
- Multiple data format options (log magnitude, phase, group delay, complex gradient, etc.).
- Configurable filter and peak-finding parameters (`FilterParams`, `PeakFinderParams`).
- Returns `ResonanceResult` objects with frequency, FWHM, Q factor, Qc, Qi, dip depth.
- Interactive GUI: `souk-mkid-finder-app`.

**Pre-Accumulator Snapshots**
- `get_accumulator_snapshots()` for acquiring high time-resolution pre-accumulation data on a single tone (1024 samples per snapshot at FFT output rate).

**Configuration Management**
- `push_config()` / `pull_config()` for transferring configs and calibration files between client and server. Configs and calibration data are held in memory; pushed configs are always saved on the server, local saving is via `save_config()`.
- `save_config()` writes the in-memory config and any pulled calibration files to a local YAML file and `calibrations/` directory.
- `apply_config()` detects changed parameters and applies hardware changes without full reinitialisation.
- Client config files live wherever the user chooses; server uses pipeline-specific directories on the RFSoC.

**Power Calibration & Optimisation**
- `set_tone_powers()` / `get_tone_powers()` with full calibration chain and selectable `reference_plane` covering the entire signal chain: TX (`'dac'`, `'rf_output'`, `'detector'`) and RX (`'cryostat_output'`, `'adc_input'`, `'accumulator'`).
- Saturation detection: `check_input_saturation()`, `check_output_saturation()`, `check_dsp_overflow()`.
- Auto-optimisation: `maximise_tx_power(headroom_db)`, `maximise_rx_power(headroom_db)`, `optimise_tx_snr()`, `optimise_rx_snr()`.
- Auto-fix: `fix_dac_saturation()`, `fix_adc_saturation()`.

**Other**
- `generate_newman_phases()` for optimal crest factor minimisation.
- `set_tones_helper()` convenience method for setting frequencies, powers (dBm), and phases in one call.
- Triggered streaming with `enable_triggered_stream()` and `send_fake_trigger()`.
- Generic parameter access via `set_parameter()` / `get_parameter()`.
- Server daemon management: `souk-enable-daemon` / `souk-disable-daemon`.

## v1.0.0 (Initial Release)

- Client-server architecture with TCP request/stream protocol.
- Basic tone management (set/get frequencies, amplitudes, phases).
- Discrete sample acquisition (`get_samples`, `parse_samples`, `export_samples`).
- Continuous and triggered streaming.
- Frequency sweeping (`perform_sweep`) and retuning (`perform_retune`).
- YAML-based configuration.
- Cross-platform support (Linux and Windows).

---

## Future Developments

Planned for upcoming releases:
- Quick on/off resonance switching for noise characterisation.  
- Automated resonator tracking (~~continuous retune loop with drift correction~~ using d2phi/df2 from frequency modulated timestreams).
- More interactive plotting features (eg step to next resonance, flag as good/bad)
- ADC calibration via loopback measurement.
- Improved VACC tone backfilling for more efficient LO slot usage.
- Dual-DAC mode support for improved dynamic range.
- Automated version numbering and release workflow.
