# Changelog & Feature List

## v1.4.0 (Current)

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
- Timing/sync management to tie together the ADC and CPU clocks
- Quick on/off resonance switching for noise measurements.  
- Blind tone management and common-mode noise removal.
- Automated resonator tracking (continuous retune loop with drift correction).
- More plots in the docs and examples.
- More interactive plotting features (eg step to next resonance, flag as good/bad)
- ADC calibration via loopback measurement.
- Improved VACC tone backfilling for more efficient LO slot usage.
- Dual-DAC mode support for improved dynamic range.
- HDF5 export format support.
- Automated version numbering and release workflow.
