# Changelog & Feature List

## v1.2.0 (Current)

**Structured Info System**
- `get_info(sections)` replaces the monolithic `get_system_information()` with 14 named sections: `server`, `versions`, `clock`, `fpga`, `rfdc`, `pipeline`, `tones`, `rf_frontend`, `lna`, `diagnostics`, `config`, `calibrations`, `resonators`, `registers`.
- Each section includes a `ready` flag indicating whether its data could be read from hardware.
- Default call excludes expensive sections (`diagnostics`, `config`, `calibrations`, `resonators`, `registers`); use `'all'` for everything.
- `health_check()` for compact intermittent polling — returns pass/fail bools for clock lock, ADC/DAC saturation, DSP overflow, RTS events, plus key state indicators.
- `rf_frontend` section now includes full signal chain description: hardware identity, attenuator backend details (I2C bus/channel or RUDAT serial numbers), live attenuator/amp state, derived gain/compression, and updownconverter characterisation (LO frequency, sideband, mixer/combiner losses, IF/RF S21).
- `lna` section includes controller status and bias readings (voltage, current) for all 14 channels.
- `config` section returns raw YAML config text with comments preserved, plus `config_matches_applied` (in-memory config vs last applied to firmware).
- `calibrations` section returns resolved per-tone calibration values (after frequency-dependent interpolation).
- `resonators` section placeholder for future resonator detuning tracking module.
- `registers` section placeholder for future full firmware register dump.
- `tone_indices` renamed to `firmware_indices` in new info output (backward-compatible key retained in legacy `get_system_information()`).
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
- `rotate_to_real_axis()` — rotate resonance to negative real axis.
- `phase_center()` — circle centering + rotation (may act on raw or deembedded data).
- `apply_phase_center_params()` — apply phase centering to timestream data.

**Resonance Finding Enhancements**
- `find_resonances(mode='targeted')` — per-tone resonance search with double/triple flagging.
- `flagged_tones` output for tones containing multiple resonances.

**Resonator Fitting (`souk_readout_tools.fitting`)**
- Khalil notch-type resonator model: `S21 = a*exp(jα)*exp(-2πjfτ)*(1 - Ql/|Qc|*exp(jφ)/(1+2jQlΔf/fr))`.
- `fit_resonance()` — single resonance nonlinear least-squares fit.
- `batch_fit()` — automatic detection and fitting of all resonances.
- `extract_parameters()` — extract fitted parameters into arrays.
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
- Clock source and lock status included in `get_system_information()`.
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
- `get_system_information()` reports software versions, git info, and RFDC RTS events.
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
