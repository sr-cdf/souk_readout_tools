# Changelog & Feature List

## Unreleased

**The accumulation counter is read over devmem, not katcp.** `acc.get_acc_cnt()`
is `read_uint('acc_cnt')` through the full casperfpga block path -- 27.1 us
against 3.7 us for a cached mmap read, and it runs twice per sample. New
`read_acc_cnt_fast` / `_blocking_wait_for_acc_fast` save **46.8 us on every
accumulation, at every tone count**, and make the 10 us poll period in
`_blocking_wait_for_acc` meaningful for the first time (each iteration
previously cost 27 us, so the period was inoperative).

**Partial accumulator reads for small tone sets.** `plan_accumulator_reads`
groups requested channels into contiguous runs, merging runs closer than
~37 channels (where bridging is cheaper than another transaction), and
`read_accumulated_data_fast` reads only those. Falls back to the unchanged bulk
read whenever that is cheaper, so a full 2048-channel comb is untouched.
`sparse_read=None|True|False`; the plan is cached per tone set.

Both together, measured on v7.12.1: 1 tone 224 -> 36 us (6.2x), 64 -> 41 (5.5x),
341 -> 65 (3.8x), 1000 -> 109 (2.6x), 2048 -> 278 (1.2x, counter only).

Partial reads are snapped to 64-byte boundaries. `mm[a:b]` is a memcpy, and
glibc's aarch64 memcpy finishes odd-length copies with overlapping unaligned
vector loads, which SIGBUS on the uncached `/dev/mem` mapping -- reading 341
channels (2728 B) bus-errors where 64 channels does not.

**Staged control-buffer writes measured and rejected.** A bulk slice write
streams ~4x faster per word than scattered stores, but constructing the payload
costs more than that saves: scatter vs staged, tx+rx, 4 tones 50/136 us, 1000
244/378, 2048 430/579. `sparse_write=None` always scatters; `True` keeps the
staged path for re-testing if the construction is ever made cheap. New
`profiling/access_size_scaling.py` measures the fixed/per-word cost model these
decisions rest on.

## v1.8.0

**Validated against firmware v7.12.1.** The minor version tracks the firmware
generation. v7.12 code support shipped in v1.7.10; this release marks it verified
on hardware. Sweep step **375 us -> 72 us (5.2x; 6.0x in C)**, read 84 -> 26.7
ns/word, write 12.3x. The 16-bit `synth_input_reorder` map needs **no** byte swap,
and sparse 32-bit writes into the 128-bit AXI4 BRAM lines do not clobber their
neighbours (byte-write strobes confirmed on hardware). `packet_error` is now 0 at
sample rates up to 2000 Hz, where v7.11 showed errors.

**The board needs more than a new `.fpg`:** casperfpga `a89480c`, control_sw
`7.12.1`, souk_readout_tools >= 1.7.10, plus `BOOT.BIN`, `top.bit` and `image.ub`.
`BOOT.BIN` is a performance dependency (200 MHz AXI clock), not a correctness gate.
New `doc/v712-notes.md` carries the register moves, the boot-source finding, the
in-place update procedure and the measured numbers; `profiling/README.md` gains the
v7.12.1 results table alongside the v7.11 rows.

**`timing.format_timing_status()` renders `get_info('timing')` as seven lines.**
The raw dict is ~60 keys across six sub-dicts. Surfaces the TAI/UTC offset
explicitly: PTP runs on TAI, so `telescope_time` is `currentUtcOffset` (37 s) ahead
of UTC, which is why chrony reports PHC0 as a falseticker unless given `offset=-37`.

## v1.7.10

**Firmware v7.12: the reorder map word width now comes from the firmware.**
v7.12 narrows `VaccReorderMultiSampleIn._map_format` to `>i2` (the
`p*_synth_input_reorder_map*_map1` registers halve, `0x2000` -> `0x1000`), but
`psb_chanselect_set_channel_inmap` / `psb_chanselect_get_channel_inmap` pinned
`_fast_inmap_fast_nbytes` / `_fast_inmap_fast_dtype` to 4-byte words and took only
the byte order from `_map_format`. Both now derive the itemsize too; behaviour on
v7.11 (`>i4`) is unchanged.

**v7.12 also needs, on the board:** casperfpga `a89480c` (`transport_localmem.MAP_SIZE`
32 MiB -> 512 MiB), control_sw `7.12.1`, and the new `BOOT.BIN` (`psu_init` enables
HPM1 at 128 bits and doubles the AXI control clock to 200 MHz) — a new `.fpg` alone is
not enough. `p*_acc0_dout0` and `p*_mix_{rx,tx}_lo0_control{0,1}` move to AXI4 BRAMs at
`0xB000_0000`+; all 224 register names are unchanged. `tcpborphserver3` needs no rebuild
(it sizes its own window from the `.fpg`).

**Fast tone writes no longer rebuild the whole-map view on every call.**
`prepare_control_buffer_data_fast` tested `tx_lo_control_buffer_mv` /
`rx_lo_control_buffer_mv` but assigned `tx_lo_control_buffer` / `rx_lo_control_buffer`.

**Profilers reach the v7.12 BRAM region.** `profiling/devmem_profile.{py,c}` `MAP_SIZE`
32 MiB -> `0x20000000`; `fpg_addrs.py` finds the v7.11 `..._lo0_control0` rename;
`dump_addrs.py` uses the `(lo, buf)` key. Both were already broken on v7.11.

**`souk_readout_tools.__version__` exists.** Read from the install metadata
(`importlib.metadata`), so `setup.py` stays the only place the version is written;
`'unknown'` when imported from a source tree with no `.egg-info`/`.dist-info`.

**`get_info('versions')` says which machine it describes.** The section is the
board's install, never the client host's; it gains a `host` key (the board's
nodename), and `info_versions` / `ReadoutClient.get_info` docstrings say so.
`souk_readout_tools_commit` is None on a non-editable pip install (no `.git`).

**A board deployed without `.git` can still report its commit.** `data/COMMIT`,
written by `git describe --tags --always --dirty` in the checkout before
deploying, is read back by `info_versions` when no repo is found (a live `.git`
still wins). Gitignored, shipped by `package_data`; see `doc/installation.md`.

## v1.7.9

**Board management over USB serial: `souk-find-boards`, `souk-board-status`,
`souk-board-power`.** New `board_management` module reaches the KRM-4ZU47DR
carrier BMC from the control PC (pyserial, a new client dependency). Read-only
by construction: `_send()` accepts only `READ_COMMANDS`, and `power_on()` /
`power_off()` are the only writers, each needing `confirm=True` and an
explicitly named board. Boards are selected by USB serial, device name, or the
board serial the BMC reports.

**Board selection is cross-platform**; udev aliases and `dialout` are not.
`_looks_like_device()` accepts `COM7` / `\\.\COM10` as well as `/dev/ttyUSB1`,
and `--udev` (rules template in `data/udev/99-souk-rfsoc-serial.rules`) refuses
to run off Linux, pointing at USB-serial selection instead. See
`doc/board_management.md`.

## v1.7.8

**Sound card as a stream-to-dac output: `alsa` backend.** New
`stream_dac.AlsaDac` (optional `pyalsaaudio`) drives any ALSA playback device;
`DAC0`/`DAC1` map to the first two audio channels, so LabJack configs port over.
The card owns the clock: `write()` only sets a target level and a writer thread
emits one ramped period per `pcm.write`. Config `dac.alsa` (`device`, `rate`,
`channels`, `period_size`, `full_scale_volts`); `make_dac_backend`'s second
argument is now `options`, not `labjack_options`. Linux only: the guarded
import falls back to `dummy` on Windows and macOS.

**Line outputs are AC-coupled** — no DC, so `idle_voltage` and `v_offset` are
inert there. `--backend alsa --selftest` (`stream_dac.alsa_selftest`) measures
`full_scale_volts` and the coupling corner; `--list-audio-devices` finds the
device string. See `doc/stream_to_dac.md`.

## v1.7.7

**Tone powers can be allocated from measured sensitivity instead of power.**
`transducer_gain()` measures `|dz/dx|` from the sweeps a power sweep already
takes, `carrier_levels()` each tone's ADC level. On a 379-tone array `|dz/dx|`
predicts the measured noise floor at r = -0.91, tone power at r = -0.13.

**`optimise_tone_powers(run, epsilon=, max_loss_db=)` chooses the powers**, and
`allocation_metrics()` / `plot_allocation()` score a comb before it is run.
`max_loss_db` limits how far any one tone may be sacrificed; bifurcation caps
override it, listing those tones in `loss_floor_overridden`. Powers past the
swept range are held at the nearest measured point and flagged in
`outside_swept_range`, not extrapolated. `objective='inverse_variance'`
maximises mapping speed but left 305 of 379 tones worse off; default
`'mean_db'` weights every detector equally.

**`epsilon` — dB of readout noise per dB of ADC fill — is measured per system,
never assumed**, and is a local slope, not a constant.
`scripts/adc_headroom/01_attenuation_ladder.py` measures it,
`02_analyse_ladder.py` reduces it, `adc_fill_response()` fits it, returning
`epsilon_high_fill` / `epsilon_low_fill` and warning when they differ. The
quantisation `adc_level` ladders give 0.57 over the first 12 dB of attenuation
and 0.97 over the last 12 for a 1000-tone comb, 0.07/0.64 for 10 tones.

**Compressed ladder steps are detected and refused.** `check_input_saturation`
cannot see them; the giveaway is noise-to-carrier that *improves* with added
attenuation. `adc_fill_response(level_db=...)` drops steps whose fill response
falls below `min_fill_response` (0.8 dB/dB).

**About 5 dB of ADC range is wasted on every on-resonance capture**, because the
RX optimiser runs with the tones parked off their dips. Measured 4.7-6.5 dB
across a 15-point blackbody ladder; reported as `unused_headroom_db`.

**Driving harder only helps if the tone is read off the dip centre.** At the dip
minimum `|dz/dx|` flattens and then falls approaching bifurcation; at the
steepest-phase point it keeps climbing. `transducer_gain(at='max')` (default)
follows the steepest point, `at='tone'` the dip centre — different best combs.

**`noise.rx_gain_ratio()` and `match_rx_gain=`** let one calibration sweep serve
a timestream taken at a different ADC drive; an injected 6 dB mismatch corrupts
the conversion by 53 sigma uncorrected, 0.19 corrected. A sweep taken at a
different *tone power* cannot be reused: a different shape, not the same shape
scaled.

Details and worked calls in `doc/tone_power_notes.md`.

## v1.7.6

**Streaming no longer dies once the accumulation counter passes 2\*\*31.**
`acc_cnt` free-runs since FPGA programming and overflows int32 after 24.9 days
at 1 kHz (4 hours at `acc_len` 4); NumPy 2 then raised `OverflowError: Python
integer ... out of bounds for int32` in `prepare_frame` on every frame and no
data was sent.

**Stream header words are read and written unsigned.** All 10 (counters, both
halves of the telescope time, the modulation tag, flags) go through one `<u4`
view of the header block in `prepare_frame`, `parse_samples`, `stream_data` and
`parse_stream_data`; only the I/Q payload is signed. Wire format unchanged, and
3 us/frame cheaper than the per-word int32 casts it replaces.

## v1.7.5

**New `timeseries` module.** Timestream and PSD analysis as a flat set of tools,
each returning a plain dict of arrays and each taking a `parse_samples` dict, a
complex array, or an `(i, q)` pair with `sample_rate=`:

- `timestream_psds`, `snapshot_psds`, `stitch_psds` — PSDs of the accumulated
  and snapshot streams, and the composite trace joining them.
- `cross_psd` — cross spectrum, `coherence`, `cross_phase` and the per-bin 2x2
  Hermitian `matrix`. Amplifier and TLS noise cancel, leaving the common
  quasiparticle term.
- `psd_band_levels`, `fit_psd_model` — per-band levels, and
  `S(f) = A f^-alpha + W` fitted in log space with errors, covariance and flags.
- `subtract_psds` — on-minus-off detector noise, shaped like a `timestream_psds`
  result so the fitters apply to it unchanged. Replaces the 100-line recipe in
  `doc/resonator_noise_workflow.md`.
- `combine_quadratures`, `estimate_beta`, `beta_from_kernels`,
  `signal_direction_from_loads` — GLS combination of the two quadratures, and
  the three tiers of `beta`. Where frequency is TLS-limited and dissipation
  amplifier-limited, `weights='closed_form'` gained 21x against 1.03x for
  scalar weights.
- `fit_gr_rolloff`, `fit_glitch_pulses` — `tau_qp` from the spectrum and from
  pulses; on a synthetic 100 µs population they agree to 100.0 vs 100.0 µs.
- `find_glitches`, `masked_psd`, `allan_deviation`, `timestream_stats`,
  `accumulator_response`, `export_psds` / `load_psds` / `export_summary`.
- Plots: `plot_allan_deviation`, `plot_timestream_histogram`, `plot_coherence`,
  `plot_composite_psd`.

Exports reload under numpy's default `allow_pickle=False`. The conventions a
plausible implementation gets wrong — the `2·beta` referral, the fixed
`tau_res` pole, deglitching before fitting, the `1/n_eff` coherence floor, the
shared on/off responsivity — are in `doc/timeseries_analysis.md`.

**`plot_timestream_psd` gains `return_data=`**, returning `(fig, result)`, and
delegates its compute loop to `timeseries.timestream_psds`. Plotted output is
unchanged, verified line-by-line over 55 cases.

**PSD compute moved `plotting/_psd.py` → `timeseries.py`** (`compute_psd`,
`compute_psd_averaged`, `compute_psd_concatenated`, `log_bin_psd`,
`carrier_power`) and readout correction `plotting/_common.py` → `noise.py`.
Both still re-exported, so existing imports are unaffected. Dependency direction
is now `plotting` → `timeseries` → `noise` → `resonator`.

**`compute_psd_concatenated` gains `detrend_blocks=`, defaulting to
`'constant'` — this changes snapshot PSD numbers.** `plot_snapshots_psd` and
`plot_batch_snapshots` now demean each block; a 3% block-to-block offset lifted
the sub-`fs/L` level ~1.8x. Pass `psd_kwargs={'detrend_blocks': False}` for the
old behaviour.

**`batch_snapshots()` now records `info` and `date` at capture time** and copies
`info` into each per-tone dict, so a single tone stays self-describing. Without
it `snapshot_psds(format='freq_diss')` fell back to the sweep midpoint, 41 dB
out; earlier captures are rebuilt from `tone_frequencies`.

**"Readout noise", not "amplifier noise", throughout.** The off-resonance
capture measures whatever the readout chain contributes, which on this
instrument may be ADC- rather than amplifier-limited. No numbers change.

**`_get_sweep_trace` deduplicated** onto the `noise.py` copy; a negative tone
index now raises instead of indexing from the end.

## v1.7.4

### Breaking changes

**`maximise_tx_power`: `headroom_db` renamed to `dac_headroom_db`.** The old name
now raises rather than being silently dropped. New `dac_saturation_threshold`
(default 0.90) sets the trip fraction for every check the routine makes; pass
~0.999 to run the DAC up to clipping.

**Default `target_anl` is now 0.1**, was `0.01` in `find_best_power`,
`analyse_power_sweep` and `plot_best_power`. Chosen powers move up ~10 dB, into
the swept window where they belong.

### Corrected measurements — earlier numbers were wrong

**Every calibrated RX power was 6.02 dB low.** The power model normalised ADC data
by 2^15, but ADC full scale is 2^14. Every RX dBm/volts/watts/S21 number rises
6.02 dB; tone powers and the saturation/headroom logic are unchanged. Confirmed
against a calibrated synthesiser: full scale at +3.37 dBm, 900 MHz.

**`calc_adc_input_power` was a further 3.01 dB low**, from a `/sqrt(2)`
inconsistent with the peak-referred `adc_dbm_to_dbfs`.
`get_tone_powers(reference_plane='adc_input'|'cryostat_output')` reads 3.01 dB
higher, `'accumulator'` 3.01 dB lower. No plotted value changes.

**ADC DDC fine-mixer scale defaults to 1P0 and is always programmed explicitly.**
`adc_mixer_scale_1p0: false` now programs `MIX_SCALE_0P7` rather than AUTO, which
read back as 0P7 and cost a silent 3 dB in every calibrated RX power. Re-measure
`adc_dbm_to_dbfs` against the scale in use; `doc/calibration.md` gains a
full-scale definition and a loopback procedure.

**Blank mixerless-module calibration no longer costs ~16 dB of tone power.**
`rf_peripherals._get_amp_s21` returned the bare amplifier gain as the whole module
S21; it now returns the module S21 excluding the variable attenuator. Model error
drops from 5–16 dB to ≤1.2 dB across all four TX/RX × enabled/bypassed states.
Affects runs whose `system_info` shows
`tx_bypass_amp_s21_source: peripheral_model` — dBm labels only.

**Fast ADC/DAC snapshots no longer come back with sample pairs transposed.** `'<h'`
unpacking swapped adjacent 16-bit samples, so **every DAC snapshot spectrum was
mirrored about DC** and ADC spectra were scrambled — treat earlier conclusions from
them accordingly. Ordered uses only; saturation and headroom results were
bit-identical. Hardware-confirmed.

**Bundled template converter calibrations replaced with measured values.**
`dac{0,1}_dbfs_to_dbm` `-6.0` → `-9.0`, `adc_dbm_to_dbfs` `+12` → `+3`, with the
`calc_*` fallback following. Both are flat approximations across a band that rolls
off ~2 dB; use a calibration file for absolute work.

**Frequency-dependent calibrations interpolate linearly.** `[[f, dB], …]` entries
and CSV cal files were resolved nearest-neighbour; both now use `np.interp`, still
clamped outside the calibrated range.

### RF power optimisation

**The power optimisers report how close each chain runs to compression, and RX
enforces it.** Whenever the frontend model supplies an input P1dB,
`maximise_tx_power` and `maximise_rx_power` log the margin and return it as
`result['{tx,rx}_compression']`. On RX, settings inside `compression_headroom_db`
(default 3 dB) are refused — the module's variable attenuator sits *after* the
preamp, so enabling the amp drops the input-referred P1dB by roughly its own gain.
`compression_check=False` turns it off; TX remains limited only by the existing
opt-in `compression_headroom_db`.

**`maximise_rx_power` tries the RX amp before it drops the attenuator.** It did the
reverse, so switching the amp on could only saturate and it was bypassed again with
no control left to recover. In loopback that parked the ADC at -6 dBFS with the
attenuator already at 0 dB. The amp is now enabled with the attenuator at maximum,
and step 4 trades that attenuation back — the order TX already used.

**`maximise_tx_power(rx_policy='maximise')` runs one RX optimisation, not a dozen.**
Every intermediate TX step called a full `maximise_rx_power()` only for the next
step to undo it: ~12 passes and ~40 s with an RF frontend. Those now take the cheap
`'protect'` check, and the full pass runs once against the final TX level.

**The RX attenuator steps back at its own 0.5 dB resolution, not in 3 dB jumps.**
The first jump now rounds towards more attenuation so it never exceeds the measured
headroom, and both step-backs move one 0.5 dB step at a time. On the reported case
(21 dB, peak -3.3 dBFS) this lands at 19.0 dB / -1.3 dBFS instead of 20.5 dB /
-2.8 dBFS. The DSA jump rounds the same way.

**`fix_dac_saturation` sizes the reduction from the measured peak instead of
halving.** It now halves only while the peak is pinned at full scale, then moves
straight to `headroom_db` below full scale from the measured peak. The reported
case costs 0.19 dB rather than 6.1 dB.

**Intermittent DAC saturation is chased in 0.5 dB steps against a longer capture,
not cut by a flat 3 dB.** It re-checks over 500 captures first; if that does not
reproduce the saturation, psb_scale is left alone. `intermittent_backoff_db`
becomes the ceiling on the total reduction rather than the amount, so recovery
costs measurement time instead of output power.

**Measured DAC headroom no longer under-reports.** `_measure_dac_headroom_db`
averages 250 snapshots like the saturation check instead of one, so the two agree
on a multitone comb's crest factor. Expect reported headroom to drop by a few dB.

**`set_tone_powers` takes `dac_saturation_threshold` too.** It was stuck at 0.90,
so asking for a single tone at the DAC calibration power failed with *"Target power
exceeds maximum achievable by 1.0 dB"*.

### Wideband sweep

**`wideband_sweep` defaults to 80% of the available bandwidth**, down from 90% — on
a 2457.6 MHz band that is 1966.1 MHz, keeping the edge tones out of the DAC
interpolation filter roll-off. Spans above 90% still warn; pass `bandwidth_hz`
explicitly for the old default.

**`wideband_sweep` takes every margin and step control it touches, and forwards
each to the call that honours it.** It accepted three and called the saturation
fixes bare, so a sweep asking for 0.1 dB of DAC headroom had its drive reset to the
server default the moment saturation was detected. Adds `dac_saturation_threshold`,
`atten_step_db`, `dsa_step_db`, `rts_as_saturation`, `compression_check`,
`tx_compression_headroom_db`, `rx_compression_headroom_db`,
`intermittent_backoff_db` and `backoff_step_db`, all defaulting to `None` and all
exposed as `--flags`. See the table in `doc/tone_power_notes.md`.

### Power-sweep analysis and plotting

**Parameter outlier clipping follows the physical trend.** `find_best_power`'s
per-parameter MAD check detrends against the five nearest powers instead of one
global Theil-Sen line, so the knee near bifurcation no longer clips the top power
step. Chosen powers move by a median of 0.00 dB.

**Power-sweep plot defaults: no all-tones overlay, always a comb-level view.**
`plot_power_sweep(save_overlay=...)` defaults to `False`. New
`plot_best_power_vs_frequency` writes `plots/best_power_vs_frequency.png` whenever
powers were chosen.

**Wideband sweeps plot in S21 units.** Per-tone TX powers are expanded onto the
concatenated trace instead of raising `ValueError`.

**Single-tone snapshot plots accept batch snapshot data.** A one-tone
`batch_snapshots()` dict is unwrapped automatically; a multi-tone one raises an
error naming the tones and pointing at `plot_batch_snapshots()`.

### Other

**`get_info('versions')` reports the git commits again.** `_get_git_commit` handles
a submodule's gitdir file and root-owned checkouts, and repository paths are no
longer hardcoded to `/home/casper`.

## v1.7.3

**One notch convention for the linear and driven models**
- `s21_model`'s linear branch put the full complex `1/Qe` in the notch denominator
  (`1/Qi + 1/Qe`); the Duffing branch used the loaded `Qr_inv = 1/Qi + Re(1/Qe)`.
  Both now use `Qr_inv` — the loaded inverse Q, not `Re(1/Qe)` itself — so the
  asymmetry is a numerator phase throughout (Probst/Swenson), in one shared
  `_resonator_factor`.
- `anl -> 0` now reproduces the linear model exactly for `phi != 0`, and fitted `fr`
  is the frequency of maximum `|dS21/df|` at any `phi`.
- `Qi`, `Qc`, `phi` and fit quality unchanged (residual RSS identical on 384 measured
  sweeps). `fr` moves by `fr*Im(1/Qe)/2`, median 5.9 kHz. The `nonlinear='auto'`
  linear-vs-Duffing `fr` gap drops from a median 10% of a linewidth to 3e-6.
- `ResonatorCalibration.frequency_origin_fraction` is 0 for a linear fit; driven fits
  are unaffected.
- `_s21_linear_jacobian` re-derived (the `phi` column now carries two terms); worst
  relative error 3e-6 against central differences. Linear fits stay ~7 ms.

## v1.7.2

**Duffing fits can no longer come back worse than their own linear seed**
- `fit_resonance_nonlinear` never scored the winning nonlinear candidate against the
  linear pre-fit it started from, so a strayed optimiser could report a large spurious
  `anl` with residuals hundreds of times worse.
- The refinement is now compared against the linear seed; if it loses, the fit is
  redone with `anl` pinned at the floor. New `FitResult.linear_seed_guard` flag;
  `param_fixed={'anl': ...}` disables the guard.
- On a 384-resonator, 16-step (-100 to -70 dBm) sweep: fires on 1.8% of fits, cuts
  worse-than-seed fits from 103 to 12, removes 65 of 81 spurious `anl > 1`. Median
  cost unchanged at 38 ms/fit.

**`sharex` / `sharey` on the two-panel timestream and snapshot plots**
- `plot_timestream_psd()`, `plot_timestream()`, `plot_snapshots_psd()` and
  `plot_snapshots()` take `sharex` and `sharey`.
- `sharex` defaults `True`. The PSD functions default `sharey=None`: on for
  `'freq_diss'`, off for `'iq'` and `'magphase'`.
- Sharing stays live as traces are overlaid. Passing an existing `fig=` can add
  sharing but cannot remove it.

**Legends no longer squash overlaid plots**
- Legends are excluded from the layout calculation and spill into up to four columns
  past eight entries, so axes keep full size however many traces are overlaid.
  Affects every plotting function.
- `tight_layout` now applies to the figure being drawn rather than matplotlib's
  current figure.

**PSD display units: dBc/Hz and dB**
- `plot_timestream_psd(psd_units=...)`: `'linear'` (default), `'db'`, or `'dbc'`. The
  dB options switch to log-x; conversion happens after log binning.
- `'dbc'` divides the raw-unit panels by carrier power `|mean(z)|**2` and halves the
  one-sided density, giving the IEEE single-sideband `L(f) = S(f)/2`. Rejected for
  `format='freq_diss'` — use `'db'` there.
- `plot_snapshots_psd()` takes the same argument, so snapshots and the accumulated
  timestream overlay in dBc/Hz: the normalisation cancels the `acc_len` offset.
- **Convention change:** `scripts/quantisation/quantisation_noise_test.py` now reports
  the same single-sideband `L(f)`. Values recorded before read 3.01 dB noisier — do
  not mix old and new runs on one panel.
- See [Resonator Noise Workflow](doc/resonator_noise_workflow.md#noise-to-carrier-units).

**Common-mode removal without resonators**
- `decorrelate_modes` and `blind_tone_modes` now apply to `format='iq'` and
  `'magphase'`, not just `'freq_diss'`. Neither needs a sweep, so loopback data can be
  decorrelated.
- New `noise.remove_iq_common_modes_svd(ts_data, n_modes, tones=...)`;
  `decorrelate_tones` defaults to every non-blind tone.
- See [Resonator Noise Workflow](doc/resonator_noise_workflow.md#without-resonators-iq-and-magphase).

**Quantisation-noise test — `scripts/quantisation/`**
- `quantisation_noise_test.py` measures whether moving digital gain downstream costs
  SNR, walking tone amplitude against `psb_scale` (and the FFT shift schedule) as a
  ladder at constant DAC drive. Reports dB of noise per dB of gain moved downstream
  (0 = free, 1 = quantisation-limited), as both a broadband floor and a
  band-integrated statistic, which do not track each other.
- Measures the reference point first and last, checks DSP overflow and DAC saturation
  at every point, and pins the RX chain. `--loopback rf` repeats through the real path.
- `quantisation_noise_test_plot.py` plots the CSV.
- See [Tone Power Notes](doc/tone_power_notes.md#where-the-digital-gain-is-taken).

**Tone-power headroom: correct, and settable everywhere.** A bare
`maximise_tx_power()` now drives the RF frontend to maximum and gives ~1.9 dB more
digital drive; pass `headroom_db`/`reference_plane` explicitly to keep the old levels.
- `maximise_tx_power()` defaults to `reference_plane='detector'`; use
  `reference_plane='dac'` or `digital_only=True` for the old digital-only behaviour.
- `headroom_db` was double-counted — the ramps stopped at the 90% saturation trip
  (0.92 dB) and then applied `headroom_db` on top, so 1 dB gave ~1.9 dB. Now measured
  from true full scale; requests inside the 0.92 dB floor clamp to it.
- Defaults unified at `firmware_lib.DEFAULT_DAC_HEADROOM_DB` /
  `DEFAULT_ADC_HEADROOM_DB` (1.0 dB), forwarded only when set.
- ADC snapshot levels are now a true fraction of full scale (88% of full scale read
  `0.44`), which had made `maximise_rx_power`'s headroom 6.02 dB optimistic. Trip
  point unchanged; `adc_levels` values and the printed threshold (now 90%) change.
- `dac_headroom_db` is measured per component, not on the complex magnitude.
- `check_input_saturation()` reports `level_saturated` and `rts_saturated` separately
  and the RX loops name the cause; `rts_as_saturation=False` uses measured levels only.
- `filterbank_headroom_db` replaces the fixed `FILTERBANK_TX_HEADROOM_DB` constant —
  amplitude-word room for the modulation drive restore, not DAC-saturation margin.
- Remaining hardcoded margins become parameters: `set_tone_powers(headroom_db)`,
  `fix_dac_saturation(headroom_db, intermittent_backoff_db)` (were `0.90` / `0.707`),
  and `atten_step_db` / `dsa_step_db` on `fix_adc_saturation` and the `rx_policy`
  protect loop.
- `wideband_sweep()` (and its CLI) and `run_power_sweep()` gain `dac_headroom_db`,
  `adc_headroom_db` and `filterbank_headroom_db`; previously all silently used the
  library defaults. `wideband_sweep(tone_powers_dbm='auto')` also forwards its own
  `reference_plane`.
- Fixed `client.set_psb_scale()` casting to `int`, so `set_psb_scale(4 * 0.99)` wrote
  3 — a 0.09 dB request became 2.5 dB. The scale is UFix16_8 (1/256).

## v1.7.1

**Match resonance lists between sweeps — `resonance_matching`**
- New module answering "which resonance here is which device there?" when the lists
  differ in length, the frequencies have moved, and some devices appear in only one
  sweep. `rm.match_index(a, b)` returns an A→B index array with `-1` for unmatched;
  `rm.match_resonances(a, b)` returns the same with diagnostics attached.
- Accepts bare frequency lists (Hz/MHz/GHz auto-detected), dicts, structured arrays,
  `.resonances` files, KIDLAB toneslists, fit-summary CSVs, power-sweep run
  directories, and parsed sweep dicts (resonances found automatically).
- Three stages: a global shift model chosen from `none`/`constant`/`proportional`/
  `affine`/`linear_frac` (preferring the simplest that fits); a tolerance from the
  measured scatter, floored at a linewidth and capped at a fraction of the spacing;
  then a global one-to-one assignment (`free`, `monotonic` or `nearest`). The
  order-preserving solution is always run as a cross-check.
- Coverage-aware: devices outside the other sweep's band report `out_of_range` rather
  than counting as lost. `groups()` reports N:M clusters, so a resolved blend or a
  merged pair is visible rather than silently mis-paired.
- `compare()` lines up any per-resonance quantity across the two lists; `transfer()`
  carries per-device information (beam-map positions, names, tone powers) onto the
  other list's ordering.
- Matches flagged `ok`/`crossed`/`ambiguous`/`outlier`/`rescued`/`unmatched_*`,
  written by `save()` and read by `load_match()` with hand-edited `b_index` honoured.
  Adjustable by hand with `set()`, `unmatch()`, `lock()` and `rematch()`; every edit
  returns a new match.
- `fingerprint='auto'` measures which resonator properties survived between the two
  sweeps and uses those to settle pairings frequency cannot — what disqualifies a
  feature is scatter, not change. Fingerprints only break ties. A rescue pass pairs
  leftovers with frequency ignored, auto-accepted only when a leave-one-out test
  scores high enough (93% on fitted parameters, 13% on survey estimates, where it
  declines and offers a shortlist).
- Five plots: `plot()`, `plot_shift()`, `plot_quality()`, `plot_compare()`,
  `plot_pair()`. New `doc/resonance_matching.md`.

## v1.7.0

**Centralised LNA bias service + v2 bias board support**
- Only one RFSoC per telescope is wired to the LNA bias board, so that machine runs
  `souk-lna-service` and every readout server reaches the board through it. New
  `server/lna_service.py`: TCP service (length-prefixed JSON) owning the board,
  serialising access and serving a background-polled status cache.
  `souk-lna-service-install` / `-start` / `-stop` / `-restart` / `-status` /
  `-remove`, unit `lna-service.service`.
- New `backend: remote` on `LNABiasController`, resolved from a per-machine
  `~/.souk_readout_tools/site.yaml` (`lna_service.host`/`.port`), with a
  `cryostat.lna_bias.host` override for bench work. Each server sends its own
  `lna_channel` on every request. Client API, mock and apply-on-config-push
  unchanged; an unreachable service degrades to failure results rather than raising.
- v2 LNA bias board support. One `_BiasMonitor` subclass covers both revisions:
  builds the v2 U6/U7 output-enable expanders only when present (auto-detected),
  batches multi-channel enables into one write per expander, and never disables
  outputs on construction — the submodule's constructor does, which would cut bias to
  all 14 LNAs on a service restart.
- New hard power control on v2: `set_lna_output_enabled(enabled, channel)` and
  `set_lna_output_enabled_all(enabled)` on client, server and controller;
  `output_enabled` added to status (`None` on v1). `soft_off` unchanged and still the
  only option on v1.
- Tracked the submodule's `bias_control_hw2v` branch: retry failures now raise
  `I2CDeviceError` rather than `ConnectionError`, and the retry log message changed,
  which had silently disabled the unpopulated-slot noise filter. Submodule pin moves
  to `94eb9c8`, also marked by the local tag `souk-readout-tools-v1.7.0` inside the
  submodule; if upstream force-pushes, recover with
  `git -C src/souk_readout_tools/server/souk-peripherals-control checkout 94eb9c8`.
- The shared I2C bus lock moved to `server/i2c_lock.py` and is now taken by
  `rf_peripherals` too, so RF module traffic cannot interleave with the LNA service's
  mux operations where both boards share bus 0.

## v1.6.7

**Power tuning: measured bifurcation, shared sweep windows, and an explicit comb
planner** (`power_sweep.py` only — no firmware or hardware change)
- `run_power_sweep(direction='both')` takes an up then a down sweep at each power
  step, so bifurcation is **measured** rather than reached by extrapolating fitted
  `anl` to `anl_bif`. The up sweep stays the canonical artifact; the down sweep is
  saved as `kind='sweep_down'` and appears as `sweeps_down`. Tones park at the edge
  each sweep starts from, and the down sweep reuses the up sweep's ADC calibration.
  `follow_dips` recenters on the down sweep by default (`follow_dips_from`).
- `hysteresis_metrics()` compares the two traces per tone and power (chi-square
  z-score, mean branch separation as a fraction of dip depth, split between minima in
  linewidths, per-direction jump ratios). `hysteresis_onset()` brackets the
  transition. `find_best_power` caps the chosen power at that onset, gains a
  `hysteresis_onset` fallback, and flags `bif_hysteresis_mismatch` /
  `anl_direction_mismatch`. The onset is an *upper bound*.
- **Parameter outlier check is now detrended.** MAD-clipping Qi and fr against a
  constant per-tone median treated physics as outliers — on a 389-tone campaign it
  excluded 227 tones at −88 dBm and all 389 at −80. Deviations are now measured about
  a robust Theil–Sen trend in power (log space for Qi/Qc): 227 → 16 excluded at
  −88 dBm, past-bifurcation rows still rejected. `param_outliers_detrend=False`
  restores the old form.
- **Fallback after a rejected ANL fit now uses the array.** The old `anl_threshold`
  fallback handed such tones the hottest power in the run (median 18 dB above the
  array). They are now placed from the tones that did work, via a robust line of
  chosen power against `−30log10(Ql) + 10log10(Qc)` (from `anl ∝ P·Ql³/Qc`), falling
  back to the population median. Rescued tones span −105.9…−99.2 dBm instead of
  −120…−82.
- **Shared sweep windows.** `follow_dips`'s conflict guard used a 1 Hz separation
  floor, so a pair landing 10 Hz apart passed, `clip_overlapping_spans` collapsed both
  spans to zero, and every fit failed for both tones for the rest of the run (14 of
  389 tones). The floor is now a fraction of the pair's sweep span (default 0.5);
  `follow_min_separation_hz` overrides it with a plain frequency.
- **Multi-resonance fitting.** A close pair distorted the single-resonator fit badly —
  fitted `anl` came out 10–100× too large. `fit_power_sweep` now restricts each fit to
  the resonance that tone reads out, cutting at the midpoint to each neighbouring dip.
  Median reduced chi2 10423 → 148; contaminated tones recovering a usable ANL fit 1 →
  7 of 15. Each split tone is also fitted unsplit and the better kept. Costs +21% fit
  time; the fitter itself is untouched.
- `resonator_catalogue()` reports one row per *resonance* rather than per tone, with
  the separation in linewidths and whether the pair could have a tone each.
- **`plan_tone_comb()`** turns that into the next run's comb, the deliberate boundary
  between discovery and configuration. `policy='keep'` (default) preserves the count,
  `'replace'` moves a tone to the better resonance in its window, `'add'` grows the
  comb subject to admission tests. Separation binds, not power: only 2 of 21 close
  pairs could be given a tone each and stay sweepable. Any tone parameter can be
  overridden per tone or per column, applied last.
- The `power_sweep` module docstring leads with the recommended pipeline — sweep,
  analyse, balance, plan — with a runnable example.
- Every part is optional; a single-direction run behaves exactly as before.

## v1.6.6

**Server-side FFM tone tracking (auto-recentering), both engines**
- New `server/tracking.py` + `readout_server.py` wiring: with frequency modulation
  running (fw default, or sw; ≥3 slots/points), the server demodulates the streamed
  cycles, filters per-tone detunings (boxcar/EWMA, runtime-configurable), applies a
  deadband plus K-consecutive-window confirmation, and stages clamped centre
  corrections. Engine-blind — fw slot tags and sw point tags share the `flag5` field.
- Two update classes with independent commit policies: `lo` (seamless — fw inactive
  mixer slot-buffer flip, sw LO-word `install_bundle` swap; auto-commit on) and `bin`
  (bin/map recenter re-arm, staged and on-demand by default). Either way staging is
  the executor-side prepare, so the commit moment is a cheap switch.
- Hot-path cost is one deque append per frame (measured 0.066 µs); estimation runs
  vectorised across all 2048 tones in the thread executor
  (`profiling/tracking_benchmark.py`).
- Auto-holds during sweep/retune and while streaming/modulation are off; back-off plus
  confirmation restart when an external command lands mid-batch; `dry_run` default on.
  Commands: `enable_tracking` / `disable_tracking` / `hold_tracking` /
  `resume_tracking` / `commit_tracking_updates`, plus `get_info('tracking')`.
- **Health monitoring:** `get_info('tracking')` returns `{summary, tones, controller,
  …}` — per-state tone counts, n_over_threshold, max/median drift, staged/commit/
  back-off counts, loop liveness, and per-tone lock `state`, detuning mean+std,
  `slope_ratio`, invalid-cycle fraction, staged centre. `health_check()` gains the same
  `summary` plus `resonators_tracking`/`max_detuning_hz`. A **lost resonance** is
  flagged by the phase slope collapsing against its on-resonance baseline
  (`unlock_slope_ratio`) and/or the invalid-cycle fraction (`unlock_invalid_fraction`)
  — not by detuning magnitude, which *compresses* far from resonance.
- **All per-tone tracking work is off the event loop.** The consumer's single
  `to_thread` hop now also runs `decide` and rebuilds the health snapshot; only the
  per-correction commit stays on the loop. `get_info('tracking')` and `health_check()`
  serve that cached snapshot in O(1) — ~0.05 µs and flat 256→2048 tones, versus
  ~14–30 ms for an on-loop rebuild. Benchmark:
  `profiling/tracking_health_benchmark.py`.
- `enable_modulation(linewidth_hz=...)` carries `params_from_sweep`'s per-tone
  linewidths on both engines, required by tracking unless given to `enable_tracking`.

**Typed stream frames (opt-in, in-band tone provenance)**
- Stream clients may send `{"subscribe": ["tone_updates"]}` to receive typed frames:
  `SNAPSHOT` on subscribe and `TONE_UPDATE` on every modulation revision change, any
  origin, carrying revision/op/kind/engine/source/changed centres/triggering
  detunings. Typed frames put a nonzero type code in the top byte of the length
  prefix; unsubscribed clients get a byte-identical legacy stream.

**stream-to-dac: live FFM demodulated output (`--mode ffm`)**
- `stream_dac.FfmConverter` demodulates the modulated stream per cycle: the
  phase-vs-offset slope is a live, self-calibrating `dphi_df`, and the centre-point
  phase against a startup baseline gives the linearised frequency shift — no
  calibration file needed.
- Output is the **absolute resonator shift** across recenters: `(center[rev] −
  center[rev0]) − probe_side_shift`, so tracking corrections do not step the analog
  output. The same updates land in the recording's `.updates.jsonl` sidecar.
- New `conversion.average_cycles` / `baseline_cycles` config keys; recipes in
  doc/stream_to_dac.md and doc/tone_tracking.md.
- Client: `stream_dac.SocketFrameSource(subscribe_updates=True)` + `.updates.jsonl`
  sidecar; `receive_stream_g3(subscribe_tone_updates=True)` writes a G3 Wiring frame
  per mid-stream `TONE_UPDATE`. Mock: tracking requests + `get_info('tracking')`.
- `doc/tone_tracking.md`: state machine, config table, staged-commit model,
  reconstruction formula, closed-loop response, protocol spec, benchmarks.

## v1.6.5

**stream-to-dac: live analog output of KID response**
- New `souk-stream-to-dac` script + `stream_dac` module: receives the TCP stream,
  converts a chosen tone's IQ to magnitude / phase / frequency shift and drives a USB
  DAC (LabJack T-series via optional `labjack-ljm`, or a dummy backend) — built for
  recording KID spectral response on an external analog logger (TOPTICA cw-THz scans).
- Reader/output thread pair with a bounded drop-oldest queue, boxcar averaging to the
  DAC update rate, watchdog plus safe-state idle voltage on any error/stall/exit, and
  drop/loop-time diagnostics.
- Raw stream always recorded in the `receive_stream` format; `--replay` plays a
  recording back through the full pipeline at true rate, `--mock` uses the emulated
  server.
- Stream IQ gets the same software readout-flattening factors as
  `parse_samples(apply_readout_correction=True)` before conversion.
- Calibrated mode uses `ResonatorCalibration.tone_converter` (exact Möbius inversion);
  `stream_dac.calibrations_from_sweep` / `save_calibrations` / `load_calibrations`
  give a sweep → `batch_fit` → `.npz` workflow.
- New `template_stream_to_dac_config.yaml`, `doc/stream_to_dac.md`, and a
  `--selftest` DAC→AIN loopback latency check.

## v1.6.4

**Per-point modulation timestreams**
- New `modulation.point_timestreams()` splits `group_cycles` output into one
  `parse_samples`-shaped timestream per probe point (with a real sub-cycle time axis),
  so the standard timestream/PSD plotters work per point.
- `demodulate`/`demodulate_timestream` accept a list of fits/calibrations for
  `calibration=` (matched by `tone_index`) and auto-wrap raw `FitResult`s.

**Timestream plotting: demodulated-stream handling**
- Plotters honour a producer-supplied `time_s` axis; `conversion_method` defaults to
  `None` and warns when conversion knobs cannot apply to already-demodulated data.
- `plot_timestream_psd(exclude_settling=True)` computes the PSD from only the
  non-settling samples at the decimated rate; non-finite PSDs now warn.
- `plot_timestream_on_resonance` detects demodulated input and no longer
  double-applies deembed/phase-centering.

**Server: firmware-modulation revision history**
- `_next_fw_modulation_revision` records center/offsets/dwell/mode per revision
  (via `state['revision_history']`) so tagged frames can be attributed offline;
  `purge_revisions` clears fw history and refuses while fw modulation is enabled.

## v1.6.3

**Fix: stale mixer dwell breaking software modulation after firmware-slot runs**
- `disable_fw_modulation` left the mixer `dwell` register at the fw `n_dwell`. The
  dwell gates *all* allowed LO switches — including the ping-pong flips driven by
  software modulation, sweeps and retunes — so after any fw-mod run, sw modulation
  showed skipped points and ragged dwells and sweeps averaged stale-frequency
  accumulations, long after modulation was reported off (`dwell` read back 11 on
  hardware). The dwell is now restored to 1 on fw disable and asserted at sw arm.

**Filterbank compensation: applied where you look at the data**
- The software readout-flattening factors are now applied at **parse time** by
  default: `parse_samples(apply_readout_correction=True)` (per sample via the
  modulation point tag) and `parse_sweep_data(apply_readout_correction=True)` (per
  sweep point). Output carries `readout_correction_applied`; `group_cycles` skips its
  own application when marked. Pass `False` for raw accumulator values.
- Sweeps/retunes resolve the standard path group-delay calibration for the image-phase
  term (previously override-only) and take a per-request `compensate_filterbank`
  override (`perform_sweep`, `perform_retune`, `wideband_sweep`).
- `plot_sweep`, `plot_timestream`, `plot_timestream_psd`,
  `plot_timestream_on_resonance` and `batch_fit` take `apply_readout_correction`
  (default on) and convert already-parsed data either way.
- Power optimisers reserve `FILTERBANK_TX_HEADROOM_DB` (1 dB) of amplitude-word
  headroom for the TX drive-restoring boost when compensation is enabled; the psb_scale
  ramp recovers the output level, so no power is lost. Explicit `set_tone_powers`
  requests that eat into the reserve warn instead of failing.

## v1.6.2

**Power-sweep API tidy (breaking)**
- `power_sweep` now reads as the workflow: `run_power_sweep` / `load_power_sweep`
  acquire, `analyse_power_sweep` / `load_analysis` (new) analyse, then optional comb
  balancing — grouped per artifact, each with a symmetric `write_*`/`load_*` pair.
- Removed the `analyze_power_sweep` alias; the spelling is `analyse_power_sweep`.
- Renamed `fit_parameter_series` → `parameter_series` and
  `allocate_balanced_tone_powers` → `balance_tone_powers`; `include_optimizer` →
  `include_optimiser`; the balancing objective `"maximize_power"` →
  `"maximise_power"`. `fit_summary_rows` is private.
- New `load_analysis(path)` (everything `analyse_power_sweep` wrote) and
  `load_fit_summary(path)` (the CSV back as a structured array, accepted directly by
  `find_best_power`).
- `analyse_power_sweep`'s `plot` flag split into `plot_fits` / `plot_best`, so power
  selection can be redone from stored fits without re-rendering every per-tone plot.
- Writers accept a file or run directory, default into `<run>/analysis/`, and return
  `Path`.

**Sweeps**
- Overlapping sweep spans are clipped so neighbouring segments do not overlap
  (`clip_overlapping_spans`, on by default); the actually-swept centers/spans are
  recorded in the sweep metadata.

## v1.6.1

**Filterbank channel-response compensation**
- Tones away from their filterbank bin centre are now gain- (and optionally phase-)
  compensated wherever mixer control words are prepared: tone setting, sweeps, and —
  the motivating case — frequency modulation riding the channel overlap, where the
  combined TX+RX response reaches −1 dB at 0.70 bin spacings (±211 kHz) and −6 dB at
  the channel edge.
- Model: 8-tap DPSS+sinc prototype (single-bank gain `G`) plus coherent synthesis-image
  recombination in the analysis bin (`H(δ) = Σₘ G(δ−2m)²·e^{4πjmBτ}`), validated in
  digital loopback (≤0.04 dB, zero phase) and RF loopback (0.05 dB / 2.7 mrad),
  frequency-independent across 0.6–2.1 GHz. See
  [filterbank_compensation.md](doc/filterbank_compensation.md).
- Corrections: TX scaling `× 1/G` in hardware restores the physical drive; the
  readout-flattening `× G/|H|` part is applied in **software** — the RX LO scale word
  is inert in the v7.11 fabric (hardware-verified; souk-firmware#117), so the server
  reports per-(tone, point) `readout_correction` factors and `modulation.group_cycles`
  applies them (flip `firmware_lib.FILTERBANK_RX_SCALE_IN_FABRIC` when a firmware
  connects the multiplier). RX phase offsets do work in hardware. A clipped TX boost
  is reported in the modulation state warnings.
- The phase term's group delay comes from `rf_frontend.path_group_delay_ns` (measure
  once with `client.measure_path_group_delay(save_to_config=True)`), evaluated per
  tone; `filterbank_group_delay_override_ns` overrides for testing. Master switch
  `filterbank_compensation` (default **on**);
  `enable_modulation(..., compensate_filterbank=False)` gives a per-request A/B. Keep
  base tone amplitudes ~1 dB below full scale for the modulation TX boost.
- Fixed the server `psb_scale` setter truncating fractional scales to int (the block
  value is UFix16.8; restoring a read-back 2.77 silently wrote 2).
- Fixed an intermittent `RuntimeError: delayed modulation preload was not serviced…`
  killing sw-modulated finite captures near their end. Data was never mislabelled;
  captures no longer advance after their final dwell.
- Standalone scripts in `scripts/filterbank/`: channel-response measurement, a
  compensation acceptance test, and an LO-stepping diagnostic.

**Other**
- rfsoc4x2: krc-utils availability checks in clock status handling.
- `so3g` is no longer a hard dependency — without it, SO 3G streaming is unavailable
  (e.g. on Windows) but everything else works.
- Docs/profiling: README and profiling scripts detail the accumulator read-back
  geometry and timing metrics.

## v1.6.0

**v7.11 multi-LO firmware support (firmware-slot frequency modulation)**
- Adapted to the v7.11 register map: `acc_len` moves from the accumulator to the mixer
  (changing it now needs an mrst sync). `sync_delay` is deprecated — the mixer owns
  the RX/TX LO delay and sets it during init; config/parameter writes warn and no-op,
  and `info_pipeline` reports the live mixer `sync_delay`, `tx_rx_skew` and
  `buffer_switch_skew`.
- Mixer LO control ping-pong buffers are now separate registers
  (`{lo}_lo{i}_control0/1`), each holding `n_slots` LO slots selected by in-register
  offset. All control-buffer read/write paths gain a `slot` argument (`slot=0`
  reproduces pre-v7.11 behaviour). `accumulator.get_new_spectra` returns a 5-tuple.

**Firmware-slot modulation engine (`engine='fw'`, hardware-validated)**
- New engine using the v7.11 mixer's four LO slots: load up to four full combs once and
  let the firmware switch between them — auto round-robin every `n_dwell`
  accumulations, or manual slot selection — with no per-sample software writes and no
  software-timing jitter. Switches land on accumulation edges; `n_settle` of 0 or 1
  suffices.
- Each accumulation is tagged from the firmware `buffer_id` register in the same devmem
  call as the IQ data, removing the one-sample tag/data lag. `get_samples` aligns its
  first sample to the start of a point-1 dwell; settling is edge-detected from tag
  changes so it self-corrects across dropped samples.
- Seamless `update_modulation` rides the inactive ping-pong buffer;
  `recenter_modulation` reloads fresh armed bins. The two engines share the `flag5` tag
  word and are mutually exclusive; tone/sweep/retune writes invalidate both.

**Unified modulation API**
- `enable_modulation(center, offsets, ..., engine='fw')` defaults to the firmware-slot
  engine; `update/recenter/disable_modulation` and `get_modulation_state` auto-detect
  the active engine (`engine=None`), with `set_modulation_slot` for fw manual mode.
- `get_info`: `tone_modulation` becomes `modulation` — one payload for whichever engine
  is active, tagged with `engine` and carrying `num_points` / `samples_per_point` /
  `offsets_hz` aliases so it is drop-in for `group_cycles` / `demodulate`.
  `modulation.active_modulation_state()` picks it out of a `get_info` payload.
- [frequency_modulation.md](doc/frequency_modulation.md) rewritten around the unified
  API with firmware-slot as the primary engine.

**Snapshots (v7.11)**
- Snapshot blocks latch the telescope time of the first sample; `get_adc/dac_snapshot`
  return it as `timestamp`. New `batch_adc_snapshots` / `batch_dac_snapshots` stream
  raw timestamped snapshots over one TCP connection via the fast devmem path.

**Server robustness**
- `set_config` verifies pipeline readiness after a push and fails loudly instead of
  persisting a dead config; a pushed config that changes the firmware image drives the
  full deprogram/reprogram path.

**Misc**
- Added `resonator_fitter.py` for compatibility with the ukkid_controller OCS agent.

## v1.5.0

**v7.10-timed-sync firmware support (timed sync & multi-board alignment)**
- TT-targeted syncs so independent RFSoC boards can align timestamps and accumulations
  to the same epoch.
- New client controls `timed_sync_needed()` / `timed_sync_ready()` / `timed_sync_arm()`
  / `timed_sync_check()` and `set_telescope_time()`. `set_telescope_time()` loads the
  current Linux time into the firmware TT counter, latched on the next PPS edge;
  `timed_sync_arm()` schedules a future PPS-latched resync and only reloads TT when
  missing or drifted by ~1 s. Requires PTP lock and a healthy on-board PPS strobe; see
  [tsu_strobe_and_timed_sync.md](doc/tsu_strobe_and_timed_sync.md).
- Board PPS generation comes from the TSU strobe path (`tsu_strobe.py`,
  `souk-tsu-strobe`) rather than an external PPS input, with scripts in
  `scripts/timed_sync/`.
- `get_info('sync')` reports timed-sync state, readiness, alignment, last sync, last TT
  load and drift; `get_info('timing')` adds `sync_readiness`. The server caches the
  PPS-aligned TT load second so `timed_sync_check()` is instant by default.
- Results include UTC strings, clearer key names (`last_pps_*`,
  `pps_boundary_offset_s`, `target_tt_*`, `armed_at_*`), and a shared default alignment
  tolerance of 0.1 ms (`timing.DEFAULT_ALIGN_TOL_S`, overridable via `align_tol_s`).
- [timing.md](doc/timing.md) gains instructions for standing up a lab PTP grandmaster
  on a Linux workstation, plus the finding that software-only timestamping did not work
  with the RFSoCs.

**TX/RX phase offset fix (firmware)**
- v7.10 fixes the TX/RX phase offset seen after mixer/tone control-buffer switches, so
  post-switch firmware syncs are no longer required. The software workaround is
  disabled by default: `compensate_rx_ticks=0`.

**Sweep & modulation stepping without per-step syncs (v7.10)**
- Sweeps, retunes and modulation default to a single start-of-run sync
  (`setup_sync=True`, `setup_mrst=False`) with no per-step syncs (`autosync=False`),
  dropping a few accumulations around each step instead;
  `settle_accumulations=4` / `chanmap_settle_accumulations=4` are the new defaults.
- Standalone tone setters (`set_tone_frequencies`, `set_tone_amplitudes`,
  `set_tone_phases`, `apply_tone_frequency_settings`, `set_tone_powers`) also default
  to `autosync=False`.
- `enable_modulation` defaults `buffer_reuse_delay_accs=3` with `samples_per_point=4`.

**Clock-source safety**
- Clock health is checked at initialise/restart and via `get_info`; there is no forced
  clock reconfiguration on every config push. If a PLL is unlocked or the live source
  differs from config, the server refuses to touch the clock and reports
  `reset_required` — changing clock configuration against a loaded pipeline can stall
  or crash the board. Seamless runtime switching between internal and external clock
  sources is no longer supported; clock changes go through reset-to-base → deprogram →
  clock-init → reprogram (`hard_reset`). New helpers `classify_clock_status`,
  `apply_clock_config`, `deprogram_fpga`, `select_clock_source`.

**Modulation / demodulation**
- Added `modulation.demodulate_timestream()` — sample-aligned demodulation without
  averaging away retained samples. Supports calibrated Möbius inversion or the
  model-free basis, plus `raw` / `interpolate` / `none` settling fill modes.
- `group_cycles()` supports `include_settling` and emits blind / regular / modulated
  tone-role metadata; `params_from_sweep()` adds mutually exclusive `point_sequence`
  and `offset_linewidths`.
- `recenter_modulation()` gains RX-tick phase compensation, client-visible warnings,
  and `purge_modulation_revisions()`.
- `enable_modulation(..., force=True)` arms even when some tones fall beyond fixed-bin
  coverage; `result.tones_beyond_coverage` reports them and warns they read back
  wrapped to the other end of the bin.

**DAC inverse sync filter**
- Config-driven DAC inverse-sync-filter control, enabled by default unless explicitly
  `False`, directly controllable from the client and reported in `get_info` / config
  sync.

**Plotting & analysis**
- Added `plotting.log_bin_psd()` for log-spaced PSD binning, timestream plotting for
  modulated-probe frequencies and precomputed frequency/dissipation columns, plus
  resonator-fitting uncertainty/model refinements.

**Resonator fitting — automatic nonlinearity estimation (`fitting.py`)**
- `nonlinear='auto'` for `fit_resonance`, `batch_fit` and `fit_sweep_stack`: tries the
  fast linear model first and escalates to the Duffing `anl` fit only when needed.
  Diagnostics in `linear_sufficiency`; linear-only results report `anl = 0`.
- Deliberately not the default and not suitable for power sweeps or `anl`-versus-power
  studies, where small but real nonlinearities would be suppressed. `nonlinear` remains
  `True` by default, including in `fit_power_sweep` / `analyse_power_sweep`.
- Linear fitting is faster via a closed-form Jacobian; `store_optimizer=False` shrinks
  large parallel-fit results.

**Mock server sample rate**
- The mock server derives sample rate from `acc_len` in both directions, and throttles
  acquisition to approximately real time so G3 streaming, `receive_stream` and
  non-burst `get_samples` take realistic wall-clock time.

**`get_sweep_data` and `get_info` speedup**
- `get_sweep_data` / `get_sweep_txt` serve a cached sweep-time system-info snapshot
  instead of re-polling hardware: ~6 s → ~5 ms. `get_info('all')` ~6 s → ~1 s via fast
  control-buffer/channel-map reads and narrower LNA I2C access.

**Parameter-series measurement tools**
- Reintroduced `measurement.py` in a deliberately simple form: `ParameterSeries`,
  `ParameterGrid`, `TimedMeasurement` and `ConditionalMeasurement`. You supply set/read
  callbacks and a `measure_func(client)`; the tools walk the parameter and save each
  result plus a `measurement.json` manifest as they go. `run(..., client=...)`
  re-points one tool at another board/pipeline. `ConditionalMeasurement` matches
  `target_values` in any order.
- The `measure_func` return value drives saving: a dict is stored as `.npz`, `None`
  stores nothing; `save_func` overrides. Saving is opt-in via `save_dir`; existing run
  folders are never overwritten unless `overwrite=True`.
- Robust long runs: `resume=True`, `retries`, `on_error="skip"`, and clean Ctrl-C
  finalisation. Optional `plot_func` and `summarise_func` (live `summary.csv`). Read
  back with `load_run`, plot with `plot_run_summary(ncols=...)`. See
  [measurements.md](doc/measurements.md).
- Resource guard for large runs: after the first step the run projects per-step memory
  and disk over the whole run, freeing each step after saving if memory would be
  exhausted and warning if disk would fill. Per-run budget via `memory_fraction`
  (default 0.8 of total system memory). Pass `estimated_step_bytes` for a pre-run
  check. The guard only adapts/warns — it never aborts.
- `load_run` caches step data only while it fits in free memory; `run.iter_data()`
  streams one step at a time, `load_run(cache=False)` / `run.clear_cache()` control it.
- Shared manifest/npz helpers now live in `measurement.py` and are imported by
  `power_sweep.py`; the power-sweep run format is unchanged.

**Removals & housekeeping**
- Replaced the earlier over-complicated measurement-run framework (`MeasurementRun` /
  `MeasurementStore` / artifact dataclasses) with the tools above.
- Tightened config-push validation, added Windows guards for POSIX-only imports, and
  introduced `config_utils.get_user_dir()` for resolving the target user's data
  directory under `sudo`.

## v1.4.0

**Fast Frequency Modulation (real-time IQ conversion & resonator tracking)**
- Modulation is a mode of the single continuous streamer: each tone is stepped through
  a small set of probe frequencies (typically 2–3) every accumulation, returned as
  ordinary frames tagged per sample with the modulation step and a settling marker
  (packed into the spare `flag5` word; decode as unsigned).
- New controls: `enable_modulation()` / `update_modulation()` / `recenter_modulation()`
  / `disable_modulation()`, plus `get_modulation_state()` and a
  `get_info('tone_modulation')` section. Arm is independent of output — start with
  `enable_stream()` or `get_samples(N)`.
- On/off at any time, and **seamless live updates** of per-tone centres/offsets with no
  intentionally dropped frames. Tones ride the ~2× filterbank overlap; a bounded
  `recenter_modulation()` reloads channel maps when a tone drifts beyond coverage.
  Per-(tone,point) bin occupancy is reported.
- Correct double-buffer ping-pong scheduler (valid for any N, including odd N across
  cycles) with an N=2 zero-rewrite fast path; per-point mixer words are computed
  relative to the armed bins so phase stays continuous as tones drift.
- New `souk_readout_tools.modulation` toolkit: `params_from_sweep`, `group_cycles`
  (`on_missing='notify'|'fill'`) and `demodulate`. The recommended de-embedded/
  phase-centred basis (via `resonator.ResonatorCalibration`) yields frequency shift and
  dissipation through the exact Möbius inversion; a model-free raw-phase fallback is
  also provided.
- Mock mode emulates modulation; `client_scripts/test_frequency_modulation.py`
  exercises the whole path. See
  [Fast Frequency Modulation](doc/frequency_modulation.md).

**Power Sweep**
- `find_best_power` repairs unphysical negative `chosen_params` (from linearly
  extrapolating a fit-summary field past the measured range) by substituting the
  cross-tone median. Signed quantities untouched; affected tones stay flagged
  `extrapolated`.

## v1.3.0

Forward-ported the `jl_ocs_devel` branch (PR #10) plus follow-on hardening.

**G3 Streaming Output (so3g)**
- `client.receive_stream_g3()` records a continuous stream as an so3g/spt3g G3 file:
  `Observation` frame, `Wiring` frame with tone metadata, and `Scan` frames carrying
  `G3SuperTimestream` payloads for I/Q, packet counters and PTP timestamps. Mock
  supported.
- `receive_stream_g3.py` client script for command-line capture, shipped as a module
  rather than an entry point. Client install now requires `so3g` (pulls in `spt3g`).

**Mock Server Mode**
- `ReadoutClient(..., mock=True)` emulates the readout server without hardware,
  swapping socket traffic for an in-process `MockReadoutServer` and defaulting to
  `127.0.0.1:10000`. Generates a synthetic resonator catalogue and emulates sweeps,
  snapshots, streams, the full `get_info()` surface and the G3 writer.

**OCS Compatibility Fitting Helpers (`client/res_fns.py`)**
- Forward-ported for `UKKIDController` compatibility (`s21_model`, etc.). Slated for
  removal once the core fitting tools are validated there.

**Core Resonator Fitting (`fitting.py`)**
- Reworked the linear and nonlinear fitters around a clearer convention
  (`fr, Qi, Qc, phi, A, alpha, tau[, anl]`). `Qc` is the fitted real coupling
  parameter; `Qe` and `Qc_abs` are derived. Setup uses explicit `initial_guess`,
  `param_bounds` and `param_fixed` dicts.
- Better nonlinear/Duffing convergence on distorted shapes via a new seed strategy,
  high-drive handling, optional subsampling and fit diagnostics.
- Process-based parallel fitting for `fit_sweep_stack()` and `batch_fit()` via
  `n_jobs`, with compact verbose progress.
- Conservative wideband peak-finder defaults for `find_resonances(mode='wideband')` and
  `batch_fit(find_resonances=True)`: inner 10–90% trim, dip finding, 1–100 dB
  prominence, 1 kHz–10 MHz width, 100 kHz minimum spacing, lowpass 0.5.

**Resonator Transforms and Plotting**
- Deembedding, phase-centering and sweep plotting propagate optional S21 uncertainties
  (`real=sigma_I`, `imag=sigma_Q`) through the same transforms.
- Updated the phase-centering convention and calibration construction so fitted
  geometry is handled consistently across fitting, plotting and freq/diss conversion.

**Noise Analysis**
- `noise.remove_common_modes_svd()` for NumPy-only common-mode subtraction across slow
  accumulated timestreams, plus
  `noise.fractional_frequency_and_dissipation_timestreams()`.
- `noise.remove_blind_tone_common_modes()` fits temporal reference modes from
  simultaneous blind-tone amplitude/phase variations and regresses them out of
  regular-tone I/Q before conversion. Blind sweep traces are not used.
- `plot_timestream_psd(..., format='freq_diss')` can overlay SVD- and
  blind-tone-cleaned spectra or plot them in place of the raw ones.

**CSV Import Hardening**
- `import_sweep` and the stream/snapshot CSV importers use `ast.literal_eval` instead
  of `eval`, removing an arbitrary-code-execution sink and fixing a latent crash on
  values that are valid strings but not valid Python expressions (`fpg_file` paths,
  ISO-8601 dates, free text).

**Client API**
- `perform_sweep()` and `perform_retune()` accept `wait=False`; `wait=True` blocks via
  `wait_for_sweep()` after a successful dispatch.

**Power Sweep Workflow Polish**
- `run_power_sweep()` defaults to pre-centering before the first saved sweep and
  following dips between power steps; the initial unsaved center-search sweep uses
  `search_span_factor=2.0`. Pass `follow_dips=False` or `search_for_center=False`.
- `run_power_sweep()` defaults `follow_min_depth_db=0.5`, so `follow_dips=True` no
  longer chases shallow noise minima. `None` restores accept-any-depth.
- New `best_power_arrays()`, `write_best_power()` and `load_best_power()` for the
  chosen power, `p_bif` and `p_bif_sub_3db` per tone, and round-tripping
  `find_best_power()` to `best_power.json`.
- `plot_fits()` disables the shared axis offset annotation on every panel.
- `plot_power_sweep()` and `plot_best_power()` gained `n_jobs` (joblib convention),
  rendering per-tone PNGs across worker processes — `savefig` is the dominant CPU cost,
  so this scales toward the core count. `analyse_power_sweep()` forwards `n_jobs`.
  Default `dpi` dropped 100 → 80 (savefig time scales with `dpi**2`); raise it for
  publication figures. Per-tone output is byte-identical serial vs parallel.
- Removed the per-tone `UserWarning: ... Axes that are not compatible with
  tight_layout` on `mag+phase+iq` layouts — those figures lay out via
  `savefig(bbox_inches='tight')`, which also stops long axis labels being clipped.
- Fixed `analyse_power_sweep(plot=True)` passing an invalid `show=` to
  `plot_power_sweep()` (now `show_overlay=`), which raised `TypeError`.

## v1.2.0

**Get Info API**
- `get_info()` accepts a single section name (`get_info("timing")`) and returns that
  section directly. List requests still work and return a list in the requested order.

**RFSoC Timing and PTP**
- New packaged `souk-timing-monitor` daemon polling `ptp4l` via `pmc` and chrony via
  `chronyc`, exposing JSON status on `/run/timing-monitor.sock`, plus packaged
  templates for `ptp4l.service`, `timing-monitor.service`, `ptp4l.conf` and the chrony
  PHC refclock drop-in. `souk-enable-timing` installs and starts them;
  `souk-test-timing-monitor` is a lab client for `status`, `ping` and streaming checks.
- New `client.get_timing_status()` and a `timing` section in `get_info()`;
  `health_check()` includes `timing_state` and `timing_ready`.
- `get_info("timing")` presents a compact grouped view — `summary`, `monitor`, `ptp`,
  `phc`, `ntp`, `chrony` — with parsed values plus raw `pmc`/`chronyc` output for
  debugging. `ready_for_firmware_sync` is the gate for firmware timestamp syncs.
- Summaries include current and last-seen PTP grandmaster identities, compact NTP
  source errors, and a rough PTP-holdover-vs-NTP comparison, with the holdover
  drift-rate and policy constants exposed. NTP entries carry configured name and
  resolved address, adjusted/measured offsets, and the measurement error bound.
- `summary.system_synced` now means actively synchronised to a current external source
  — false during PTP holdover. `chrony.synced` still reports chrony's local view.
- Status includes `ptp_data_fresh`, `ptp_ingress_time_ns`, `ptp_seconds_since_ingress`
  and `ptp_seconds_since_ingress_update`; stale `gmPresent` and `master_offset` are
  masked when `TIME_STATUS_NP.ingress_time` shows no recent ingress. Raw `pmc` and
  `chronyc tracking` / `sources -v` outputs are stored.
- Holdover policy is configurable via CLI options and reported as
  `ptp_holdover_window_s` / `ptp_holdover_error_rate_ppm`; once PTP lock has been
  observed and chrony stays on `PHC0`, the monitor stays in `ptp_holdover` beyond the
  nominal window and reports `ptp_holdover_expired`.
- Starting with no GM but chrony on `PHC0` reports `phc_free_run` until a PTP lock has
  been observed; a non-zero `ingress_time` must keep advancing to stay fresh. With
  nothing selected yet, the monitor reports `initializing` during a configurable grace
  period before falling back to `free_run`.
- The packaged chrony PHC refclock uses `prefer` but not `trust`, so NTP can reject a
  free-running PHC, and defaults to `offset 0`; the lab-only `offset -37` workaround
  installs with `souk-enable-timing --offset=-37`. Added timing documentation and a
  site commissioning checklist.

## v1.1.1

**Removed Deprecated APIs**
- `get_system_information()` and `get_server_status()` removed. Use
  `get_info(sections=...)` and `health_check()`.
- Sweep, snapshot, stream and parsed-data payloads carry the structured info under the
  `info` key, replacing the legacy flat `system_information` key.

**LNA Bias Setting Failure Reporting**
- `set_lna_bias_voltage()` / `set_lna_bias_voltage_all()` propagate hardware-side
  rejections as proper `status: 'error'` responses rather than burying the failure in a
  `status: 'success'` payload. Each per-channel result carries a `success: bool`.
- LNA bias config gains an explicit backend plus `bias_voltage_v` (default 1.5 V),
  `method` and `blind`; config application sets the configured LNA channel when the
  `i2c` backend is enabled.
- Explicit `fixed` backends for LNA bias and RF attenuators, so non-controllable values
  are deliberate rather than blank backend leaves.
- Added `soft_off_lna_bias()` / `soft_off_lna_bias_all()` and `souk-find-lnas --status`.

**RF Peripheral Configuration & Discovery**
- RF frontend config separates `mixerless_module`, `attenuator` and `bypass_amps`.
  Attenuator values live at `rf_frontend.attenuator.tx_value_db` / `rx_value_db`;
  measured bypass-amp S21 under `rf_frontend.mixerless_module`.
- The controller no longer uses a software mimic after hardware init failures — failed
  hardware stays unavailable and set/get calls raise instead of returning fake success.
- Added `souk-find-bypass-amps` and `souk-rf-peripherals-status`; extended
  `souk-find-attenuators` with `--status`. Power-management helpers distinguish
  controllable attenuators from bypass-amp support.

**Config / Runtime State Split**
- `pull_config()` (and `get_info()`'s `config` section) return the active config file
  only; runtime hardware changes are reported through the live status/info APIs.
- `sync_config_from_system()` captures live state into `client.config` and returns it;
  `sync_config_to_local(save_as=...)` is the clearer alias for creating a new local
  config. With blind tone metadata active it preserves the regular/blind split and
  writes `blind_frequencies`, `blind_amplitudes`, `blind_phases` and `blind_spans` into
  `firmware.defaults`.

**Sweep Plotter Speed-up**
- `plot_sweep()`'s `show_errors` defaults to `None`, auto-disabling error fills for
  wideband sweeps where `fill_between` over hundreds of thousands of points dominates
  render time. Per-tone sweeps unchanged.

**Sweep & Tone Update Robustness**
- Frequency-only tone updates preserve existing amplitudes and phases when the tone
  count is unchanged.
- Fast sweep setup can carry amplitudes/phases through every sweep point, and protects
  the VACC from shared-bin amplitude overflow by scaling sweep amplitudes and
  temporarily compensating `psb_scale` when possible.
- Fast sweep/retune paths use the fast tone-frequency writer and direct channel-map
  updates.

**Blind Tone Management**
- Blind tone support for fixed gain/phase monitor tones, configured via
  `firmware.defaults.blind_frequencies`, `blind_amplitudes`, `blind_phases` and
  `blind_spans`.
- The firmware is programmed with one combined tone list, so blind tones are included
  in Newman phase generation, VACC/shared-bin protection, calibrated power setting,
  dynamic-range optimisation, timestreams and sweeps. Retune and tracking keep blind
  tones fixed while updating only regular tones.
- New helpers: `suggest_blind_frequencies()`, `set_blind_tones()`, `get_blind_tones()`,
  `remove_blind_tones()`, `get_tone_metadata()`, `get_blind_tone_indices()`,
  `get_regular_tone_indices()`. `suggest_blind_frequencies()` jitters the even target
  positions by default, reducing regular-grid intermodulation spur alignment.

**Peak Finder Edge Trim**
- `PeakFinderParams` accepts `f_low` / `f_high` (Hz) to drop peaks outside a chosen
  band, applied after `find_peaks` so prominence/width context still uses the full band.

**Other Fixes & Improvements**
- `maximise_tx_power()` accepts `compression_headroom_db` to cap total TX frontend
  input power below the input-referred 1 dB compression point. The RF peripheral
  wrapper corrects the mixerless module amplifier P1dB arithmetic without modifying the
  upstream submodule.
- Mixerless-module measured calibration overrides under `rf_frontend.mixerless_module`:
  per-state TX/RX amp S21, bypass delta S21, input-referred P1dB and module group
  delay. S21/group-delay keys accept scalars, inline frequency tables or calibration
  filenames.
- Wideband sweep with `tone_powers=None` no longer reapplies tone powers.
- Fixed a late-import bug in `firmware_lib`.
- `perform_sweep()` / `perform_retune()` can refresh the ADC calibration before
  sweeping for improved S21 stability.
- Improved cable-delay fit; `remove_group_delay()` honours frequency-dependent
  calibrations.
- Fixed `generate_newman_phases` to return phases in the original frequency order, not
  the sorted order.

**Structured Info System**
- `get_info(sections)` provides structured system state in 15 named sections:
  `server`, `versions`, `clock`, `fpga`, `rfdc`, `pipeline`, `tones`, `rf_frontend`,
  `lna`, `rfsoc_sensors`, `diagnostics`, `config`, `calibrations`, `resonators`,
  `registers`. Each includes a `ready` flag. The default call excludes the expensive
  sections; use `'all'` for everything.
- `rfsoc_sensors` reports on-chip PS/PL SYSMON die temperatures (C) and supply voltages
  (V) via IIO sysfs, keyed by raw sensor names so duplicate short names (`vccams`,
  `vccint`) stay distinct.
- `health_check()` for compact polling — pass/fail bools for clock lock, ADC/DAC
  saturation, DSP overflow and RTS events, plus key state indicators.
- `rf_frontend` includes the full signal chain: hardware identity, attenuator backend
  details, live attenuator/amp state, derived gain/compression, and updownconverter
  characterisation. `lna` includes controller status and bias readings for all 14
  channels. `config` returns the active config file as YAML text. `calibrations`
  returns resolved per-tone values after frequency-dependent interpolation.
  `resonators` and `registers` are placeholders.
- `tone_indices` renamed to `firmware_indices` in `info['tones']`. Server activity
  flags (`streaming`, `triggered_streaming`, `sweeping`) exposed in `server`.
  `firmware_interface_ready` / `firmware_fast_interface_ready` checks added.

**Pipeline Parameter Access**
- New client methods `set_sync_delay()`/`get_sync_delay()`,
  `set_acc_len()`/`get_acc_len()`, `set_internal_loopback()`/`get_internal_loopback()`,
  `set_psb_scale()`/`get_psb_scale()`, `set_psb_fftshift()`/`get_psb_fftshift()`,
  `set_pfb_fftshift()`/`get_pfb_fftshift()`. Previously settable only via
  `apply_config()`.

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
