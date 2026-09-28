# Live timestream plotting

`souk-live-plot` displays time traces, PSDs and ASDs from a live stream file.
Start by pointing it at a binary capture that another process is writing:

```bash
souk-live-plot --stream-file /path/to/capture --follow --history 60 --span 10
```

The viewer reads the latest minute from the file, displays a ten-second time
window, and follows new records as they are written. The companion
`/path/to/capture.json` supplies the sample rate, tone count and other metadata.
The default layout has amplitude above phase, with time on the left and PSD
on the right. You can display one tone, selected tones, or every tone.

For a completed capture, omit `--follow`; exported samples use `--file`:

```bash
souk-live-plot --stream-file /path/to/capture
souk-live-plot --file samples.npy
```

You can also pass hardware connection details to start a stream receiver and
plot its incoming data directly; see [Optional stream receiver](#optional-stream-receiver).

From a source checkout, use the module command instead:

```bash
PYTHONPATH=src .venv/bin/python -m souk_readout_tools.client.client_scripts.live_plot \
  --stream-file /path/to/capture --follow --history 60 --span 10
```

On a desktop or with display forwarding, use an interactive Matplotlib
backend. If this environment only has the noninteractive Agg backend, install
a GUI binding first (PyQt5 is already a normal client-install dependency):

```bash
.venv/bin/python -m pip install PyQt5
```

`--backend QtAgg` or `--backend TkAgg` selects an installed backend. In IPython,
enable one with e.g. `%matplotlib qt`. Reinstall the package to register the
`souk-live-plot` command if running an older installation.

The toolbar coordinate readout uses ASCII numbers (for example `-1e-06`)
to avoid a Qt5/fontconfig crash when looking up the Unicode minus glyph.
Plot labels and the standard pan/zoom controls are unaffected.

## Layouts and quantities

```bash
# One time trace per selected tone
souk-live-plot --stream-file captures/live --follow --signal i --layout time --tones all

# Amplitude above phase
souk-live-plot --stream-file captures/live --follow --format magphase --layout time

# Time above ASD for one quantity
souk-live-plot --stream-file captures/live --follow --signal phase --layout time-asd

# I/Q rows; time/PSD columns
souk-live-plot --stream-file captures/live --follow --format iq --layout time-psd

# Fractional frequency above dissipation, using the local linear estimate
souk-live-plot --stream-file captures/live --follow --format freq_diss --layout time --sweep sweep.npy

# Exact fitted Mobius conversion, time/ASD columns
souk-live-plot --stream-file captures/live --follow --format freq_diss --layout time-asd \
  --sweep sweep.npy --calibrations resonators.npz --conversion mobius

# Frequency and dissipation in linewidths (Qr from the calibrations)
souk-live-plot --stream-file captures/live --follow --format linewidths --layout time-asd \
  --sweep sweep.npy --calibrations resonators.npz

# Explicit panel order and columns
souk-live-plot --stream-file captures/live --follow --panels logmag:time phase:time --columns 1
```

One to four panels are supported. Explicit panels are written `signal:domain`
in row-major order. Domains are `time`, `psd`, and `asd`.

| Signal | Meaning / units |
|---|---|
| `i`, `q` | Parsed I/Q in accumulator counts |
| `amp` (also `mag`, `magnitude`) | Absolute I+jQ, counts |
| `phase` | Unwrapped phase, radians |
| `logmag` | 20 log10(amplitude / one count), dB |
| `centered_phase` | Phase after subtracting the sweep circle center |
| `centered_rotated_phase` | Centered phase after the fixed sweep rotation |
| `frequency_shift` | Resonance-frequency shift, Hz |
| `fractional_frequency` | Resonance-frequency shift divided by the reference tone frequency |
| `dissipation` | Matched loss coordinate, Δ(1 / 2Qi) |
| `frequency_linewidths` (also `freq_lw`) | Resonance-frequency shift in linewidths, fractional shift × Qr |
| `dissipation_linewidths` (also `diss_lw`) | Qr·Δ(1 / 2Qi), dissipation on the same scale |

The two linewidth signals take each tone's loaded Q from `--calibrations`
(fits or `ResonatorCalibration`), with either conversion; `--format
linewidths` selects the pair.

Centered phase needs a sweep or explicit `phase_center_params` in Python;
geometry is fitted once to the sweep, not to the moving live arc. The sweep
and stream must use the same IQ coordinate system. Frequency/dissipation
conversion reuses the existing resonator/timeseries conventions. Linearized
conversion needs a sweep; Mobius also needs per-tone `ResonatorCalibration`
objects or fits. Precomputed demodulated frequency/dissipation fields in
supplied captures take precedence over reconversion.

Create `resonators.npz` from a saved sweep before using `--conversion mobius`:

```python
from souk_readout_tools.client.readout_client import ReadoutClient
from souk_readout_tools.stream_dac import calibrations_from_sweep, save_calibrations

sweep = ReadoutClient.import_sweep('sweep.npy')
calibrations = calibrations_from_sweep(sweep)
save_calibrations('resonators.npz', calibrations)
```

This fits the sweep and saves the per-tone resonator calibration parameters
in a NumPy `.npz` archive. It is generated offline, separately from the stream
capture and its `.json` metadata. Keep the sweep's tone indices aligned with
the stream; `sorted(calibrations)` shows which tones have calibrations. For
Python use, pass the resulting dictionary directly as `LivePlot(...,
calibrations=calibrations, sweep_data=sweep, conversion_method='mobius')`.
Linearized conversion needs only `--sweep`, with no calibration archive.

The standard parser's readout correction is applied. Fast-modulation streams
can be displayed raw or demodulated; see the next section.
The sweep calibration remains fixed for a live session; after retuning the
resonators or changing that calibration, use a new viewer with the appropriate
sweep. Modulation centres, offsets and RX correction factors follow the
revision-tagged stream updates. Frequency-shift sign and the factor 1/2 in
dissipation match `timeseries.py`.

## Controls and time windows

- **Pause / Play** freezes/resumes the displayed snapshot. File reading
  (or optional direct acquisition and recording) continues while paused.
  The frozen snapshot stays
  available for navigation even as the live ring rolls over.
- **Follow** resumes scrolling with the configured time span and restores full
  spectral frequency limits.
- **Full X** fits the entire displayed history (or frequency range), then holds
  the limits. **Auto Y** resumes continuous vertical autoscaling.
- The normal Matplotlib pan, zoom, home/back/forward and save controls work.
  Changing an axis limit holds that coordinate until Follow or Auto Y is used.
- **Tones** accepts `all`, comma-separated IDs or inclusive ranges such as
  `0-3,8`. These are user-tone indices, not firmware channel numbers.
- **Span (s)** changes the scrolling width. The panel fields accept a new
  `signal:domain` and apply it when Enter is pressed. The number of panels is
  selected at startup (or by constructing another viewer).

`--history 60 --span 10` retains one minute but shows the most recent ten
seconds. Pan back within the retained history, or pause to hold a snapshot.
Retention is selected at startup. `--x-axis sample` uses sample numbers from
stream start; `time` uses elapsed sample time; `telescope_time` uses PTP ticks
converted to seconds at 30.72 MHz. Span is expressed in seconds in all modes.
When following a file, sample/time offsets include the number of records
skipped at startup; losses before that retained tail are not reconstructed.
Telescope timestamps must be valid on the source to be a useful axis.

PSDs use `timeseries.compute_psd` (Welch); ASDs are its square root. Units are
signal-units²/Hz or signal-units/√Hz. A PSD of `logmag` is a spectrum of the dB
signal, in dB²/Hz; it is not a dB display of the amplitude PSD. Spectra use the
whole retained history by default, or `--psd-window 20` for its last 20 seconds.
Zooming the time panel does not change the spectral input. `--nperseg 1024`
controls frequency resolution and is shortened while the buffer fills.

Counter gaps appear as NaN slots at their correct sample times. Counter wrap
is handled; backwards/reset counters insert a discontinuity marker because
the elapsed time is unknown. Spectra use `timeseries.masked_psd` to reject
Welch segments containing missing/nonfinite samples. The status line reports
rejected segments, insufficient data, disconnects and receiver errors.
Time traces draw every sample by default, with renderer path simplification
also disabled. To explicitly reduce drawing work, pass `--max-plot-points 4000`
(or `LivePlot(max_plot_points=4000)` in Python). This retains minima/maxima
from bins across the retained history, so displayed points can be unevenly
spaced and zooming does not recover omitted points. The status line shows
**Display reduced** whenever reduction is active. Bins containing gaps are
left blank so reduction cannot visually join them. This reduces drawing work
and plotted-array size, not raw history storage; PSDs use the full signal.

All streamed tones are retained so the display selection can change at any
time. The default raw history allocation limit is 512 MiB; parsed snapshots
and plotted arrays need additional memory. An oversized request fails with an
estimate and a suggested shorter history. `--max-memory-mb` changes the limit.
Displaying fewer tones reduces analysis/rendering work but not raw storage.
Select a small subset for high-rate or high-tone-count streams; GUI refresh
rate (`--interval 0.2`, in seconds) is independent of acquisition rate.

## Fast frequency modulation

Raw probe-point I/Q, amplitude and phase remain the default. Select
`--data-mode demodulated` to call the existing `modulation.group_cycles` and
`demodulate_timestream` tools, using `--demod-method fast` (default) or
`accurate`. This works with a growing binary file or the optional receiver;
the stream metadata must contain the active modulation state, including probe
offsets, dwell length and settling count.

```bash
# Sample-aligned demodulated magnitude/phase, preserving gaps at settling samples
souk-live-plot --stream-file capture --follow --tones 0-3 --data-mode demodulated

# Fast model-free frequency/dissipation and detuning, linewidth in Hz
souk-live-plot --stream-file capture --follow --tones 0-3 \
  --data-mode demodulated --demod-method fast --linewidth-hz 100000 \
  --panels fractional_frequency:time dissipation:time detuning:time --columns 1

# Calibrated Mobius conversion; no reference sweep needed for demodulation
souk-live-plot --stream-file capture --follow --data-mode demodulated \
  --conversion mobius --calibrations resonators.npz --format freq_diss
```

`LivePlot(data_mode='demodulated', demod_method='fast', linewidth_hz=...)`
provides the same options in Python; `linewidth_hz` can also be a per-tone array
in original tone order. The **Data** button or `viewer.set_data_mode(...)`
switches raw/demodulated display. Choose I/Q, amplitude or phase panels before
switching back to raw; frequency/dissipation conversion of raw modulated IQ
is rejected instead of being mislabelled as demodulated data.

Demodulation preserves the original sample times and rate. It does not average
away dwell samples. Only complete cycles with the expected point and settling
pattern are used; partial edge cycles and cycles containing lost packets are
left as NaN. The first usable per-point reference is held fixed within each
modulation revision. The viewer subscribes to in-band `SNAPSHOT` and
`TONE_UPDATE` records, switches offsets and RX correction at the tagged
revision boundary, and adds applied centre steps to the resonance-side
frequency-shift trace. An unknown revision stays blank until its full state
arrives. A manually recorded file needs its `.updates.jsonl` sidecar
(`receive_stream(..., subscribe_tone_updates=True)`) for this reconstruction.

Settling treatment is explicit: `--settling none` (default) leaves settling
samples as NaN; `--settling raw` demodulates their actual transient IQ while
excluding them from slope/reference estimation. No interpolation is performed.
PSDs reject segments containing NaNs, so frequent settling gaps can leave no
usable segment at the chosen `--nperseg`. Using `raw` keeps those transients
in the spectrum; it does not make them valid steady-state measurements.

`frequency_shift` is resonance-side motion; `fractional_frequency` divides it
by the reference tone frequency. `dissipation` is the matched loss coordinate
Delta(1 / (2 Qi)). Model-free conversion uses the probe-point gradient;
`--conversion mobius` uses the supplied resonator calibrations.

Plot the derivative-based tuning diagnostics directly, without a linewidth
or fit:

```bash
souk-live-plot --stream-file capture --follow --tones 0-3 \
  --data-mode demodulated --demod-method fast \
  --panels responsivity:time curvature:time --columns 1
```

- `responsivity` / `dphi_df`: signed phase slope, rad/Hz.
- `curvature` / `d2phi_df2`: phase second derivative, rad/Hz².
- `curvature_over_slope`: second derivative divided by first, 1/Hz.

For symmetric offsets `-h, 0, +h`, the fast estimator uses
`(phase_plus - phase_minus)/(2*h)` and
`(phase_plus - 2*phase_zero + phase_minus)/h**2`, averaging repeated
measurements of the same offset. Curvature crosses zero at the phase
inflection point (peak phase responsivity for the ideal resonator model).
It is a tuning-error diagnostic, not itself a distance in Hz. Detuning inverts
the circle-centred notch phase `-2 atan(2x)` exactly:
`detuning_linewidths = -curvature / (2 * slope * |slope|)`, and
`detuning_hz` is that times the linewidth. With `--calibrations` (any
`--conversion`) the derivative estimates use the de-embedded, centred phase.

These diagnostics are estimated once per modulation cycle, held across that
cycle for time plots, and use the cycle rate for PSD/ASD. Choose e.g.
`--panels responsivity:time responsivity:psd curvature:time curvature:psd`
for a four-panel view. Frequency/dissipation traces remain sample-aligned.

`detuning` (alias `detuning_hz`) and `detuning_linewidths` use the existing
cycle estimator: positive means probe above resonance. In the model-free case
it requires a linewidth and sufficient distinct offsets for curvature; without
these, estimates remain NaN. Pass `--calibrations`: the per-cycle slope,
curvature and detuning are then taken on the circle-centred phase with each
tone's fitted linewidth, whatever `--conversion` is. That is the same
estimate the server tracker acts on. Without calibrations the raw phase is
used; its detuning is taken about the origin rather than the circle centre and
reads 3x to ~100x high (worse for deeper dips), so only its sign and zero mean
anything.
Detuning is held across each cycle for the time plot, explicitly noted in the
status; its PSD/ASD uses one estimate per cycle at the cycle rate, preserving
missing cycles as gaps. It is not presented as a sample-rate measurement.

To move the modulation centres toward **zero of the calibrated phase
curvature** shown by `--calibrations ... --panels curvature:time`, call
`client.update_modulation()` while streaming. The RFSoC server fits the
selected resonators from its latest sweep held in memory, caches those fits,
and uses its live, settling-excluded I/Q accumulator to estimate calibrated
phase curvature. It applies one correction of at most half a fitted linewidth
per tone. The viewer follows the resulting revision in the stream without a
restart. No client capture or calibration archive lookup is involved. Run a
targeted sweep on the server first if it has no valid sweep in memory. Tones
without usable fits are listed as skipped. Explicit `center=` or `offsets=`
still sends the requested values. The plot and server use the same calibrated
phase calculation when their fits come from the same sweep with the same fit
settings; an archive from a different sweep can have a different phase zero.

`client.enable_tracking()` can use fitted linewidths from that server sweep
when they were not passed to `enable_modulation`. Tracking currently
estimates **raw-I/Q** phase curvature, which can have a different zero from
the Mobius plot. Its default `dry_run=True` observes and stages corrections
without applying them; the no-argument `update_modulation()` path above uses
the calibrated plot basis.

## Following a growing file

`--stream-file PATH --follow` starts with up to `--history` seconds of the
most recent complete records, then reads newly appended records. It seeks to
the recent tail, so it does not load an entire long recording. The in-memory
ring remains bounded. EOF is temporary: the viewer waits for more data until
you close it. A partially written last record waits for completion, and all
file handles are read-only. New data becomes visible after the writer flushes
it. Truncating or replacing the file stops the reader with an error; start a
new viewer for the new capture. Metadata and calibration are fixed for a
viewing session.

Pause freezes the displayed snapshot while file reading continues. The
**Follow** button controls axis scrolling; the CLI `--follow` option selects
the growing-file source. Closing the viewer stops only its file reader.

Without `--follow`, file inspection loads the supplied capture into memory
and initially fits its whole span. Standard raw file captures have counter
gaps filled with NaNs using `timeseries.fill_gaps`; duplicate/backward counters
or more than one million missing samples require preparing the capture
explicitly. For caller-supplied or demodulated data, supply a uniform sample
grid with NaNs for missing samples before `update`; an irregular time axis
alone does not make a uniform-rate PSD valid. A separate Qt application with
file browsers and dropdown selectors remains a future extension; this version
uses Matplotlib widgets.

## Python use

```python
from souk_readout_tools.plotting import LivePlot, Panel

viewer = LivePlot(tones=[0, 2], history_seconds=60, view_seconds=10)
viewer.start_file('/path/to/capture')
try:
    viewer.show()
finally:
    viewer.close()

# A custom layout with calibration already loaded; also reads only the file.
viewer = LivePlot(
    panels=[Panel('fractional_frequency', 'time'),
            Panel('fractional_frequency', 'asd'),
            Panel('dissipation', 'time'), Panel('dissipation', 'asd')],
    layout=(2, 2), tones=[0], sweep_data=sweep, calibrations=calibrations,
    conversion_method='mobius')
viewer.start_file('/path/to/capture')
viewer.show()
viewer.close()

# Reuse with parsed samples on the GUI thread.
from souk_readout_tools.client.readout_client import ReadoutClient
viewer = LivePlot(['amp:time', 'phase:time'], tones='all')
viewer.update(ReadoutClient.import_samples('samples.npy'))
viewer.full_span()
viewer.show()
```

`LivePlot.figure`, `.axes` and `.lines` expose the Matplotlib objects. Passing
an empty `figure=` and `controls=False` allows embedding in a Qt canvas with
external controls. `set_tones`, `set_panel`, `set_view_seconds`, `set_paused`,
`follow`, `full_span` and `autoscale_y` provide the same actions programmatically.
Use these on the GUI thread and retain the viewer object while the window is
open. `start_file` runs disk reading in a worker; no hardware client instance
is created.

## Optional stream receiver

Passing a hardware configuration with `-C`, or an address and request port
with `-a` and `-r`, starts a stream receiver using `ReadoutClient.receive_stream`
and plots the incoming samples directly:

```bash
# Start a receiver for a server that is already streaming
souk-live-plot -C my_config.yaml --tones 0,2,5 --history 60 --span 10

# Also ask the server to begin transmitting
souk-live-plot -a 10.11.11.11 -r 10000 --enable-stream --tones 0-3

# Receive, display and save the stream for 120 seconds
souk-live-plot -C my_config.yaml --record captures/live --duration 120
```

Hardware details start the receiver; `--enable-stream` additionally requests
that the server begin transmitting. Closing the window stops this receiver
and any recording it owns; server streaming remains enabled. For a simulated
source, use `souk-live-plot --mock`.

The command-line viewer receives in a **separate process**, which forwards
frames to the plot in ~50 ms batches. A receiver thread inside the plotting
process competes with the refresh for Python's GIL: with a slow refresh
(long history, many tones, demodulation) it fell to ~200 frames/s, the
server's send buffer filled and the server dropped the client. In Python use
`viewer.start_process(connect)`, where `connect` is a picklable callable
returning a ready client:

```python
import functools
from souk_readout_tools.client.readout_client import ReadoutClient
viewer = LivePlot(tones=[0, 1], data_mode='demodulated')
viewer.start_process(functools.partial(ReadoutClient, config_file='my_config.yaml'),
                     filename='captures/live')      # optional recording
viewer.show()
```

`plot_live(client, ...)` and `viewer.start(client)` still receive on a thread
with an existing client object; they suit low rates or quick refreshes.
`plot_live(..., show=False)` starts reception without calling `show`.

Snapshots for display copy only the tones the panels show, so the refresh
cost scales with displayed tones, not streamed ones (60 s at 2500 Hz, 3 of
380 tones: 0.06 s instead of 1 s).

The receiver extension is also usable independently:

```python
from threading import Event
stop = Event()
client.receive_stream(frame_callback=callback, stop_event=stop,
                      write_to_file=False)
# callback(payload_bytes, metadata) runs once per complete IQ frame.
# Set stop from another thread to interrupt even an idle/partial socket read.
```

Callbacks run in the calling receiver thread and should be fast. Payloads own
their bytes; they exclude the TCP length prefix. Metadata has the same schema
as the capture sidecar. Default `receive_stream` disk behavior is unchanged.
