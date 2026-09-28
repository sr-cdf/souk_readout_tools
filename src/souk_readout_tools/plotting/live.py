"""Interactive Matplotlib time/PSD/ASD panels for a readout stream.

``LivePlot.start_file(path)`` follows an existing binary stream capture into
a bounded in-memory history without connecting to hardware. ``plot_live(client)``
also supports direct reception using ``client.receive_stream``, and
``LivePlot.start_process(connect)`` receives in a separate process. ``LivePlot.update(samples)`` displays an existing
parsed capture, or data supplied by another acquisition loop. The plotting
object and its controls stay on the GUI thread; only receiving runs in a worker.
See ``doc/live_plotting.md`` for layouts, conversions and controls.
"""

from dataclasses import dataclass
import multiprocessing
import queue
import threading
import time
import warnings

import numpy as np

from ._common import _get_pyplot
from ._live_buffer import StreamBuffer
from ._live_signals import SignalExtractor, SIGNAL_LABELS, canonical_signal
from .timestream import TT_CLOCK_HZ
from ..timeseries import compute_psd, masked_psd


def parse_tones(value):
    """Parse ``all``, ``0,2,5`` or inclusive ranges such as ``0-3,8``."""
    if value is None or (isinstance(value, str) and value.strip().lower() == 'all'):
        return None
    if isinstance(value, str):
        result = []
        for part in value.split(','):
            ends = part.strip().split('-')
            if len(ends) == 1:
                result.append(int(ends[0]))
            elif len(ends) == 2:
                first, last = map(int, ends)
                if last < first:
                    raise ValueError('Tone ranges must be increasing')
                result.extend(range(first, last + 1))
            else:
                raise ValueError('Use tone numbers, ranges, or all')
    else:
        result = [int(t) for t in value]
    if not result or min(result) < 0:
        raise ValueError('Select at least one nonnegative tone index')
    return list(dict.fromkeys(result))


@dataclass(frozen=True)
class Panel:
    """A panel displaying one quantity for the selected tones.

    ``signal`` is one of ``i``, ``q``, ``amp``, ``phase``, ``logmag``,
    ``centered_phase``, ``centered_rotated_phase``, ``frequency_shift``,
    ``fractional_frequency``, or ``dissipation``. ``domain`` is ``time``,
    ``psd`` or ``asd``. Optional ``tones`` overrides the global selection.
    """

    signal: str = 'amp'
    domain: str = 'time'
    tones: object = None

    def __post_init__(self):
        object.__setattr__(self, 'signal', canonical_signal(self.signal))
        if self.domain not in ('time', 'psd', 'asd'):
            raise ValueError('Panel domain must be time, psd or asd')
        if self.tones is not None:
            selection = parse_tones(self.tones)
            object.__setattr__(self, 'tones', 'all' if selection is None else tuple(selection))


def _panel(value):
    if isinstance(value, Panel):
        return value
    if isinstance(value, str):
        parts = value.split(':')
        if len(parts) == 1:
            return Panel(parts[0])
        if len(parts) == 2:
            return Panel(*parts)
        raise ValueError('Use signal:time, signal:psd or signal:asd')
    if isinstance(value, dict):
        return Panel(**value)
    return Panel(*value)


def _positive(value, name):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be positive and finite')
    return value


def _thin_trace(x, y, max_points):
    """Retain extrema and a gap marker per bin for drawing only."""
    if max_points is None or len(x) <= max_points:
        return x, y
    indices = [0]
    for group in np.array_split(np.arange(len(x)), max(1, max_points // 3)):
        values = y[group]
        finite = np.isfinite(values)
        if not finite.all():
            # Leave the whole drawing bin blank: keeping a single gap plus
            # extrema could visually bridge other gaps inside the same bin.
            indices.append(group[np.flatnonzero(~finite)[0]])
        else:
            indices.extend((group[np.argmin(values)], group[np.argmax(values)]))
    indices.append(len(x) - 1)
    indices = np.unique(indices)
    return x[indices], y[indices]


def _spectrum(values, rate, nperseg):
    """Use the analysis estimators without joining samples across gaps."""
    if len(values) < 2:
        return np.empty(0), np.empty(0), 'waiting for samples'
    nperseg = min(nperseg, len(values))
    bad = ~np.isfinite(values)
    if not bad.any():
        f, p = compute_psd(values, rate, nperseg=nperseg)
        note = ''
    else:
        # The UI reports rejection rather than emitting the same warning on
        # every refresh. Reject whole segments; never interpolate a PSD gap.
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', message='segment rejection discarded.*')
            try:
                result = masked_psd(values, rate, bad, nperseg=nperseg)
            except ValueError as exc:
                if 'Welch segments contain a flagged sample' not in str(exc):
                    raise
                return np.empty(0), np.empty(0), 'no gap-free Welch segments'
        f, p = result['f'], result['psd']
        note = f"{result['lost_fraction']:.0%} Welch segments rejected"
    keep = (f > 0) & np.isfinite(p) & (p > 0)
    return f[keep], p[keep], note


def _forward_stream(connect, connection, stop, filename, duration, info, num_tones):
    """Child process for ``LivePlot.start_process``: receive, forward in batches.

    Sends ``('frames', metadata or None, [payload, ...])`` about every 50 ms
    (metadata with the first batch only; ``receive_stream`` fixes it before the
    first frame), ``('update', record)`` at each revision boundary, and
    ``('error', message)`` on failure. A separate sender
    thread does the pipe writes, so a slow viewer never stalls the socket.
    """
    outbox = queue.Queue()

    def send():
        while (message := outbox.get()) is not None:
            connection.send(message)

    sender = threading.Thread(target=send, daemon=True)
    sender.start()
    batch, sent_metadata, first_metadata, last = [], False, None, time.monotonic()

    def flush():
        nonlocal batch, sent_metadata, last
        if batch:
            outbox.put(('frames', None if sent_metadata else first_metadata, batch))
            batch, sent_metadata, last = [], True, time.monotonic()

    def forward(payload, metadata):
        nonlocal first_metadata
        if first_metadata is None:
            first_metadata = metadata
        batch.append(payload)
        if time.monotonic() - last >= 0.05:
            flush()

    def forward_update(record):
        flush()
        outbox.put(('update', record))

    try:
        client = connect()
        client.receive_stream(
            num_tones=num_tones, filename=filename, duration=duration, info=info,
            frame_callback=forward, update_callback=forward_update,
            subscribe_tone_updates=not getattr(client, 'mock', False),
            stop_event=stop, write_to_file=filename is not None)
        flush()
    except Exception as exc:
        outbox.put(('error', f'{type(exc).__name__}: {exc}'))
    outbox.put(None)
    sender.join()


class LivePlot:
    """A reusable Matplotlib viewer with independent history and visible span.

    Parameters
    ----------
    panels : sequence, optional
        ``Panel`` objects, ``(signal, domain)`` pairs or ``'signal:domain'``
        strings, in row-major order. Default: amplitude/phase rows and time/PSD
        columns. One to four panels are supported.
    layout : (rows, columns), optional
        Defaults to (2, 2) for four panels, otherwise vertically stacked.
    tones : sequence of int or str, optional
        User-tone indices, e.g. ``[0, 3]`` or ``'0-3,8'``; default tone 0.
        ``None`` or ``'all'`` displays every tone.
    history_seconds, view_seconds : float
        Retained live history (60 s) and scrolling time-axis width (10 s).
    psd_seconds : float, optional
        Trailing interval used for spectra; default all retained history.
        Independent of toolbar zoom and ``view_seconds``.
    x_axis : {'time', 'sample', 'telescope_time'}
        Stream-relative seconds, absolute sample number, or PTP seconds.
    nperseg : int
        Welch segment length, shortened while the buffer is filling.
    interval_ms : float
        GUI refresh interval (default 200 ms).
    sweep_data, calibrations, conversion_method, reference_tone_frequency,
        smooth_window_hz
        Passed to ``resonator.convert_samples`` (see ``resonator.QUANTITIES``
        for the names and signs). ``calibrations`` -- a record, fits or
        ``ResonatorCalibration`` objects -- defaults to the one the stream
        carries. Centred panels use it, else a model-free calibration of the
        sweep. Frequency conversion needs a sweep ('linearized', 'lookup') or
        calibrations ('phase', 'mobius').
    data_mode : {'raw', 'demodulated'}
        Raw probe samples (default), or sample-aligned modulation demodulation.
    demod_method : {'fast', 'accurate'}
        Existing modulation phase-derivative estimator; default fast.
    settling : {'none', 'raw'}
        Leave settling samples as NaN (default), or demodulate their raw IQ.
        Neither option averages away samples or interpolates gaps.
    linewidth_hz : float or array-like, optional
        Positive per-tone linewidth for cycle detuning on a tone without a
        calibration. With ``calibrations`` the per-cycle slope, curvature and
        detuning are taken on the circle-centred phase with each tone's fr/Ql,
        whatever ``conversion_method`` is; raw-phase detuning reads 3x to
        ~100x high. Detuning spectra use cycle rate.
    max_plot_points : int or None
        Default None draws every sample. An explicit limit (>= 6) enables
        extrema-based time-trace reduction across retained history. Selected
        points can be unevenly spaced; the status line reports reduction.
        This affects drawing only, not raw storage or spectral calculations.
    max_bytes : int
        Maximum raw history allocation; default 512 MiB. Parsed snapshots and
        plotting arrays need additional temporary memory.
    figure : matplotlib.figure.Figure, optional
        Empty figure, for embedding in a future Qt application.
    controls : bool
        Include Matplotlib buttons and editable panel/selection fields.

    Notes
    -----
    Keep a reference to this object while the window is open. All methods
    except the receiver callback must be called on the GUI thread.
    """

    def __init__(self, panels=None, *, layout=None, tones=(0,),
                 history_seconds=60, view_seconds=10, psd_seconds=None,
                 x_axis='time', nperseg=1024, interval_ms=200,
                 max_plot_points=None, max_bytes=512 * 1024**2,
                 data_mode='raw', demod_method='fast', settling='none', linewidth_hz=None,
                 sweep_data=None, calibrations=None, conversion_method='linearized',
                 reference_tone_frequency=None, smooth_window_hz=1000,
                 figure=None, controls=True):
        self.panels = [_panel(p) for p in (panels if panels is not None else (
            'amplitude:time', 'amplitude:psd', 'unwrapped_phase:time',
            'unwrapped_phase:psd'))]
        if not 1 <= len(self.panels) <= 4:
            raise ValueError('Choose between one and four panels')
        layout = layout or ((2, 2) if len(self.panels) == 4 else (len(self.panels), 1))
        if len(layout) != 2 or min(layout) < 1 or np.prod(layout) != len(self.panels):
            raise ValueError('layout rows * columns must equal the number of panels')
        if x_axis not in ('time', 'sample', 'telescope_time'):
            raise ValueError('x_axis must be time, sample or telescope_time')
        self.tones = parse_tones(tones)
        self.history_seconds = _positive(history_seconds, 'history_seconds')
        self.view_seconds = _positive(view_seconds, 'view_seconds')
        self.psd_seconds = None if psd_seconds is None else _positive(psd_seconds, 'psd_seconds')
        self.interval_ms = _positive(interval_ms, 'interval_ms')
        self.nperseg = int(nperseg)
        self.max_plot_points = None if max_plot_points is None else int(max_plot_points)
        if self.nperseg < 2:
            raise ValueError('nperseg must be >= 2')
        if self.max_plot_points is not None and self.max_plot_points < 6:
            raise ValueError('max_plot_points must be None or >= 6')
        if data_mode not in ('raw', 'demodulated'):
            raise ValueError('data_mode must be raw or demodulated')
        from ..modulation import StreamDemodulator
        self.data_mode = data_mode
        # Demodulated data: the calibrations choose the centred basis and
        # 'mobius' the exact inversion; otherwise the three-point 'phase'
        # form (calibrated tones) or the 'linear' projection (the rest).
        self._demodulator = StreamDemodulator(
            demod_method, settling, linewidth_hz, calibration=calibrations,
            conversion='mobius' if conversion_method == 'mobius' else None)
        self.x_axis = x_axis
        self.buffer = StreamBuffer(self.history_seconds, max_bytes=max_bytes)
        self.extractor = SignalExtractor(
            sweep_data=sweep_data, calibrations=calibrations,
            conversion_method=conversion_method,
            reference_tone_frequency=reference_tone_frequency,
            smooth_window_hz=smooth_window_hz)
        plt = _get_pyplot()
        self.figure = figure if figure is not None else plt.figure(figsize=(12, 8))
        if self.figure.axes:
            raise ValueError('figure must be empty')
        self.axes = list(self.figure.subplots(*layout, squeeze=False).flat)
        self.figure.subplots_adjust(bottom=0.27 if controls else 0.12,
                                    top=0.92, hspace=0.38, wspace=0.3)
        self.lines = [{} for _ in self.panels]
        self.follow_x = [True] * len(self.panels)
        self.auto_y = [True] * len(self.panels)
        self.paused = False
        self.error = None
        self._samples = None
        self._input_samples = None
        self._pending_samples = None
        self._updating = False
        self._closed = False
        self._thread = None
        self._process = self._process_stop = None
        self._source_kind = 'stream'
        self._sample_index_offset = 0
        self._stop = threading.Event()
        self._finished = threading.Event()
        self._last_received = -1
        self._plot_note = ''
        self._control_error = ''
        self._widgets = []
        self._status = self.figure.text(0.08, 0.02, 'Waiting for data', fontsize=9)
        self.figure.suptitle('SOUK live timestream')
        for index, ax in enumerate(self.axes):
            self._configure_axis(index)
            ax.callbacks.connect('xlim_changed', lambda _, i=index: self._manual_limit(i, 'x'))
            ax.callbacks.connect('ylim_changed', lambda _, i=index: self._manual_limit(i, 'y'))
        if controls:
            self._make_controls()
        # Qt's toolbar QLabel can crash in fontconfig fallback for Unicode
        # minus signs from the default coordinate formatter. Keep cursor text
        # ASCII without changing plot tick labels or pan/zoom behavior.
        for ax in self.figure.axes:
            ax.format_coord = lambda x, y: f'x={x:.12g} y={y:.12g}'
        self._close_cid = self.figure.canvas.mpl_connect('close_event', lambda _: self.close())
        self._timer = self.figure.canvas.new_timer(interval=int(self.interval_ms))
        self._timer.add_callback(self.refresh)

    def _configure_axis(self, index):
        ax, panel = self.axes[index], self.panels[index]
        name, unit = SIGNAL_LABELS[panel.signal]
        spectral = panel.domain != 'time'
        ax.set_xscale('log' if spectral else 'linear')
        ax.set_yscale('log' if spectral else 'linear')
        if spectral:
            unit = f'{unit}²/Hz' if panel.domain == 'psd' else f'{unit}/√Hz'
            ax.set_xlabel('Frequency (Hz)')
            ax.set_ylabel(f'{name} {panel.domain.upper()} ({unit})')
        else:
            ax.set_xlabel({'time': 'Time from stream start (s)',
                           'sample': 'Sample number',
                           'telescope_time': 'Telescope time (s)'}[self.x_axis])
            ax.set_ylabel(f'{name} ({unit})')
        ax.grid(True, alpha=0.25)

    def _manual_limit(self, index, coordinate):
        if not self._updating:
            (self.follow_x if coordinate == 'x' else self.auto_y)[index] = False

    def _make_controls(self):
        from matplotlib.widgets import Button, TextBox

        def button(rect, label, callback):
            widget = Button(self.figure.add_axes(rect), label)
            widget.on_clicked(callback)
            self._widgets.append(widget)
            return widget

        self._mode_button = button(
            [0.08, 0.225, 0.22, 0.035], f'Data: {self.data_mode}',
            lambda _: self._toggle_data_mode())
        self._pause_button = button([0.08, 0.18, 0.11, 0.035], 'Pause', lambda _: self.set_paused(not self.paused))
        button([0.20, 0.18, 0.11, 0.035], 'Follow', lambda _: self.follow())
        button([0.32, 0.18, 0.11, 0.035], 'Full X', lambda _: self.full_span())
        button([0.44, 0.18, 0.11, 0.035], 'Auto Y', lambda _: self.autoscale_y())

        def textbox(rect, label, initial, callback):
            widget = TextBox(self.figure.add_axes(rect), label, initial=initial)
            def submit(value):
                try:
                    callback(value)
                    self._control_error = ''
                except (ValueError, KeyError, IndexError) as exc:
                    self._control_error = str(exc)
                self._update_status()
                self.figure.canvas.draw_idle()
            widget.on_submit(submit)
            self._widgets.append(widget)

        textbox([0.63, 0.18, 0.12, 0.035], 'Tones ',
                'all' if self.tones is None else ','.join(map(str, self.tones)), self.set_tones)
        textbox([0.85, 0.18, 0.09, 0.035], 'Span (s) ', str(self.view_seconds), self.set_view_seconds)
        width = 0.84 / len(self.panels)
        for i, panel in enumerate(self.panels):
            textbox([0.10 + i * width, 0.105, width - 0.045, 0.035], f'P{i+1} ',
                    f'{panel.signal}:{panel.domain}', lambda value, index=i: self.set_panel(index, value))
        self.figure.text(0.08, 0.065, 'Panels: signal:time / signal:psd / signal:asd    '
                         'Tones: all, 0,2 or 0-3    Pan/zoom holds axis limits', fontsize=9)

    def _start_worker(self, receive, name):
        if self._thread is not None or self._closed:
            raise RuntimeError('Create a new LivePlot to start another source')

        def run():
            try:
                receive()
            except Exception as exc:
                self.error = exc
            finally:
                self._finished.set()

        self._thread = threading.Thread(target=run, name=name, daemon=True)
        self._thread.start()
        self._timer.start()
        return self

    def start(self, client, *, filename=None, duration=None, info=None, num_tones=None):
        """Start receiving; optionally record the same stream to ``filename``.

        The server must already be streaming. Closing the figure stops this
        receiver; it does not change the server's stream state. For a capture
        already being written by another process, use ``start_file`` instead.
        """
        if duration is not None:
            duration = _positive(duration, 'duration')

        def receive():
            client.receive_stream(
                num_tones=num_tones, filename=filename, duration=duration,
                info=info, frame_callback=self.buffer.append_frame,
                update_callback=self.buffer.append_update,
                subscribe_tone_updates=not getattr(client, 'mock', False),
                stop_event=self._stop, write_to_file=filename is not None)

        return self._start_worker(receive, 'souk-live-receiver')

    def start_process(self, connect, *, filename=None, duration=None, info=None,
                      num_tones=None):
        """Like ``start``, but receive in a separate process.

        ``connect`` is a picklable zero-argument callable returning a ready
        ``ReadoutClient``, e.g. ``functools.partial(ReadoutClient,
        config_file='my_config.yaml')``; the child process builds its own
        client with it. A receiver thread in this process competes with the
        plotting for the GIL and drops frames once refreshes are slow (long
        history, many tones, demodulation); a receiver process does not.
        Closing the viewer stops the process, and any recording it owns.
        """
        if self._thread is not None or self._closed:
            raise RuntimeError('Create a new LivePlot to start another source')
        if duration is not None:
            duration = _positive(duration, 'duration')
        context = multiprocessing.get_context('spawn')
        receiver, sender = context.Pipe(duplex=False)
        stop = context.Event()
        process = context.Process(
            target=_forward_stream, name='souk-live-receiver', daemon=True,
            args=(connect, sender, stop, filename, duration, info, num_tones))
        process.start()
        sender.close()          # so the pipe reports EOF once the child exits
        self._process, self._process_stop = process, stop

        def receive():
            metadata = None
            try:
                while not self._stop.is_set():
                    if not receiver.poll(0.1):
                        continue
                    try:
                        message = receiver.recv()
                    except EOFError:
                        return
                    if message[0] == 'error':
                        raise RuntimeError(message[1])
                    if message[0] == 'update':
                        self.buffer.append_update(message[1])
                        continue
                    metadata = message[1] or metadata
                    for payload in message[2]:
                        self.buffer.append_frame(payload, metadata)
            finally:
                stop.set()
                process.join(timeout=2)
                if process.is_alive():
                    process.terminate()
                receiver.close()

        return self._start_worker(receive, 'souk-live-pipe')

    def start_file(self, filename, *, poll_interval=0.1):
        """Follow an existing binary capture without connecting to hardware.

        ``filename`` is the binary file written by ``receive_stream``; its
        ``filename + '.json'`` metadata file must exist alongside it. Start at
        the most recent ``history_seconds`` of complete records, then read new
        records as they appear. Partial trailing records wait for completion.
        EOF is temporary: keep following until ``close()``. A truncated or
        replaced file ends the source with an error instead of mixing captures.
        """
        from ._live_file import follow_stream_file

        if self._thread is not None or self._closed:
            raise RuntimeError('Create a new LivePlot to start another source')
        poll_interval = _positive(poll_interval, 'poll_interval')
        self._source_kind = 'file'

        def append(payload, metadata):
            self._sample_index_offset = metadata['sample_index_offset']
            self.buffer.append_frame(payload, metadata)

        def receive():
            follow_stream_file(filename, append, self._stop,
                               update_callback=self.buffer.append_update,
                               history_seconds=self.history_seconds,
                               poll_interval=poll_interval)

        return self._start_worker(receive, 'souk-file-reader')

    def _snapshot(self):
        samples = self.buffer.snapshot(
            tones=self._display_tones(),
            apply_readout_correction=self.data_mode != 'demodulated')
        if samples is not None and self._sample_index_offset:
            samples['sample_index'] += self._sample_index_offset
            samples['time_s'] += self._sample_index_offset / samples['sample_rate']
            for record in samples.get('tone_updates', ()):
                record['sample_index'] += self._sample_index_offset
        return samples

    def refresh(self):
        """GUI timer callback; receiving continues while display is paused."""
        if self._closed:
            return
        try:
            received = self.buffer.total_received
            if not self.paused and received != self._last_received:
                samples = self._snapshot()
                if samples is not None:
                    self.update(samples)
                    self._last_received = received
        except Exception as exc:
            self._plot_note = f'Plot error: {exc}'
        self._update_status()
        self.figure.canvas.draw_idle()

    def _selection(self, panel):
        if panel.tones == 'all':
            return None
        return panel.tones if panel.tones is not None else self.tones

    def update(self, samples):
        """Display a parsed ``parse_samples``/``parse_stream`` capture.

        This also supports file inspection without a client. Call on the GUI
        thread. The supplied dictionary is retained until the next update.
        """
        if self.paused:
            self._pending_samples = samples
            return self
        self._input_samples = samples
        self._samples = (self._demodulate_samples(samples)
                         if self.data_mode == 'demodulated' else samples)
        self._render()
        self._update_status()
        self.figure.canvas.draw_idle()
        return self

    def _display_tones(self):
        """Tones shown by any panel, or None when a panel shows all tones."""
        selections = [self._selection(panel) for panel in self.panels]
        if any(selection is None for selection in selections):
            return None
        return sorted({tone for selection in selections for tone in selection})

    def _demodulate_samples(self, samples):
        return self._demodulator(samples, tones=self._display_tones())

    def set_data_mode(self, mode):
        """Switch raw/demodulated display, retaining per-revision references."""
        if mode not in ('raw', 'demodulated'):
            raise ValueError('data_mode must be raw or demodulated')
        old_mode, old_samples = self.data_mode, self._samples
        try:
            self.data_mode = mode
            if self._input_samples is not None:
                source = (self._snapshot() if self.buffer.total_received
                          else self._input_samples)
                self._input_samples = source
                self._samples = (self._demodulate_samples(source)
                                 if mode == 'demodulated' else source)
            self._redraw()
        except Exception:
            self.data_mode, self._samples = old_mode, old_samples
            raise
        if hasattr(self, '_mode_button'):
            self._mode_button.label.set_text(f'Data: {mode}')

    def _toggle_data_mode(self):
        try:
            self.set_data_mode('demodulated' if self.data_mode == 'raw' else 'raw')
            self._control_error = ''
        except (ValueError, KeyError) as exc:
            self._control_error = str(exc)
        self._update_status()
        self.figure.canvas.draw_idle()

    def _time_axis(self, samples):
        n = int(samples['num_samples'])
        if self.x_axis == 'sample':
            return np.asarray(samples.get('sample_index', np.arange(n)), dtype=float)
        if self.x_axis == 'telescope_time':
            x = np.asarray(samples['telescope_time'], dtype=float) / TT_CLOCK_HZ
            x[np.asarray(samples.get('packet_missing', np.zeros(n)), dtype=bool)] = np.nan
            return x
        return np.asarray(samples.get('time_s', np.arange(n) / samples['sample_rate']), dtype=float)

    def _render(self):
        if self._samples is None:
            return
        samples = self._samples
        # Only convert a tone for quantities actually requested for that tone.
        # An IQ-only panel must not require another panel's calibration.
        groups = {}
        for panel in self.panels:
            selection = self._selection(panel)
            key = None if selection is None else tuple(sorted(selection))
            groups.setdefault(key, set()).add(panel.signal)
        values = {}
        for tones, signals in groups.items():
            for signal, rows in self.extractor.extract(samples, signals, tones=tones).items():
                values.setdefault(signal, {}).update(rows)
        rate = _positive(samples['sample_rate'], 'sample_rate')
        time_x = self._time_axis(samples)
        notes = {samples['demod_note']} if 'demod_note' in samples else set()
        if self.extractor.note:
            notes.add(self.extractor.note)
        if not samples.get('modulation_demodulated') and np.any(
                np.asarray(samples.get('modulation_point', [])) > 0):
            notes.add('Raw modulated probe samples')
        spectra = {}
        self._updating = True
        try:
            for i, (ax, panel, lines) in enumerate(zip(self.axes, self.panels, self.lines)):
                selection = self._selection(panel)
                rows = {key: row for key, row in values[panel.signal].items()
                        if selection is None or int(key) in selection}
                changed = set(lines) != set(rows)
                for key in list(lines):
                    if key not in rows:
                        lines.pop(key).remove()
                for key, row in rows.items():
                    row = np.asarray(row, dtype=float)
                    if panel.domain == 'time':
                        x, y = _thin_trace(time_x, row, self.max_plot_points)
                        if len(x) < len(row):
                            notes.add(f'Display reduced: at most {self.max_plot_points} '
                                      'points/trace (extrema; uneven spacing)')
                    else:
                        cache_key = panel.signal, key
                        if cache_key not in spectra:
                            spectral_row, spectral_rate = row, rate
                            cycle_fields = samples.get('cycle_spectra', {})
                            # Demodulated data's tone is the modulation centre.
                            field = {'detuning_hz': 'center_detuning_hz',
                                     'detuning_linewidths': 'center_detuning_linewidths',
                                     }.get(panel.signal, panel.signal)
                            if field in cycle_fields:
                                spectral_row = cycle_fields[field][key]
                                spectral_rate = samples['cycle_rate']
                            count = (len(spectral_row) if self.psd_seconds is None else
                                     max(2, int(self.psd_seconds * spectral_rate)))
                            spectra[cache_key] = _spectrum(
                                spectral_row[-count:], spectral_rate, self.nperseg)
                        x, p, note = spectra[cache_key]
                        y = np.sqrt(p) if panel.domain == 'asd' else p
                        if note:
                            notes.add(note)
                    if key not in lines:
                        lines[key], = ax.plot([], [], lw=0.9, label=f'Tone {int(key)}',
                                              color=f'C{int(key) % 10}')
                    lines[key].set_data(x, y)
                    # Do not silently simplify vertices in the renderer either.
                    lines[key].get_path().should_simplify = False
                if changed:
                    legend = ax.get_legend()
                    if legend is not None:
                        legend.remove()
                    if lines:
                        ax.legend(loc='upper right', fontsize='small', ncol=min(4, len(lines)))
                ax.relim()
                if self.auto_y[i]:
                    ax.set_autoscaley_on(True)
                    ax.autoscale_view(scalex=False, scaley=True)
                if self.follow_x[i]:
                    if panel.domain == 'time':
                        valid_x = time_x[np.isfinite(time_x)]
                        if valid_x.size:
                            end = valid_x[-1]
                            span = self.view_seconds * (rate if self.x_axis == 'sample' else 1)
                            ax.set_xlim(end - span, end)
                    else:
                        ax.set_autoscalex_on(True)
                        ax.autoscale_view(scalex=True, scaley=False)
        finally:
            self._updating = False
        self._plot_note = '; '.join(sorted(notes))

    def _redraw(self):
        self._render()
        self._update_status()
        self.figure.canvas.draw_idle()

    def set_paused(self, paused=True):
        """Freeze the displayed snapshot; receiving and ring eviction continue."""
        self.paused = bool(paused)
        if hasattr(self, '_pause_button'):
            self._pause_button.label.set_text('Play' if self.paused else 'Pause')
        if not self.paused:
            if self._pending_samples is not None:
                pending, self._pending_samples = self._pending_samples, None
                self.update(pending)
            self._last_received = -1
            self.refresh()
        else:
            self._update_status()
            self.figure.canvas.draw_idle()

    def set_tones(self, tones):
        selection = parse_tones(tones)
        if self._samples is not None and selection is not None:
            available = set(map(int, self._input_samples['i_data']))
            # A live buffer can fetch tones absent from the current snapshot.
            if self._thread is None and not set(selection) <= available:
                raise ValueError(f'Tones available: {sorted(available)}')
        old = self.tones
        old_samples = self._samples
        self.tones = selection
        try:
            if self._thread is not None and not self.paused:
                samples = self._snapshot()
                if samples is not None:
                    self._input_samples = samples
                    self._samples = (self._demodulate_samples(samples)
                                     if self.data_mode == 'demodulated' else samples)
            if self.data_mode == 'demodulated' and self._input_samples is not None:
                self._samples = self._demodulate_samples(self._input_samples)
            self._redraw()
        except Exception:
            self.tones = old
            self._samples = old_samples
            raise

    def set_panel(self, index, panel):
        panel = _panel(panel)
        old = self.panels[index]
        old_samples = self._samples
        self.panels[index] = panel
        try:
            # Snapshots hold only the displayed tones; fetch any the new panel adds.
            if self._thread is not None and not self.paused:
                samples = self._snapshot()
                if samples is not None:
                    self._input_samples = self._samples = samples
            if self.data_mode == 'demodulated' and self._input_samples is not None:
                self._samples = self._demodulate_samples(self._input_samples)
            if self._samples is not None:
                self.extractor.extract(self._samples, [panel.signal], tones=self._selection(panel))
        except Exception:
            self.panels[index] = old
            self._samples = old_samples
            raise
        self._updating = True
        try:
            self._configure_axis(index)
            self.follow_x[index] = self.auto_y[index] = True
            self._redraw()
        finally:
            self._updating = False

    def set_view_seconds(self, seconds):
        self.view_seconds = _positive(seconds, 'view_seconds')
        self.follow()

    def follow(self):
        """Resume automatic x limits (scrolling time, full spectral frequency)."""
        self.follow_x[:] = [True] * len(self.panels)
        self._redraw()

    def full_span(self):
        """Fit x to the whole displayed history, then hold those limits."""
        self._updating = True
        try:
            for i, ax in enumerate(self.axes):
                ax.relim()
                ax.set_autoscalex_on(True)
                ax.autoscale_view(scalex=True, scaley=False)
                ax.set_autoscalex_on(False)
                self.follow_x[i] = False
        finally:
            self._updating = False
        self.figure.canvas.draw_idle()

    def autoscale_y(self):
        """Resume automatic y limits; toolbar y navigation holds them again."""
        self.auto_y[:] = [True] * len(self.panels)
        self._redraw()

    def _update_status(self):
        if self.error is not None:
            state = f'Source error: {self.error}'
        elif self.paused:
            action = 'file reading continues' if self._source_kind == 'file' else 'acquisition continues'
            state = f'Paused display ({action})' if not self._finished.is_set() and self._thread else 'Paused display'
        elif self._finished.is_set():
            state = 'Source ended'
        elif self._thread is not None:
            if self._source_kind == 'file':
                state = 'Following stream file' if self.buffer.total_received else 'Waiting for complete file records'
            else:
                state = 'Live' if self.buffer.total_received else 'Waiting for stream'
        else:
            state = 'File / supplied data' if self._samples is not None else 'Waiting for data'
        details = []
        if self._samples is not None:
            n = self._samples['num_samples']
            details.append(f"{n} samples / {n / self._samples['sample_rate']:.2f} s displayed history")
        if self.buffer.total_received:
            details.append(f'{self.buffer.total_received} received, {self.buffer.missing_samples} missing')
        details.extend(n for n in (self._plot_note, self._control_error) if n)
        self._status.set_text(' | '.join([state] + details))

    def show(self, block=True):
        """Show the Matplotlib window using the active interactive backend."""
        _get_pyplot().show(block=block)
        return self

    def close(self):
        """Stop the file reader or receiver and GUI timer. Safe to call repeatedly."""
        self._closed = True
        self._timer.stop()
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=0.5)
        if self._process is not None:
            # Let the receiver close its recording cleanly before exit.
            self._process_stop.set()
            self._process.join(timeout=3)


def plot_live(client, *, filename=None, duration=None, info=None, num_tones=None,
              show=True, block=True, **kwargs):
    """Create a ``LivePlot``, receive via ``client.receive_stream``, and show it.

    Pass ``show=False`` to embed or drive refresh manually. The server should
    already be streaming (``client.enable_stream()``). ``filename`` optionally
    records the stream; otherwise acquisition uses memory only.
    """
    viewer = LivePlot(**kwargs)
    viewer.start(client, filename=filename, duration=duration, info=info, num_tones=num_tones)
    if show:
        try:
            viewer.show(block=block)
        finally:
            if block:
                viewer.close()
    return viewer
