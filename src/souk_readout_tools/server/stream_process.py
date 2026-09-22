"""The stream loop in its own process (opt-in, for testing).

Turned on by ``firmware.stream_process: true`` in the server config, or
``SOUK_STREAM_PROCESS=1`` in the environment (which wins). The readout server
then keeps everything except the frame loop -- requests, sweeps,
``get_samples``, tracking estimation and commits, info -- and starts a second
process that owns the stream port and does only this, per accumulation::

    wait -> read -> build the frame -> send it -> tracking tap

In one process, every request, tracking window and health snapshot shares the
GIL and the event loop with the frame loop, and anything that holds either for
more than ~250 us at 2500 Hz costs accumulations (doc/v712-notes.md, "The
per-iteration deadline was self-imposed"). Here nothing on the request side can.

What crosses between the two processes:

- **A small shared block** (``/dev/shm/souk_stream_p<pipeline>``): the stream
  mode (idle / plain / fw), the eight stream flags, the fw revision and
  ``n_settle``, a tone-set generation number, the pause handshake and the
  loop-timing ring. The frame loop reads the words it needs every frame, so a
  flag the server sets is in the very next frame, as before.
- **A pipe** for anything ordered or larger: new tone indices, typed frames
  (TONE_UPDATE, SNAPSHOT), tracking start / stop / window swaps, the stream's
  sparse-read override. The frame loop reads it whenever it yields.

While a new tone set is on its way through the pipe, frames carry
FLAG_SET_FREQS -- the generation in the shared block runs ahead of the one the
loop has applied -- so no frame is read with stale indices unflagged.

The frame and client code is the server's own (``prepare_frame``,
``handle_stream_client`` and friends are borrowed from ``ReadoutServer``), so
the two modes cannot build or send different frames.

Before anything that can reprogram the FPGA or re-initialise its clocks, the
server pauses this process and waits for it to confirm it has stopped touching
the fabric (a read of unclocked fabric hangs the board). It re-opens its own
fast interface when the server's is healthy again.

Not supported with the stream process (the server refuses them): the sw
modulation engine, which steps hardware from the frame loop, and triggered
streams. ``get_info('loop_timing')`` reports the stream process's frames.

CPUs: the stream process pins itself to ``SOUK_STREAM_CPUS`` and the server
moves itself to ``SOUK_WORKER_CPUS`` (on the KRM, ``souk-enable-tuning`` sets
both, and ``SOUK_STREAM_RT_PRIO``). Without them the two share a core, and the
kernel preempts the frame loop whenever the server works.
"""

import asyncio
import json
import mmap
import multiprocessing
import os
import time
import traceback

import numpy as np

from souk_readout_tools.server import readout_server as rs
from souk_readout_tools.server import tracking

# Control words (int64) at the start of the shared block.
MODE, FLAGS, FW_REV, FW_N_SETTLE, TONE_GEN, PAUSE_REQ, PAUSE_ACK, LT_N, READY, STARTED, \
    TRACK_REQ, TRACK_ACK, LOSS_N = range(13)
CTRL_WORDS = 16
# A finished tracking window's description, written by the stream process when
# it swaps buffers (float64 words): seq, mode (0 boxcar / 1 ewma / -1 empty),
# boxcar buffer, revision (-1 = none), frames tapped, frames skipped, blocks,
# points, then blocks*points counts and, for an ewma, as many weights.
TRACK_MAX_BN = 256
TRACK_INFO_WORDS = 8 + 2 * TRACK_MAX_BN
MODE_IDLE, MODE_PLAIN, MODE_FW = 0, 1, 2
MODE_NAMES = {MODE_IDLE: 'idle', MODE_PLAIN: 'plain', MODE_FW: 'fw'}

PAUSE_TIMEOUT_S = 2.0      # the loop checks for a pause every frame, or 10 ms idle
# Before its frame loop first runs the process is still importing (seconds on
# the board) -- not touching the fabric, but not answering either.
PAUSE_TIMEOUT_STARTING_S = 60.0


def wanted(config):
    """Whether this server should run its stream loop in a separate process."""
    env = os.environ.get('SOUK_STREAM_PROCESS', '').strip().lower()
    if env:
        return env not in ('0', 'false', 'no', 'off')
    return bool((config or {}).get('firmware', {}).get('stream_process', False))


class SharedBlock:
    """A file in /dev/shm, mapped by both processes.

    ``nbytes`` creates it (zero-filled, replacing any stale one); without it an
    existing file is opened.
    """

    def __init__(self, path, nbytes=None):
        flags = os.O_RDWR | (os.O_CREAT | os.O_TRUNC if nbytes is not None else 0)
        fd = os.open(path, flags, 0o600)
        try:
            if nbytes is not None:
                os.ftruncate(fd, nbytes)
            self.mm = mmap.mmap(fd, os.fstat(fd).st_size)
        finally:
            os.close(fd)
        self.path = path

    def control(self):
        """The control words, as a memoryview of int64 (Python-int access)."""
        return memoryview(self.mm)[:8 * CTRL_WORDS].cast('q')

    def track_info(self):
        """The finished tracking window's description (see TRACK_INFO_WORDS)."""
        return np.ndarray((TRACK_INFO_WORDS,), dtype=np.float64,
                          buffer=self.mm, offset=8 * CTRL_WORDS)

    def loop_timing(self):
        """The loop-timing ring (same layout as ``ReadoutServer.loop_timing``)."""
        return np.ndarray((rs.LOOP_TIMING_FRAMES, 6), dtype=np.float64,
                          buffer=self.mm, offset=8 * (CTRL_WORDS + TRACK_INFO_WORDS))

    def loss_records(self):
        """The loss records (same layout as ``ReadoutServer.loss_records``)."""
        return np.ndarray((rs.LOSS_RECORDS, 2 + 6 * rs.LOSS_RECORD_FRAMES), dtype=np.float64,
                          buffer=self.mm, offset=8 * (CTRL_WORDS + TRACK_INFO_WORDS
                                                      + rs.LOOP_TIMING_FRAMES * 6))

    def unlink(self):
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass


# ---------------------------------------------------------------------------
# The server's side
# ---------------------------------------------------------------------------

def _same_tones(a, b):
    if a is None or b is None:
        return a is None and b is None
    return np.array_equal(a, b)


class StreamProcess:
    """The readout server's handle on its stream process.

    ``settings``: ``fw_config_file``, ``pipeline_id``, ``address``, ``port``,
    ``zdma``, ``zdma_udmabuf``, ``zdma_channel``, and ``fake`` (a dict for the
    off-board stand-in accumulator, tests only).
    """

    def __init__(self, settings):
        self.settings = dict(settings)
        self.path = f"/dev/shm/souk_stream_p{int(settings['pipeline_id'])}"
        self.block = SharedBlock(self.path, 8 * (CTRL_WORDS + TRACK_INFO_WORDS
                                                 + rs.LOOP_TIMING_FRAMES * 6
                                                 + rs.LOSS_RECORDS * (2 + 6 * rs.LOSS_RECORD_FRAMES)))
        self.ctrl = self.block.control()
        self.track_info = self.block.track_info()
        self.loop_timing = self.block.loop_timing()
        self.loss_records = self.block.loss_records()
        # The pipe exists before the process does, so state published before
        # start() is waiting for it when it begins.
        self._ctx = multiprocessing.get_context('spawn')
        self.conn, self._child_conn = self._ctx.Pipe()
        self.proc = None
        self.tone_gen = 0                  # 0 = no tone set sent yet
        self._last_tones = None
        self._pause_id = 0
        self._next_id = 0
        self._pending = {}                 # request id -> future
        self.snapshot_frame = None         # callable -> SNAPSHOT typed-frame bytes

    def start(self, paused):
        """Spawn the process; ``paused`` while the fabric is not known good."""
        if paused:
            self._pause_id += 1
            self.ctrl[PAUSE_REQ] = self._pause_id
        self.proc = self._ctx.Process(
            target=main, args=(self._child_conn, self.path, self.settings),
            name=f"souk-stream-p{self.settings['pipeline_id']}", daemon=True)
        self.proc.start()
        self._child_conn.close()
        loop = asyncio.get_running_loop()
        loop.add_reader(self.conn.fileno(), self._on_pipe)
        loop.add_reader(self.proc.sentinel, self._on_exit)
        print(f'stream process started (pid {self.proc.pid}'
              f"{', paused' if paused else ''})")

    def stop(self):
        if self.alive():
            self._send(('stop',))
        if self.proc is not None:
            self.proc.join(timeout=2.0)
            if self.proc.is_alive():
                self.proc.terminate()
        self.block.unlink()

    def alive(self):
        return self.proc is not None and self.proc.is_alive()

    def frames_recorded(self):
        """Rows written to the loop-timing ring since the stream started
        (updated whenever the frame loop yields)."""
        return self.ctrl[LT_N]

    def losses_recorded(self):
        """Loss records written since the stream started (updated with LT_N)."""
        return self.ctrl[LOSS_N]

    def status(self):
        c = self.ctrl
        return {'pid': self.proc.pid if self.proc else None,
                'alive': self.alive(),
                'ready': bool(c[READY]),
                'paused': bool(c[PAUSE_REQ]),
                'mode': MODE_NAMES.get(c[MODE], c[MODE])}

    # -- state the frame loop reads every frame --------------------------------

    def publish(self, streaming, fw, flags, fw_rev, fw_n_settle):
        """The stream mode, the flag bits and the fw revision, as of now."""
        c = self.ctrl
        c[FLAGS] = flags
        c[FW_REV] = fw_rev
        c[FW_N_SETTLE] = fw_n_settle
        c[MODE] = MODE_IDLE if not streaming else (MODE_FW if fw else MODE_PLAIN)

    def publish_tones(self, indices):
        """A new active tone set: flag frames at once, send the indices.

        The same set again (a tracking LO commit re-states it) is not a
        change, and must not flag frames.
        """
        indices = None if indices is None else np.asarray(indices)
        if self.tone_gen and _same_tones(indices, self._last_tones):
            return
        self._last_tones = indices
        self.tone_gen += 1
        self.ctrl[TONE_GEN] = self.tone_gen
        self._send(('tones', self.tone_gen, indices))

    # -- pausing around FPGA programming --------------------------------------

    def pause(self):
        """Return once the process has stopped touching the fabric.

        Raises if it does not confirm in ``PAUSE_TIMEOUT_S``: better to refuse
        the reprogram than to program under a live reader.
        """
        if not self.alive():
            return
        self._pause_id += 1
        self.ctrl[PAUSE_REQ] = self._pause_id
        t0 = time.monotonic()
        while self.ctrl[PAUSE_ACK] != self._pause_id:
            if not self.alive():
                return
            limit = PAUSE_TIMEOUT_S if self.ctrl[STARTED] else PAUSE_TIMEOUT_STARTING_S
            if time.monotonic() - t0 > limit:
                raise RuntimeError('stream process did not pause; not touching the FPGA')
            time.sleep(0.001)

    def resume(self):
        """Let it re-open its interface (the server's is healthy again)."""
        self.ctrl[PAUSE_REQ] = 0

    # -- pipe ------------------------------------------------------------------

    def _send(self, msg):
        try:
            self.conn.send(msg)
        except (BrokenPipeError, OSError) as e:
            print(f'stream process pipe: {e}')

    def send(self, kind, *args):
        self._send((kind,) + args)

    def send_typed(self, frame_bytes, client_id=None):
        """A typed frame for every subscribed client, or for one."""
        self._send(('typed', client_id, frame_bytes))

    async def request(self, kind, *args, timeout=2.0):
        self._next_id += 1
        req = self._next_id
        fut = asyncio.get_running_loop().create_future()
        self._pending[req] = fut
        self._send((kind, req) + args)
        try:
            return await asyncio.wait_for(fut, timeout)
        finally:
            self._pending.pop(req, None)

    def _on_pipe(self):
        try:
            while self.conn.poll():
                msg = self.conn.recv()
                if msg[0] == 'reply':
                    fut = self._pending.pop(msg[1], None)
                    if fut is not None and not fut.done():
                        fut.set_result(msg[2])
                elif msg[0] == 'subscribe':
                    if self.snapshot_frame is not None:
                        self.send_typed(self.snapshot_frame(), msg[1])
        except (EOFError, OSError):
            asyncio.get_running_loop().remove_reader(self.conn.fileno())
        except Exception as e:
            print(f'stream process message: {e}')
            print(traceback.format_exc())

    def _on_exit(self):
        asyncio.get_running_loop().remove_reader(self.proc.sentinel)
        print(f'STREAM PROCESS EXITED (code {self.proc.exitcode}); no frames '
              'will be streamed until the server restarts')

    # -- tracking ----------------------------------------------------------------

    def start_tracking(self, spec):
        return RemoteIQAccumulator(self, spec)


class RemoteIQAccumulator:
    """The server's view of an ``IQAccumulator`` running in the stream process.

    The sums live in a shared file; the stream process taps into them and, on
    request, hands over a window by saying which buffer and what counts. The
    server then reads the sums in place and, for a boxcar, zeroes that buffer
    (``TrackingRuntime.window_iq``) before asking for the next one.
    """

    def __init__(self, proc, spec):
        if int(spec['blocks']) * int(spec['num_points']) > TRACK_MAX_BN:
            raise ValueError(f'with the stream process, blocks x points must be '
                             f'<= {TRACK_MAX_BN}')
        self.proc = proc
        self.spec = dict(spec)
        shape = (int(spec['blocks']), int(spec['num_points']), 2 * int(spec['n_tones']))
        self.block = SharedBlock(proc.path + '_tracking', tracking.IQAccumulator.nbytes(
            spec['blocks'], spec['num_points'], spec['n_tones'], spec['iq_filter']))
        if spec['iq_filter'] == 'boxcar':
            self.sets = np.ndarray((2,) + shape, dtype=np.int64, buffer=self.block.mm)
            self.sums = None
        else:
            self.sets = None
            self.sums = np.ndarray(shape, dtype=np.float64, buffer=self.block.mm)
        self.enabled = True
        self.frames_tapped = self.frames_skipped = 0
        proc.send('track_start', self.spec, self.block.path)

    def tap(self, data, point, settling=False, revision=0):
        """No-op: the continuous stream is tapped in the stream process (a
        ``get_samples`` capture in the server does not feed tracking)."""

    async def next_window(self):
        """Ask the stream process to hand over a window, through the shared
        block rather than the pipe: it swaps buffers right after a frame, writes
        the counts next to the control words and acknowledges, so the frame
        loop makes no system call and unpickles nothing for it."""
        c, info = self.proc.ctrl, self.proc.track_info
        req = c[TRACK_REQ] + 1
        c[TRACK_REQ] = req
        t0 = time.monotonic()
        while c[TRACK_ACK] != req or int(info[0]) != req:
            if time.monotonic() - t0 > 2.0 or not self.proc.alive():
                return None
            await asyncio.sleep(0.002)
        mode = int(info[1])
        if mode < 0:
            return None
        b, n = int(info[6]), int(info[7])
        self.frames_tapped, self.frames_skipped = int(info[4]), int(info[5])
        counts = info[8:8 + b * n].astype(np.int64).reshape(b, n)
        revision = None if info[3] < 0 else int(info[3])
        if mode == 0:
            return ('boxcar', self.sets[int(info[2])], counts, revision)
        weights = info[8 + TRACK_MAX_BN:8 + TRACK_MAX_BN + b * n].reshape(b, n).copy()
        return ('ewma', self.sums.copy(), weights, counts, revision)

    def take_window(self):
        raise RuntimeError('a remote accumulator is taken with next_window()')

    def reset(self, revision=None):
        self.proc.send('track_reset', revision)

    def stop(self):
        self.enabled = False
        self.proc.send('track_stop')
        self.block.unlink()


# ---------------------------------------------------------------------------
# The stream process
# ---------------------------------------------------------------------------

class _SharedFlag:
    """A stream flag the server sets, read from the shared block."""

    def __init__(self, ctrl, bit):
        self.ctrl = ctrl
        self.bit = bit

    def is_set(self):
        return bool((self.ctrl[FLAGS] >> self.bit) & 1)


class _SetFreqsFlag(_SharedFlag):
    """FLAG_SET_FREQS, also raised while a new tone set is in the pipe."""

    def __init__(self, ctrl, bit, streamer):
        super().__init__(ctrl, bit)
        self.streamer = streamer

    def is_set(self):
        return (bool((self.ctrl[FLAGS] >> self.bit) & 1)
                or self.ctrl[TONE_GEN] != self.streamer.tone_gen)


class Streamer:
    """The frame loop and the stream clients, in the stream process."""

    # The server's own frame and client code, so both modes build and send the
    # same bytes. They only use the attributes set up in __init__.
    stream_process = None      # (they check it; this *is* the stream process)
    prepare_frame = rs.ReadoutServer.prepare_frame
    handle_stream_client = rs.ReadoutServer.handle_stream_client
    stream_keepalive = rs.ReadoutServer.stream_keepalive
    _size_send_buffer = rs.ReadoutServer._size_send_buffer
    _set_streaming = rs.ReadoutServer._set_streaming
    _write_to_stream_clients = rs.ReadoutServer._write_to_stream_clients
    _yield_due = rs.ReadoutServer._yield_due
    _record_loss = rs.ReadoutServer._record_loss
    _drain_stream_clients = rs.ReadoutServer._drain_stream_clients
    _write_typed_update_frame = rs.ReadoutServer._write_typed_update_frame

    def __init__(self, conn, block_path, settings):
        self.conn = conn
        self.settings = settings
        self.block = SharedBlock(block_path)
        self.ctrl = self.block.control()
        self.track_info = self.block.track_info()
        self.loop_timing = self.block.loop_timing()
        self.loss_records = self.block.loss_records()
        self.stream_flags = [_SharedFlag(self.ctrl, i) for i in range(8)]
        self.stream_flags[rs.FLAG_SET_FREQS] = _SetFreqsFlag(self.ctrl, rs.FLAG_SET_FREQS, self)
        self.stream_clients = []
        self._clients_by_id = {}
        self.e_stream_enabled = asyncio.Event()
        self.e_stream_stopped = asyncio.Event()
        self.e_stream_stopped.set()
        self._stopped = asyncio.Event()
        self.active_tone_indices = None
        self.tone_gen = 0
        self.tracking = None
        self._track_block = None
        self.sparse_read = None
        self.r_fast = None
        self.fp = None                     # fast read params; None while paused
        self._mode = MODE_IDLE
        self._frame_cache = None
        self._mod_dwell_tag = -1
        self._mod_dwell_pos = 0
        self._loop_timing_n = 0
        self._loop_timing_last = None
        self._loss_records_n = 0
        self._frames_since_yield = 0
        self._last_yield = 0.0

    async def run(self):
        loop = asyncio.get_running_loop()
        loop.add_reader(self.conn.fileno(), self._on_pipe)
        server = await asyncio.start_server(self.handle_stream_client,
                                            self.settings['address'],
                                            self.settings['port'])
        print(f"stream process listening on {self.settings['address']}:"
              f"{self.settings['port']}")
        frames = asyncio.create_task(self._frames())
        watch = asyncio.create_task(self._pause_watch())
        await self._stopped.wait()
        frames.cancel()
        watch.cancel()
        server.close()

    async def _frames(self):
        c = self.ctrl
        rev_seen = None
        c[STARTED] = 1                     # the pause request is checked from here on
        while True:
            try:
                if c[PAUSE_REQ]:
                    await self._paused()
                    continue
                if self.fp is None:
                    self._open_interface()
                mode = c[MODE]
                if mode != self._mode:
                    self._change_mode(mode)
                if mode == MODE_IDLE:
                    await asyncio.sleep(0.01)
                    continue
                if mode == MODE_FW:
                    rev = c[FW_REV]
                    if rev != rev_seen:
                        # A new arm or update: restart the settling edge tracker.
                        self._mod_dwell_tag, self._mod_dwell_pos, rev_seen = -1, 0, rev
                    payload, cnt, err = self.prepare_frame(
                        self.fp, fw_modulation=True, revision=rev, fw_n_settle=c[FW_N_SETTLE])
                else:
                    payload, cnt, err = self.prepare_frame(self.fp)
                self._write_to_stream_clients(payload)
                if c[TRACK_REQ] != c[TRACK_ACK]:
                    self._swap_tracking_window()
                if self._yield_due(len(payload)):
                    c[LT_N], c[LOSS_N] = self._loop_timing_n, self._loss_records_n
                    await self._drain_stream_clients()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                print(f'Error streaming data: {e}')
                print(traceback.format_exc())
                await asyncio.sleep(0.1)

    def _change_mode(self, mode):
        was, now = self._mode != MODE_IDLE, mode != MODE_IDLE
        if now and not was:
            self._loop_timing_n, self._loop_timing_last, self._loss_records_n = 0, None, 0
            self._set_streaming(True)
        elif was and not now:
            self._set_streaming(False)
        self.ctrl[LT_N], self.ctrl[LOSS_N] = self._loop_timing_n, self._loss_records_n
        self._mode = mode

    def _open_interface(self):
        s = self.settings
        if s.get('fake'):
            self.fp = {}
        else:
            fl = rs.firmware_lib
            fl.ZDMA_ENABLED = bool(s.get('zdma'))
            fl.ZDMA_UDMABUF = s.get('zdma_udmabuf', 'udmabuf0')
            fl.ZDMA_CHANNEL = int(s.get('zdma_channel', 0))
            self.r_fast = fl.create_fast_readout_interface(s['fw_config_file'], s['pipeline_id'])
            self.fp = fl.get_fast_read_params(self.r_fast)
        self.fp['sparse_read'] = self.sparse_read
        self.fp['catch_up'] = True         # read late rather than lose it (acc_due_s)
        self.ctrl[READY] = 1

    def _swap_tracking_window(self):
        """Hand the server a tracking window through the shared block (see
        RemoteIQAccumulator.next_window). Done right after a frame is sent."""
        c, t = self.ctrl, self.track_info
        req = c[TRACK_REQ]
        info = self.tracking.take_window_info() if self.tracking is not None else None
        if info is None:
            t[1] = -1
        else:
            counts = info['counts']
            b, n = len(counts), len(counts[0])
            t[1] = 0 if info['mode'] == 'boxcar' else 1
            t[2] = info.get('set', 0)
            t[3] = -1 if info['revision'] is None else info['revision']
            t[4], t[5], t[6], t[7] = info['frames_tapped'], info['frames_skipped'], b, n
            t[8:8 + b * n] = [x for row in counts for x in row]
            if 'weights' in info:
                t[8 + TRACK_MAX_BN:8 + TRACK_MAX_BN + b * n] = [x for row in info['weights'] for x in row]
        t[0] = req                         # the description is complete ...
        c[TRACK_ACK] = req                 # ... and then it is acknowledged

    async def _pause_watch(self):
        """Acknowledge a pause even while the frame loop is held up in a drain.

        Safe because every fabric access is in synchronous code: whenever this
        runs, the frame loop is parked at an await, between reads, and it checks
        the pause request before its next read. ``fp = None`` makes it re-open
        the interface only after the pause has been lifted.
        """
        c = self.ctrl
        while True:
            await asyncio.sleep(0.05)
            if c[PAUSE_REQ] and c[PAUSE_ACK] != c[PAUSE_REQ]:
                self.fp = self.r_fast = None
                c[READY] = 0
                c[PAUSE_ACK] = c[PAUSE_REQ]

    async def _paused(self):
        """Stop touching the fabric until the server says it is good again."""
        c = self.ctrl
        self.fp = self.r_fast = None
        c[READY] = 0
        if self._mode != MODE_IDLE:
            self._change_mode(MODE_IDLE)
        while c[PAUSE_REQ]:
            c[PAUSE_ACK] = c[PAUSE_REQ]
            await asyncio.sleep(0.005)

    # -- the pipe --------------------------------------------------------------

    def _on_pipe(self):
        # One message per call: the event loop only calls this when the pipe is
        # readable, and calls it again while more is waiting. Connection.poll()
        # builds a selector and makes several system calls each time -- enough
        # on this ARM, twice per message, to push a frame past its deadline.
        try:
            self._handle(self.conn.recv())
        except (EOFError, OSError):
            asyncio.get_running_loop().remove_reader(self.conn.fileno())
            self._stopped.set()            # the server has gone
        except Exception as e:
            print(f'stream process message: {e}')
            print(traceback.format_exc())

    def _handle(self, msg):
        kind = msg[0]
        if kind == 'tones':
            self.active_tone_indices, self.tone_gen = msg[2], msg[1]
        elif kind == 'typed':
            _, client_id, frame = msg
            if client_id is None:
                self._write_typed_update_frame(frame)
            else:
                writer = self._clients_by_id.get(client_id)
                if writer in self.stream_clients:
                    writer.write(frame)
        elif kind == 'sparse_read':
            self.sparse_read = msg[1]
            if self.fp is not None:
                self.fp['sparse_read'] = msg[1]
        elif kind == 'track_start':
            _, spec, path = msg
            self._track_block = SharedBlock(path)
            self.tracking = tracking.IQAccumulator(**spec, buffer=self._track_block.mm)
        elif kind == 'track_stop':
            self.tracking = self._track_block = None
        elif kind == 'track_reset':
            if self.tracking is not None:
                self.tracking.reset(msg[1])
        elif kind == 'reset_loop_timing':
            self._loop_timing_n, self._loop_timing_last, self._loss_records_n = 0, None, 0
            self.ctrl[LT_N] = self.ctrl[LOSS_N] = 0
        elif kind == 'stop':
            self._stopped.set()

    def _handle_stream_subscribe(self, writer, line):
        """A client's subscribe line: mark it, and ask the server for the
        SNAPSHOT it gets first (only the server knows the modulation state)."""
        try:
            wants = json.loads(line.decode()).get('subscribe')
        except Exception:
            print(f"stream client {getattr(writer, 'addr', '?')} sent an "
                  "unparseable line; ignoring")
            return
        subscribed = 'tone_updates' in wants if isinstance(wants, list) else bool(wants)
        if subscribed and not writer.typed_updates:
            writer.typed_updates = True
            self._clients_by_id[id(writer)] = writer
            self.conn.send(('subscribe', id(writer)))


class _FakeAccumulator:
    """Off-board stand-in for the accumulator, for tests (settings['fake']).

    Accumulations arrive at ``rate`` Hz on this process's clock and are read
    late rather than skipped, like the real catch-up wait. Each frame's I
    words hold the count and its Q words the tone index; the LO slot steps
    every ``dwell`` accumulations through ``n_slots``.
    """

    def __init__(self, rate=1000.0, n_slots=4, dwell=1, n_default=4):
        self.period = 1.0 / float(rate)
        self.n_slots, self.dwell, self.n_default = int(n_slots), int(dwell), int(n_default)
        self.t0 = time.perf_counter()

    def read(self, fp, num_tones=None, tone_indices=None, get_buf_id=False,
             sparse_read=None):
        cnt = int((time.perf_counter() - self.t0) / self.period)
        if fp.get('acc_last_read') == cnt:
            cnt += 1
            time.sleep(max(0.0, self.t0 + cnt * self.period - time.perf_counter()))
        fp['acc_last_read'] = cnt
        fp['t_acc_seen'] = time.perf_counter()
        ti = (np.arange(self.n_default) if tone_indices is None
              else np.asarray(tone_indices))
        data = np.empty(2 * len(ti), dtype='<i4')
        data[0::2] = cnt & 0x7FFFFFFF
        data[1::2] = ti
        out = (cnt, data, False, cnt)
        return out + (0, (cnt // self.dwell) % self.n_slots) if get_buf_id else out


def main(conn, block_path, settings):
    """Entry point of the stream process (``multiprocessing`` spawn)."""
    cpus = set(settings.get('cpus') or ()) or rs._cpus_from_env('SOUK_STREAM_CPUS')
    if settings.get('fake'):
        fake = _FakeAccumulator(**settings['fake'])
        rs.firmware_lib = type('fake_firmware_lib', (), {
            'read_accumulated_data_fast': staticmethod(fake.read)})
    else:
        from souk_readout_tools import firmware_lib
        rs.firmware_lib = firmware_lib      # the borrowed frame code looks it up there
    if cpus:
        for tid in os.listdir('/proc/self/task'):
            try:
                os.sched_setaffinity(int(tid), cpus)
            except OSError:
                pass
    # Opt-in real-time priority. The kernel here is PREEMPT_NONE and its SLAB
    # allocator reaps each CPU's caches every 2 s (cache_reap, ~100-300 us on a
    # stream core); a SCHED_OTHER frame loop waking meanwhile waits for it to
    # finish, and loses or tears an accumulation. A SCHED_FIFO one gets the core
    # back at the reaper's next cond_resched(). Safe only because the wait
    # sleeps between accumulations (firmware_lib ACC_LATE_WAKE_S): a spinning
    # real-time task is throttled by the kernel, 50 ms in every second.
    prio = int(os.environ.get('SOUK_STREAM_RT_PRIO', '0') or 0)
    if prio > 0:
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(prio))
            print(f'stream process: SCHED_FIFO priority {prio}')
        except OSError as e:
            print(f'stream process: could not set SCHED_FIFO {prio}: {e}')
    streamer = Streamer(conn, block_path, settings)
    try:
        asyncio.run(streamer.run())
    finally:
        streamer.ctrl[READY] = 0
