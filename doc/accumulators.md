# Note on using second accumulator for resonator tracking

Firstly, for background, we show how to use the accumulators in the `souk_mkid_readout` package from `souk-firmware`. 

Then we discuss how to speed up accumulator readout by using a different transport layer.

Then we discuss what the `souk_readout_tools` package does to work even faster.

We show how both accumulators can be used in parallel in `souk_readout_tools` to correct for changes in resonator transmission at a slow rate while saving accumulated data at a high rate.

Finally we discuss actions required in the software to support this.


## Using souk_mkid_readout

To start with, we get a standard CASPER readout host, I call it `r`, sometimes it's called `s`. If running in a python shell on the RFSoC it is created like this:
```python
r = souk_mkid_readout.SoukMkidReadout('localhost', configfile=''/home/casper/src/souk-firmware/software/control_sw/config/souk-single-pipeline-krm.yaml',pipeline_id=0)
```

If running on a remote host with `souk_mkid_readout` installed, it will be something like:
```python
r = souk_mkid_readout.SoukMkidReadout('10.11.11.11', configfile='/path/to/souk-firmware/software/control_sw/config/souk-single-pipeline-krm.yaml',pipeline_id=0)
```
where the IP address is that of the RFSoC board.

Amongst other things, `r` contains a list of accumulators in the DSP pipeline: 

```python
print(r.accumulators)
[<souk_mkid_readout.blocks.accumulator.WindowedAccumulator object at 0xffff87c1fcd0>, <souk_mkid_readout.blocks.accumulator.WindowedAccumulator object at 0xffff87c1fdf0>]
```

We can refer to them as `a0` and `a1`:

```python
a0 = r.accumulators[0]
a1 = r.accumulators[1]
```

The accumulation length sets how many averages are applied to the upstream data before the result is available. On startup this value is 32768 averages. If the system is clocked at 4915.2MHz, the 8192-point PFB FFT will output one channel at 600kHz, which is reduced to 600kHz/32768 = 18.310546875 Hz by the accumulator. For lab testing and development we typically set the averages to 1000 which gives a data rate of 600Hz.

We can get and set the accumulation length like this:

```python
a0.get_acc_len()
32768
a0.set_acc_len(1000)
a0.get_acc_len()
1000
```

Each accumulator has a counter which increments by one every time a new accumulation is completed. It can be read out like this:

```python
a0.get_acc_cnt()
281823
```

To read out data from an accumlator we can use the `get_new_spectra` method. For example if one tone has been set at index 0, we might see somethoing like this:

```python
a0.get_new_spectra()
2025-11-25 15:32:20,545 - souk_mkid_readout.blocks.block:localhost:p0_acc0 - WARNING - Accumulation counter changed while reading data!
(array([-2544.-2091.j,     0.   +0.j,     0.   +0.j, ...,     0.   +0.j,
            0.   +0.j,     0.   +0.j]),
 None,
 None)
```

Where the returned array holds the accumulated complex values for each of the 2048 available tones. 

The other two return values are for optional GPIO register and timestamp values. These can be accessed by passing the parameters `gpio_count` (a list of GPIO indices) and get_tt (a boolean), for example:

```python
a0.get_new_spectra(gpio_count=[0,1], get_tt=True)
2025-11-25 15:37:44,995 - souk_mkid_readout.blocks.block:localhost:p0_acc0 - WARNING - Accumulation counter changed while reading data!
(array([301.+2992.j,   0.   +0.j,   0.   +0.j, ...,   0.   +0.j,
          0.   +0.j,   0.   +0.j]),
 [0, 0],
 225017715084)

```

You may have noticed the warning message about the accumulation counter changing while reading data.

This occurs because the default interface to the underlying FPGA registers is relatively slow, and if the accumulation length is short (output data rate is high) then the accumulator may update its output state while we are still reading the data.

Looking deeper into the source code, we see that the `_wait_for_acc` method is called to wait for a new accumulation to be ready, then the accumulated data is collected with the `_read_bram` method
```python
    def get_new_spectra(self, gpio_count=[], get_tt=False):
        """
        Wait for a new accumulation to be ready then read it.

        :param gpio_count: List of GPIO counter registers to return with the
            accumulator data. E.g., [0,1,3] will return counters for pulse
            edges on GPIOs 0, 1, and 3.

        :get_tt: If True, return timestamp corresponding to last sample of accumulation.
        :type get_tt: Bool

        :return: spectra_data, gpio_counts, timestamp,
            spectra_data is an array of `self.n_chans` complex-values.
            If gpio_count is not an empty list gpio_values is a list
            of the same length as gpio_count. Otherwise gpio_count is None
            If get_tt, timestamp is the accumulation timestamp. Otherwise it is None
        :rtype: numpy.ndarray[, gpio_counters]

        """
        self._wait_for_acc()
        d, timestamp  = self._read_bram(get_tt=get_tt)
        counts = None
        if gpio_count != []:
            counts = []
            for i in gpio_count:
                counts += [self.read_gpio_counter(i)]
        return d, counts, timestamp
```

`_wait_for_acc` polls the accumulation counter until it changes. The default poll_period is very long (0.1s) so we override this to a value much shorter than the accumulation time eg, 1e-4 s:

```python
    def _wait_for_acc(self, poll_period_s=0.1):
        """
        Block until a new accumulation completes, then return
        the count index.

        :param poll_period_s: The polling rate of the new accumulation counter, in seconds.
        :type poll_period_s: float

        :return: Current accumulation count
        :rtype: int
        """
        cnt0 = self.get_acc_cnt()
        cnt1 = self.get_acc_cnt()
        # Counter overflow protection
        if cnt1 < cnt0:
            cnt1 += 2**32
        while cnt1 < ((cnt0+1) % (2**32)):
            time.sleep(poll_period_s)
            cnt1 = self.get_acc_cnt()
        return cnt1

```


`_read_bram` gets the array of accumulated data from the FPGA block RAM (BRAM) and also performs the check for accumulation counter changes (`start_acc_cnt` and `stop_acc_cnt`) during the readout. There is some serial/parallel structuring to the way the data is stored in BRAM, but essentially it reads out all the data into a numpy array and returns it along with the GPIOs and timestamp if requested:

```python
    def _read_bram(self, get_tt=False):
        """ 
        Read RAM containing accumulated spectra.

        :get_tt: If True, return timestamp corresponding to last sample of accumulation.
        :type get_tt: Bool

        :return: data, timestamp tuple
            data is an array of complex valued data, in int32 format. Array
            dimensions are [FREQUENCY CHANNEL].
            timestamp is the accumulation timestamp, or None if get_tt is false.
        :rtype: numpy.array, int
        """
        dout = np.zeros(self.n_chans, dtype=complex)
        start_acc_cnt = self.get_acc_cnt()
        wordsize = np.dtype(self._dtype).itemsize
        if self._is_complex:
            wordsize *= 2
        for i in range(self._n_parallel_chans):
            ramname = f'dout{i}'
            d = np.frombuffer(self.read(ramname, self._n_serial_chans*wordsize), dtype=self._dtype)
            if self._is_complex:
                dout[i::self._n_parallel_chans].real = d[0::2]
                dout[i::self._n_parallel_chans].imag = d[1::2]
            else:
                dout[i::self._n_parallel_chans].real = d[:]
        if get_tt:
            tt = self.read_tt()
        else:
            tt = None
        stop_acc_cnt = self.get_acc_cnt()
        if start_acc_cnt != stop_acc_cnt:
            self.logger.warning('Accumulation counter changed while reading data!')
        return dout, tt

```

The `read` command in this method is defined in the transport layer of the `casperfpga` stack and we can see that this readout host uses `KatcpTransport`:

```python
print(a0.host.transport)
KatcpTransport(localhost):7147 - connected
```

```python
Signature: a0.host.transport.read(device_name, size, offset=0)
Source:   
    def read(self, device_name, size, offset=0):
        """
        Read size-bytes of binary data with carriage-return escape-sequenced.
       
        :param device_name: name of memory device from which to read
        :param size: how many bytes to read
        :param offset: start at this offset
        :return: binary data string
        """
        reply, _ = self.katcprequest(
            name='read', request_timeout=self._timeout, require_ok=True,
            request_args=(device_name, str(offset), str(size)))
        return reply.arguments[1]
File:      ~/py3.8venv/lib/python3.8/site-packages/casperfpga/transport_katcp.py
Type:      method
```

This is a message based protocol which talks to CASPER's `tcpborphserver3` server on the RFSoC over TCP/IP. `tcpborphserver3` handles the reading/writing of all of the FPGA registers/BRAMs and returns information to the host. This is useful for controlling the board (or many boards) remotely, but various protocol overheads and network latencies prevent us from rapidly reading/writing large blocks of data, as we would like to do with the accumulator BRAMs.

To get around this, we can use the more direct `LocalMemTransport` which provides direct access to the FPGA's AXI-Lite memory-mapped registers -- as long as the host code is running directly on the RFSoC's ARM CPU. This requires root privileges as it utilises the linux system's `/dev/mem` interface, and it is much faster for reading out large blocks of data.

Passing `local=True` when creating a new readout host `r_fast` in a python session started with sudo privelleges will ensure`LocalMemTransport` is used:

```python
r_fast = souk_mkid_readout.SoukMkidReadout('localhost', configfile='/home/casper/src/souk-firmware/software/control_sw/config/souk-single-pipeline-krm.yaml', pipeline_id=0, local=True)
2025-11-25 16:21:45.52 ERROR localhost transport_localmem.py:88 - Could not find device: sys_board_id
2025-11-25 16:21:45.52 ERROR localhost transport_localmem.py:88 - Could not find device: sys_board_id
2025-11-25 16:21:45,605 - souk_mkid_readout.souk_mkid_readout:localhost:0 - WARNING - Tried to get adc_clk_hz and failed

r_fast.accumulators
[<souk_mkid_readout.blocks.accumulator.WindowedAccumulator at 0xffff778c4e50>,
 <souk_mkid_readout.blocks.accumulator.WindowedAccumulator at 0xffff778c4e80>]


a0_fast = r_fast.accumulators[0]
a1_fast = r_fast.accumulators[1]

print(a0_fast.host.transport)
<casperfpga.transport_localmem.LocalMemTransport object at 0xffff7dda3b20>
```


The `read` method in this transport layer looks like this:

```python
Signature: a0_fast.host.transport.read(device_name, size, offset=0)
Source:   
    def read(self, device_name, size, offset=0):
        """
        Read size-bytes of binary data.

        :param device_name: name of memory device from which to read
        :param size: how many bytes to read
        :param offset: start at this offset, offset in bytes
        :return: binary data string
        """
        addr = self._get_device_address(device_name) + offset
        return self.axil_mm[addr : addr + size]
File:      ~/py3.8venv/lib/python3.8/site-packages/casperfpga/transport_localmem.py
Type:      method
```


And when we read out the accumulators we now see the warning about the accumulation counter on `a0_fast` but not on `a1_fast` which has the higher accumulation length:

```python
a0_fast.set_acc_len(1000)
a1_fast.set_acc_len(32768)

a0_fast.get_new_spectra()
2025-11-25 16:54:17,126 - souk_mkid_readout.blocks.block:localhost:p0_acc0 - WARNING - Accumulation counter changed while reading data!
(array([15504.+27.j,     0. +0.j,     0. +0.j, ...,     0. +0.j,
            0. +0.j,     0. +0.j]),
 None,
 None)

a1_fast.get_new_spectra()
(array([4654.-22363.j,    0.    +0.j,    0.    +0.j, ...,    0.    +0.j,
           0.    +0.j,    0.    +0.j]),
 None,
 None)
```


We can do a quick time comparison of the two transport layers like this:

```python
%timeit a0.get_new_spectra() # KatcpTransport on RFSoC through localhost
[... warning messages ...]
116 ms ± 3.53 ms per loop (mean ± std. dev. of 7 runs, 10 loops each)

%timeit a0_fast.get_new_spectra() # LocalMemTransport on RFSoC through /dev/mem
[... warning messages ...]
18.3 ms ± 4.26 µs per loop (mean ± std. dev. of 7 runs, 100 loops each)

```

Switching from KatcpTransport to LocalMemTransport has reduced the readout time from ~116 ms to ~18.3 ms (~8 per second to ~55 per second), a factor of ~6.3 speedup. 

Further optimisations are possible by avoiding some of the overheads in `souk_mkid_readout`. These have been implemented in the `souk_readout_tools` package where we achieve continuous readout at around 1000 accumulations per second. 


## Using souk_readout_tools


In `souk_mkid_readout` we talked directly to the FPGA accumulators from a
Python shell, using the CASPER software tools either over KATCP or via `LocalMemTransport`. The
`souk_readout_tools` package wraps the same hardware path in a small
client–server protocol so that

- the RFSoC ARM runs a daemon (`readout_server.py`) that reads the
    accumulators as fast as possible using a stripped back memory-mapped firmware
    interface, and
- remote clients (`readout_client.py`) pull those accumulated
    spectra either in finite chunks over the **request channel** or as a
    continuous stream over the **stream channel**.

On the firmware side the fast path still ends in the same BRAMs we saw
earlier, but the transport and framing are now optimised for moving lots
of accumulated spectra around with low overhead.

### Fast accumulator access inside the server

When the readout server starts it creates two firmware handles in
`ReadoutServer.init_server`:

```python
self.r      = firmware_lib.create_standard_readout_interface(fw_config_file)
self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file)
```

- `self.r` uses the normal CASPER transport stack and is used for control
    operations (setting tones, configuring DSP blocks, etc.).
- `self.r_fast` is configured specifically for high‑rate accumulator
    readout (direct access to the memory‑mapped BRAMs on the RFSoC).

The server never calls the slower `get_new_spectra` path. Instead it
pre‑computes a small structure describing how to read the accumulator
BRAM efficiently and reuses it for every sample:

```python
fast_read_params = firmware_lib.get_fast_read_params(self.r_fast)
cnt, data, err = firmware_lib.read_accumulated_data_fast(fast_read_params,
                                                                                                                 burst=burst)
```

- `get_fast_read_params` inspects the firmware layout once (number of
    tones, BRAM names, word widths, etc.).
- `read_accumulated_data_fast` uses those parameters to
    - wait for a new accumulation to complete,
    - perform a single, contiguous BRAM read into a NumPy array, and
    - return the accumulator counter `cnt`, the raw interleaved I/Q data
        `data` and a boolean `err` flag if the counter changed mid‑read.

This is the equivalent of calling `_wait_for_acc` and `_read_bram`
in `souk_mkid_readout`, but with far less Python and transport overhead
per accumulation.

### Server framing: headers and flags

Every time the server reads the accumulators it builds a fixed‑width
frame that contains both data and a small header. This happens in
`ReadoutServer.prepare_frame`:

```python
num_headers = 10
cnt, data, err = firmware_lib.read_accumulated_data_fast(fast_read_params,
                                                         burst=burst)

frame = np.zeros(len(data) + num_headers, dtype='<i4')
frame[:len(data)] = data           # interleaved I/Q for all tones
frame[-1]  = err                   # packet error flag
frame[-2]  = cnt                   # accumulation counter
frame[-10] = int(self.stream_flags[FLAG_SERVER_REQUEST].is_set())
...
data_bytes = frame.tobytes()
data_len   = struct.pack('>I', len(data_bytes))
payload    = data_len + data_bytes
```

The `stream_flags` entries record whether any tone updates or other
server‑side events were in progress while this spectrum was acquired
(`FLAG_SET_FREQS`, `FLAG_SET_AMPS`, `FLAG_SET_PHASES`, `FLAG_CAL_FREEZE`,
etc.). Clients can use those flags to veto samples that overlap with a
retune or calibration change.

The important point is that the accelerator read and the packetisation
are tightly coupled in one place so that **all** downstream consumers
see the same, consistent view of each accumulation.

### Chunked acquisition over the request channel (`get_samples`)

The simplest way to fetch accumulated data from the server is a
finite‑length transfer over the request socket using the
`get_samples` request. On the server side this is handled in
`ReadoutServer.handle_request_client`:

```python
elif request == 'get_samples':
        num_samples = message.get('num_samples')
        burst       = message.get('burst', False)
        task = asyncio.create_task(self.get_samples(writer, num_samples,
                                                    burst=burst))
```

The `get_samples` coroutine then performs the actual loop:

```python
async def get_samples(self, writer, num_samples, burst=False):
        fast_read_params = firmware_lib.get_fast_read_params(self.r_fast)
        for _ in range(num_samples):
                payload, cnt, err = self.prepare_frame(fast_read_params,
                                                       burst=burst)
                writer.write(payload)
                await writer.drain()
```

From the client’s point of view this looks like a simple RPC. In
`ReadoutClient.get_samples` we

- open a TCP connection to the request server,
- send a JSON message `{'request': 'get_samples', 'num_samples': N,
    'burst': False}` prefixed with a 4‑byte length, and
- read back `N` frames, each preceded by its own 4‑byte length:

```python
message = {'request': 'get_samples', 'num_samples': num_samples,
                     'burst': burst}
...
alldatalen = 2048*2*4 + 10*4    # I/Q for 2048 tones + 10 headers
data_raw  = bytearray(alldatalen * num_samples)
view      = memoryview(data_raw)

for j in range(num_samples):
        raw_datalen = s.recv(4)
        next_datalen = struct.unpack('>I', raw_datalen)[0]
        received_len = 0
        while received_len < next_datalen:
                packet_len = s.recv_into(view[packet_offset+received_len:],
                                                next_datalen - received_len)
                ...
```

Once the raw bytes are collected, the helper
`ReadoutClient.parse_samples` turns them back into per‑tone timestreams:

```python
sample_data = client.get_samples(num_samples=500)
data        = ReadoutClient.parse_samples(sample_data,
                                num_tones=len(client.get_tone_frequencies()))

z0 = data['i_data']['0000'] + 1j*data['q_data']['0000']
t  = np.arange(data['num_samples']) / data['sample_rate']
```

Internally `parse_samples` understands the same frame layout created in
`prepare_frame`:

- it views the `data_raw` buffer as `<i4`,
- splits even/odd entries into I/Q channels for each tone, and
- pulls the last ten words out as `packet_counter`, `packet_error` and
    the eight `stream_flags`.

This request‑channel path is ideal for quickly acquiring a finite block
of accumulated spectra for plotting, estimating noise spectra, or
driving a sweep/retune algorithm.

### Continuous streaming over the stream channel

For long‑duration monitoring the same accumulator frames can be sent
continuously over a separate stream socket. On the server side this is
handled by the `stream_data` task started in `ReadoutServer.async_main`:

```python
self.stream_task = asyncio.create_task(self.stream_data())
...
async def stream_data(self):
        fast_read_params = firmware_lib.get_fast_read_params(self.r_fast)
        while True:
                if self.stream_enabled.is_set():
                        payload, cnt, err = self.prepare_frame(fast_read_params)
                        for client in self.stream_clients:
                                client.write(payload)
                                await client.drain()
                else:
                        await asyncio.sleep(0.1)
```

Clients attach to this stream port using
`ReadoutClient.receive_stream`:

```python
client.enable_stream()  # RPC over request channel
client.receive_stream(num_tones=256, 
                      filename='/tmp/souk_stream') # data over stream channel
                                                   # needs ctrl-c to stop.
```

`receive_stream`:

- opens a TCP connection to the stream server,
- repeatedly reads `[length][frame]` pairs, and
- writes each frame to disk as raw `<i4` along with a small JSON
    sidecar that records the layout and system information.

Offline, `ReadoutClient.parse_stream` can reconstruct the same
dictionary structure as `parse_samples`, but now for many more samples
than you would normally request with a single `get_samples` call.

Operationally the two channels are used together:

- the **request channel** carries control RPCs (tone updates, config
    pushes, sweeps) and finite‑length `get_samples` pulls, and
- the **stream channel** carries a continuous feed of accumulator
    frames to any number of listeners.

Both ultimately rely on the same fast accumulator read path and frame
layout implemented in `ReadoutServer.prepare_frame`.

### Tone control in souk_readout_tools

Tone configuration has two complementary paths in `souk_readout_tools`:

1. **External control via the request channel** for general setup and
     user‑driven changes.
2. **Internal control from server tasks** (no client connection
     required) for automatic tracking and retuning.

For external control, all tone settings still flow through the server’s
`handle_request_client` logic using simple `get`/`set` parameters that
map directly onto `firmware_lib` helpers:

- `set_tone_frequencies`, `set_tone_amplitudes`, `set_tone_phases`,
    `set_tone_powers` in `ReadoutClient` call the generic `set` RPC.
- `ReadoutServer.handle_request_client` receives those `set` requests
    and forwards them to `firmware_lib.set_tone_*`, while also raising the
    appropriate `stream_flags` so that the headers in each accumulator
    frame record when frequencies or amplitudes are being changed.

For internal control the server can also call the same `firmware_lib`
functions directly from its own coroutines, without any client being
connected. The sweep and retune paths in `readout_server.py` are
the current examples of this:

- `ReadoutServer.sweep` builds a grid of tone frequencies, uses
    `prepare_sweep_settings_fast` / `apply_sweep_step_fast` (fast analogs
    of the accumulator helpers) to step through them quickly, and calls
    `read_accumulated_data_fast` at each point to build up a
    frequency–response data cube.
- `ReadoutServer.retune` does the same but also analyses the accumulated 
   sweeps and calls `firmware_lib.set_tone_frequencies` to move the tones 
   onto the resonance peaks, again entirely inside the daemon.

In a tracking loop (for example a `tracking_task` coroutine
running alongside `stream_data`) we might expect the server to

- keep a **fast accumulator**, accumulator 0, streaming continuously at high rate to
    clients over the stream channel, independent of any particular 
    control connection,
- use the **second accumulator** (or suitably averaged/filtered views of the
    same fast data) to monitor slow, high‑S/N variations in each
    resonator’s transmission and phase, and
- adjust the resonator tones from inside the daemon using the same
    `set_tone_frequencies` / `prepare_tone_frequencies_fast` /
    `apply_tone_frequencies_fast` machinery that is already used for
    sweeps and retunes, without requiring any external RPCs while the
    loop is running.



## Actions required in the software to support dual accumulators

To fully support dual accumulators in `souk_readout_tools`, the following
actions are required:

- Identify and list all current single‑accumulator assumptions in firmware_lib.py, server/readout_server.py, and client/readout_client.py (e.g. hard‑coded acc[0], fixed frame length, single fast_read_params usage).

- Define a configuration scheme in the YAML (data/config/*.yaml) and ReadoutServer.init_server to distinguish fast_accumulator_index and slow_accumulator_index, plus suggested default acc_len values for each.

- Extend the fast‑read helpers in firmware_lib so get_fast_read_params and read_accumulated_data_fast can be instantiated for an arbitrary accumulator index, and plan to hold two parameter sets inside ReadoutServer (one per accumulator).
Decide on the tracking‑loop data architecture: either (A) keep stream frames as they are and let the tracking task read accumulator 1 “side‑band” (no client API change), or (B) design a new frame layout that carries both accumulators’ data plus an “frame_version/acc_id” header field, and outline matching updates to ReadoutClient.parse_*.

- Design the tracking task within ReadoutServer (e.g. tracking_task coroutine) that periodically reads the slow accumulator via its fast_read_params, estimates per‑tone drifts (phase/amplitude or frequency), and calls existing firmware_lib tone update helpers (set_tone_frequencies / prepare_*_fast / apply_*_fast) to adjust tones, while using stream_flags to mark frames during which retuning is in progress.

- Define how clients will configure and monitor tracking: add request‑channel RPCs (e.g. enable_tracking, disable_tracking, get_tracking_status, get_tracking_config) in ReadoutServer.handle_request_client and ReadoutClient, including what status fields to expose (on/off, last update time, mean drift, slow accumulator acc_len, etc.).


## Notes following meeting with Sam and Jamie on 2025-12-08

We considered the approach to dual-accumulator support in souk_readout_tools.

We are looking at adding a prepare_slow_frame method that:
- reads the slow accumulator
- does some optional filtering/averaging
- decides if tone updates are needed
- applies tone updates if needed
- transmits updated frequencies to clients over the stream channel with a new FLAG_TRACKING_UPDATE flag set.
- the tracking update flag indicates the data represents tone frequencies and not raw accumulator data.

This would be called from within a tracking_task coroutine which runs alongside stream_data.

We may want to perform some profiling to optimise the speed of the slow accumulator read and tone update path.


