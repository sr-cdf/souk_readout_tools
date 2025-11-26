# note on using second accumulator for resonator tracking

Firstly we show how to use the accumulators in the `souk_mkid_readout` package from `souk-firmware`. 

Then we discuss how to speed up accumulator readout by using a different transport layer.

Then we discuss what the `souk_readout_tools` package does to work even faster.

Finally we show how both accumulators can be used in parallel in `souk_readout_tools` to correct for changes in resonator transmission at a slow rate while saving accumulated data at a high rate.


## Using souk_mkid_readout

To start with, we get a standard CASPER readout host, I call it `r`, sometimes it's called `s`. If running in a python shell on the RFSoC it is created like this:
```
r = souk_mkid_readout.SoukMkidReadout('localhost', configfile=''/home/casper/src/souk-firmware/software/control_sw/config/souk-single-pipeline-krm.yaml',pipeline_id=0)
```

If running on a remote host with `souk_mkid_readout` installed, it will be something like:
```
r = souk_mkid_readout.SoukMkidReadout('10.11.11.11', configfile='/path/to/souk-firmware/software/control_sw/config/souk-single-pipeline-krm.yaml',pipeline_id=0)
```
where the IP address is that of the RFSoC board.

Amongst other things, `r` contains a list of accumulators in the DSP pipeline: 

```
print(r.accumulators)
[<souk_mkid_readout.blocks.accumulator.WindowedAccumulator object at 0xffff87c1fcd0>, <souk_mkid_readout.blocks.accumulator.WindowedAccumulator object at 0xffff87c1fdf0>]
```

We can refer to them as `a0` and `a1`:

```
a0 = r.accumulators[0]
a1 = r.accumulators[1]
```

The accumulation length sets how many averages are applied to the upstream data before the result is available. On startup this value is 32768 averages. If the system is clocked at 4915.2MHz, the 8192-point PFB FFT will output one channel at 600kHz, which is reduced to 600kHz/32768 = 18.310546875 Hz by the accumulator. For lab testing and development we typically set the averages to 1000 which gives a data rate of 600Hz.

We can get and set the accumulation length like this:

```
a0.get_acc_len()
32768
a0.set_acc_len(1000)
a0.get_acc_len()
1000
```

Each accumulator has a counter which increments by one every time a new accumulation is completed. It can be read out like this:

```
a0.get_acc_cnt()
281823
```

To read out data from an accumlator we can use the `get_new_spectra` method. For example if one tone has been set at index 0, we might see somethoing like this:

```
a0.get_new_spectra()
2025-11-25 15:32:20,545 - souk_mkid_readout.blocks.block:localhost:p0_acc0 - WARNING - Accumulation counter changed while reading data!
(array([-2544.-2091.j,     0.   +0.j,     0.   +0.j, ...,     0.   +0.j,
            0.   +0.j,     0.   +0.j]),
 None,
 None)
```

Where the returned array holds the accumulated complex values for each of the 2048 available tones. 

The other two return values are for optional GPIO register and timestamp values. These can be accessed by passing the parameters `gpio_count` (a list of GPIO indices) and get_tt (a boolean), for example:

```
a0.get_new_spectra(gpio_count=[0,1], get_tt=True)
2025-11-25 15:37:44,995 - souk_mkid_readout.blocks.block:localhost:p0_acc0 - WARNING - Accumulation counter changed while reading data!
(array([301.+2992.j,   0.   +0.j,   0.   +0.j, ...,   0.   +0.j,
          0.   +0.j,   0.   +0.j]),
 [0, 0],
 225017715084)

```

You may have noticed the warning message about the accumulation counter changing while reading data.

This occurs because the default interface to the underlying FPGA registers is relatively slow, and if the accumulation length is short (output data rate is high) then the accumulator may update its output registers while we are still reading out the data.

Looking deeper into the source code, we see that the `_wait_for_acc` method is called to wait for a new accumulation to be ready, then the accumulated data is collected with the `_read_bram` method
```
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

```
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

```
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

```
print(a0.host.transport)
KatcpTransport(localhost):7147 - connected
```

This is a message based TCP/IP protocol which talks to CASPER's `tcpborphserver3` server on the RFSoC. `tcpborphserver3` handles the reading/writing of all of the FPGA registers and returns information to the host. This is useful for controlling the board (or many boards) remotely, but various overheads and latencies make it relatively slow for reading out large blocks of data quickly, like we want to do with the accumulator BRAMs.

To get around this, we can use the more direct `LocalMemTransport` which provides direct access to the FPGA's memory-mapped registers when the host code is running on the RFSoC itself. This requires root privileges as it uses the linux system's `/dev/mem` interface, but is much faster for reading out large blocks of data.

Passing `local=True` when creating the readout host `r` will ensure`LocalMemTransport` is used:

```
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

Note that we now see the warning about the accumulation counter changing during readout on `a0_fast` but not on `a1_fast` which has the higher accumulation length. 

We can do a quick time comparison of the two transport layers like this:

```
%timeit a0.get_new_spectra() # KatcpTransport on RFSoC through localhost
[... warning messages ...]
116 ms ± 3.53 ms per loop (mean ± std. dev. of 7 runs, 10 loops each)

%timeit a0_fast.get_new_spectra() # LocalMemTransport on RFSoC through /dev/mem
[... warning messages ...]
18.3 ms ± 4.26 µs per loop (mean ± std. dev. of 7 runs, 100 loops each)

```

Switching from KatcpTransport to LocalMemTransport has reduced the readout time from ~116 ms to ~18.3 ms (~8 per second to ~55 per second), a factor of ~6.3 speedup. 

Further optimisations are possible by interfacing directly with the underlying memory mapped BRAM and avoiding overheads with the CASPER and have been implemented in the `souk_readout_tools` package which is described in the next section.


## Using souk_readout_tools



