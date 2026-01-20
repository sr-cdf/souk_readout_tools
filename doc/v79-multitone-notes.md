# Notes on attempting the multitone feature in v7.9 firmware

v7.9 Firmware

Add "multitone" capability. Rather than requiring one tone per PSB bin allow each tone to be summed into a vector accumulator which feeds the PSB, in such a way that it's possible to have arbitrarily many tones in each bin.

Intially tested with souk_mkid_readout and after some debugging and fixes to the chanreorder block software, to correctly interpret inmap vs outmap semantics, this appears to be working.

Changes made to souk_readout_tools/firmware_lib.py to support the new firmware features are summarised below.

Testing of souk_readout_tools/firmware_lib.py changes is ongoing.

---

## Summary of Changes

### Multitone Operation (v7.9)

1. **VACC (Vector Accumulator) for PSB**: Tones are now summed into a vector accumulator before the PSB, allowing multiple tones to map to the same FFT bin.

2. **PSB Channel Selection - Inmap vs Outmap**: 
   - **Previously (outmap)**: `outmap[output_position] = fft_bin` — specified which FFT bin emerges at each output position
   - **Now (inmap)**: `inmap[lo_index] = fft_bin` — specifies which FFT bin each LO index contributes to
   - The inmap approach allows multiple LO indices to map to the same FFT bin (multitone capability)

3. **VACC Constraint**: When multiple tones feed the same FFT bin, their LO indices must be separated by at least 6 (due to dual-port RAM timing in the VACC). This is the `min_tone_separation` parameter.

4. **Discard Bin**: Unused LO indices map to a "discard bin" (`n_chans_out - 1`) instead of `-1`

### Corresponding Changes in `firmware_lib.py`

| Function | Change |
|----------|--------|
| `psb_chanselect_set_channel_inmap()` | **New** - Sets PSB channel mapping using inmap semantics |
| `psb_chanselect_get_channel_inmap()` | **New** - Reads PSB channel mapping as inmap |
| `compute_vacc_tone_indices()` | **New** - Computes optimal LO indices respecting VACC min separation constraint |
| `prepare_tone_frequency_settings()` | Updated to use `chanmap_psb_inmap` and `tone_indices` |
| `prepare_tone_frequency_settings_fast()` | Updated to use inmap and pass `tone_indices` |
| `apply_tone_frequency_settings()` | Updated to call `psb_chanselect_set_channel_inmap()` |
| `get_tone_frequencies()` | Updated to read from inmap; indexes by `psb_tones_active` (non-contiguous with VACC) |
| `get_tone_amplitudes()` | Updated to index by active tone indices (non-contiguous) |
| `get_tone_phases()` | Updated to index by active tone indices (non-contiguous) |
| `set_tone_amplitudes()` | Updated to place values at correct LO indices |
| `set_tone_phases()` | Updated to place values at correct LO indices |
| `prepare_sweep_settings_fast()` | Updated with VACC-aware tone index assignment |

### Key Concept: Tone Indices

With VACC, tone indices (LO indices) may be **non-contiguous**. For example, if 3 tones all map to the same FFT bin, they might be assigned LO indices `[0, 6, 12]` instead of `[0, 1, 2]` to satisfy the minimum separation constraint.

Functions that read/write tone parameters must now:
1. Determine active tone indices from the inmap (not just count tones)
2. Index into full-sized arrays at these specific positions
3. Build full-sized arrays when writing, placing values at correct LO indices

**NOTE**: The `compute_vacc_tone_indices` function does not currently implement backfilling of unused LO slots, which could lead to inefficient use of available tones if the number of tone per bin is high. This can be improved in future iterations.

---


## Initial testing with `souk_mkid_readout`

### Setup  

```python

(venv) sam@aigdetmux:~/souk/souk-firmware/software/control_sw$ ipython
Python 3.12.7 (main, Jun 18 2025, 13:16:51) [GCC 14.2.0]
Type 'copyright', 'credits' or 'license' for more information
IPython 8.29.0 -- An enhanced Interactive Python. Type '?' for help.

In [1]: import souk_mkid_readout
/home/sam/souk/venv/lib/python3.12/site-packages/casperfpga/tengbe.py:5: UserWarning: pkg_resources is deprecated as an API. See https://setuptools.pypa.io/en/latest/pkg_resources.html. The pkg_resources package is slated for removal as early as 2025-11-30. Refrain from using this package or pin to Setuptools<81.
  from pkg_resources import resource_filename

In [2]: r0 = souk_mkid_readout.SoukMkidReadout('10.11.11.11',configfile='/home/sam/souk/souk-firmware/softw
   ...: are/control_sw/config/souk-dual-pipeline-krm.yaml',pipeline_id=0)

In [3]: r0.program()
2026-01-13 11:59:38,301 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Programming with /home/sam/souk/souk-firmware/firmware/src/souk_dual_pipeline_krm/outputs/souk_dual_pipeline_krm_2026-01-08_1518.fpg

In [4]: r0.initialize_shared_blocks()
2026-01-13 11:59:43,992 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing shared block (writable): common
2026-01-13 11:59:44,006 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing shared block (writable): adc_snapshot
2026-01-13 11:59:44,006 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing shared block (writable): dac_snapshot
2026-01-13 11:59:44,006 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing shared block (writable): zoomfft
2026-01-13 11:59:44,013 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing shared block (writable): zoomacc
2026-01-13 11:59:44,014 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing shared block (writable): gen_cordic
2026-01-13 11:59:44,036 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing shared block (writable): gen_lut
2026-01-13 11:59:44,170 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing shared block (writable): autocorr

In [5]: r0.initialize_pipeline_blocks()
2026-01-13 11:59:50,518 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 sync
2026-01-13 11:59:50,536 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 input
2026-01-13 11:59:50,538 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 pfb
2026-01-13 11:59:50,543 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 pfbtvg
2026-01-13 11:59:50,545 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 chanselect
2026-01-13 11:59:50,598 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 mixer
2026-01-13 11:59:50,977 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 psb_chanselect
2026-01-13 11:59:51,103 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 psb
2026-01-13 11:59:51,108 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 psbscale
2026-01-13 11:59:51,110 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 accumulator0
2026-01-13 11:59:51,129 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 accumulator1
2026-01-13 11:59:51,149 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 output
2026-01-13 11:59:51,151 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Initializing pipeline block (writable): p0 out_delay
2026-01-13 11:59:51,154 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Detecting and compensating RX vs TX pipeline skew, p0
2026-01-13 11:59:52,361 - souk_mkid_readout.blocks.block:10.11.11.11:p0_sync - WARNING - Timed out waiting for sync pulse
2026-01-13 11:59:52,622 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Set sync delay to 5712 FPGA clocks, p0
2026-01-13 11:59:52,622 - souk_mkid_readout.souk_mkid_readout:10.11.11.11:0 - INFO - Performing software global reset, p0
2026-01-13 11:59:53,830 - souk_mkid_readout.blocks.block:10.11.11.11:p0_sync - WARNING - Timed out waiting for sync pulse

In [6]: r0.output.use_psb()

In [7]: r0.fpga.print_status()
antname: None
fw_build_time: 2026-01-08T15:17:13
fw_supported: True
fw_type: 3
fw_version: 7.9.2.0
host: aigdetmux:10.11.11.11
programmed: True
sw_version: 7.4.2.0.post90+gb091e680.dirty
timestamp: 2026-01-13T12:01:10.889133

# note the sw_version is 90 commits post the latest tag as
#  there is no 7.9 tag yet


```

### using `set_multi_tone(freqs_hz, phase_offsets_rads=None, amplitudes=None, los=['rx', 'tx'])`

#### 1500.0 MHz

`r0.set_multi_tone([1500e6])`

A single tone is visible at 1500MHz

```python
r0._get_closest_psb_bin(1500e6)
np.int64(904)

(904+4096)*r0.adc_clk_hz/8192
1500000000.0
```

**outmap:**

```python
r0.psb_chanselect.get_channel_outmap()
array([-1, -1, -1, ..., -1, -1, -1])

np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)
(array([224]),)

r0.psb_chanselect.get_channel_outmap()[np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)]
array([8])
```

Why bin 224 when the FFT chan is 904?

Why channel 8?

**inmap**
```python
r0.psb_chanselect.get_channel_inmap()
array([2047, 2047, 2047, ..., 2047, 2047, 2047])

np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)
(array([8]),)

r0.psb_chanselect.get_channel_inmap()[np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)]
array([224])
```

Fine, inmap is just inverse of outmap.

#### 1500.4 MHz - one bin higher

`r0.set_multi_tone([1500.4e6])`

A single tone is visible at 1500.4 MHz

```python
r0._get_closest_psb_bin(1500.4e6)
p.int64(905)
```

**outmap**

```python
r0.psb_chanselect.get_channel_outmap()
array([-1, -1, -1, ..., -1, -1, -1])

np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)
(array([225]),)

r0.psb_chanselect.get_channel_outmap()[np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)]
array([8])
```

**inmap**

```python
r0.psb_chanselect.get_channel_inmap()
array([2047, 2047, 2047, ..., 2047, 2047, 2047])

np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)
(array([8]),)

r0.psb_chanselect.get_channel_inmap()[np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)]
array([225])
```

#### 1500.0 + 1500.4 MHz - different bins
```python
r0.set_multi_tone([1500e6, 1500.4e6])
```
Two tones are visible at 1500.0 and 1500.4, MHz

**outmap**
```python
np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)
(array([224, 225]),)

r0.psb_chanselect.get_channel_outmap()[np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)]
array([8, 9])
```
**inmap**
```python
np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)
(array([8, 9]),)
r0.psb_chanselect.get_channel_inmap()[np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)]
array([224, 225])
```

#### 1500.1 MHz - same bin as 1500.0 MHz

```python
r0.set_multi_tone([1500.1e6])

r0._get_closest_psb_bin(1500.1e6)
p.int64(904)

```

A single tone is visible at 1500.1 MHz

**outmap**

```python

np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)
(array([224]),)

r0.psb_chanselect.get_channel_outmap()[np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)]
array([8])
```

**inmap**

```python
np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)
(array([8]),)

r0.psb_chanselect.get_channel_inmap()[np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)]
array([224])
```




#### 1500.0 and 1500.1 MHz - both in same bin

```python
r0.set_multi_tone([1500e6, 1500.1e6])
```

Only one tone visible at 1500.1 MHz

**outmap**
```python
np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)
(array([224]),)

r0.psb_chanselect.get_channel_outmap()[np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)]
array([9])

```

**inmap**
```python
np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)
(array([9]),)

r0.psb_chanselect.get_channel_inmap()[np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)]
array([224])
```

Note we are seeing output channel 9 here, not 8 as with single tone settings - the previous tone (1500.0 MHz) appears to have only been partially cleared.



### Using `set_tone(tone_id, freq_hz, phase_offset_rads=0.0, amp=1.0)`

Reprogram and reinitialise to clear previous state, then continue...

#### 1500.0 MHz

```python

r0.set_tone(0, 1500e6)
2026-01-13 12:19:58,230 - souk_mkid_readout.blocks.block:10.11.11.11:p0_chan_select - INFO - Setting output 0 to channel 904
2026-01-13 12:19:58,242 - souk_mkid_readout.blocks.block:10.11.11.11:p0_synth_input_reorder - INFO - Setting single channel input 0 -> output 904
```

A single tone is visible at 1500 MHz

**outmap**

```python
np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)
(array([224]),)

r0.psb_chanselect.get_channel_outmap()[np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)]
array([8])
```

**inmap**

```python
np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)
(array([8]),)

r0.psb_chanselect.get_channel_inmap()[np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)]
array([224])
```

all good so far

#### 1500.1 MHz

Try overwriting this tone with a new one in the same bin:

```python

r0.set_tone(0,1500.1e6)
2026-01-13 12:24:47,902 - souk_mkid_readout.blocks.block:10.11.11.11:p0_chan_select - INFO - Setting output 0 to channel 904
2026-01-13 12:24:47,915 - souk_mkid_readout.blocks.block:10.11.11.11:p0_synth_input_reorder - INFO - Setting single channel input 0 -> output 904
```

A single tone is visible at 1500.1 MHz

**outmap**

```python
np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)
(array([ 56, 224]),)

r0.psb_chanselect.get_channel_outmap()[np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)]
array([32,  8])
```

Note the extra non-zero bin at 56 in the outmap pointing to 32 - this is likely due to the previous tone at 1500.0 MHz not being cleared. But 224 still points to 8.

**inmap**

```python
np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)
(array([ 8, 32]),)

r0.psb_chanselect.get_channel_inmap()[np.nonzero(r0.psb_chanselect.get_channel_inmap()!=2047)]
array([224,  56])
```

And 8 still points to 224



### Folowing chat with Jack it appears that:

We need to call set inmap directly, 

We need to reorder tones by every 8th or something like that.

Now i understand why only one tone per bin works with set_multi_tone:

The following loop just overwrites the element of the `chanmap_psb` array corresponding to the same bin `tx_nearest_bin` multiple times:


```python
        for fn, freq_hz in enumerate(freqs_hz):
            ### Configure receiving side
            rx_nearest_bin, rx_freq_offset_hz = self._get_closest_pfb_bin(freq_hz)
            chanmap_in[fn] = rx_nearest_bin
            lo_freqs_hz[fn] = rx_freq_offset_hz
            ### Configure transmit side
            tx_nearest_bin = self._get_closest_psb_bin(freq_hz)
            chanmap_psb[tx_nearest_bin] = fn
```


## github comments with suggested fixes

### comment

(904 - 8) / 4 = 224

So the inmap index (mixer index) is 8 higher than expected if we expect it to be zero for a single tone, and the inmap value (fft bin index into which the that mixer is directed) is a quarter of the [expected bin index minus 8].

### comment
chanreorder.VaccReorderMultiSampleIn.get_channel_inmap() uses the original get_channel_outmap code.

it decodes the inmap as if it were an outmap.

The following code correctly interprets and decodes the serial_maps into an inmap:
```python
    def get_channel_inmap(self):
        """
        Get the currently loaded reorder map.

        :return: The reorder map currently loaded. Entry `i` in this map
            corresponds to the output channel to which input `i` contributes.
        :rtype: list
        """
        # Read the reorder memory contents
        nbytes = self._reorder_depth * np.dtype(self._map_format).itemsize
        serial_maps = np.zeros([self._expansion_factor, self._reorder_depth], dtype=int)
        for i in range(self._expansion_factor):
            serial_maps[i] = np.frombuffer(self.read(f'map{i}_{self._map_reg}', nbytes), dtype=self._map_format)

        # Precompute the mapping from (block_id, block_offset) -> output channel
        outchans = np.arange(self.n_chans_out)
        block_id = (outchans // self.n_parallel_samples) % self._expansion_factor
        block_s_offset = outchans // self.n_parallel_chans_out
        block_p_offset = outchans % self.n_parallel_samples
        block_offset = block_s_offset * self.n_parallel_samples + block_p_offset
        
        offset_to_outchan = {}
        for out_ch in range(self.n_chans_out):
            offset_to_outchan[(block_id[out_ch], block_offset[out_ch])] = out_ch
        
        # Default stored value is (n_chans_in - 1) = 2047
        default_val = self.n_chans_in - 1
        
        inmap = np.ones(self.n_chans_in, dtype=int) * (self.n_chans_out - 1)
        for i in range(self.n_chans_in):  # i = input channel index
            # Check which expansion block has a non-default value for input i
            for exp_idx in range(self._expansion_factor):
                stored_val = serial_maps[exp_idx, i]
                if stored_val != default_val:
                    # This block was written to - decode the output channel
                    key = (exp_idx, stored_val)
                    if key in offset_to_outchan:
                        inmap[i] = offset_to_outchan[key]
                    break
        
        return inmap    
```

This then changes the outmap length from get_channel_outmap to be the correct 8192 (fft bins), not the previous 2048 (mixer chans) meaning the line which fixes the final element of the outmap to be length 1 (to ensure flatness) is now choosing the wrong element and get_channel_outmap is not returning numpy arrays when applying single tones.

This can be fixed with the following code, which forces the correct final bin to see [-1] if all the other bins are length-1.
```python

    def get_channel_outmap(self):
        """
        Read the currently loaded reorder map.

        :return: The reorder map currently loaded. Entry `i` in this map is the
            input channel index which contributes to output channel `i`.
            If each output has at most one input, returns a 1D numpy array.
            If any output has multiple inputs, returns a list of lists.
        :rtype: np.ndarray or list
        """
        inmap = self.get_channel_inmap()
        discard_bin = self.n_chans_out - 1
        outmap = [[] for _ in range(self.n_chans_out)]
        for i, v in enumerate(inmap):
            # Don't accumulate into the discard bin
            if v != discard_bin:
                outmap[v] += [i]
        # For consistency with other reorder blocks, use -1 to mean "not used".
        # The discard bin is always marked as unused.
        outmap[discard_bin] = [-1]
        for i, v in enumerate(outmap):
            if v == []:
                outmap[i] = [-1]
        # If each output has exactly one input, return a 1D numpy array for API consistency
        if all(len(v) == 1 for v in outmap):
            return np.array([v[0] for v in outmap], dtype=int)
        else:
            return outmap
```

These fixes also fix the issue with leftover values when overwriting single tones with the same id in set_tone():

```python

In [11]: r0.set_tone(0, 1500e6)
    ...: 
2026-01-14 15:47:18,475 - souk_mkid_readout.blocks.block:10.11.11.11:p0_chan_select - INFO - Setting output 0 to channel 904
2026-01-14 15:47:18,487 - souk_mkid_readout.blocks.block:10.11.11.11:p0_synth_input_reorder - INFO - Setting single channel input 0 -> output 904

In [20]: r0.set_tone(0,1500.1e6)
    ...: 
2026-01-14 15:48:11,979 - souk_mkid_readout.blocks.block:10.11.11.11:p0_synth_input_reorder - INFO - Setting single channel input -1 -> output 904
2026-01-14 15:48:12,146 - souk_mkid_readout.blocks.block:10.11.11.11:p0_chan_select - INFO - Setting output 0 to channel 904
2026-01-14 15:48:12,157 - souk_mkid_readout.blocks.block:10.11.11.11:p0_synth_input_reorder - INFO - Setting single channel input 0 -> output 904

In [21]: np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)
Out[21]: (array([904]),)

In [22]: r0.psb_chanselect.get_channel_outmap()[np.nonzero(r0.psb_chanselect.get_channel_outmap()!=-1)]
Out[22]: array([0])

```
However, set_tone uses psb_chanreorder.set_single_tone, which also expects the old behaviour and will only apply one tone at a time, so we fix it with this:

souk_mkid_readout.py:
```python

    def set_tone(self, tone_id, freq_hz, phase_offset_rads=0.0, amp=1.0):
        """
        Configure both TX and RX paths for a tone at frequency ``freq_hz``
        with ID ``tone_id``.

        :param tone_id: Index number of tone to set
        :type tone_id: int

        :param freq_hz: Tone frequency, in Hz. Or, use ``None`` to disable
            this tone index.
        :type freq_hz: float

        :param phase_offset_rads: Phase offset of tone, in radians.
        :type phase_offset_rads: float

        :param amp: Tone amplitude, (<=1.0)
        :type amp: float
        """

        assert tone_id < N_TONE, f'Only tone IDs 0..{N_TONE-1} supported'
        # Disable anywhere either synthesizer is already using this tone ID
        # TODO: is this the best behaviour?
        chanmap = self.psb_chanselect.get_channel_outmap()
        
        # Handle both numpy array and list of lists cases
        if isinstance(chanmap, np.ndarray):
            # Simple case: each bin has one tone
            for b in np.where(chanmap == tone_id)[0]:
                self.psb_chanselect.set_single_channel(b, -1)
        else:
            # List of lists case: bins can have multiple tones
            for b, tones in enumerate(chanmap):
                if tone_id in tones:
                    # Remove this tone from the bin
                    new_tones = [t for t in tones if t != tone_id]
                    if len(new_tones) == 0:
                        new_tones = [-1]
                    chanmap[b] = new_tones
            # Write the updated map
            self.psb_chanselect.set_channel_outmap(chanmap)
        
        if freq_hz is None:
            return
        ### Configure receiving side
        rx_nearest_bin, rx_freq_offset_hz = self._get_closest_pfb_bin(freq_hz)
        # Put this bin in the correct tone slot
        self.chanselect.set_single_channel(tone_id, rx_nearest_bin)
        # Configure the mixer at this ID to the appropriate offset freq
        self.mixer.set_chan_freq(tone_id, freq_offset_hz=rx_freq_offset_hz,
                                 phase_offset=phase_offset_rads,
                                 sample_rate_hz=self.adc_clk_hz)
        self.mixer.set_amplitude_scale(tone_id, amp)
        
        ### Configure transmit side
        # Index of nearest bin
        tx_nearest_bin = self._get_closest_psb_bin(freq_hz)
        # Get index of nearest bin, and place tone in this bin for relevant
        # synth bank.
        self.psb_chanselect.set_single_channel(tx_nearest_bin, tone_id)
```

and this

chanreorder.py:

```python
 def set_single_channel(self, outidx, inidx):
        """
        Set output channel number ``outidx`` to input number ``inidx``.
        Do this by reading the total channel map, modifying a single entry,
        and writing back.

        Example usage:
            # Set the first channel out of the reorder to 33
            ```set_single_channel(0, 33)``

        :param outidx: Index of output channel to set.
        :type outidx: int

        :param inidx: Input channel index to select.
        :type inidx: int
        """
        self.logger.info(f'Setting single channel input {inidx} -> output {outidx}')
        outmap = self.get_channel_outmap()
        # If the output channel is undriven (driven by input -1)
        # set it to be driven by the input index.
        # Otherwise, add the input index to the list of other drivers
        current_val = outmap[outidx]
        if current_val == -1 or inidx == -1:
            outmap[outidx] = inidx
        else:
            # Handle both numpy array and list of lists cases
            if isinstance(outmap, np.ndarray):
                # Convert to list of lists for modification
                outmap = [[v] for v in outmap]
            outmap[outidx] = outmap[outidx] + [inidx]
        self.set_channel_outmap(outmap)

```

The following code implements set_multi_tone_vacc, which works for multiple tones per bin although its not fully tested. min_tone_seperation works with 8 and not with 2, havent tried others,

Note that overflow happens if the sum of the amplitudes is greater 1.0 (and the phases align)> Visible as large distortion and spurs on spectrum analyser. I suppose its wishful thinking to hope these integers could be somehow widened in the Vacc? Or an overflow flag made available?

```python
    def set_multi_tone_vacc(self, freqs_hz, phase_offsets_rads=None, amplitudes=None, los=['rx', 'tx'], min_tone_separation=8, return_tone_id_map=True):
        """
        Configure both TX and RX paths for multiple tones, supporting multiple tones per FFT bin.
        Handles the VACC constraint that consecutive LO indices cannot feed the same bin.

        :param freqs_hz: Tone frequencies, in Hz.
        :type freqs_hz: list of float

        :param phase_offsets_rads: Phase offset of tones, in radians. If none is
            provided, offsets of 0 are used.
        :type phase_offsets_rads: list of float

        :param amplitudes: Relative amplitude of tones, provided as a list
            of floats between 0 and 1. If none is provided, amplitudes of 1.0
            are used.
        :type amplitudes: list of float

        :param los: List of LOs to write to. Can be ['rx'], ['tx'] or ['rx', 'tx']
        :type los: list

        :param min_tone_separation: Minimum separation between LO indices that feed
            the same FFT bin (due to VACC dual-port RAM timing). Default is 2.
        :type min_tone_separation: int
        """
        n_tones = len(freqs_hz)
        if phase_offsets_rads is None:
            phase_offsets_rads = np.zeros(n_tones, dtype=float)
        if amplitudes is None:
            amplitudes = np.ones(n_tones, dtype=float)
        
        assert len(freqs_hz) == n_tones
        assert len(phase_offsets_rads) == n_tones
        assert len(amplitudes) == n_tones
        
        # Group tones by their target RX FFT bin
        bin_to_tones = {}  # bin_index -> list of (tone_idx, freq_offset_hz)
        
        for tone_idx, freq_hz in enumerate(freqs_hz):
            rx_nearest_bin, rx_freq_offset_hz = self._get_closest_pfb_bin(freq_hz)
            if rx_nearest_bin not in bin_to_tones:
                bin_to_tones[rx_nearest_bin] = []
            bin_to_tones[rx_nearest_bin].append((tone_idx, rx_freq_offset_hz))
        
        # Build the inmap: for each LO index, specify which FFT bin it feeds
        # Initialize all LOs to feed the "discard" bin (last output channel)
        inmap = np.ones(N_TONE, dtype=int) * (self.psb_chanselect.n_chans_out - 1)
        
        # Also track chanselect outmap (which RX PFB bin feeds each LO)
        chanmap_in = -1 * np.ones(self.chanselect.n_chans_out, dtype=int)
        lo_freqs_hz = np.zeros(N_TONE, dtype=float)
        
        # Assign LO indices to bins, respecting the VACC constraint
        # For bins with multiple tones, we need to ensure LO indices are separated
        lo_idx = 0
        tone_to_lo = {}  # original tone index -> assigned LO index
        
        for rx_bin, tone_list in bin_to_tones.items():
            n_tones_in_bin = len(tone_list)
            
            for i, (orig_tone_idx, freq_offset_hz) in enumerate(tone_list):
                # Find a suitable LO index
                # If multiple tones in this bin, ensure spacing
                if i > 0:
                    # Need to skip ahead to maintain min_tone_separation from previous LO in same bin
                    prev_lo = tone_to_lo[tone_list[i-1][0]]
                    required_lo = prev_lo + min_tone_separation
                    if lo_idx < required_lo:
                        lo_idx = required_lo
                
                if lo_idx >= N_TONE:
                    raise ValueError(f"Ran out of LO slots. Reduce number of tones or tones per bin.")
                
                # Assign this LO to this bin
                tone_to_lo[orig_tone_idx] = lo_idx
                inmap[lo_idx] = rx_bin
                chanmap_in[lo_idx] = rx_bin  # This LO gets data from this RX bin
                lo_freqs_hz[lo_idx] = freq_offset_hz
                
                lo_idx += 1
        
        # Write RX channel selection (which PFB bin feeds each mixer/LO)
        self.chanselect.set_channel_outmap(chanmap_in)
        
        # Write mixer tones (only for used LOs, but set_freqs expects full array)
        # We need to set phases and amplitudes for the assigned LO indices
        lo_phases = np.zeros(N_TONE, dtype=float)
        lo_amps = np.zeros(N_TONE, dtype=float)
        for orig_idx, lo_idx in tone_to_lo.items():
            lo_phases[lo_idx] = phase_offsets_rads[orig_idx]
            lo_amps[lo_idx] = amplitudes[orig_idx]
        
        self.mixer.set_freqs(lo_freqs_hz, lo_phases, lo_amps, self.adc_clk_hz, los)
        
        # Write the inmap for the VACC reorder (PSB side)
        self.psb_chanselect.set_channel_inmap(inmap)
        
        if return_maps:
            return tone_to_lo  # Return mapping so user knows which LO each tone ended up on
        else:
            return

```
Concluding remarks:

I can now happily set tones in the same bin, as long as they dont overflow.

I believe everything is still backwards compatible.

I will crack on with updating my faster implementation seperately.

I still want to test the fast dump burst thing tomorrow, if that works then we are probably ok to go with the merge, if you happy with the above changes.

### comment 

Testing set_multi_tone_vacc in more detail, appears that min_tone_separation=6 is the lowest value that works. Anything lower and some tones simply disappear.


### comment

Also, note that in the above implementation, unused LO slots that fall in the gaps between same-bin LOs are never back filled, so the number of available slots is reduced. There are fixes for this, but it seems unlikely to cause any problems right now, so happy to leave that for another day.

### comment

copilot made some additional fixes. now pulled into souk-firmware v7.9-multitone branch.


## Using `souk_readout_tools`