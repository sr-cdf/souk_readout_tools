"""
This module provides a set of functions that are used by the readout server to interact with the SOUK firmware.


"""

import warnings
import numpy as np
import time
#import asyncio

import souk_mkid_readout
from souk_mkid_readout.souk_mkid_readout import *

import calibration

autosync_time_delay = 0.001 #seconds

adc_saturation_bits = 16 # note the adc gives 16 bit data but is a 14 bit converter
dac_saturation_bits = 16 # note the dac takes 16 bit data but is a 14 bit converter

def cplx2uint(d,nbits,fmt='>u4'):
    """
    Vectorized: Convert a floating point real, imag pair
        to a UFix<nbits>_<nbits-1> CASPER-standard complex number.
    
    fmt should be '>u4' with the standard firmware interface or '<u4' with the fast mmap-ed interface
    """
    tnb = 2**nbits
    tnm1b = 2**(nbits-1)
    tnm1bm1 = tnm1b-1
    real = (np.round(d.real * tnm1b)).astype(int)
    imag = (np.round(d.imag * tnm1b)).astype(int)
    # Saturate
    real[real > tnm1bm1] = tnm1bm1
    imag[imag > tnm1bm1] = tnm1bm1 
    real[real<0] += tnb
    imag[imag<0] += tnb
    return ((real << nbits) + imag).astype(fmt)

def uint2cplx(d, nbits):
    """
    Vectorised: Convert a CASPER-standard UFix<nbits>_<nbits-1>
        complex number to a real, imag pair.
    """
    tnb = 2**nbits
    tnbm1 = tnb-1
    tnm1b = 2**(nbits-1)
    # tnm1bm1 = tnm1b-1
    real = (d.astype(int) >> nbits) & tnbm1
    imag = d.astype(int) & tnbm1
    real[real >= tnm1b] -= tnb
    imag[imag >= tnm1b] -= tnb
    return (real + 1j*imag) / tnm1b

def _format_phase_steps(phase, phase_bp, fmt='>i4'):
    """
    Vectorised: Given a desired phase step, format as appropriate
        integers which are interpretable by the mixer firmware

        :param phase: phase[s] to step per clock cycle, in radians
        :type phase: float, or array of floats

        :param phase_bp: binary points of the phase accumulator
        :type phase_bp: int

        :return: phase_int -- the integers to be written
            to firmware. Is either an integer (if `phase'
            is an integer. Else an array of integers.)
        :rtype: int (or array(dtype=int))

    fmt should be '>i4' with the standard firmware interface or '<i4' with the fast mmap-ed interface
    
     """
    phase_scaled = phase / np.pi
    phase_scaled = ((phase_scaled + 1) % 2) - 1
    phase_int = (phase_scaled * (2**phase_bp)).astype(fmt)
    return phase_int

def _format_phase_offsets(phase_offsets, phase_offset_bp):
    """
    Vectorised: Given a desired phase offset, format as appropriate
        integers which are interpretable by the mixer firmware

        :param phase_offset: phase offset[s] of tones, in radians
        :type phase_offset: float, or array of floats

        :param phase_offset_bp: binary points of the phase offset
        :type phase_offset_bp: int

        :return: phase_offset_int -- the integers to be written
            to firmware. Is either an integer (if `phase_offset'
            is an integer. Else an array of integers.
        :rtype: int (or array(dtype=int))
     """
    phase_offset_scaled = phase_offsets / np.pi
    phase_offset_scaled = ((phase_offset_scaled + 1) % 2) - 1
    phase_offset_int = (phase_offset_scaled * (2**phase_offset_bp)).astype('>i4')
    return phase_offset_int

def _format_amp_scale(amplitude_scale_factors,n_scale_bits):
    """
    Vectorised:    Given a desired scale factor, format as an appropriate
        integer which is interpretable by the mixer firmware.

        :param v: Scale factors to apply to the tones
        :type v: array of floats

        :param n_scale_bits: Number of bits to use for the scale factor
        :type n_scale_bits: int

        :return: Integer scale[s]
        :rtype: array of ints
    """
    v = amplitude_scale_factors * 2**n_scale_bits
    v = np.round(v).astype(int)
    # saturate
    scale_max = 2**n_scale_bits - 1
    v[v > scale_max] = scale_max
    return v.astype('>u4')

def _invert_format_phase_steps(phase_int,phase_bp):
    """
    Vectorised: Given a phase step integer, or array of integers, as read from the firmware,
      invert the formatting applied by `_format_phase_steps'
    """
    phase_scaled = phase_int.astype(float) / (2**phase_bp)
    #dont need to invert this: phase_scaled = ((phase_scaled + 1) % 2) - 1
    phase = phase_scaled * np.pi
    return phase

def _invert_format_phase_offsets(phase_offset_int,phase_offset_bp):
    """
    Vectorised: Given a phase offset integer or array of integers, as read from the firmware,
      invert the formatting applied by `_format_phase_offsets'
    """
    phase_offset_scaled = phase_offset_int.astype(float) / (2**phase_offset_bp)
    #dont need to invert this: phase_offset_scaled = ((phase_offset_scaled + 1) % 2) - 1
    phase_offset = phase_offset_scaled * np.pi
    return phase_offset

def _invert_format_amp_scale(scale_factors_int,n_scale_bits):
    """
    Vectorised: Given a scale factor integer or array of integers, as read from the firmware,
        invert the formatting applied by `_format_amp_scale'
    """
    scale_factors = scale_factors_int.astype(float) / 2**n_scale_bits
    return scale_factors


def _wait_for_acc(r,accnum=0,poll_period_s=0.1):
    return r.accumulators[accnum]._wait_for_acc(poll_period_s)
    

def _blocking_sleep(duration, get_now=time.perf_counter):
    now = get_now()
    end = now + duration
    while now < end:
        now = get_now()


def _blocking_wait_for_acc(acc,poll_period_s=0.1):
    """
    Function to wait for the next accumulation.
    Uses a blocking sleep method to improve performance (by reducing context switches?).
    """
    cnt0 = acc.get_acc_cnt()
    cnt1 = acc.get_acc_cnt()
    # Counter overflow protection
    if cnt1 < cnt0:
        cnt1 += 2**32
    while cnt1 < ((cnt0+1) % (2**32)):
        # #time.sleep(poll_period_s)
        _blocking_sleep(poll_period_s)
        cnt1 = acc.get_acc_cnt()
    return cnt1


def create_standard_readout_interface(fw_config_file):
    r = SoukMkidReadout('localhost',configfile=fw_config_file)
    return r

def create_fast_readout_interface(fw_config_file):
    r_fast = SoukMkidReadout('localhost',configfile=fw_config_file,local=True)
    return r_fast

def needs_programming(r):
    if not hasattr(r, 'accumulators'):
        return True
    if not r.fpga.is_programmed():
        return True
    return False

def reload_firmware(config_dict): 
    fw_config_file = config_dict['firmware']['fw_config_file']
    r = create_standard_readout_interface(fw_config_file)
    r_fast = create_fast_readout_interface(fw_config_file)
    r.program()
    initialise_firmware(r,config_dict=config_dict)
    return r, r_fast

def initialise_firmware(r,config_dict):

    fwconf = config_dict['firmware']
    fwkeys = fwconf.keys()

    if 'defaults' in fwkeys:
        defaults = fwconf['defaults']
    else:
        defaults = {}

    if 'fw_config_file' in fwkeys:
        fw_config_file = fwconf['fw_config_file']
    else:
        raise ValueError('Firmware configuration file not specified in firmware configuration')
    if 'pipeline_id' in fwkeys:
        pipeline_id = fwconf['pipeline_id']
    else:
        raise ValueError('Pipeline ID not specified in firmware configuration')
    if 'dac0_tile' in fwkeys:
        dac0_tile = fwconf['dac0_tile']
    else:
        raise ValueError('DAC0 tile not specified in firmware configuration')
    if 'dac0_block' in fwkeys:
        dac0_block = fwconf['dac0_block']
    else:
        raise ValueError('DAC0 block not specified in firmware configuration')
    if 'dac1_tile' in fwkeys:
        dac1_tile = fwconf['dac1_tile']
    else:
        raise ValueError('DAC1 tile not specified in firmware configuration')
    if 'dac1_block' in fwkeys:
        dac1_block = fwconf['dac1_block']
    else:
        raise ValueError('DAC1 block not specified in firmware configuration')
    if 'adc_tile' in fwkeys:
        adc_tile = fwconf['adc_tile']
    else:
        raise ValueError('ADC tile not specified in firmware configuration')
    if 'adc_block' in fwkeys:
        adc_block = fwconf['adc_block']
    else:
        raise ValueError('ADC block not specified in firmware configuration')

    dac0_calibration_file = fwconf.get('dac0_calibration_file',None)
    dac1_calibration_file = fwconf.get('dac1_calibration_file',None)
    adc_calibration_file = fwconf.get('adc_calibration_file',None)
    
    
    sync_delay = defaults.get('sync_delay',None)
    acc_len = defaults.get('acc_len',None)
    dac_duc_mixer_frequency_hz = defaults.get('dac_duc_mixer_frequency_hz',None)
    adc_ddc_mix_frequency_hz = defaults.get('adc_ddc_mix_frequency_hz',None)
    nyquist_zone = defaults.get('nyquist_zone',None)
    dac_mixer_scale_1p0 = defaults.get('dac_mixer_scale_1p0',None)
    adc_mixer_scale_1p0 = defaults.get('adc_mixer_scale_1p0',None)
    dsa = defaults.get('dsa',None)
    vop = defaults.get('vop',None)
    internal_loopback = defaults.get('internal_loopback',None)
    psb_scale = defaults.get('psb_scale',None)
    psb_fftshift = defaults.get('psb_fftshift',None)
    pfb_fftshift = defaults.get('pfb_fftshift',None)
    frequencies = defaults.get('frequencies',[])
    amplitudes = defaults.get('amplitudes',[])
    phases = defaults.get('phases',[])

    r.initialize()

    r.output.use_psb()

    if sync_delay is not None:
        r.sync.set_delay(sync_delay)
        time.sleep(autosync_time_delay)
        r.sync.arm_sync(wait=False)
        time.sleep(autosync_time_delay)
        r.sync.sw_sync()
    if acc_len is not None:
        r.accumulators[0].set_acc_len(acc_len)

    if dac_duc_mixer_frequency_hz is not None:
        r.rfdc.core.set_fine_mixer_freq(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE,dac_duc_mixer_frequency_hz/1e6)
        r.rfdc.core.set_fine_mixer_freq(dac1_tile,dac1_block,r.rfdc.core.DAC_TILE,dac_duc_mixer_frequency_hz/1e6)
    if adc_ddc_mix_frequency_hz is not None:
        r.rfdc.core.set_fine_mixer_freq(adc_tile,adc_block,r.rfdc.core.ADC_TILE,adc_ddc_mix_frequency_hz/1e6)
    if nyquist_zone is not None:
        set_nyquist_zone(r,config_dict,nyquist_zone,inv_sinc=False)
    if dac_mixer_scale_1p0 is not None:
        if dac_mixer_scale_1p0:
            r.rfdc.core.set_mixer_scale(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE,r.rfdc.core.MIX_SCALE_1P0)
            r.rfdc.core.set_mixer_scale(dac1_tile,dac1_block,r.rfdc.core.DAC_TILE,r.rfdc.core.MIX_SCALE_1P0)
        else:
            r.rfdc.core.set_mixer_scale(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE,r.rfdc.core.MIX_SCALE_AUTO)
            r.rfdc.core.set_mixer_scale(dac1_tile,dac1_block,r.rfdc.core.DAC_TILE,r.rfdc.core.MIX_SCALE_AUTO)
    if adc_mixer_scale_1p0 is not None:
        if adc_mixer_scale_1p0:
            r.rfdc.core.set_mixer_scale(adc_tile,adc_block,r.rfdc.core.ADC_TILE,r.rfdc.core.MIX_SCALE_1P0)
        else:
            r.rfdc.core.set_mixer_scale(adc_tile,adc_block,r.rfdc.core.ADC_TILE,r.rfdc.core.MIX_SCALE_AUTO)
    if dsa is not None:
        r.rfdc.core.set_dsa(adc_tile, adc_block, dsa)
    if vop is not None:
        r.rfdc.core.set_vop(dac0_tile, dac0_block, vop)
        r.rfdc.core.set_vop(dac1_tile, dac1_block, vop)
    if internal_loopback is not None:
        if internal_loopback:
            r.input.enable_loopback()
        else:
            r.input.disable_loopback()
    if psb_scale is not None:
        r.psbscale.set_scale(psb_scale)
    if psb_fftshift is not None:
        r.psb.set_fftshift(psb_fftshift)
    if pfb_fftshift is not None:
        r.pfb.set_fftshift(pfb_fftshift)
    if frequencies:
        set_tone_frequencies(r, config_dict, frequencies)
    if amplitudes:
        set_tone_amplitudes(r, config_dict, amplitudes)
    if phases:
        set_tone_phases(r,config_dict, phases)
    
    dac_saturation = check_output_saturation(r,iterations=1,saturation_bits=dac_saturation_bits)
    adc_saturation = check_input_saturation(r,iterations=1,saturation_bits=adc_saturation_bits)
    dsp_overflow = check_dsp_overflow(r)
    print(f'DAC levels: {dac_saturation}')
    print(f'ADC levels: {adc_saturation}')
    print(f'DSP overflow: {dsp_overflow}')
    
    return

def get_system_information(r,config_dict):
    
    pipeline_id = config_dict['firmware']['pipeline_id']
    dac0_tile = config_dict['firmware']['dac0_tile']
    dac0_block = config_dict['firmware']['dac0_block']
    dac1_tile = config_dict['firmware']['dac1_tile']
    dac1_block = config_dict['firmware']['dac1_block']
    adc_tile = config_dict['firmware']['adc_tile']
    adc_block = config_dict['firmware']['adc_block']

    info = {}
    info['fpga_status'] = r.fpga.get_status()[0]
    info['pipeline_id'] = r.pipeline_id
    info['output_mode'] = r.output.get_status()[0]['mode']
    info['sync_delay'] = r.sync.get_delay()
    info['acc_len'] = r.accumulators[0].get_acc_len()
    info['acc_freq'] = get_sample_rate(r)
    info['internal_loopback'] = r.input.loopback_enabled()
    info['psb_scale'] = r.psbscale.get_scale()
    info['psb_fftshift'] = r.psb.get_fftshift()
    info['pfb_fftshift'] = r.pfb.get_fftshift()
    info['dsa'] = r.rfdc.core.get_dsa(adc_tile,adc_block)['dsa']
    info['vop_dac0'] = r.rfdc.core.get_output_current(dac0_tile,dac0_block)['current']
    info['vop_dac1'] = r.rfdc.core.get_output_current(dac1_tile,dac1_block)['current']
    info['dac_duc_mixer_frequency_hz'] = float(r.rfdc.core.get_mixer_settings(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE)['Freq'])*1e6
    info['adc_ddc_mix_frequency_hz'] = float(r.rfdc.core.get_mixer_settings(adc_tile,adc_block,r.rfdc.core.ADC_TILE)['Freq'])*1e6
    info['nyquist_zone_adc'] = r.rfdc.core.get_nyquist_zone(adc_tile,adc_block,r.rfdc.core.ADC_TILE)
    info['nyquist_zone_dac0'] = r.rfdc.core.get_nyquist_zone(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE)
    info['nyquist_zone_dac1'] = r.rfdc.core.get_nyquist_zone(dac1_tile,dac1_block,r.rfdc.core.DAC_TILE)
    info['mixer_scale_1p0_dac0'] = r.rfdc.core.get_mixer_settings(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE)['FineMixerScale'] == r.rfdc.core.MIX_SCALE_1P0
    info['mixer_scale_1p0_dac1'] = r.rfdc.core.get_mixer_settings(dac1_tile,dac1_block,r.rfdc.core.DAC_TILE)['FineMixerScale'] == r.rfdc.core.MIX_SCALE_1P0
    info['mixer_scale_1p0_adc'] = r.rfdc.core.get_mixer_settings(adc_tile,adc_block,r.rfdc.core.ADC_TILE)['FineMixerScale'] == r.rfdc.core.MIX_SCALE_1P0
    info['mixer_qmc_settings_dac0'] = r.rfdc.core.get_qmc_settings(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE)
    info['mixer_qmc_settings_dac1'] = r.rfdc.core.get_qmc_settings(dac1_tile,dac1_block,r.rfdc.core.DAC_TILE)
    info['mixer_qmc_settings_adc'] = r.rfdc.core.get_qmc_settings(adc_tile,adc_block,r.rfdc.core.ADC_TILE)
    info['tone_frequencies'] = get_tone_frequencies(r,config_dict).tolist()
    info['tone_amplitudes'] = get_tone_amplitudes(r,config_dict).tolist()
    info['tone_phases'] = get_tone_phases(r,config_dict).tolist()
    print('system information:')
    for key, value in info.items():
        print(f'{key}: {value}\n')

    return info

def read_parameter(r, param_name):
    if hasattr(r, param_name):
        return getattr(r, param_name)
    else:
        print(f'Parameter not found {param_name}')
        return None

def write_parameter(r, param_name, param_value):
    if hasattr(r, param_name):
        setattr(r, param_name, param_value)
    else:
        print(f'Parameter not found {param_name}')
        return None    
     

def get_sample_rate(r):
    fft_bw = r.adc_clk_hz / (N_RX_FFT / N_RX_OVERSAMPLE)
    acc_len = r.accumulators[0].get_acc_len()
    return fft_bw / acc_len


def set_sample_rate(r,sample_rate_hz):
    print(sample_rate_hz)
    fft_bw = r.adc_clk_hz / (N_RX_FFT / N_RX_OVERSAMPLE)
    acc_len = fft_bw / sample_rate_hz
    print(sample_rate_hz,r.adc_clk_hz,fft_bw, acc_len)
    if acc_len % 1 != 0:
        nearest_int = int(round(acc_len))
        nearest_rate = fft_bw / float(nearest_int)
        print(f'Accumulation length {acc_len} must be an integer, trying {nearest_int} = {nearest_rate} Hz')
        acc_len = nearest_int
    if acc_len %4 != 0:
        print(f'Accumulation length {acc_len} must be a multiple of 4, rounding up')
        acc_len = 4 * int(np.ceil(acc_len / 4))
    if acc_len < 1:
        acc_len = 1
    if acc_len > 2**16-1:
        acc_len = 2**16-1
    acc_freq = fft_bw / float(acc_len)
    acc_len = int(acc_len)
    print(f'setting acc len {acc_len} = {acc_freq} Hz')
    r.accumulators[0].set_acc_len(acc_len)
    return acc_freq


def set_nyquist_zone(r,config_dict,nyquist_zone,inv_sinc=False):
    """
    Set the Nyquist zone for both DACs and the ADC in the RFSOC.

    """
    dac0_tile = int(config_dict['firmware']['dac0_tile'])
    dac0_block = int(config_dict['firmware']['dac0_block'])
    dac1_tile = int(config_dict['firmware']['dac1_tile'])
    dac1_block = int(config_dict['firmware']['dac1_block'])
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])

    if nyquist_zone not in [1,2]:
        raise ValueError(f'Invalid Nyquist zone {nyquist_zone}')
    
    r.rfdc.core.set_nyquist_zone(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE,nyquist_zone)
    r.rfdc.core.set_nyquist_zone(dac1_tile,dac1_block,r.rfdc.core.DAC_TILE,nyquist_zone)
    r.rfdc.core.set_nyquist_zone(adc_tile,adc_block,r.rfdc.core.ADC_TILE,nyquist_zone)

    if nyquist_zone == 1:
        # mix baseband to center of 1st zone (+Fs*1/4, where Fs=2*r.adc_clk_hz)
        r.rfdc.core.set_fine_mixer_freq(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE,
                                        +2*r.adc_clk_hz*1/4/1e6)
        r.rfdc.core.set_fine_mixer_freq(dac1_tile,dac1_block,r.rfdc.core.DAC_TILE,
                                        +2*r.adc_clk_hz*1/4/1e6)
        # mix center of 1st zone down to baseband (-Fs*1/4, where Fs=2*r.adc_clk_hz)
        r.rfdc.core.set_fine_mixer_freq(adc_tile,adc_block,r.rfdc.core.ADC_TILE,
                                        -2*r.adc_clk_hz*1/4/1e6)

        if inv_sinc:
            r.rfdc.core.set_invsinc_fir(dac0_tile,dac0_block,r.rfdc.core.INVSINC_FIR_NYQUIST1)
            r.rfdc.core.set_invsinc_fir(dac1_tile,dac1_block,r.rfdc.core.INVSINC_FIR_NYQUIST1)
        else:
            r.rfdc.core.set_invsinc_fir(dac0_tile,dac0_block,r.rfdc.core.INVSINC_FIR_DISABLED)
            r.rfdc.core.set_invsinc_fir(dac1_tile,dac1_block,r.rfdc.core.INVSINC_FIR_DISABLED)

    elif nyquist_zone == 2:
        # mix baseband to center of 2nd zone and flip (-Fs*3/4, where Fs=2*r.adc_clk_hz)
        r.rfdc.core.set_fine_mixer_freq(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE,
                                        -2*r.adc_clk_hz*3/4/1e6)
        r.rfdc.core.set_fine_mixer_freq(dac1_tile,dac1_block,r.rfdc.core.DAC_TILE,
                                        -2*r.adc_clk_hz*3/4/1e6)
        #mix center of 2nd zone down to baseband and flip (+Fs*3/4, where Fs=2*r.adc_clk_hz)
        r.rfdc.core.set_fine_mixer_freq(adc_tile,adc_block,r.rfdc.core.ADC_TILE,
                                        +2*r.adc_clk_hz*3/4/1e6)
        # use high pass image reject filer. Has no effect?
        # r.rfdc.core.set_imr_mode(0,0,1)

        if inv_sinc:
            r.rfdc.core.set_invsinc_fir(dac0_tile,dac0_block,r.rfdc.core.INVSINC_FIR_NYQUIST2)
            r.rfdc.core.set_invsinc_fir(dac1_tile,dac1_block,r.rfdc.core.INVSINC_FIR_NYQUIST2)
        else:
            r.rfdc.core.set_invsinc_fir(dac0_tile,dac0_block,r.rfdc.core.INVSINC_FIR_DISABLED)
            r.rfdc.core.set_invsinc_fir(dac1_tile,dac1_block,r.rfdc.core.INVSINC_FIR_DISABLED)
        
    return


def get_tone_frequencies(r, config_dict, detailed_output=False):
    """
    Query the RFSOC for the current tone frequencies.

    Reads the phase increment values from the LOs in 
    the mixer, and the polyphase filterbank channel frequencies 
    and calculates the digital baseband tone frequencies. 
    
    The digital baseband tones are then converted 
    to the analog output frequencies by adding the RFDC DUC frequency offset and 
    accounting for the selected Nyquist zone. 
    
    If the analog updown converter (UDC) is connected, the analog output 
    frequencies are then converted to the RF frequencies by adding the appropriate
    frequency offset given UDC LO frequency and selected sideband.

    TODO: account for dual dac mode, for now assume all on dac 0

    """
    #config
    udc_connected = config_dict['rf_frontend']['connected']
    udc_lo_frequency = config_dict['rf_frontend']['tx_mixer_lo_frequency_hz']
    udc_sideband = config_dict['rf_frontend']['tx_mixer_sideband']
    udc_connected = False if not udc_connected else udc_connected
    udc_lo_frequency = 0 if not udc_lo_frequency else float(udc_lo_frequency)
    udc_sideband = 1 if not udc_sideband else int(udc_sideband)

    dac_tile = int(config_dict['firmware']['dac0_tile'])
    dac_block = int(config_dict['firmware']['dac0_block'])
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    
    #constants
    p = r.pipeline_id
    nc = r.mixer.n_chans
    fft_period_s = r.mixer._n_upstream_chans / r.mixer._upstream_oversample_factor / r.adc_clk_hz
    fft_rbw_hz = 1./fft_period_s
    all_tx_bin_centers_hz = np.fft.fftfreq(2 * N_TX_FFT, 1. / r.adc_clk_hz)
    all_rx_bin_centers_hz = np.fft.fftfreq(N_RX_FFT, 1. / r.adc_clk_hz)
    duc_settings = r.rfdc.core.get_mixer_settings(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
    ddc_settings = r.rfdc.core.get_mixer_settings(adc_tile,adc_block,r.rfdc.core.ADC_TILE)
    dac_nyquist_zone = r.rfdc.core.get_nyquist_zone(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
    adc_nyquist_zone = r.rfdc.core.get_nyquist_zone(adc_tile,adc_block,r.rfdc.core.ADC_TILE)

    #read mixer lo phase_increment values
    phase_inc_tx   = np.frombuffer(r.mixer.read(f'tx_lo{p}_phase_inc',4*nc),dtype='>i4')
    phase_inc_rx   = np.frombuffer(r.mixer.read(f'rx_lo{p}_phase_inc',4*nc),dtype='>i4')
    phase_inc_tx   = _invert_format_phase_steps(phase_inc_tx,r.mixer._phase_bp)
    phase_inc_rx   = _invert_format_phase_steps(phase_inc_rx,r.mixer._phase_bp)
    
    #read mixer lo ri_step values
    ri_steps_tx    = np.frombuffer(r.mixer.read(f'tx_lo{p}_ri_step',4*nc),dtype='>u4')
    ri_steps_rx    = np.frombuffer(r.mixer.read(f'rx_lo{p}_ri_step',4*nc),dtype='>u4')
    ri_steps_tx    = uint2cplx(ri_steps_tx, r.mixer._n_ri_step_bits)
    ri_steps_rx    = uint2cplx(ri_steps_rx, r.mixer._n_ri_step_bits)
    
    #convert to ri_steps to phase angles
    phase_steps_tx = np.angle(ri_steps_tx)
    phase_steps_rx = np.angle(ri_steps_rx)

    #convert to offset frequencies
    offset_freqs_hz_tx    = phase_inc_tx * fft_rbw_hz / 2 / np.pi 
    offset_freqs_hz_rx    = phase_inc_rx * fft_rbw_hz / 2 / np.pi
    offset_freqs_hz_tx_ri    = phase_steps_tx * fft_rbw_hz / 2 / np.pi 
    offset_freqs_hz_rx_ri    = phase_steps_rx * fft_rbw_hz / 2 / np.pi
    
    #get the filterbank channels
    chanmap_psb  = r.psb_chanselect.get_channel_outmap()
    chanmap_pfb  = r.chanselect.get_channel_outmap()
    psb_chans_active = np.nonzero(chanmap_psb+1)[0]
    pfb_chans_active = np.nonzero(chanmap_pfb+1)[0]
    psb_channels = psb_chans_active[np.argsort(chanmap_psb[psb_chans_active])]
    pfb_channels = chanmap_pfb[pfb_chans_active]
    if np.all(chanmap_psb == 2047):
        warnings.warn('Possibly attempting to get frequencies when none are set.')
        psb_channels = np.copy(pfb_channels)

    #get the number of active channels (assumes anything not -1 is a channel)
    num_tones_tx = len(psb_channels)
    num_tones_rx = len(pfb_channels)
    if num_tones_tx != num_tones_rx:
        warnings.warn(f'Number of tones in tx ({num_tones_tx}) and rx ({num_tones_rx}) do not match.')
    
    #get filterbank center frequencies
    tx_bin_centers_hz = all_tx_bin_centers_hz[psb_channels]
    rx_bin_centers_hz = all_rx_bin_centers_hz[pfb_channels]

    #get the digital baseband frequencies
    dbb_freqs_tx = tx_bin_centers_hz + offset_freqs_hz_tx[:num_tones_tx]
    dbb_freqs_rx = rx_bin_centers_hz + offset_freqs_hz_rx[:num_tones_rx]

    #get the analog output/input frequencies
    duc_freqs = dbb_freqs_tx + 1e6*duc_settings['Freq']
    ddc_freqs = dbb_freqs_rx - 1e6*ddc_settings['Freq']

    #adjust for the given selection of nyquist zone.
    if dac_nyquist_zone == 1:
        dac_out_freqs = duc_freqs
    elif dac_nyquist_zone == 2:
        dac_out_freqs = 2*r.adc_clk_hz - duc_freqs
    else:
        raise ValueError(f'Invalid DAC nyquist zone ({dac_nyquist_zone})')
    if adc_nyquist_zone == 1:
        adc_in_freqs = ddc_freqs
    elif adc_nyquist_zone == 2:
        adc_in_freqs = 2*r.adc_clk_hz - ddc_freqs
    else:
        raise ValueError(f'Invalid ADC nyquist zone ({adc_nyquist_zone})')
    
    #get the rf frequencies given any analog up/down conversion
    udc_freqs_tx = udc_lo_frequency + udc_sideband * dac_out_freqs 
    udc_freqs_rx = udc_lo_frequency + udc_sideband * adc_in_freqs    
    
    output_freqs = udc_freqs_tx if udc_connected else dac_out_freqs
    if detailed_output:
        details = {'tx':{},'rx':{}}
        details['tx']['mixer_lo_phase_increment'] = phase_inc_tx.tolist()[:num_tones_tx]
        details['tx']['mixer_lo_ri_step'] = [(i,q) for i,q in zip(ri_steps_tx.real.tolist(),ri_steps_tx.imag.tolist())][:num_tones_tx]
        details['tx']['mixer_lo_phase_step'] = phase_steps_tx.tolist()[:num_tones_tx]
        details['tx']['mixer_lo_offset_freq'] = offset_freqs_hz_tx.tolist()[:num_tones_tx]
        details['tx']['filterbank_center_freq'] = tx_bin_centers_hz.tolist()
        details['tx']['digital_baseband_freq'] = dbb_freqs_tx.tolist()
        details['tx']['analog_output_freq'] = dac_out_freqs.tolist()
        details['tx']['rf_output_freq'] = udc_freqs_tx.tolist()
        details['rx']['mixer_lo_phase_increment'] = phase_inc_rx.tolist()[:num_tones_rx]
        details['rx']['mixer_lo_ri_step'] = [(i,q) for i,q in zip(ri_steps_rx.real.tolist(),ri_steps_rx.imag.tolist())][:num_tones_rx]
        details['rx']['mixer_lo_phase_step'] = phase_steps_rx.tolist()[:num_tones_rx]
        details['rx']['mixer_lo_offset_freq'] = offset_freqs_hz_rx.tolist()[:num_tones_rx]
        details['rx']['filterbank_center_freq'] = rx_bin_centers_hz.tolist()
        details['rx']['digital_baseband_freq'] = dbb_freqs_rx.tolist()
        details['rx']['analog_input_freq'] = adc_in_freqs.tolist()
        details['rx']['rf_input_freq'] = udc_freqs_rx.tolist()
        return output_freqs,details
    else:
        return output_freqs

def prepare_tone_frequency_settings(r, config_dict, tone_frequencies):
    """
    Prepare the tone frequency settings for applying to the RFSOC.

    Given a set of desired RF tone frequencies, the function calculates the
    required DAC/ADC analog frequencies given any analog up/down conversion.
    The required DAC/ADC digital frequencies are then calculated given
    the selected Nyquist zone and the digital baseband frequencies are calculated given
    the RFDC DUC/DDC setting. Finally the filterbank center frequencies and the mixer LO 
    offsets are translated to the formatted channel maps and phase accumulator increments
    and returned for setting in the RFSOC firmware.
    """
    #config
    udc_connected = config_dict['rf_frontend']['connected']
    udc_lo_frequency = config_dict['rf_frontend']['tx_mixer_lo_frequency_hz']
    udc_sideband = config_dict['rf_frontend']['tx_mixer_sideband']
    udc_connected = False if not udc_connected else udc_connected
    udc_lo_frequency = 0 if not udc_lo_frequency else float(udc_lo_frequency)
    udc_sideband = 1 if not udc_sideband else int(udc_sideband)
    dac_tile = int(config_dict['firmware']['dac0_tile'])
    dac_block = int(config_dict['firmware']['dac0_block'])
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])

    #constants
    tone_frequencies = np.atleast_1d(tone_frequencies)
    nc = r.mixer.n_chans
    fft_period_s = r.mixer._n_upstream_chans / r.mixer._upstream_oversample_factor / r.adc_clk_hz
    fft_rbw_hz = 1./fft_period_s
    all_tx_bin_centers_hz = np.fft.fftfreq(2 * N_TX_FFT, 1. / r.adc_clk_hz)
    all_rx_bin_centers_hz = np.fft.fftfreq(N_RX_FFT, 1. / r.adc_clk_hz)
    duc_settings = r.rfdc.core.get_mixer_settings(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
    ddc_settings = r.rfdc.core.get_mixer_settings(adc_tile,adc_block,r.rfdc.core.ADC_TILE)
    dac_nyquist_zone = r.rfdc.core.get_nyquist_zone(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
    adc_nyquist_zone = r.rfdc.core.get_nyquist_zone(adc_tile,adc_block,r.rfdc.core.ADC_TILE)
    chanmap_psb = np.full(r.psb_chanselect.n_chans_out, -1, dtype=int)
    chanmap_pfb  = np.full(r.chanselect.n_chans_out, -1, dtype=int)
    num_tones = len(tone_frequencies)
    channels = np.arange(num_tones)

    #get the DAC/ADC analog frequencies given any analog up/down conversion
    if udc_connected:
        dac_out_freqs = (tone_frequencies - udc_lo_frequency) / udc_sideband
        adc_in_freqs = (tone_frequencies - udc_lo_frequency) / udc_sideband
    else:
        dac_out_freqs = tone_frequencies
        adc_in_freqs = tone_frequencies
    
    #get the DAC/ADC digitial frequencies given the Nyquist zone
    if dac_nyquist_zone == 1:
        duc_freqs = dac_out_freqs
    elif dac_nyquist_zone == 2:
        duc_freqs = 2*r.adc_clk_hz - dac_out_freqs
    else:
        raise ValueError(f'Invalid DAC nyquist zone ({dac_nyquist_zone})')
    if adc_nyquist_zone == 1:
        ddc_freqs = adc_in_freqs
    elif adc_nyquist_zone == 2:
        ddc_freqs = 2*r.adc_clk_hz - adc_in_freqs
    else:
        raise ValueError(f'Invalid ADC nyquist zone ({adc_nyquist_zone})')

    #get the digital baseband frequencies given the DUC/DDC settings
    dbb_freqs_tx = duc_freqs - 1e6*duc_settings['Freq']
    dbb_freqs_rx = ddc_freqs + 1e6*ddc_settings['Freq']

    #check all tones are in within the baseband bandwidth
    txbbmin=np.min(all_tx_bin_centers_hz)
    txbbmax=np.max(all_tx_bin_centers_hz)+fft_rbw_hz
    rxbbmin=np.min(all_rx_bin_centers_hz)
    rxbbmax=np.max(all_rx_bin_centers_hz)+fft_rbw_hz

    if (dbb_freqs_tx > txbbmax).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth')
    if (dbb_freqs_tx < txbbmin).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth')
    if (dbb_freqs_rx > rxbbmax).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth')
    if (dbb_freqs_rx < rxbbmin).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth')

    #get the nearest filterbank center frequencies for each tone
    # Calculate the distance from each frequency to all bin centers
    diff_tx = dbb_freqs_tx[:, np.newaxis] - all_tx_bin_centers_hz
    diff_rx = dbb_freqs_rx[:, np.newaxis] - all_rx_bin_centers_hz
    # Find the index of the minimum squared difference
    tx_nearest_bins = np.argmin(diff_tx**2, axis=-1)
    rx_nearest_bins = np.argmin(diff_rx**2, axis=-1)
    
    #get the offsets between the digital baseband and the filterbank center frequencies
    tx_freq_offsets_hz = diff_tx[np.arange(len(dbb_freqs_tx)), tx_nearest_bins]
    rx_freq_offsets_hz = diff_rx[np.arange(len(dbb_freqs_rx)), rx_nearest_bins]
    
    #get the phase increments and ri steps for the mixer LOs
    phase_incs_tx = tx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    phase_incs_rx = rx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    ri_steps_tx = np.cos(phase_incs_tx) + 1j*np.sin(phase_incs_tx)
    ri_steps_rx = np.cos(phase_incs_rx) + 1j*np.sin(phase_incs_rx)
    
    #format the phase increments and ri steps for the mixer LOs
    phase_incs_tx_formatted = _format_phase_steps(phase_incs_tx,r.mixer._phase_bp)
    phase_incs_rx_formatted = _format_phase_steps(phase_incs_rx,r.mixer._phase_bp)
    ri_steps_tx_formatted = cplx2uint(ri_steps_tx, r.mixer._n_ri_step_bits)
    ri_steps_rx_formatted = cplx2uint(ri_steps_rx, r.mixer._n_ri_step_bits)

    #set the filterbank channel maps
    chanmap_psb[tx_nearest_bins] = channels
    chanmap_pfb[channels] = rx_nearest_bins

    tone_settings_dict = {'phase_incs_tx_formatted':phase_incs_tx_formatted,
                         'phase_incs_rx_formatted':phase_incs_rx_formatted,
                         'ri_steps_tx_formatted':ri_steps_tx_formatted,
                         'ri_steps_rx_formatted':ri_steps_rx_formatted,
                         'chanmap_psb':chanmap_psb,
                         'chanmap_pfb':chanmap_pfb,
                         'num_tones':num_tones}
    
    details = {'tx':{},'rx':{},'num_tones':num_tones}
    details['tx']['digital_baseband_freq'] = dbb_freqs_tx.tolist()
    details['tx']['filterbank_center_freq'] = all_tx_bin_centers_hz[tx_nearest_bins].tolist()
    details['tx']['filterbank_channel_outmap'] = chanmap_psb.tolist()
    details['tx']['freq_offset'] = tx_freq_offsets_hz.tolist()
    details['tx']['mixer_lo_phase_increment'] = phase_incs_tx.tolist()
    details['tx']['mixer_lo_ri_step'] = [(i,q) for i,q in zip(ri_steps_tx.real.tolist(),ri_steps_tx.imag.tolist())]
    details['rx']['digital_baseband_freq'] = dbb_freqs_rx.tolist()
    details['rx']['filterbank_center_freq'] = all_rx_bin_centers_hz[rx_nearest_bins].tolist()
    details['rx']['filterbank_channel_outmap'] = chanmap_pfb.tolist()
    details['rx']['freq_offset'] = rx_freq_offsets_hz.tolist()
    details['rx']['mixer_lo_phase_increment'] = phase_incs_rx.tolist()
    details['rx']['mixer_lo_ri_step'] = [(i,q) for i,q in zip(ri_steps_rx.real.tolist(),ri_steps_rx.imag.tolist())]
    return tone_settings_dict, details

def apply_tone_frequency_settings(r, tone_settings_dict, autosync=True):
    """
    Apply the tone frequency settings to the RFSOC.

    Keys in the dictionary may be:
    'phase_incs_tx', 'phase_incs_rx', 'ri_steps_tx', 'ri_steps_rx', 'chanmap_psb', 'chanmap_pfb', 'num_tones
    """
    phase_incs_tx = tone_settings_dict.get('phase_incs_tx_formatted')
    phase_incs_rx = tone_settings_dict.get('phase_incs_rx_formatted')
    ri_steps_tx   = tone_settings_dict.get('ri_steps_tx_formatted')
    ri_steps_rx   = tone_settings_dict.get('ri_steps_rx_formatted')
    chanmap_psb   = tone_settings_dict.get('chanmap_psb')
    chanmap_pfb   = tone_settings_dict.get('chanmap_pfb')
    num_tones     = tone_settings_dict.get('num_tones')

    if num_tones is None:
        num_tones = max((len(phase_incs_tx),len(phase_incs_tx),len(ri_steps_tx),len(ri_steps_tx),))

    if not chanmap_psb is None:
        r.psb_chanselect.set_channel_outmap(np.copy(chanmap_psb))
    if not chanmap_pfb is None:
        r.chanselect.set_channel_outmap(np.copy(chanmap_pfb))

    for i in range(min(r.mixer._n_parallel_chans, num_tones)):   
        if not phase_incs_tx is None:
            r.mixer.write(f'tx_lo{i}_phase_inc', phase_incs_tx[i::r.mixer._n_parallel_chans].tobytes())
        if not ri_steps_tx is None:
            r.mixer.write(f'tx_lo{i}_ri_step',     ri_steps_tx[i::r.mixer._n_parallel_chans].tobytes())
        if not phase_incs_rx is None:
            r.mixer.write(f'rx_lo{i}_phase_inc', phase_incs_rx[i::r.mixer._n_parallel_chans].tobytes())
        if not ri_steps_rx is None:
            r.mixer.write(f'rx_lo{i}_ri_step',     ri_steps_rx[i::r.mixer._n_parallel_chans].tobytes())
    
    if autosync:
        # time.sleep(autosync_time_delay)
        r.sync.arm_sync(wait=False)
        time.sleep(autosync_time_delay)
        r.sync.sw_sync()

    return

def prepare_tone_frequency_settings_fast(r, config_dict, tone_frequencies,detailed_output=False):
    """
    Prepare the tone frequency settings for applying to the RFSOC.

    Given a set of desired RF tone frequencies, the function calculates the
    required DAC/ADC analog frequencies given any analog up/down conversion.
    The required DAC/ADC digital frequencies are then calculated given
    the selected Nyquist zone and the digital baseband frequencies are calculated given
    the RFDC DUC/DDC setting. Finally the filterbank center frequencies and the mixer LO 
    offsets are translated to the formatted channel maps and phase accumulator increments
    and returned for setting in the RFSOC firmware.
    """
    #config
    udc_connected = config_dict['rf_frontend']['connected']
    udc_lo_frequency = config_dict['rf_frontend']['tx_mixer_lo_frequency_hz']
    udc_sideband = config_dict['rf_frontend']['tx_mixer_sideband']
    udc_connected = False if not udc_connected else udc_connected
    udc_lo_frequency = 0 if not udc_lo_frequency else float(udc_lo_frequency)
    udc_sideband = 1 if not udc_sideband else int(udc_sideband)
    dac_tile = int(config_dict['firmware']['dac0_tile'])
    dac_block = int(config_dict['firmware']['dac0_block'])
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    duc_frequency = config_dict['firmware']['defaults']['dac_duc_mixer_frequency_hz']
    ddc_frequency = config_dict['firmware']['defaults']['adc_ddc_mixer_frequency_hz']
    dac_nyquist_zone = config_dict['firmware']['defaults']['nyquist_zone']
    adc_nyquist_zone = config_dict['firmware']['defaults']['nyquist_zone']

    #constants
    nc = r.mixer.n_chans
    tone_frequencies = np.atleast_1d(tone_frequencies)
    fft_period_s = r.mixer._n_upstream_chans / r.mixer._upstream_oversample_factor / r.adc_clk_hz
    fft_rbw_hz = 1./fft_period_s
    all_tx_bin_centers_hz = np.fft.fftfreq(2 * N_TX_FFT, 1. / r.adc_clk_hz)
    all_rx_bin_centers_hz = np.fft.fftfreq(N_RX_FFT, 1. / r.adc_clk_hz)
    #duc_settings = r.rfdc.core.get_mixer_settings(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
    #ddc_settings = r.rfdc.core.get_mixer_settings(adc_tile,adc_block,r.rfdc.core.ADC_TILE)
    #dac_nyquist_zone = r.rfdc.core.get_nyquist_zone(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
    #adc_nyquist_zone = r.rfdc.core.get_nyquist_zone(adc_tile,adc_block,r.rfdc.core.ADC_TILE)
    chanmap_psb = np.full(r.psb_chanselect.n_chans_out, -1, dtype=int)
    chanmap_pfb  = np.full(r.chanselect.n_chans_out, -1, dtype=int)
    num_tones = len(tone_frequencies)
    channels = np.arange(num_tones)

    #get the DAC/ADC analog frequencies given any analog up/down conversion
    if udc_connected:
        dac_out_freqs = (tone_frequencies - udc_lo_frequency) / udc_sideband
        adc_in_freqs = (tone_frequencies - udc_lo_frequency) / udc_sideband
    else:
        dac_out_freqs = tone_frequencies
        adc_in_freqs = tone_frequencies
    
    #get the DAC/ADC digitial frequencies given the Nyquist zone
    if dac_nyquist_zone == 1:
        duc_freqs = dac_out_freqs
    elif dac_nyquist_zone == 2:
        duc_freqs = 2*r.adc_clk_hz - dac_out_freqs
    else:
        raise ValueError(f'Invalid DAC nyquist zone ({dac_nyquist_zone})')
    if adc_nyquist_zone == 1:
        ddc_freqs = adc_in_freqs
    elif adc_nyquist_zone == 2:
        ddc_freqs = 2*r.adc_clk_hz - adc_in_freqs
    else:
        raise ValueError(f'Invalid ADC nyquist zone ({adc_nyquist_zone})')

    #get the digital baseband frequencies given the DUC/DDC settings
    # dbb_freqs_tx = duc_freqs - 1e6*duc_settings['Freq']
    # dbb_freqs_rx = ddc_freqs + 1e6*ddc_settings['Freq']
    dbb_freqs_tx = duc_freqs - duc_frequency
    dbb_freqs_rx = ddc_freqs + ddc_frequency

    #check all tones are in within the baseband bandwidth
    txbbmin=np.min(all_tx_bin_centers_hz)
    txbbmax=np.max(all_tx_bin_centers_hz)+fft_rbw_hz
    rxbbmin=np.min(all_rx_bin_centers_hz)
    rxbbmax=np.max(all_rx_bin_centers_hz)+fft_rbw_hz

    if (dbb_freqs_tx > txbbmax).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth')
    if (dbb_freqs_tx < txbbmin).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth')
    if (dbb_freqs_rx > rxbbmax).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth')
    if (dbb_freqs_rx < rxbbmin).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth')

    #get the nearest filterbank center frequencies for each tone
    # Calculate the distance from each frequency to all bin centers
    diff_tx = dbb_freqs_tx[:, np.newaxis] - all_tx_bin_centers_hz
    diff_rx = dbb_freqs_rx[:, np.newaxis] - all_rx_bin_centers_hz
    # Find the index of the minimum squared difference
    tx_nearest_bins = np.argmin(diff_tx**2, axis=-1)
    rx_nearest_bins = np.argmin(diff_rx**2, axis=-1)
    #get the offsets between the digital baseband and the filterbank center frequencies
    tx_freq_offsets_hz = diff_tx[channels, tx_nearest_bins]
    rx_freq_offsets_hz = diff_rx[channels, rx_nearest_bins]
    
    #get the phase increments and ri steps for the mixer LOs
    phase_incs_tx = tx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    phase_incs_rx = rx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    ri_steps_tx = np.cos(phase_incs_tx) + 1j*np.sin(phase_incs_tx)
    ri_steps_rx = np.cos(phase_incs_rx) + 1j*np.sin(phase_incs_rx)
    
    #zero pad out to nchans
    phase_incs_tx = np.pad(phase_incs_tx, (0,nc-len(phase_incs_tx)), 'constant', constant_values=(0,0))
    phase_incs_rx = np.pad(phase_incs_rx, (0,nc-len(phase_incs_rx)), 'constant', constant_values=(0,0))
    ri_steps_tx = np.pad(ri_steps_tx, (0,nc-len(ri_steps_tx)), 'constant', constant_values=(0,0))
    ri_steps_rx = np.pad(ri_steps_rx, (0,nc-len(ri_steps_rx)), 'constant', constant_values=(0,0))


    #format the phase increments and ri steps for the mixer LOs
    phase_incs_tx_formatted = _format_phase_steps(phase_incs_tx,r.mixer._phase_bp,fmt='<i4')
    phase_incs_rx_formatted = _format_phase_steps(phase_incs_rx,r.mixer._phase_bp,fmt='<i4')
    ri_steps_tx_formatted = cplx2uint(ri_steps_tx, r.mixer._n_ri_step_bits,fmt='<u4')
    ri_steps_rx_formatted = cplx2uint(ri_steps_rx, r.mixer._n_ri_step_bits,fmt='<u4')

    #set the filterbank channel maps
    chanmap_psb[tx_nearest_bins] = channels
    chanmap_pfb[channels] = rx_nearest_bins

    tone_settings_dict = {'phase_incs_tx_formatted':phase_incs_tx_formatted,
                         'phase_incs_rx_formatted':phase_incs_rx_formatted,
                         'ri_steps_tx_formatted':ri_steps_tx_formatted,
                         'ri_steps_rx_formatted':ri_steps_rx_formatted,
                         'chanmap_psb':chanmap_psb,
                         'chanmap_pfb':chanmap_pfb,
                         'num_tones':num_tones}
    
    if detailed_output:
        details = {'tx':{},'rx':{},'num_tones':num_tones}
        details['tx']['digital_baseband_freq'] = dbb_freqs_tx.tolist()
        details['tx']['filterbank_center_freq'] = all_tx_bin_centers_hz[tx_nearest_bins].tolist()
        details['tx']['filterbank_channel_outmap'] = chanmap_psb.tolist()
        details['tx']['freq_offset'] = tx_freq_offsets_hz.tolist()
        details['tx']['mixer_lo_phase_increment'] = phase_incs_tx.tolist()
        details['tx']['mixer_lo_ri_step'] = [(i,q) for i,q in zip(ri_steps_tx.real.tolist(),ri_steps_tx.imag.tolist())]
        details['rx']['digital_baseband_freq'] = dbb_freqs_rx.tolist()
        details['rx']['filterbank_center_freq'] = all_rx_bin_centers_hz[rx_nearest_bins].tolist()
        details['rx']['filterbank_channel_outmap'] = chanmap_pfb.tolist()
        details['rx']['freq_offset'] = rx_freq_offsets_hz.tolist()
        details['rx']['mixer_lo_phase_increment'] = phase_incs_rx.tolist()
        details['rx']['mixer_lo_ri_step'] = [(i,q) for i,q in zip(ri_steps_rx.real.tolist(),ri_steps_rx.imag.tolist())]
        return tone_settings_dict, details
    else:
        return tone_settings_dict


def prepare_sweep_settings_fast(r, config_dict, sweep_frequencies, detailed_output=False):
    num_points,num_tones = sweep_frequencies.shape
    channels = np.arange(num_tones)
    points = np.arange(num_points)

    #config
    udc_connected = config_dict['rf_frontend']['connected']
    udc_lo_frequency = config_dict['rf_frontend']['tx_mixer_lo_frequency_hz']
    udc_sideband = config_dict['rf_frontend']['tx_mixer_sideband']
    udc_connected = False if not udc_connected else udc_connected
    udc_lo_frequency = 0 if not udc_lo_frequency else float(udc_lo_frequency)
    udc_sideband = 1 if not udc_sideband else int(udc_sideband)
    dac_tile = int(config_dict['firmware']['dac0_tile'])
    dac_block = int(config_dict['firmware']['dac0_block'])
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    duc_frequency = config_dict['firmware']['defaults']['dac_duc_mixer_frequency_hz']
    ddc_frequency = config_dict['firmware']['defaults']['adc_ddc_mixer_frequency_hz']
    dac_nyquist_zone = config_dict['firmware']['defaults']['nyquist_zone']
    adc_nyquist_zone = config_dict['firmware']['defaults']['nyquist_zone']

    #constants
    nc = r.mixer.n_chans
    fft_period_s = r.mixer._n_upstream_chans / r.mixer._upstream_oversample_factor / r.adc_clk_hz
    fft_rbw_hz = 1./fft_period_s
    fft_tx_nbins = 2 * N_TX_FFT
    fft_rx_nbins = N_RX_FFT
    all_tx_bin_centers_hz = np.fft.fftfreq(fft_tx_nbins, 1. / r.adc_clk_hz)
    all_rx_bin_centers_hz = np.fft.fftfreq(fft_rx_nbins, 1. / r.adc_clk_hz)

    chanmap_psb = np.full((num_points,r.psb_chanselect.n_chans_out), -1, dtype=int)
    chanmap_pfb  = np.full((num_points,r.chanselect.n_chans_out), -1, dtype=int)

    skip_chanmap_psb=np.zeros(num_points,dtype=bool)
    skip_chanmap_pfb=np.zeros(num_points,dtype=bool)

    phase_incs_tx_formatted_padded = np.zeros((num_points,nc),dtype='<i4')+32767
    phase_incs_rx_formatted_padded = np.zeros((num_points,nc),dtype='<i4')+32767
    ri_steps_tx_formatted_padded = np.zeros((num_points,nc),dtype='<u4')+65535
    ri_steps_rx_formatted_padded = np.zeros((num_points,nc),dtype='<u4')+65535
    
    #get the DAC/ADC analog frequencies given any analog up/down conversion
    if udc_connected:
        dac_out_freqs = (sweep_frequencies - udc_lo_frequency) / udc_sideband
        adc_in_freqs = (sweep_frequencies - udc_lo_frequency) / udc_sideband
    else:
        dac_out_freqs = sweep_frequencies
        adc_in_freqs = sweep_frequencies
    
    #get the DAC/ADC digitial frequencies given the Nyquist zone
    if dac_nyquist_zone == 1:
        duc_freqs = dac_out_freqs
    elif dac_nyquist_zone == 2:
        duc_freqs = 2*r.adc_clk_hz - dac_out_freqs
    else:
        raise ValueError(f'Invalid DAC nyquist zone ({dac_nyquist_zone})')
    if adc_nyquist_zone == 1:
        ddc_freqs = adc_in_freqs
    elif adc_nyquist_zone == 2:
        ddc_freqs = 2*r.adc_clk_hz - adc_in_freqs
    else:
        raise ValueError(f'Invalid ADC nyquist zone ({adc_nyquist_zone})')
        
    #get the digital baseband frequencies given the DUC/DDC settings
    # dbb_freqs_tx = duc_freqs - 1e6*duc_settings['Freq']
    # dbb_freqs_rx = ddc_freqs + 1e6*ddc_settings['Freq']
    dbb_freqs_tx = duc_freqs - duc_frequency
    dbb_freqs_rx = ddc_freqs + ddc_frequency

    #check all tones are in within the baseband bandwidth
    txbbmin=np.min(all_tx_bin_centers_hz)
    txbbmax=np.max(all_tx_bin_centers_hz)+fft_rbw_hz
    rxbbmin=np.min(all_rx_bin_centers_hz)
    rxbbmax=np.max(all_rx_bin_centers_hz)+fft_rbw_hz

    if (dbb_freqs_tx > txbbmax).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth')
    if (dbb_freqs_tx < txbbmin).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth')
    if (dbb_freqs_rx > rxbbmax).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth')
    if (dbb_freqs_rx < rxbbmin).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth')
    
    # tx_nearest_bins = np.zeros(dbb_freqs_tx.shape,dtype=int)
    # tx_freq_offsets_hz =np.zeros_like(dbb_freqs_tx)
    # for p in points:
    #     for c in channels:
    #         tx_nearest_bins[p,c] = np.argmin(np.abs(dbb_freqs_tx[p,c] - all_tx_bin_centers_hz))
    #         tx_freq_offsets_hz[p,c] = dbb_freqs_tx[p,c] - all_tx_bin_centers_hz[tx_nearest_bins[p,c]]

    # rx_nearest_bins = np.zeros(dbb_freqs_rx.shape,dtype=int)
    # rx_freq_offsets_hz =np.zeros_like(dbb_freqs_rx)
    # for p in points:
    #     for c in channels:
    #         rx_nearest_bins[p,c] = np.argmin(np.abs(dbb_freqs_rx[p,c] - all_rx_bin_centers_hz))
    #         rx_freq_offsets_hz[p,c] = dbb_freqs_rx[p,c] - all_rx_bin_centers_hz[rx_nearest_bins[p,c]]

    # get the nearest filterbank center frequencies for each tone
    tx_nearest_bins =  np.round(np.clip(dbb_freqs_tx/r.adc_clk_hz*fft_tx_nbins,-fft_tx_nbins/2,fft_tx_nbins/2-1)).astype(int)
    tx_neg_bins = tx_nearest_bins<0
    tx_nearest_bins[tx_neg_bins] += fft_tx_nbins
    rx_nearest_bins =  np.round(np.clip(dbb_freqs_rx/r.adc_clk_hz*fft_rx_nbins,-fft_rx_nbins/2,fft_rx_nbins/2-1)).astype(int)
    rx_neg_bins = rx_nearest_bins<0
    rx_nearest_bins[rx_neg_bins] += fft_rx_nbins
        
    # #get the offsets between the digital baseband and the filterbank center frequencies
    tx_freq_offsets_hz = dbb_freqs_tx - tx_nearest_bins/fft_tx_nbins*r.adc_clk_hz
    tx_freq_offsets_hz[tx_neg_bins] += r.adc_clk_hz
    rx_freq_offsets_hz = dbb_freqs_rx - rx_nearest_bins/fft_rx_nbins*r.adc_clk_hz
    rx_freq_offsets_hz[rx_neg_bins] += r.adc_clk_hz

    #get the phase increments and ri steps for the mixer LOs
    phase_incs_tx = tx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    phase_incs_rx = rx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    ri_steps_tx = np.cos(phase_incs_tx) + 1j*np.sin(phase_incs_tx)
    ri_steps_rx = np.cos(phase_incs_rx) + 1j*np.sin(phase_incs_rx)
    # ri_steps_tx = np.exp(1j*phase_incs_tx)
    # ri_steps_rx = np.exp(1j*phase_incs_rx)
    
    #format the phase increments and ri steps for the mixer LOs
    phase_incs_tx_formatted = _format_phase_steps(phase_incs_tx,r.mixer._phase_bp,fmt='<i4')
    phase_incs_rx_formatted = _format_phase_steps(phase_incs_rx,r.mixer._phase_bp,fmt='<i4')
    ri_steps_tx_formatted = cplx2uint(ri_steps_tx, r.mixer._n_ri_step_bits,fmt='<u4')
    ri_steps_rx_formatted = cplx2uint(ri_steps_rx, r.mixer._n_ri_step_bits,fmt='<u4')

    for p in points:
        #zero pad out to nchans for the fast write
        phase_incs_tx_formatted_padded[p,:len(phase_incs_tx_formatted[p])] = phase_incs_tx_formatted[p]
        phase_incs_rx_formatted_padded[p,:len(phase_incs_rx_formatted[p])] = phase_incs_rx_formatted[p]
        ri_steps_tx_formatted_padded[p,:len(ri_steps_tx_formatted[p])] = ri_steps_tx_formatted[p]
        ri_steps_rx_formatted_padded[p,:len(ri_steps_rx_formatted[p])] = ri_steps_rx_formatted[p]

        #set the filterbank channel maps
        chanmap_psb[p,tx_nearest_bins[p]] = channels
        chanmap_pfb[p,channels] = rx_nearest_bins[p]

    for p in points:
        if p==0:
            pass
        if (chanmap_psb[p] == chanmap_psb[p-1]).all():
            skip_chanmap_psb[p]=True
        if (chanmap_pfb[p] == chanmap_pfb[p-1]).all():
            skip_chanmap_pfb[p]=True

    sweep_settings_dict = {'phase_incs_tx_formatted':phase_incs_tx_formatted_padded,
                         'phase_incs_rx_formatted':phase_incs_rx_formatted_padded,
                         'ri_steps_tx_formatted':ri_steps_tx_formatted_padded,
                         'ri_steps_rx_formatted':ri_steps_rx_formatted_padded,
                         'chanmap_psb':chanmap_psb,
                         'chanmap_pfb':chanmap_pfb,
                         'skip_chanmap_psb':skip_chanmap_psb,
                         'skip_chanmap_pfb':skip_chanmap_pfb,
                         'num_tones':num_tones}

    return sweep_settings_dict

def apply_sweep_step_fast(r, r_fast, sweep_settings, step_index, autosync=True):
    phase_incs_tx_formatted = sweep_settings.get('phase_incs_tx_formatted')
    phase_incs_rx_formatted = sweep_settings.get('phase_incs_rx_formatted')
    ri_steps_tx_formatted   = sweep_settings.get('ri_steps_tx_formatted')
    ri_steps_rx_formatted   = sweep_settings.get('ri_steps_rx_formatted')
    chanmap_psb   = sweep_settings.get('chanmap_psb')
    chanmap_pfb   = sweep_settings.get('chanmap_pfb')
    skip_chanmap_psb = sweep_settings.get('skip_chanmap_psb')
    skip_chanmap_pfb = sweep_settings.get('skip_chanmap_pfb')
    num_tones     = sweep_settings.get('num_tones')
    
    c1=not skip_chanmap_psb[step_index]
    c2=not skip_chanmap_pfb[step_index]
    if c1:
        r.psb_chanselect.set_channel_outmap(np.copy(chanmap_psb[step_index]))
        # while not (r.psb_chanselect.get_channel_outmap()==chanmap_psb[step_index]).all():
        #     print('waiting for psb chanmap to update')
        #     time.sleep(0.001)
        # print('psb chanmap updated')
    if c2:
        r.chanselect.set_channel_outmap(np.copy(chanmap_pfb[step_index]))
        # while not (r.chanselect.get_channel_outmap()==chanmap_pfb[step_index]).all():
        #     print('waiting for pfb chanmap to update')
        #     time.sleep(0.001)
        # print('pfb chanmap updated')
    # if c1 or c2:
    #     print('\n\n\nchanmap set\n\n\n')
    #     r.sync.arm_sync(wait=False)
    #     time.sleep(1)
    #     r.sync.sw_sync()



    fast_write_mixer(r_fast, 
                      phase_incs_tx_formatted[step_index],
                        phase_incs_rx_formatted[step_index],
                          ri_steps_tx_formatted[step_index],
                            ri_steps_rx_formatted[step_index])

    if autosync:
        # time.sleep(autosync_time_delay)
        r_fast.sync.arm_sync(wait=False)
        time.sleep(autosync_time_delay)
        r_fast.sync.sw_sync()

    return

def get_bram_addresses_mixer(r_fast):
    phase_addrs_tx = []
    phase_addrs_rx = []
    ri_step_addrs_tx = []
    ri_step_addrs_rx = []
    nbytes = r_fast.mixer._n_serial_chans * 4 # phases in 4 byte words
    for i in range(r_fast.mixer._n_parallel_chans):
        ramname = f'{r_fast.mixer.prefix}tx_lo{i}_phase_inc'
        phase_addrs_tx += [r_fast.mixer.host.transport._get_device_address(ramname)]
        ramname = f'{r_fast.mixer.prefix}rx_lo{i}_phase_inc'
        phase_addrs_rx += [r_fast.mixer.host.transport._get_device_address(ramname)]
        ramname = f'{r_fast.mixer.prefix}tx_lo{i}_ri_step'
        ri_step_addrs_tx += [r_fast.mixer.host.transport._get_device_address(ramname)]
        ramname = f'{r_fast.mixer.prefix}rx_lo{i}_ri_step'
        ri_step_addrs_rx += [r_fast.mixer.host.transport._get_device_address(ramname)]
    bram_addresses_mixer = {'phase_addrs_tx':phase_addrs_tx,
                            'phase_addrs_rx':phase_addrs_rx,
                            'ri_step_addrs_tx':ri_step_addrs_tx,
                            'ri_step_addrs_rx':ri_step_addrs_rx,
                            'nbytes':nbytes}
    return bram_addresses_mixer

def fast_write_mixer(r_fast, phase_incs_tx_formatted,phase_incs_rx_formatted,ri_steps_tx_formatted,ri_steps_rx_formatted):

    if not hasattr(r_fast,'bram_addresses_mixer'):
        r_fast.bram_addresses_mixer = get_bram_addresses_mixer(r_fast)

    phase_addrs_tx = r_fast.bram_addresses_mixer['phase_addrs_tx']
    phase_addrs_rx = r_fast.bram_addresses_mixer['phase_addrs_rx']
    ri_step_addrs_tx = r_fast.bram_addresses_mixer['ri_step_addrs_tx']
    ri_step_addrs_rx = r_fast.bram_addresses_mixer['ri_step_addrs_rx']
    nbytes = r_fast.bram_addresses_mixer['nbytes']
    
    phase_incs_tx_formatted=phase_incs_tx_formatted.reshape(r_fast.mixer._n_parallel_chans, r_fast.mixer._n_serial_chans)
    phase_incs_rx_formatted=phase_incs_rx_formatted.reshape(r_fast.mixer._n_parallel_chans, r_fast.mixer._n_serial_chans)
    ri_steps_tx_formatted=ri_steps_tx_formatted.reshape(r_fast.mixer._n_parallel_chans, r_fast.mixer._n_serial_chans)
    ri_steps_rx_formatted=ri_steps_rx_formatted.reshape(r_fast.mixer._n_parallel_chans, r_fast.mixer._n_serial_chans)
    
    # Seemingly can't write more than 512 bytes in one go.
    # Assume nbytes is a multiple of 512
    # n_write = (nbytes // 512)
    maxwrite=512
    n_write = (nbytes // maxwrite)
    write_idxs = np.arange(n_write)
    readback_delay = 0.00001
    max_retries = 1000
    for i in range(len(phase_addrs_tx)):
        phase_incs_tx_bytes = phase_incs_tx_formatted[i].tobytes()
        phase_incs_rx_bytes = phase_incs_rx_formatted[i].tobytes()
        ri_steps_tx_bytes = ri_steps_tx_formatted[i].tobytes()
        ri_steps_rx_bytes = ri_steps_rx_formatted[i].tobytes()
        for j in write_idxs:
            raw = phase_incs_tx_bytes[j*maxwrite:(j+1)*maxwrite]
            r_fast.mixer.host.transport.axil_mm[phase_addrs_tx[i]+j*maxwrite:phase_addrs_tx[i] +(j+1)*maxwrite] = raw
            time.sleep(readback_delay)
            ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_tx[i]+j*maxwrite:phase_addrs_tx[i] +(j+1)*maxwrite]
            retry_count=0
            while ret!=raw:
                #retry write
                retry_count+=1
                r_fast.mixer.host.transport.axil_mm[phase_addrs_tx[i]+j*maxwrite:phase_addrs_tx[i] +(j+1)*maxwrite] = raw
                time.sleep(readback_delay*retry_count)
                ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_tx[i]+j*maxwrite:phase_addrs_tx[i] +(j+1)*maxwrite]
                if retry_count>max_retries:
                    raise IOError(f'Failed to write phase_incs_tx {j} to BRAM after {max_retries} tries')
        for j in write_idxs:
            raw = phase_incs_rx_bytes[j*maxwrite:(j+1)*maxwrite]
            r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite] = raw
            time.sleep(readback_delay)
            ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite]
            retry_count=0
            while ret!=raw:
                #retry write
                retry_count+=1
                r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite] = raw
                time.sleep(readback_delay*retry_count)
                ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite]
                if retry_count>max_retries:
                    raise IOError(f'Failed to write phase_incs_rx {j} to BRAM after {max_retries} tries')
        for j in write_idxs:
            raw = ri_steps_tx_bytes[j*maxwrite:(j+1)*maxwrite]
            r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite] = raw
            time.sleep(readback_delay)
            ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite]
            retry_count=0
            while ret!=raw:
                #retry write
                retry_count+=1
                r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite] = raw
                time.sleep(readback_delay*retry_count)
                ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite]
                if retry_count>max_retries:
                    raise IOError(f'Failed to write ri_steps_tx {j} to BRAM after {max_retries} tries')
        for j in write_idxs:
            raw = ri_steps_rx_bytes[j*maxwrite:(j+1)*maxwrite]
            r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite] = raw
            time.sleep(readback_delay)
            ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite]
            retry_count=0
            while ret!=raw:
                #retry write
                retry_count+=1
                r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite] = raw
                time.sleep(readback_delay*retry_count)
                ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite]
                if retry_count>max_retries:
                    raise IOError(f'Failed to write ri_steps_rx {j} to BRAM after {max_retries} tries')
            
    #     for j in write_idxs:
    #         raw = phase_incs_rx_bytes[j*maxwrite:(j+1)*maxwrite]
    #         r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite] = raw
    #         time.sleep(0.00001)
    #         ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite]
    #         if ret==raw:
    #             pass #print(f'phase_incs_rx {j:2d} write successful')
    #         else:
    #             # print(f'phase_incs_rx {j:2d} write failed')
    #             for xx in range(10):
    #                 # print('retrying write', xx)
    #                 r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite] = raw
    #                 time.sleep(0.00001)
    #                 ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite]
    #                 if ret==raw:
    #                     # print('retry successful')
    #                     break
    #             if xx==9:
    #                 print('\t\t\t\tretry failed')
            
    #         # while r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*512:phase_addrs_rx[i] +(j+1)*512] != raw:
    #         #     time.sleep(0.00001)
    #         #     r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*512:phase_addrs_rx[i] +(j+1)*512]=raw
                
    #         # r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*512:phase_addrs_rx[i] +(j+1)*512] = phase_incs_rx_bytes[j*512:(j+1)*512]
    #         # # while not (np.frombuffer(r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*512:phase_addrs_rx[i] +(j+1)*512],dtype='<i4').copy() == np.frombuffer(phase_incs_rx_bytes[j*512:(j+1)*512],dtype='<i4').copy()).all():
    #         # #     print('waiting for phase_incs_rx to update')
    #         # #     time.sleep(0.001)
    #         # r_fast.mv_as_int[(ri_step_addrs_tx[i]+j*512)//4:(ri_step_addrs_tx[i] +(j+1)*512)//4] = memoryview(ri_steps_tx_bytes[(j*512):((j+1)*512)]).cast('I')
    #     for j in write_idxs:
    #         raw = ri_steps_tx_bytes[j*maxwrite:(j+1)*maxwrite]
    #         r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite] = raw
    #         time.sleep(0.00001)
    #         ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite]
    #         if ret==raw:
    #             pass #print(f'ri_steps_tx   {j:2d} write successful')
    #         else:
    #             # print(f'ri_steps_tx   {j:2d} write failed')
    #             for xx in range(10):
    #                 # print('retrying write', xx)
    #                 r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite] = raw
    #                 time.sleep(0.00001)
    #                 ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite]
    #                 if ret==raw:
    #                     # print('retry successful')
    #                     break
    #             if xx==9:
    #                 print('\t\t\t\tretry failed')
    #         # while r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*512:ri_step_addrs_tx[i] +(j+1)*512] != raw:
    #         #     time.sleep(0.00001)
    #         #     r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*512:ri_step_addrs_tx[i] +(j+1)*512]=raw
                
    #         # r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*512:ri_step_addrs_tx[i] +(j+1)*512] = ri_steps_tx_bytes[j*512:(j+1)*512]
    #         # # while not (np.frombuffer(r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*512:ri_step_addrs_tx[i] +(j+1)*512],dtype='<i4').copy() == np.frombuffer(ri_steps_tx_bytes[j*512:(j+1)*512],dtype='<i4').copy()).all():
    #         # #     print('waiting for ri_steps_tx to update')
    #         # #     time.sleep(0.001)
    #         # r_fast.mv_as_int[(ri_step_addrs_rx[i]+j*512)//4:(ri_step_addrs_rx[i] +(j+1)*512)//4] = memoryview(ri_steps_rx_bytes[(j*512):((j+1)*512)]).cast('I')
    #     for j in write_idxs:
    #         raw = ri_steps_rx_bytes[j*maxwrite:(j+1)*maxwrite]
    #         r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite] = raw
    #         time.sleep(0.00001)
    #         ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite]
    #         if ret==raw:
    #             pass #print(f'ri_steps_rx   {j:2d} write successful')
    #         else:
    #             # print(f'ri_steps_rx   {j:2d} write failed')
    #             for xx in range(10):
    #                 # print('retrying write', xx)
    #                 r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite] = raw
    #                 time.sleep(0.00001)
    #                 ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite]
    #                 if ret==raw:
    #                     # print('retry successful')
    #                     break
    #             if xx==9:
    #                 print('\t\t\t\tretry failed')
    #         # while r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*512:ri_step_addrs_rx[i] +(j+1)*512] != raw:
    #         #     time.sleep(0.00001)
    #         #     r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*512:ri_step_addrs_rx[i] +(j+1)*512]=raw
                
    #         # r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*512:ri_step_addrs_rx[i] +(j+1)*512] = ri_steps_rx_bytes[j*512:(j+1)*512]
    #         # # while not (np.frombuffer(r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*512:ri_step_addrs_rx[i] +(j+1)*512],dtype='<i4').copy() == np.frombuffer(ri_steps_rx_bytes[j*512:(j+1)*512],dtype='<i4').copy()).all():
    #         # #     print('waiting for ri_steps_rx to update')
    #         # #     time.sleep(0.001)
    # # r_fast.mixer.host.transport.axil_mm.flush()



def apply_tone_frequency_settings_fast(r, r_fast, fast_tone_frequency_settings, autosync=True):
    phase_incs_tx_formatted = fast_tone_frequency_settings.get('phase_incs_tx_formatted')
    phase_incs_rx_formatted = fast_tone_frequency_settings.get('phase_incs_rx_formatted')
    ri_steps_tx_formatted   = fast_tone_frequency_settings.get('ri_steps_tx_formatted')
    ri_steps_rx_formatted   = fast_tone_frequency_settings.get('ri_steps_rx_formatted')
    chanmap_psb   = fast_tone_frequency_settings.get('chanmap_psb')
    chanmap_pfb   = fast_tone_frequency_settings.get('chanmap_pfb')
    # num_tones     = fast_tone_frequency_settings.get('num_tones')
    c1 = chanmap_psb is not None
    c2 = chanmap_pfb is not None

    if c1 is not None:
        # print('chanmap_psb set')
        r.psb_chanselect.set_channel_outmap(np.copy(chanmap_psb))
    if c2 is not None:
        # print('chanmap_pfb set')
        r.chanselect.set_channel_outmap(np.copy(chanmap_pfb))
    # if c1 or c2:
    #     # print('chanmap set')
    #     r.sync.arm_sync(wait=False)
    #     time.sleep(1)
    #     r.sync.sw_sync()

    fast_write_mixer(r_fast,
                      phase_incs_tx_formatted,
                        phase_incs_rx_formatted,
                          ri_steps_tx_formatted,
                            ri_steps_rx_formatted)

    if autosync:
        # time.sleep(autosync_time_delay)
        r_fast.sync.arm_sync(wait=False)
        time.sleep(autosync_time_delay)
        r_fast.sync.sw_sync()


def set_tone_frequencies(r, config_dict, tone_frequencies, autosync=True, detailed_output=False):
    """
    Set the tone frequencies in the RFSOC.
    
    Given a set of desired RF tone frequencies, the function calculates the
    required DAC/ADC analog frequencies given any analog up/down conversion.
    The required DAC/ADC digital frequencies are then calculated given
    the selected Nyquist zone and the digital baseband frequencies are calculated given
    the RFDC DUC/DDC setting. Finally the filterbank center frequencies and the mixer LO 
    offsets are translated to the formatted channel maps and phase accumulator increments
    and written to the RFSOC firmware.

    TODO: account for dual dac mode, for now assume all on dac 0
    
    """
    
    tone_frequency_settings, details = prepare_tone_frequency_settings(r, config_dict, tone_frequencies)
    apply_tone_frequency_settings(r, tone_frequency_settings, autosync=autosync)

    if detailed_output:
        return details
    else:
        return

def set_tone_frequencies_fast(r, r_fast, config_dict, tone_frequencies, autosync=True):
    """
    Set the tone frequencies in the RFSOC using the fast firmware interface.
    
    Given a set of desired RF tone frequencies, the function calculates the
    required DAC/ADC analog frequencies given any analog up/down conversion.
    The required DAC/ADC digital frequencies are then calculated given
    the selected Nyquist zone and the digital baseband frequencies are calculated given
    the RFDC DUC/DDC setting. Finally the filterbank center frequencies and the mixer LO
    offsets are translated to the formatted channel maps and phase accumulator increments
    and written to the fast firmware interface.
    """

    tone_frequency_settings = prepare_tone_frequency_settings_fast(r, config_dict, tone_frequencies)
    apply_tone_frequency_settings_fast(r, r_fast, tone_frequency_settings, autosync=autosync)

    return

# def get_fast_write_params(r, r_fast, config_dict, frequencies):

#     """
#     Get the parameters required to write the mixer LO phase increments, ri steps and 
#     filterbank channel maps for sets of tone frequencies to the fast firmware interface.
#     params:
#     r: firmware interface object
#     r_fast: fast firmware interface object
#     config_dict: configuration dictionary
#     frequencies: tone frequencies, ndarray of shape (n_tones, n_tone_sets))
#     """

#     #config
#     udc_connected = config_dict['rf_frontend']['connected']
#     udc_lo_frequency = config_dict['rf_frontend']['tx_mixer_lo_frequency_hz']
#     udc_sideband = config_dict['rf_frontend']['tx_mixer_sideband']
#     udc_connected = False if not udc_connected else udc_connected
#     udc_lo_frequency = 0 if not udc_lo_frequency else float(udc_lo_frequency)
#     udc_sideband = 1 if not udc_sideband else int(udc_sideband)
#     dac_tile = int(config_dict['firmware']['dac0_tile'])
#     dac_block = int(config_dict['firmware']['dac0_block'])
#     adc_tile = int(config_dict['firmware']['adc_tile'])
#     adc_block = int(config_dict['firmware']['adc_block'])


#     #constants
#     frequencies = np.atleast_1d(frequencies)
#     nc = r.mixer.n_chans
#     fft_period_s = r.mixer._n_upstream_chans / r.mixer._upstream_oversample_factor / r.adc_clk_hz
#     fft_rbw_hz = 1./fft_period_s
#     all_tx_bin_centers_hz = np.fft.fftfreq(2 * N_TX_FFT, 1. / r.adc_clk_hz)
#     all_rx_bin_centers_hz = np.fft.fftfreq(N_RX_FFT, 1. / r.adc_clk_hz)
#     duc_settings = r.rfdc.core.get_mixer_settings(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
#     ddc_settings = r.rfdc.core.get_mixer_settings(adc_tile,adc_block,r.rfdc.core.ADC_TILE)
#     dac_nyquist_zone = r.rfdc.core.get_nyquist_zone(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
#     adc_nyquist_zone = r.rfdc.core.get_nyquist_zone(adc_tile,adc_block,r.rfdc.core.ADC_TILE)
#     chanmap_psb = np.full(r.psb_chanselect.n_chans_out, -1, dtype=int)
#     chanmap_pfb  = np.full(r.chanselect.n_chans_out, -1, dtype=int)
#     num_tones = frequencies.shape[0]
#     channels = np.arange(num_tones)

#     #get the DAC/ADC analog frequencies given any analog up/down conversion
#     if udc_connected:
#         dac_out_freqs = (frequencies - udc_lo_frequency) / udc_sideband
#         adc_in_freqs = (frequencies - udc_lo_frequency) / udc_sideband
#     else:
#         dac_out_freqs = frequencies
#         adc_in_freqs = frequencies
    
#     #get the DAC/ADC digitial frequencies given the Nyquist zone
#     if dac_nyquist_zone == 1:
#         duc_freqs = dac_out_freqs
#     elif dac_nyquist_zone == 2:
#         duc_freqs = 2*r.adc_clk_hz - dac_out_freqs
#     else:
#         raise ValueError(f'Invalid DAC nyquist zone ({dac_nyquist_zone})')
#     if adc_nyquist_zone == 1:
#         ddc_freqs = adc_in_freqs
#     elif adc_nyquist_zone == 2:
#         ddc_freqs = 2*r.adc_clk_hz - adc_in_freqs
#     else:
#         raise ValueError(f'Invalid ADC nyquist zone ({adc_nyquist_zone})')

#     #get the digital baseband frequencies given the DUC/DDC settings
#     dbb_freqs_tx = duc_freqs - 1e6*duc_settings['Freq']
#     dbb_freqs_rx = ddc_freqs + 1e6*ddc_settings['Freq']

#     #check all tones are in within the baseband bandwidth
#     txbbmin=min(all_tx_bin_centers_hz)
#     txbbmax=max(all_tx_bin_centers_hz)+fft_rbw_hz
#     rxbbmin=min(all_rx_bin_centers_hz)
#     rxbbmax=max(all_rx_bin_centers_hz)+fft_rbw_hz

#     if any(dbb_freqs_tx > txbbmax):
#         raise ValueError(f'TX frequencies exceed baseband bandwidth')
#     if any(dbb_freqs_tx < txbbmin):
#         raise ValueError(f'TX frequencies exceed baseband bandwidth')
#     if any(dbb_freqs_rx > rxbbmax):
#         raise ValueError(f'RX frequencies exceed baseband bandwidth')
#     if any(dbb_freqs_rx < rxbbmin):
#         raise ValueError(f'RX frequencies exceed baseband bandwidth')

#     #get the nearest filterbank center frequencies for each tone
#     # Calculate the distance from each frequency to all bin centers
#     diff_tx = dbb_freqs_tx[..., np.newaxis] - all_tx_bin_centers_hz
#     diff_rx = dbb_freqs_rx[..., np.newaxis] - all_rx_bin_centers_hz

#     # Find the index of the minimum squared difference
#     tx_nearest_bins = np.argmin(diff_tx ** 2,  axis=-1)
#     rx_nearest_bins = np.argmin(diff_rx ** 2, axis=-1)
    
#     #get the offsets between the digital baseband and the filterbank center frequencies
#     # tx_freq_offsets_hz = diff_tx[np.arange(len(dbb_freqs_tx)), tx_nearest_bins]
#     # rx_freq_offsets_hz = diff_rx[np.arange(len(dbb_freqs_rx)), rx_nearest_bins]
#     tx_freq_offsets_hz = np.take_along_axis(diff_tx, 
#                                             tx_nearest_bins[..., np.newaxis],
#                                             axis=-1).squeeze(-1)
#     rx_freq_offsets_hz = np.take_along_axis(diff_rx,
#                                             rx_nearest_bins[..., np.newaxis],
#                                             axis=-1).squeeze(-1)

#     #get the phase increments and ri steps for the mixer LOs
#     phase_incs_tx = tx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
#     phase_incs_rx = rx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
#     ri_steps_tx = np.cos(phase_incs_tx) + 1j*np.sin(phase_incs_tx)
#     ri_steps_rx = np.cos(phase_incs_rx) + 1j*np.sin(phase_incs_rx)
    
#     #format the phase increments and ri steps for the mixer LOs
#     phase_incs_tx = _format_phase_steps(phase_incs_tx,r.mixer._phase_bp)
#     phase_incs_rx = _format_phase_steps(phase_incs_rx,r.mixer._phase_bp)
#     ri_steps_tx = cplx2uint(ri_steps_tx, r.mixer._n_ri_step_bits)
#     ri_steps_rx = cplx2uint(ri_steps_rx, r.mixer._n_ri_step_bits)

#     #set the filterbank channel maps
#     chanmap_psb[tx_nearest_bins] = channels
#     chanmap_pfb[channels] = rx_nearest_bins

    
#     fast_write_params={}
#     phase_addrs_tx, phase_addrs_rx, ri_step_addrs_tx, ri_step_addrs_rx, nbytes = get_bram_addresses_mixer(r_fast)
#     fast_write_params['phase_addrs_tx'] = phase_addrs_tx
#     fast_write_params['phase_addrs_rx'] = phase_addrs_rx
#     fast_write_params['ri_step_addrs_tx'] = ri_step_addrs_tx
#     fast_write_params['ri_step_addrs_rx'] = ri_step_addrs_rx
#     fast_write_params['nbytes'] = nbytes


def get_fast_sweep_params(r_fast, config_dict,tone_frequencies):
    pass



def get_tone_amplitudes(r,config_dict,num_tones=None):
    """
    Query the RFSOC for the current tone amplitude scale factors.
    """
    if num_tones is None:
        #get the number of active filterbanck channels (assumes anything not -1 is a channel)
        chanmap_psb  = r.psb_chanselect.get_channel_outmap()
        chanmap_pfb  = r.chanselect.get_channel_outmap()
        psb_chans_active = np.nonzero(chanmap_psb+1)[0]
        pfb_chans_active = np.nonzero(chanmap_pfb+1)[0]
        if np.all(chanmap_psb == 2047):
            warnings.warn('Possibly attempting to get amplitudes when no tones are set.')
            psb_chans_active = np.copy(pfb_chans_active)  
        num_tones_tx = len(psb_chans_active)
        num_tones_rx = len(pfb_chans_active)
        if num_tones_tx != num_tones_rx:
            warnings.warn(f'Number of tones in tx ({num_tones_tx}) and rx ({num_tones_rx}) do not match.')
        num_tones = num_tones_tx
    if num_tones == 0:
        return np.array([],dtype=float)
    
    scaling_tx = np.zeros(num_tones_tx,dtype='>u4')
    scaling_rx = np.zeros(num_tones_rx,dtype='>u4')
    for i in range(min(r.mixer._n_parallel_chans, num_tones)):   
        scaling_tx[i::r.mixer._n_parallel_chans] = np.frombuffer(r.mixer.read(f'tx_lo{i}_scale',4*num_tones_tx),dtype='>u4')
        scaling_rx[i::r.mixer._n_parallel_chans] = np.frombuffer(r.mixer.read(f'rx_lo{i}_scale',4*num_tones_rx),dtype='>u4')
    scaling_tx = _invert_format_amp_scale(scaling_tx, r.mixer._n_scale_bits)
    scaling_rx = _invert_format_amp_scale(scaling_rx, r.mixer._n_scale_bits)
    return scaling_tx[:num_tones]

def set_tone_amplitudes(r, config_dict, tone_amplitudes,autosync=True):
    """
    Set the tone amplitude scale factors in the RFSOC.
    """
    tone_amplitudes = np.atleast_1d(tone_amplitudes)
    num_tones = len(tone_amplitudes)
    scaling = _format_amp_scale(tone_amplitudes, r.mixer._n_scale_bits)
    for i in range(min(r.mixer._n_parallel_chans, num_tones)):   
        r.mixer.write(f'tx_lo{i}_scale', scaling[i::r.mixer._n_parallel_chans].tobytes())
        r.mixer.write(f'rx_lo{i}_scale', scaling[i::r.mixer._n_parallel_chans].tobytes())

    if autosync:
        # time.sleep(autosync_time_delay)
        r.sync.arm_sync(wait=False)
        time.sleep(autosync_time_delay)
        r.sync.sw_sync()

    return

def get_tone_phases(r, config_dict, num_tones=None):
    """
    Query the RFSOC for the current tone phase offsets.
    Note that the returned values are in the range [-pi,pi] regardless of how they were set.
    """
    if num_tones is None:
        #get the number of active filterbanck channels (assumes anything not -1 is a channel)
        chanmap_psb  = r.psb_chanselect.get_channel_outmap()
        chanmap_pfb  = r.chanselect.get_channel_outmap()
        psb_chans_active = np.nonzero(chanmap_psb+1)[0]
        pfb_chans_active = np.nonzero(chanmap_pfb+1)[0] 
        if np.all(chanmap_psb == 2047):
            warnings.warn('Possibly attempting to get phases when no tones are set.')
            psb_chans_active = np.copy(pfb_chans_active)  
        num_tones_tx = len(psb_chans_active)
        num_tones_rx = len(pfb_chans_active)
        if num_tones_tx != num_tones_rx:
            warnings.warn(f'Number of tones in tx ({num_tones_tx}) and rx ({num_tones_rx}) do not match.')
        num_tones = num_tones_tx
    if num_tones == 0:
        return np.array([],dtype=float)
    
    phase_offsets_tx = np.zeros(num_tones_tx,dtype='>i4')
    phase_offsets_rx = np.zeros(num_tones_rx,dtype='>i4')
    for i in range(min(r.mixer._n_parallel_chans, num_tones)):   
        phase_offsets_tx[i::r.mixer._n_parallel_chans] = np.frombuffer(r.mixer.read(f'tx_lo{i}_phase_offset',4*num_tones_tx),dtype='>i4')
        phase_offsets_rx[i::r.mixer._n_parallel_chans] = np.frombuffer(r.mixer.read(f'rx_lo{i}_phase_offset',4*num_tones_rx),dtype='>i4')
    phase_offsets_tx = _invert_format_phase_offsets(phase_offsets_tx, r.mixer._phase_offset_bp)
    phase_offsets_rx = _invert_format_phase_offsets(phase_offsets_rx, r.mixer._phase_offset_bp)
    return phase_offsets_tx[:num_tones]

def set_tone_phases(r, config_dict, tone_phases, autosync=True):
    """
    Set the tone phase offsets in the RFSOC.
    """
    tone_phases = np.atleast_1d(tone_phases)
    num_tones = len(tone_phases)
    phase_offsets = _format_phase_offsets(tone_phases,r.mixer._phase_offset_bp)
    for i in range(min(r.mixer._n_parallel_chans, num_tones)):
        r.mixer.write(f'tx_lo{i}_phase_offset', phase_offsets[i::r.mixer._n_parallel_chans].tobytes())
        r.mixer.write(f'rx_lo{i}_phase_offset', phase_offsets[i::r.mixer._n_parallel_chans].tobytes())
    if autosync:
        # time.sleep(autosync_time_delay)
        r.sync.arm_sync(wait=False)
        time.sleep(autosync_time_delay)
        r.sync.sw_sync()
    return

def check_input_saturation(r,iterations=1,saturation_bits=adc_saturation_bits):
    """
    Check to see if the input ADC is saturating.

    TODO: extend this to check for amplifier saturation
    """
    ss_0 = r.adc_snapshot.get_snapshot() / 2**(saturation_bits-1)
    ss=np.zeros((iterations,ss_0.size),dtype=ss_0.dtype)
    ss[0]=ss_0
    for i in range(1,iterations):
        ss[i]=r.adc_snapshot.get_snapshot() / 2**(saturation_bits-1)
    imax = np.max(ss.real)
    imin = np.min(ss.real)
    qmax = np.max(ss.imag)
    qmin = np.min(ss.imag)
    i_over = imax >= 1.0
    i_under = imin <= -1.0
    q_over = qmax >= 1.0
    q_under = qmin <= -1.0
    any_saturation = bool(i_over|i_under|q_over|q_under)
    integration_time = ss.size/r.adc_clk_hz
    details = {'imax_fs':imax,'imin_fs':imin,'qmax_fs':qmax,'qmin_fs':qmin,
               'integration_time':integration_time}
    return any_saturation, details

def check_output_saturation(r,iterations=1,saturation_bits=dac_saturation_bits):
    """
    Check to see if the output DACs are saturating.

    TODO: extend this to check for amplifier saturation
    """
    #r.input.enable_loopback()
    #any_saturation, details = check_input_saturation(r,iterations=iterations,saturation_bits=saturation_bits)
    #r.input.disable_loopback()
    ss0_0,ss1_0 = r.dac_snapshot.get_snapshot() / 2**(saturation_bits-1)
    ss0=np.zeros((iterations,ss0_0.size),dtype=ss0_0.dtype)
    ss1=np.zeros((iterations,ss1_0.size),dtype=ss1_0.dtype)
    ss0[0]=ss0_0
    ss1[0]=ss1_0
    for i in range(1,iterations):
        ss0[i],ss1[i]=r.dac_snapshot.get_snapshot() / 2**(saturation_bits-1)
    i0max,i1max = np.max(ss0.real),np.max(ss1.real)
    i0min,i1min = np.min(ss0.real),np.min(ss1.real)
    q0max,q1max = np.max(ss0.imag),np.max(ss1.imag)
    q0min,q1min = np.min(ss0.imag),np.min(ss1.imag)
    i0_over,i1_over = i0max >= 1.0, i1max >= 1.0
    i0_under,i1_under = i0min <= -1.0, i1min <= -1.0
    q0_over,q1_over = q0max >= 1.0, q1max >= 1.0
    q0_under,q1_under = q0min <= -1.0, q1min <= -1.0
    any0_saturation = i0_over|i0_under|q0_over|q0_under
    any1_saturation = i1_over|i1_under|q1_over|q1_under
    any_saturation = bool(any0_saturation|any1_saturation)
    integration_time = ss0.size/r.adc_clk_hz
    details = {'i0max_fs':i0max,'i0min_fs':i0min,'q0max_fs':q0max,'q0min_fs':q0min,
               'i1max_fs':i1max,'i1min_fs':i1min,'q1max_fs':q1max,'q1min_fs':q1min,
               'integration_time':integration_time}
    return any_saturation, details

    
def check_dsp_overflow(r,duration_s=0.1):
    """
    Check to see if any of the digital signal processing blocks have overflowed.
    """
    psbscale_overflow0 = r.psbscale.get_overflow_count()
    psb_overflow0 = r.psb.get_overflow_count()
    pfb_overflow0 = r.pfb.get_overflow_count()
    time.sleep(duration_s)
    psbscale_overflow1 = r.psbscale.get_overflow_count()
    psb_overflow1 = r.psb.get_overflow_count()
    pfb_overflow1 = r.pfb.get_overflow_count()
    
    r.psb.reset_overflow_count()
    r.pfb.reset_overflow_count()

    psbscale_delta = psbscale_overflow1 - psbscale_overflow0
    psb_delta = psb_overflow1 - psb_overflow0
    pfb_delta = pfb_overflow1 - pfb_overflow0

    tx_overflow = psb_delta | psbscale_delta
    rx_overflow = pfb_delta

    any_overflow = bool(tx_overflow | rx_overflow)
    details = {'psbscale_ovf_count_start':psbscale_overflow0,
                'psbscale_ovf_count_end':psbscale_overflow1,
                'psbscale_ovf_delta':psbscale_delta,
                'psb_ovf_count_start':psb_overflow0,
                'psb_ovf_count_end':psb_overflow1,
                'psb_ovf_delta':psb_delta,
                'pfb_ovf_count_start':pfb_overflow0,
                'pfb_ovf_count_end':pfb_overflow1,
                'pfb_ovf_delta':pfb_delta}
    return any_overflow, details

def read_accumulated_data(r,num_tones=None):
    """
    Read one sample of accumulated data from the RFSOC using the slower CASPER interface.
    """
    data = r.accumulators[0].get_new_spectra()
    if num_tones is None:
        return data
    else:
        return data[:num_tones]

def get_fast_read_params(r_fast):
    """
    Get the parameters required to perform fast readout of the RFSOC.
    """
    acc = r_fast.accumulators[0]
    nbytes = acc._n_serial_chans * np.dtype(acc._dtype).itemsize
    if acc._is_complex:
        nbytes *= 2
    addrs = [acc.host.transport._get_device_address(f'{acc.prefix}dout{i}') for i in range(acc._n_parallel_chans)]
    for i in range(1,acc._n_parallel_chans):
        assert addrs[i] == addrs[i-1] + nbytes
    nbranch = len(addrs)
    params = {'acc':acc,
              'addrs':addrs,
              'nbytes':nbytes,
              'nbranch':nbranch,
              'base_addr':addrs[0]}
    
    return params

def read_accumulated_data_fast(fast_read_params,num_tones=None):
    """
    Read one sample of accumulated data from the RFSOC 
    utilising the faster katcp local memory transport.
    """
    acc=fast_read_params['acc']
    addrs=fast_read_params['addrs']
    nbytes=fast_read_params['nbytes']
    nbranch=fast_read_params['nbranch']
    base_addr=fast_read_params['base_addr']
    err = False


    # acc._wait_for_acc(0.00001)
    start_acc_cnt = _blocking_wait_for_acc(acc,0.00001)

    if nbranch==1:
        raw = acc.host.transport.axil_mm[base_addr:base_addr + nbytes]
        dout = np.frombuffer(raw, dtype='<i4')
    else:
        dout = np.zeros(2*acc.n_chans, dtype='<i4') # 2*4 bytes for real+imag
        for i in range(nbranch):
            raw = acc.host.transport.axil_mm[addrs[i]:addrs[i] + nbytes]
            dout[i::nbranch] = np.frombuffer(raw, dtype='<i4')
    stop_acc_cnt = acc.get_acc_cnt()
    if start_acc_cnt != stop_acc_cnt:
        acc.logger.warning('Accumulation counter changed while reading data!')
        err=True

    if num_tones is None:
        return start_acc_cnt, dout, err
    else:
        return start_acc_cnt, dout[:2*num_tones], err
    

def perform_sweep(r, r_fast, config_dict, centers, spans, points, samples_per_point, direction):
    """
    A blocking call to perform a frequency sweep of the RFSOC.
    An asynchronous version of this function is available in the readout_server code.
     
    """
    centers = np.atleast_1d(centers)
    spans = np.atleast_1d(spans)
    if len(spans)==1:
        spans = np.full(len(centers),spans[0])
    assert len(centers) == len(spans)
    num_points=int(points)
    samples_per_point=int(samples_per_point)
    assert direction in ('up','down')

    num_tones = len(centers) 
    channels = np.arange(num_tones,dtype=int)
    sweepfreqs = np.zeros((num_tones,num_points),dtype=float)
    for t in range(num_tones):
        cf=centers[t]
        sp=spans[t]
        sweepfreqs[t] = np.linspace(cf-sp/2.,cf+sp/2.,num_points)
        if direction=='down':
            sweepfreqs[t] = sweepfreqs[t][::-1]

    acc_counts = np.zeros((num_points,samples_per_point),dtype=int)
    sweep_data = np.zeros((num_tones,num_points,samples_per_point),dtype=complex)
    acc_errs = np.zeros((num_points,samples_per_point),dtype=bool)

    initial_freqs = get_tone_frequencies(r, config_dict)
    if len(initial_freqs)==0:
        initial_freqs = centers

    fast_read_params = get_fast_read_params(r_fast) 
    for p in range(num_points):
        set_tone_frequencies(r,
                             config_dict,
                             sweepfreqs[:,p],
                             autosync=True)
        
        for s in range(samples_per_point):
            cnt,data,err = read_accumulated_data_fast(r_fast,
                                                      fast_read_params,
                                                      num_tones=num_tones)
            acc_counts[p,s] = cnt
            sweep_data[:,p,s] = data[::2]+1j*data[1::2]
            acc_errs[p,s] = err

    set_tone_frequencies(r,config_dict,initial_freqs,autosync=True)
    
    sweep_responses = np.mean(sweep_data.real,axis=1) + 1j*np.mean(sweep_data.imag,axis=1)
    sweep_stds = np.std(sweep_data.real,axis=1) + 1j*np.std(sweep_data.imag,axis=1)

    results = {
        'sweep_frequencies': sweepfreqs,
        'sweep_responses': sweep_responses,
        'sweep_stds': sweep_stds,
        'samples_per_point': samples_per_point,
        'samples_per_second': get_sample_rate(r_fast),
        'accumulation_counts': acc_counts,
        'accumulation_errors': acc_errs
        }
    return results 

def perform_retune(r, r_fast,config_dict, centers, spans, points, samples_per_point, direction, method,smooth_len=3):
    """
    A blocking call to perform a frequency retune of the RFSOC.
    SImply performs a sweep and then retunes to the frequencies of maximum gradient or minimum magnitude.
    An asynchronous version of this function is available in the readout_server code.
    """
    
    #results = r.retune(center, span, points, samples_per_point,direction,method)
    if method not in ('max_gradient','min_mag'):
        raise ValueError(f'Invalid retune method "{method}", must be "max_gradient" or "min_mag"')
    results = perform_sweep(r,r_fast,centers, spans, points, samples_per_point, direction)
    
    if method == 'max_gradient':
        retune_freqs = np.zeros_like(results['sweep_frequencies'])
        for t in range(len(centers)):
            freqs = results['sweep_frequencies'][t]
            grads = np.abs(np.gradient(results['sweep_responses'][t]))
            max_grad = np.argmax(grads)
            retune_freqs[t] = freqs[max_grad]
    elif method == 'min_mag':
        retune_freqs = np.zeros_like(results['sweep_frequencies'])
        for t in range(len(centers)):
            freqs = results['sweep_frequencies'][t]
            mags = np.abs(results['sweep_responses'][t])
            min_mag = np.argmin(mags)
            retune_freqs[t] = freqs[min_mag]

    set_tone_frequencies(r,config_dict,retune_freqs)
    results['retune_freqs'] = retune_freqs

    return results



def wait_for_gpio_pulse(r, gpio_pin,fake_trigger_event=None):
    """
    Wait for a trigger signal to be detected on the given GPIO pin.
    """

    trigger0 = r.accumulators[0].read_gpio_counter(gpio_pin)
    #print(f'Waiting for trigger (GPIO_{gpio_pin}) to change, currently {trigger0}')
    while True:
        trigger1 = r.accumulators[0].read_gpio_counter(gpio_pin)
        if trigger1 != trigger0:
            break
        _blocking_sleep(0.00001)
        if fake_trigger_event is not None:
            if fake_trigger_event.is_set():
                fake_trigger_event.clear()
                print('Fake trigger signal detected, exiting wait.')
                return True
    print(f'Trigger (GPIO_{gpio_pin}) changed, now {trigger1}, delta = {trigger1-trigger0}')
    return True

def set_cal_freeze(r,config_dict,freeze):
    """
    Set the adc calibration freeze state in the RFSOC.
    """
    try:
        freeze = int(bool(freeze))
    except ValueError:
        raise ValueError(f'Invalid freeze value ({freeze}), must be boolean convertible')
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    r.rfdc.core.set_cal_freeze(adc_tile,adc_block,freeze)
    return

def get_cal_freeze(r,config_dict):
    """
    Get the adc calibration freeze state in the RFSOC.
    """
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    freeze = r.rfdc.core.get_cal_freeze(adc_tile,adc_block)
    print('freeze',freeze)
    return bool(int(freeze['CalFrozen']))

def get_tone_powers(r,config_dict,detailed_output=False):
    
    dac_tile = int(config_dict['firmware']['dac0_tile'])
    dac_block = int(config_dict['firmware']['dac0_block'])
    
    #get live params from firmware
    amps=get_tone_amplitudes(r,config_dict)
    freqs,freq_details=get_tone_frequencies(r,config_dict,detailed_output=True)
    psb_fftshift=r.psb.get_fftshift()
    psb_scale=r.psbscale.get_scale()
    mixer_settings=r.rfdc.core.get_mixer_settings(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
    mixer_scale_is_1p0 = mixer_settings['FineMixerScale'] == r.rfdc.core.MIX_SCALE_1P0
    qmc_settings = r.rfdc.core.get_qmc_settings(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
    mixer_qmc_gain = qmc_settings['GainCorrectionFactor'] if qmc_settings['EnableGain'] else 1.0
    vop_current = int(r.rfdc.core.get_output_current(dac_tile,dac_block)['current'])
    
    #get fixed params from config
    dac_fs_bits = config_dict['firmware']['dac_fullscale_bits']
    vop_current_fs = config_dict['firmware']['vop_current_fullscale']
    dac_dbfs_to_dbm = config_dict['firmware']['dac0_dbfs_to_dbm']
    tx_combiner_loss_db = config_dict['rf_frontend']['tx_combiner_loss_db']
    tx_attenuator_value_db = config_dict['rf_frontend']['tx_attenuator_value_db']
    tx_if_s21_db = config_dict['rf_frontend']['tx_if_s21_db']
    tx_mixer_conversion_loss_db = config_dict['rf_frontend']['tx_mixer_conversion_loss_db']
    tx_rf_s21_db = config_dict['rf_frontend']['tx_rf_s21_db']
    cryostat_input_s21_db = config_dict['cryostat']['input_s21_db']
    cryostat_connected = config_dict['cryostat']['connected']
    rf_frontend_connected = config_dict['rf_frontend']['connected']

    if dac_dbfs_to_dbm is None:
        #set defaults if not provided in config
        dac_dbfs_to_dbm = 0
    elif isinstance(dac_dbfs_to_dbm,str):
        #perform nearest neighbour interpolation on calibration data if filenames stored in config file directly
        cal_f,cal_db = np.loadtxt(dac_dbfs_to_dbm,ndmin=2).T
        dac_dbfs_to_dbm = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(dac_dbfs_to_dbm):
        #perform nearest neighbour interpolation on calibration data if arrays stored in config file directly
        cal_f,cal_db = np.array(dac_dbfs_to_dbm,ndmin=2).T
        dac_dbfs_to_dbm = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_combiner_loss_db is None:
        tx_combiner_loss_db = 0
    elif isinstance(tx_combiner_loss_db,str):
        cal_f,cal_db = np.loadtxt(tx_combiner_loss_db,ndmin=2).T
        tx_combiner_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_combiner_loss_db):
        cal_f,cal_db = np.array(tx_combiner_loss_db,ndmin=2).T
        tx_combiner_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_attenuator_value_db is None:
        tx_attenuator_value_db = 0

    if tx_if_s21_db is None:
        tx_if_s21_db = 0
    elif isinstance(tx_if_s21_db,str):
        cal_f,cal_db = np.loadtxt(tx_if_s21_db,ndmin=2).T
        tx_if_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_if_s21_db):
        cal_f,cal_db = np.array(tx_if_s21_db,ndmin=2).T
        tx_if_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_mixer_conversion_loss_db is None:
        tx_mixer_conversion_loss_db = 0
    elif isinstance(tx_mixer_conversion_loss_db,str):
        cal_f,cal_db = np.loadtxt(tx_mixer_conversion_loss_db,ndmin=2).T
        tx_mixer_conversion_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_mixer_conversion_loss_db):
        cal_f,cal_db = np.array(tx_mixer_conversion_loss_db,ndmin=2).T
        tx_mixer_conversion_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_rf_s21_db is None:
        tx_rf_s21_db = 0
    elif isinstance(tx_rf_s21_db,str):
        cal_f,cal_db = np.loadtxt(tx_rf_s21_db,ndmin=2).T
        tx_rf_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])
    elif not np.isscalar(tx_rf_s21_db):
        cal_f,cal_db = np.array(tx_rf_s21_db,ndmin=2).T
        tx_rf_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])
    
    if cryostat_input_s21_db is None:
        cryostat_input_s21_db = 0
    elif isinstance(cryostat_input_s21_db,str):
        cal_f,cal_db = np.loadtxt(cryostat_input_s21_db,ndmin=2).T
        cryostat_input_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])
    elif not np.isscalar(cryostat_input_s21_db):
        cal_f,cal_db = np.array(cryostat_input_s21_db,ndmin=2).T
        cryostat_input_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])

    if not rf_frontend_connected:
        tx_combiner_loss_db = np.zeros_like(freqs)
        tx_attenuator_value_db = np.zeros_like(freqs)
        tx_if_s21_db = np.zeros_like(freqs)
        tx_mixer_conversion_loss_db = np.zeros_like(freqs)
        tx_rf_s21_db = np.zeros_like(freqs)
    if not cryostat_connected:
        cryostat_input_s21_db = np.zeros_like(freqs)
    
    powers,details = calibration.calc_tone_powers(amps,
                                        psb_fftshift,
                                        psb_scale,
                                        mixer_scale_is_1p0,
                                        mixer_qmc_gain,
                                        vop_current,
                                        vop_current_fs,
                                        dac_dbfs_to_dbm,
                                        tx_combiner_loss_db,
                                        tx_attenuator_value_db,
                                        tx_if_s21_db,
                                        tx_mixer_conversion_loss_db,
                                        tx_rf_s21_db,
                                        cryostat_input_s21_db,
                                        dac_fs_bits,
                                        detailed_output=True)
    if detailed_output:
        return powers,details
    else:
        return powers
    
def set_tone_powers(r,config_dict,powers_dbm):
    """
    Set the tone powers .
    """
    powers_dbm = np.atleast_1d(powers_dbm)
    
    dac_tile = int(config_dict['firmware']['dac0_tile'])
    dac_block = int(config_dict['firmware']['dac0_block'])
    
    #get live params from firmware
    freqs,freq_details=get_tone_frequencies(r,config_dict,detailed_output=True)
    psb_fftshift=r.psb.get_fftshift()
    psb_scale=r.psbscale.get_scale()
    mixer_settings=r.rfdc.core.get_mixer_settings(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
    mixer_scale_is_1p0 = mixer_settings['FineMixerScale'] == r.rfdc.core.MIX_SCALE_1P0
    qmc_settings = r.rfdc.core.get_qmc_settings(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
    mixer_qmc_gain = qmc_settings['GainCorrectionFactor'] if qmc_settings['EnableGain'] else 1.0
    vop_current = int(r.rfdc.core.get_output_current(dac_tile,dac_block)['current'])
    
    #get fixed params from config
    dac_fs_bits = config_dict['firmware']['dac_fullscale_bits']
    vop_current_fs = config_dict['firmware']['vop_current_fullscale']
    dac_dbfs_to_dbm = config_dict['firmware']['dac0_dbfs_to_dbm']
    tx_combiner_loss_db = config_dict['rf_frontend']['tx_combiner_loss_db']
    tx_attenuator_value_db = config_dict['rf_frontend']['tx_attenuator_value_db']
    tx_if_s21_db = config_dict['rf_frontend']['tx_if_s21_db']
    tx_mixer_conversion_loss_db = config_dict['rf_frontend']['tx_mixer_conversion_loss_db']
    tx_rf_s21_db = config_dict['rf_frontend']['tx_rf_s21_db']
    cryostat_input_s21_db = config_dict['cryostat']['input_s21_db']
    cryostat_connected = config_dict['cryostat']['connected']
    rf_frontend_connected = config_dict['rf_frontend']['connected']

    if dac_dbfs_to_dbm is None:
        #set defaults if not provided in config
        dac_dbfs_to_dbm = 0
    elif isinstance(dac_dbfs_to_dbm,str):
        #perform nearest neighbour interpolation on calibration data if filenames stored in config file directly
        cal_f,cal_db = np.loadtxt(dac_dbfs_to_dbm,ndmin=2).T
        dac_dbfs_to_dbm = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(dac_dbfs_to_dbm):
        #perform nearest neighbour interpolation on calibration data if arrays stored in config file directly
        cal_f,cal_db = np.array(dac_dbfs_to_dbm,ndmin=2).T
        dac_dbfs_to_dbm = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_combiner_loss_db is None:
        tx_combiner_loss_db = 0
    elif isinstance(tx_combiner_loss_db,str):
        cal_f,cal_db = np.loadtxt(tx_combiner_loss_db,ndmin=2).T
        tx_combiner_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_combiner_loss_db):
        cal_f,cal_db = np.array(tx_combiner_loss_db,ndmin=2).T
        tx_combiner_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_attenuator_value_db is None:
        tx_attenuator_value_db = 0

    if tx_if_s21_db is None:
        tx_if_s21_db = 0
    elif isinstance(tx_if_s21_db,str):
        cal_f,cal_db = np.loadtxt(tx_if_s21_db,ndmin=2).T
        tx_if_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_if_s21_db):
        cal_f,cal_db = np.array(tx_if_s21_db,ndmin=2).T
        tx_if_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_mixer_conversion_loss_db is None:
        tx_mixer_conversion_loss_db = 0
    elif isinstance(tx_mixer_conversion_loss_db,str):
        cal_f,cal_db = np.loadtxt(tx_mixer_conversion_loss_db,ndmin=2).T
        tx_mixer_conversion_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_mixer_conversion_loss_db):
        cal_f,cal_db = np.array(tx_mixer_conversion_loss_db,ndmin=2).T
        tx_mixer_conversion_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_rf_s21_db is None:
        tx_rf_s21_db = 0
    elif isinstance(tx_rf_s21_db,str):
        cal_f,cal_db = np.loadtxt(tx_rf_s21_db,ndmin=2).T
        tx_rf_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])
    elif not np.isscalar(tx_rf_s21_db):
        cal_f,cal_db = np.array(tx_rf_s21_db,ndmin=2).T
        tx_rf_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])
    
    if cryostat_input_s21_db is None:
        cryostat_input_s21_db = 0
    elif isinstance(cryostat_input_s21_db,str):
        cal_f,cal_db = np.loadtxt(cryostat_input_s21_db,ndmin=2).T
        cryostat_input_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])
    elif not np.isscalar(cryostat_input_s21_db):
        cal_f,cal_db = np.array(cryostat_input_s21_db,ndmin=2).T
        cryostat_input_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])


    if not rf_frontend_connected:
        tx_combiner_loss_db = np.zeros_like(freqs)
        tx_attenuator_value_db = np.zeros_like(freqs)
        tx_if_s21_db = np.zeros_like(freqs)
        tx_mixer_conversion_loss_db = np.zeros_like(freqs)
        tx_rf_s21_db = np.zeros_like(freqs)
    if not cryostat_connected:
        cryostat_input_s21_db = np.zeros_like(freqs)
    

    amps,amp_details = calibration.calc_tone_amplitudes(powers_dbm=powers_dbm,
                                            psb_fftshift=psb_fftshift, 
                                            psb_scale=psb_scale,
                                            mixer_scale_is_1p0=mixer_scale_is_1p0,
                                            mixer_qmc_gain=mixer_qmc_gain,
                                            vop_current=vop_current,
                                            vop_current_fs=vop_current_fs,
                                            dac_dbfs_to_dbm=dac_dbfs_to_dbm,
                                            tx_combiner_loss_db=tx_combiner_loss_db,
                                            tx_attenuator_value_db=tx_attenuator_value_db,
                                            tx_if_s21_db=tx_if_s21_db,
                                            tx_mixer_conversion_loss_db=tx_mixer_conversion_loss_db,
                                            tx_rf_s21_db=tx_rf_s21_db,
                                            cryostat_input_s21_db=cryostat_input_s21_db,
                                            dac_fs_bits=dac_fs_bits,
                                            detailed_output=True)
    
    set_tone_amplitudes(r,config_dict,amps)

    return amp_details 






#include private functions when import * for debugging, to be removed later
__all__ = list(globals().keys())