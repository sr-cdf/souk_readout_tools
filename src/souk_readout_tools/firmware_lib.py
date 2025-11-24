"""
This module provides a set of functions that are used by the readout server to interact with the SOUK firmware.


"""

import warnings
import numpy as np
import time
import os
import yaml
import struct
#import asyncio

try:
    import souk_mkid_readout
    from souk_mkid_readout.souk_mkid_readout import *
except ImportError:
    print("firmware_lib.py: Warning: Importing firmware lib without souk_mkid_readout support.")
    
from souk_readout_tools import calibration

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

USER_DIR = os.path.expanduser('~/.souk_readout_tools/')

autosync_time_delay = 0.001 #seconds

adc_saturation_bits = 16 # note the adc gives 16 bit data but is a 12 or 14 bit converter
dac_saturation_bits = 16 # note the dac takes 16 bit data but is a 12 or 14 bit converter

def cplx2uint(d,nbits):
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
    return ((real << nbits) + imag)

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

        :param fmt: format of the integer to be written to firmware, 
            use >u4 for standard interface or <u4 for fast local mmap-based interface
        :type fmt: str
        
        :return: phase_int -- the integers to be written
            to firmware. Is either an integer (if `phase'
            is an integer. Else an array of integers.)
        :rtype: int (or array(dtype=<fmt>))    
     """
    phase_scaled = phase / np.pi
    phase_scaled = ((phase_scaled + 1) % 2) - 1
    phase_int = (phase_scaled * (2**phase_bp))
    return phase_int.astype(fmt)

def _format_phase_offsets(phase_offsets, phase_offset_bp,fmt='>i4'):
    """
    Vectorised: Given a desired phase offset, format as appropriate
        integers which are interpretable by the mixer firmware

        :param phase_offset: phase offset[s] of tones, in radians
        :type phase_offset: float, or array of floats

        :param phase_offset_bp: binary points of the phase offset
        :type phase_offset_bp: int

        :param fmt: format of the integer to be written to firmware, 
            use >u4 for standard interface or <u4 for fast local mmap-based interface
        :type fmt: str

        :return: phase_offset_int -- the integers to be written
            to firmware. Is either an integer (if `phase_offset'
            is an integer. Else an array of integers.
        :rtype: int (or array(dtype=<fmt>))
     """
    phase_offset_scaled = phase_offsets / np.pi
    phase_offset_scaled = ((phase_offset_scaled + 1) % 2) - 1
    phase_offset_int = (phase_offset_scaled * (2**phase_offset_bp))
    return phase_offset_int.astype(fmt)

def _format_ri_steps(ri_steps, ri_step_bp,fmt='>u4'):
    """
    Vectorised: Given a desired RI step, format as appropriate
        integers which are interpretable by the mixer firmware

        :param ri_steps: RI step[s] to step per parallel sample, in radians
        :type ri_steps: float, or array of floats

        :param ri_step_bp: binary points of the RI step
        :type ri_step_bp: int

        :param fmt: format of the integer to be written to firmware, 
            use >u4 for standard interface or <u4 for fast local mmap-based interface
        :type fmt: str

        :return: ri_steps_int -- the integers to be written
            to firmware. Is either an integer (if `ri_steps'
            is an integer. Else an array of integers.)
        :rtype: int (or array(dtype=<fmt>))
    """
    return  cplx2uint(ri_steps, ri_step_bp).astype(fmt)


def _format_amp_scale(amplitude_scale_factors,n_scale_bits,fmt='>u4'):
    """
    Vectorised:    Given a desired scale factor, format as an appropriate
        integer which is interpretable by the mixer firmware.

        :param v: Scale factors to apply to the tones
        :type v: array of floats

        :param n_scale_bits: Number of bits to use for the scale factor
        :type n_scale_bits: int

        :param fmt: format of the integer to be written to firmware, 
            use >u4 for standard interface or <u4 for fast local mmap-based interface
        :type fmt: str


        :return: Integer scale[s]
        :rtype: array of ints
    """
    v = amplitude_scale_factors * 2**n_scale_bits
    v = np.round(v).astype(int)
    # saturate
    scale_max = 2**n_scale_bits - 1
    v[v > scale_max] = scale_max
    return v.astype(fmt)



def _invert_format_phase_steps(phase_int,phase_bp,fmt='>i4'):
    """
    Vectorised: Given a phase step integer, or array of integers, as read from the firmware,
        invert the formatting applied by `_format_phase_steps'
        fmt should be '>i4' with the standard firmware interface or '<i4' with the fast mmap-ed interface
    """
    phase_scaled = phase_int.view(fmt).astype(float) / (2**phase_bp)
    #dont need to invert this: phase_scaled = ((phase_scaled + 1) % 2) - 1
    phase = phase_scaled * np.pi
    return phase

def _invert_format_phase_offsets(phase_offset_int,phase_offset_bp,fmt='>i4'):
    """
    Vectorised: Given a phase offset integer or array of integers, as read from the firmware,
        invert the formatting applied by `_format_phase_offsets'
        fmt should be '>i4' with the standard firmware interface or '<i4' with the fast mmap-ed interface

    """
    phase_offset_scaled = phase_offset_int.view(fmt).astype(float) / (2**phase_offset_bp)
    #dont need to invert this: phase_offset_scaled = ((phase_offset_scaled + 1) % 2) - 1
    phase_offset = phase_offset_scaled * np.pi
    return phase_offset

def _invert_format_ri_steps(ri_steps_int, ri_step_bp, fmt='>u4'):
    """
    Vectorised: Given a RI step integer or array of integers, as read from the firmware,
        invert the formatting applied by `_format_ri_steps'
        fmt should be '>u4' with the standard firmware interface or '<u4' with the fast mmap-ed interface
    """
    ri_steps = uint2cplx(ri_steps_int.view(fmt), ri_step_bp)
    return ri_steps

def _invert_format_amp_scale(scale_factors_int,n_scale_bits,fmt='>u4'):
    """
    Vectorised: Given a scale factor integer or array of integers, as read from the firmware,
        invert the formatting applied by `_format_amp_scale'
        fmt should be '>u4' with the standard firmware interface or '<u4' with the fast mmap-ed interface
    """
    scale_factors = scale_factors_int.view(fmt).astype(float) / 2**n_scale_bits
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

def needs_programming(r,config_dict):
    
    currentfpg = r.fpgfile
    try:
        currentfpg = os.readlink(currentfpg)
    except OSError:
        pass
    with open(config_dict['firmware']['fw_config_file'],'r') as file:
        newfpg = yaml.safe_load(file)['fpgfile']
    newfpg = newfpg.replace('../','').replace('./','/home/casper/src/souk-firmware/')
    try:
        newfpg = os.readlink(newfpg)
    except OSError:
        pass
        
    newfpg = os.path.basename(newfpg)
    currentfpg = os.path.basename(currentfpg)

    print('************************************************')
    print('needs_programming?')
    print('current fpg:',currentfpg)
    print('new fpg:',newfpg)
    print('************************************************')

    if newfpg != currentfpg:
        print('yes, current fpg is not the requested one')
        return True
    
    if not hasattr(r, 'accumulators'):
        print('yes, accumulators not found')
        return True
    
    if not r.fpga.is_programmed():
        print('yes, FPGA is not programmed')
        return True
    print('no')
    return False

def needs_initialising(r,config_dict):
    """
    Check if the firmware needs to be initialised.
    """
    print('************************************************')
    print('needs_initialising?')
    print('************************************************')

    if not hasattr(r, 'accumulators'):
        print('yes, accumulators not found')
        return True
    print('no')
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
    info['fpg_file'] = r.fpgfile
    info['pipeline_id'] = r.pipeline_id
    info['adc_clk_hz'] = r.adc_clk_hz
    info['output_mode'] = r.output.get_status()[0]['mode']
    info['sync_delay'] = r.sync.get_delay()
    info['acc_len'] = r.accumulators[0].get_acc_len()
    info['acc_freq'] = get_sample_rate(r)
    info['internal_loopback'] = r.input.loopback_enabled()
    info['psb_scale'] = r.psbscale.get_scale()
    info['psb_fftshift'] = r.psb.get_fftshift()
    info['pfb_fftshift'] = r.pfb.get_fftshift()

    #rfdc info will be missing keys if the dac and adc tiles/blocks are not set to match those in the firmware
    #lets check we can read dsa on the adc and vop on each dac before trying to read them

    correct_adc = r.rfdc.core.get_dsa(adc_tile,adc_block).get('dsa',None) is not None
    correct_dac0 = r.rfdc.core.get_output_current(dac0_tile,dac0_block).get('current',None) is not None
    correct_dac1 = r.rfdc.core.get_output_current(dac1_tile,dac1_block).get('current',None) is not None
    if correct_adc and correct_dac0 and correct_dac1:
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
    else:
        info['dsa'] = 0
        info['vop_dac0'] = 0
        info['vop_dac1'] = 0
        info['dac_duc_mixer_frequency_hz'] = 0
        info['adc_ddc_mix_frequency_hz'] = 0
        info['nyquist_zone_adc'] = 1
        info['nyquist_zone_dac0'] = 1
        info['nyquist_zone_dac1'] = 1
        info['mixer_scale_1p0_dac0'] = None
        info['mixer_scale_1p0_dac1'] = None
        info['mixer_scale_1p0_adc'] = None
        info['mixer_qmc_settings_dac0'] = None
        info['mixer_qmc_settings_dac1'] = None
        info['mixer_qmc_settings_adc'] = None
        print(bcolors.FAIL+'CRITICAL WARNING - RFDC settings not found, check that the DAC and ADC tiles/blocks are set correctly in the config to match the firmware'+bcolors.ENDC)
        print('Continuing regardless but the system will not work.')
    info['tone_frequencies'] = get_tone_frequencies(r,config_dict).tolist()
    info['tone_amplitudes'] = get_tone_amplitudes(r,config_dict).tolist()
    info['tone_phases'] = get_tone_phases(r,config_dict).tolist()
    print('system information:')
    for key, value in info.items():
        print(f'{key}: {value}\n')

    return info

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

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

def read_raw_control_buffer_data(r,buf,los=['tx','rx']):
    """
    From FW V7.5, all tone parameter settings are applied in one contiguous buffer and updates 
    to all tone parameters can now be written in one chunk.

    There are actually two consecutive buffers which can be switched between, so any updates 
    are applied instantly as opposed to register-by-register.

    There are still seperate buffers for tx and rx settings.

    Parameters:
    r: readout object
    buf: int, index of buffer to read from, 0 or 1.
    los: list of strings, either 'tx' or 'rx' to read the control values for the respective LO

    Returns a dictionary of the lo control values with the following keys:
    - 'tx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling' 
    - 'rx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'
    Each of these keys contains a numpy array with the values for each tone in firmware format
    
    """
    formatted_lo_control_values={'tx':{},'rx':{}}
        
    if buf not in [0,1]:
        raise ValueError(f"Buffer index must be 0 or 1. Not {buf}.")
    
    for lo in los:
        if lo not in ['tx','rx']:
            raise ValueError(f"Only LOs 'rx' and 'tx' are understood. Not {lo}.")
   
        
        n_tone = r.mixer.n_chans
        # In set_freqs, n_tone = n_chans and tones are interleaved in groups of _n_parallel_chans.
        # Each register slice written by set_freqs comes from one parallel stream and has length equal to n_serial_chans.
        Np = r.mixer._n_parallel_chans
        Ns = r.mixer._n_serial_chans    # number of tone samples per parallel slice

        # Prepare arrays to hold the interlaced results.
        all_phase_steps_int = np.empty(n_tone, dtype='>u4')
        all_ri_steps_int = np.empty(n_tone, dtype='>u4')
        all_phase_offsets_int = np.empty(n_tone, dtype='>u4')
        all_scaling_int = np.empty(n_tone, dtype='>u4')

        # Each parallel stream slice has been written to register: f'{lo}_lo{i}_control'
        # at an offset of: 4 * _CONTROL_N_WORDS * (buf * _n_serial_chans + i)
        # and with a length of: 4 * _CONTROL_N_WORDS * s bytes.
        slice_len_bytes = 4 * r.mixer._CONTROL_N_WORDS * Ns

        for i in range(Np):
            offset = 4 * r.mixer._CONTROL_N_WORDS * (buf * r.mixer._n_serial_chans + i)
            reg = f'{lo}_lo{i}_control'
            data = r.mixer.read(reg, slice_len_bytes, offset=offset)
            # Unpack as a uint32 array (big-endian). Total element count should be s * _CONTROL_N_WORDS.
            arr = np.frombuffer(data, dtype='>u4')
            # Reshape into (N, _CONTROL_N_WORDS): one row per tone in this slice.
            arr = arr.reshape((Ns, r.mixer._CONTROL_N_WORDS))
            # The fields are fixed:
            #   Column 0: phase increment
            #   Column 1: RI step
            #   Column 2: phase offset
            #   Column 3: amplitude scale
            # Place these values into the full arrays in positions corresponding to this parallel index.
            all_phase_steps_int[i::Np] = arr[:, r.mixer._PHASE_INC_WORD_OFFSET]
            all_ri_steps_int[i::Np] = arr[:, r.mixer._RI_STEP_WORD_OFFSET]
            all_phase_offsets_int[i::Np] = arr[:, r.mixer._PHASE_OFFSET_WORD_OFFSET]
            all_scaling_int[i::Np] = arr[:, r.mixer._SCALE_WORD_OFFSET]


        # # From set_freqs, phase_steps (which become phase_inc here) were computed as:
        # #   phase_steps = (freq / fft_rbw_hz) * 2π
        # # So we invert that to recover the frequency:
        # fft_period_s = r.mixer._n_upstream_chans / r.mixer._upstream_oversample_factor / sample_rate_hz
        # fft_rbw_hz = 1. / fft_period_s
        # freqs_hz = (phase_steps / (2 * np.pi)) * fft_rbw_hz

        formatted_lo_control_values[lo]={
            'formatted_phase_steps': all_phase_steps_int,
            'formatted_ri_steps': all_ri_steps_int,
            'formatted_phase_offsets': all_phase_offsets_int,
            'formatted_scaling': all_scaling_int
        }
    return formatted_lo_control_values


def read_raw_control_buffer_data_fast(r_fast,buf,los=['tx','rx']):
    """
    From FW V7.5, all tone parameter settings are applied in one contiguous buffer and updates 
    to all tone parameters can now be written in one chunk.

    There are actually two consecutive buffers which can be switched between, so any updates 
    are applied instantly as opposed to register-by-register.

    There are still seperate buffers for tx and rx settings.

    Parameters:
    r: readout object
    buf: int, index of buffer to read from, 0 or 1.
    los: list of strings, either 'tx' or 'rx' to read the control values for the respective LO

    Returns a dictionary of the lo control values with the following keys:
    - 'tx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling' 
    - 'rx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'
    Each of these keys contains a numpy array with the values for each tone in firmware format
    
    """
    formatted_lo_control_values={'tx':{},'rx':{}}
        
    if buf not in [0,1]:
        raise ValueError(f"Buffer index must be 0 or 1. Not {buf}.")
    
    for lo in los:
        if lo not in ['tx','rx']:
            raise ValueError(f"Only LOs 'rx' and 'tx' are understood. Not {lo}.")
        
        offset = 0x80000 if lo=='rx' else 0x90000
        offset += 0x8000*buf
        length = 0x8000
        data = r_fast.mixer.host.transport.axil_mm[int(offset):int(offset+length)]
        arr = np.frombuffer(data, dtype='<u4')
        all_phase_steps_int = arr[r_fast.mixer._PHASE_INC_WORD_OFFSET::4]
        all_ri_steps_int = arr[r_fast.mixer._RI_STEP_WORD_OFFSET::4]
        all_phase_offsets_int = arr[r_fast.mixer._PHASE_OFFSET_WORD_OFFSET::4]
        all_scaling_int = arr[r_fast.mixer._SCALE_WORD_OFFSET::4]

        formatted_lo_control_values[lo]={
            'formatted_phase_steps': all_phase_steps_int,
            'formatted_ri_steps': all_ri_steps_int,
            'formatted_phase_offsets': all_phase_offsets_int,
            'formatted_scaling': all_scaling_int
        }
    return formatted_lo_control_values


def interpret_raw_control_buffer_data(r,formatted_lo_control_values):
    """
    Interpret the values read from the lo control buffer.

    Parameters:
    r: readout object
    formatted_lo_control_values: dictionary with keys 'tx' and 'rx'
     - 'tx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'
     - 'rx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'

    Returns:
    lo_control_values: dictionary with keys 'tx' and 'rx'
     - 'tx': dictionary with keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
     - 'rx': dictionary with keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'

    """
    lo_control_values = {'tx':{},'rx':{}}
    for lo in ['tx','rx']:
        if lo not in formatted_lo_control_values.keys():
            raise ValueError(f"Only LOs 'rx' and 'tx' are understood. Not {lo}.")
        try:
            all_phase_steps_int = formatted_lo_control_values[lo]['formatted_phase_steps']
            all_ri_steps_int = formatted_lo_control_values[lo]['formatted_ri_steps']
            all_phase_offsets_int = formatted_lo_control_values[lo]['formatted_phase_offsets']
            all_scaling_int = formatted_lo_control_values[lo]['formatted_scaling']
    
            phase_steps = _invert_format_phase_steps(all_phase_steps_int.ravel(), r.mixer._phase_bp)
            ri_steps = _invert_format_ri_steps(all_ri_steps_int,r.mixer._n_ri_step_bits)
            phase_offsets = _invert_format_phase_offsets(all_phase_offsets_int.ravel(), r.mixer._phase_offset_bp)
            scaling = _invert_format_amp_scale(all_scaling_int.ravel(), r.mixer._n_scale_bits)

            lo_control_values[lo]['phase_steps'] = phase_steps
            lo_control_values[lo]['ri_steps'] = ri_steps
            lo_control_values[lo]['phase_offsets'] = phase_offsets
            lo_control_values[lo]['scaling'] = scaling
        except KeyError as e:
            continue

    return lo_control_values


def interpret_raw_control_buffer_data_fast(r_fast,formatted_lo_control_values):
    """
    Interpret the values read from the lo control buffer.

    Parameters:
    r_fast: readout object
    formatted_lo_control_values: dictionary with keys 'tx' and 'rx'
     - 'tx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'
     - 'rx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'

    Returns:
    lo_control_values: dictionary with keys 'tx' and 'rx'
     - 'tx': dictionary with keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
     - 'rx': dictionary with keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'

    """
    lo_control_values = {'tx':{},'rx':{}}
    for lo in ['tx','rx']:
        if lo not in formatted_lo_control_values.keys():
            raise ValueError(f"Only LOs 'rx' and 'tx' are understood. Not {lo}.")
        try:
            all_phase_steps_int = formatted_lo_control_values[lo]['formatted_phase_steps']
            all_ri_steps_int = formatted_lo_control_values[lo]['formatted_ri_steps']
            all_phase_offsets_int = formatted_lo_control_values[lo]['formatted_phase_offsets']
            all_scaling_int = formatted_lo_control_values[lo]['formatted_scaling']
    
            phase_steps = _invert_format_phase_steps(all_phase_steps_int.ravel(), r_fast.mixer._phase_bp,fmt='<i4')
            ri_steps = _invert_format_ri_steps(all_ri_steps_int,r_fast.mixer._n_ri_step_bits,fmt='<u4')
            phase_offsets = _invert_format_phase_offsets(all_phase_offsets_int.ravel(), r_fast.mixer._phase_offset_bp,fmt='<i4')
            scaling = _invert_format_amp_scale(all_scaling_int.ravel(), r_fast.mixer._n_scale_bits,fmt='<u4')

            lo_control_values[lo]['phase_steps'] = phase_steps
            lo_control_values[lo]['ri_steps'] = ri_steps
            lo_control_values[lo]['phase_offsets'] = phase_offsets
            lo_control_values[lo]['scaling'] = scaling
        except KeyError as e:
            continue

    return lo_control_values        

def prepare_control_buffer_data(r,buf,lo_control_values):
    """
    Prepare a formatted control buffer to write to the firmware.

    Parameters:
    r: readout object
    buf: int, index of buffer to write to, 0 or 1.
    lo_control_values: dictionary with keys 'tx' and 'rx'
     - 'tx': dictionary with keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
     - 'rx': dictionary with keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
    
     returns a numpy array of the formatted control buffer.

     If any keys not given, the values are read from the specified control buffer.
    """
    
    if buf not in [0,1]:
        raise ValueError(f"Buffer index must be 0 or 1. Not {buf}.")
    v={'tx':{},'rx':{}}    
    for lo in ['tx','rx']:
        if lo not in lo_control_values.keys():
            raise ValueError(f"Only LOs 'rx' and 'tx' are understood. Not {lo}.")
        phase_steps = lo_control_values[lo].get('phase_steps')
        ri_steps = lo_control_values[lo].get('ri_steps')
        phase_offsets = lo_control_values[lo].get('phase_offsets')
        scaling = lo_control_values[lo].get('scaling')
        if any([i is None for i in [phase_steps,ri_steps,phase_offsets,scaling]]):
            existing = interpret_raw_control_buffer_data(r, read_raw_control_buffer_data(r,buf,los=[lo]))
            if phase_steps is None:
                phase_steps = existing[lo]['phase_steps']
            if ri_steps is None:
                ri_steps = existing[lo]['ri_steps']
            if phase_offsets is None:
                phase_offsets = existing[lo]['phase_offsets']
            if scaling is None:
                scaling = existing[lo]['scaling']
        
        phase_steps_formatted = _format_phase_steps(phase_steps, r.mixer._phase_bp,fmt='<i4')
        phase_offsets_formatted = _format_phase_offsets(phase_offsets, r.mixer._phase_offset_bp,fmt='<i4')
        ri_steps_formatted = _format_ri_steps(ri_steps,r.mixer._n_ri_step_bits,fmt='<u4')
        scaling_formatted = _format_amp_scale(scaling, r.mixer._n_scale_bits,fmt='<u4') 

        n_tone = r.mixer.n_chans

        if len(phase_steps_formatted) != n_tone:
            phase_steps_formatted = np.concatenate([phase_steps_formatted, np.zeros(n_tone - len(phase_steps_formatted), dtype=phase_steps_formatted.dtype)])
        if len(phase_offsets_formatted) != n_tone:
            phase_offsets_formatted = np.concatenate([phase_offsets_formatted, np.zeros(n_tone - len(phase_offsets_formatted), dtype=phase_offsets_formatted.dtype)])
        if len(ri_steps_formatted) != n_tone:
            ri_steps_formatted = np.concatenate([ri_steps_formatted, np.zeros(n_tone - len(ri_steps_formatted), dtype=ri_steps_formatted.dtype)])
        if len(scaling_formatted) != n_tone:
            scaling_formatted = np.concatenate([scaling_formatted, np.zeros(n_tone - len(scaling_formatted), dtype=scaling_formatted.dtype)])
        
        v[lo] = np.zeros(int(np.ceil(n_tone / r.mixer._n_parallel_chans)) * r.mixer._CONTROL_N_WORDS, dtype='>u4')
        for i in range(min(r.mixer._n_parallel_chans, n_tone)):
            v[lo][r.mixer._SCALE_WORD_OFFSET :: r.mixer._CONTROL_N_WORDS] = scaling_formatted[i::r.mixer._n_parallel_chans]
            v[lo][r.mixer._PHASE_INC_WORD_OFFSET :: r.mixer._CONTROL_N_WORDS] = phase_steps_formatted[i::r.mixer._n_parallel_chans]
            v[lo][r.mixer._PHASE_OFFSET_WORD_OFFSET :: r.mixer._CONTROL_N_WORDS] = phase_offsets_formatted[i::r.mixer._n_parallel_chans]
            v[lo][r.mixer._RI_STEP_WORD_OFFSET :: r.mixer._CONTROL_N_WORDS] = ri_steps_formatted[i::r.mixer._n_parallel_chans]
    return v
    
def prepare_control_buffer_data_fast(r_fast,buf,lo_control_values):
    """
    Faster version of prepare_control_buffer 

    Does not prepare the full buffer, only returns the given formatted values and their indices

    Parameters:
    r: readout object
    buf: int, index of buffer to write to, 0 or 1.
    lo_control_values: dictionary with keys 'tx' and 'rx'
     - 'tx': dictionary with optional keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
     - 'rx': dictionary with optional keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
    
    Returns:
    v: dictionary with keys 'tx' and 'rx'
     - 'tx': numpy array of the formatted control buffer for tx
     - 'rx': numpy array of the formatted control buffer for rx
    i: dictionary with keys 'tx' and 'rx'
     - 'tx': numpy array of the indices for the formatted control buffer for tx
     - 'rx': numpy array of the indices for the formatted control buffer for rx

    """
    # if not hasattr(r_fast.mixer,'tx_lo_control_buffer_mv'):
    #     r_fast.mixer.tx_lo_control_buffer = np.frombuffer(memoryview(r_fast.mixer.host.transport.axil_mm[0x90000:0x100000]),dtype='<u4')
    # if not hasattr(r_fast.mixer,'rx_lo_control_buffer_mv'):
    #     r_fast.mixer.rx_lo_control_buffer = np.frombuffer(memoryview(r_fast.mixer.host.transport.axil_mm[0x80000:0x90000]),dtype='<u4')
    if not hasattr(r_fast.mixer,'tx_lo_control_buffer_mv'):
        r_fast.mixer.tx_lo_control_buffer = np.frombuffer(memoryview(r_fast.mixer.host.transport.axil_mm),dtype='<u4')
    if not hasattr(r_fast.mixer,'rx_lo_control_buffer_mv'):
        r_fast.mixer.rx_lo_control_buffer = np.frombuffer(memoryview(r_fast.mixer.host.transport.axil_mm),dtype='<u4')
    v={}  
    i={}  
    for lo in ['tx','rx']:
        if lo not in lo_control_values.keys():
            raise ValueError(f"Only LOs 'rx' and 'tx' are understood. Not {lo}.")
        phase_steps = lo_control_values[lo].get('phase_steps')
        ri_steps = lo_control_values[lo].get('ri_steps')
        phase_offsets = lo_control_values[lo].get('phase_offsets')
        scaling = lo_control_values[lo].get('scaling')

        phase_steps_formatted = _format_phase_steps(phase_steps, r_fast.mixer._phase_bp,fmt='<i4') if phase_steps is not None else []
        phase_offsets_formatted = _format_phase_offsets(phase_offsets, r_fast.mixer._phase_offset_bp,fmt='<i4') if phase_offsets is not None else []
        ri_steps_formatted = _format_ri_steps(ri_steps,r_fast.mixer._n_ri_step_bits,fmt='<u4') if ri_steps is not None else []
        scaling_formatted = _format_amp_scale(scaling, r_fast.mixer._n_scale_bits,fmt='<u4') if scaling is not None else []

        n_phase_steps = len(phase_steps_formatted)
        n_phase_offsets = len(phase_offsets_formatted)
        n_ri_steps = len(ri_steps_formatted)
        n_scaling = len(scaling_formatted) 
        
        # v[lo] = np.zeros(int(np.ceil(n_tone / r.mixer._n_parallel_chans)) * r.mixer._CONTROL_N_WORDS, dtype='>u4')
        # for i in range(min(r.mixer._n_parallel_chans, n_tone)):
        #     v[lo][r.mixer._SCALE_WORD_OFFSET :: r.mixer._CONTROL_N_WORDS] = scaling_formatted[i::r.mixer._n_parallel_chans]
        #     v[lo][r.mixer._PHASE_INC_WORD_OFFSET :: r.mixer._CONTROL_N_WORDS] = phase_steps_formatted[i::r.mixer._n_parallel_chans]
        #     v[lo][r.mixer._PHASE_OFFSET_WORD_OFFSET :: r.mixer._CONTROL_N_WORDS] = phase_offsets_formatted[i::r.mixer._n_parallel_chans]
        #     v[lo][r.mixer._RI_STEP_WORD_OFFSET :: r.mixer._CONTROL_N_WORDS] = ri_steps_formatted[i::r.mixer._n_parallel_chans]

        v[lo] = np.empty(n_phase_steps+n_phase_offsets+n_ri_steps+n_scaling, dtype='>u4')
        v[lo][0:n_phase_steps] = phase_steps_formatted
        v[lo][n_phase_steps:n_phase_steps+n_phase_offsets] = phase_offsets_formatted
        v[lo][n_phase_steps+n_phase_offsets:n_phase_steps+n_phase_offsets+n_ri_steps] = ri_steps_formatted
        v[lo][n_phase_steps+n_phase_offsets+n_ri_steps:n_phase_steps+n_phase_offsets+n_ri_steps+n_scaling] = scaling_formatted

        i[lo] = np.empty(n_phase_steps+n_phase_offsets+n_ri_steps+n_scaling, dtype=int)
        i[lo][0:n_phase_steps] = np.arange(n_phase_steps)*r_fast.mixer._CONTROL_N_WORDS+r_fast.mixer._PHASE_INC_WORD_OFFSET
        i[lo][n_phase_steps:n_phase_steps+n_phase_offsets] = np.arange(n_phase_offsets)*r_fast.mixer._CONTROL_N_WORDS+r_fast.mixer._PHASE_OFFSET_WORD_OFFSET
        i[lo][n_phase_steps+n_phase_offsets:n_phase_steps+n_phase_offsets+n_ri_steps] = np.arange(n_ri_steps)*r_fast.mixer._CONTROL_N_WORDS+r_fast.mixer._RI_STEP_WORD_OFFSET
        i[lo][n_phase_steps+n_phase_offsets+n_ri_steps:n_phase_steps+n_phase_offsets+n_ri_steps+n_scaling] = np.arange(n_scaling)*r_fast.mixer._CONTROL_N_WORDS+r_fast.mixer._SCALE_WORD_OFFSET
    return v, i

def write_control_buffer_data(r,buf,v):
    """
    Write a formatted control buffer to the firmware.
    Parameters:
    r: readout object
    buf: int, index of buffer to write to, 0 or 1.
    v: numpy array of the formatted control buffer.
    
    """
    n_tone = r.mixer.n_chans
    if buf not in [0,1]:
        raise ValueError(f"Buffer index must be 0 or 1. Not {buf}.")
    for lo in ['tx','rx']:    
        for i in range(min(r.mixer._n_parallel_chans, n_tone)):
            reg = f'{lo}_lo{i}_control'
            offset = 4 * r.mixer._CONTROL_N_WORDS * (buf * r.mixer._n_serial_chans + i)
            r.mixer.write(reg, v[lo].tobytes(),offset=offset)
    return

def write_control_buffer_data_fast(r_fast,buf,v,indices):
    """
    Write a formatted control buffer to the firmware.
    Parameters:
    r: readout object
    buf: int, index of buffer to write to, 0 or 1.
    v: dictionary with keys 'tx' and 'rx'
        - 'tx': numpy array of the formatted control buffer values for tx
        - 'rx': numpy array of the formatted control buffer values for rx
    
    indices: dictionary with keys 'tx' and 'rx'
        - 'tx': numpy array of the indices for the formatted control buffer for tx
        - 'rx': numpy array of the indices for the formatted control buffer for rx
    
    """
    n_tone = r_fast.mixer.n_chans
    if buf not in [0,1]:
        raise ValueError(f"Buffer index must be 0 or 1. Not {buf}.")
    
    if indices is None:
        for lo in ['tx','rx']:    
            start=0x80000 if lo=='rx' else 0x90000
            start+= 0x8000*buf
            length=0x8000
            r_fast.mixer.host.transport.axil_mm[int(start):int(start+length)] = v[lo].astype('<u4').tobytes()
    else:
        # for lo in ['tx','rx']:     
            # start=0x80000 if lo=='rx' else 0x90000
            # start+= 0x8000*buf
            # r_fast.mixer.host.transport.axil_mm[start+indices[lo]] = v[lo].astype('<u4').tobytes()
        start_tx = 0x90000//4 + 0x8000*buf//4
        start_rx = 0x80000//4 + 0x8000*buf//4
        r_fast.mixer.tx_lo_control_buffer[start_tx+indices['tx']] = v['tx'].astype('<u4')
        r_fast.mixer.rx_lo_control_buffer[start_rx+indices['rx']] = v['rx'].astype('<u4')
            
    return
        

def write_to_current_control_buffer(r,formatted_lo_control_values):
    """
    Write pre-formatted values to the current lo control buffer.

    Parameters:
    r: readout object
    formatted_lo_control_values: dictionary with keys 'tx' and 'rx'
     - 'tx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'
     - 'rx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'

    """
    # Get the current buffer index
    buf = r.mixer.get_current_buffer()
    if buf is None:
        raise RuntimeError('No current buffer found in mixer, cannot write frequencies')
    # Write to the current buffer
    write_control_buffer_data(r,buf,formatted_lo_control_values)

def write_to_next_control_buffer(r,formatted_lo_control_values):
    """
    Write values to the next lo control buffer.

    Parameters:
    r: readout object
    formatted_lo_control_values: dictionary with keys 'tx' and 'rx'
     - 'tx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'
     - 'rx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'
    # Note that this will not be applied until the buffer is switched.
    # This is useful for preparing the next buffer while the current one is being used.
    """
    # Get the next buffer index
    buf = r.mixer.get_current_buffer()
    if buf is None:
        raise RuntimeError('No next buffer found in mixer, cannot write frequencies')
    next_buffer = (buf + 1) % 2
    # Write to the next buffer
    write_control_buffer_data(r,next_buffer,formatted_lo_control_values)

def read_from_current_control_buffer(r,los=['tx','rx']):
    """
    Read the current lo control buffer, and return interepted values.

    Parameters:
    r: readout object
    los: list of strings, either 'tx' or 'rx' to read the control values for the respective LO

    Returns a dictionary of the lo control values with the following keys:
    - 'tx': dictionary with keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
    - 'rx': dictionary with keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
    Each of these keys contains a numpy array with the values for each tone.
    
    """
    buf = r.mixer.get_current_buffer()
    if buf is None:
        raise RuntimeError('No current buffer found in mixer, cannot read frequencies')
    return interpret_raw_control_buffer_data(r,read_raw_control_buffer_data(r,buf,los=los))

def read_from_next_control_buffer(r,los=['tx','rx']):
    """
    Read the next lo control buffer, and return interepted values.

    Parameters:
    r: readout object
    los: list of strings, either 'tx' or 'rx' to read the control values for the respective LO
    Returns a dictionary of the lo control values with the following
    keys:
    - 'tx': dictionary with keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
    - 'rx': dictionary with keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
    Each of these keys contains a numpy array with the values for each tone.
    
    """
    buf = r.mixer.get_current_buffer()
    if buf is None:
        raise RuntimeError('No next buffer found in mixer, cannot read frequencies')
    next_buffer = (buf + 1) % 2
    return interpret_raw_control_buffer_data(r,read_raw_control_buffer_data(r,next_buffer,los=los))


def switch_control_buffer(r):
    """
    Switch the current lo control buffer.

    Parameters:
    r: readout object

    """
    buf = r.mixer.get_current_buffer()
    next_buffer = (buf + 1) % 2
    r.mixer.set_current_buffer(next_buffer)
    return next_buffer

def set_control_buffer_idx(r,buf):
    """
    Set the current lo control buffer.

    Parameters:
    r: readout object
    buf: int, index of buffer to set, 0 or 1.

    """
    if buf not in [0,1]:
        raise ValueError(f"Buffer index must be 0 or 1. Not {buf}.")
    r.mixer.set_current_buffer(buf)
    return buf

def set_control_buffer_idx_fast(r_fast,buf):
    """
    Set the current lo control buffer.

    Parameters:
    r: readout object
    buf: int, index of buffer to set, 0 or 1.

    """
    r_fast.mixer.set_current_buffer(buf)
    return


def get_control_buffer_idx(r):
    """
    Get the index of the current control buffer
    Parameters:
    r: readout object
    Returns:
    buf: int, index of current buffer, 0 or 1.

    """
    buf = r.mixer.get_current_buffer()
    if buf is None:
        raise RuntimeError('No current buffer found in mixer, cannot read frequencies')
    return buf

def get_next_buffer_idx(r):
    """
    Get the index of the next control buffer
    Parameters:
    r: readout object
    Returns:
    buf: int, index of next buffer, 0 or 1.

    """
    buf = r.mixer.get_current_buffer()
    if buf is None:
        raise RuntimeError('No current buffer found in mixer, cannot read frequencies')
    next_buffer = (buf + 1) % 2
    return next_buffer


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
    if dac_nyquist_zone is None:
        print(bcolors.FAIL+'CRITICAL WARNING, misconfigured DAC tile/block, nyquist zone not found, assuming zone 1'+bcolors.ENDC)
        dac_nyquist_zone = 1
    if adc_nyquist_zone is None:
        print(bcolors.FAIL+'CRITICAL WARNING, misconfigured ADC tile/block, nyquist zone not found, assuming zone 1'+bcolors.ENDC)
        adc_nyquist_zone = 1

    
    # moved to single control buffer in v7.5
    # #read mixer lo phase_increment values
    # phase_inc_tx   = np.frombuffer(r.mixer.read(f'tx_lo{p}_phase_inc',4*nc),dtype='>i4')
    # phase_inc_rx   = np.frombuffer(r.mixer.read(f'rx_lo{p}_phase_inc',4*nc),dtype='>i4')
    # phase_inc_tx   = _invert_format_phase_steps(phase_inc_tx,r.mixer._phase_bp)
    # phase_inc_rx   = _invert_format_phase_steps(phase_inc_rx,r.mixer._phase_bp)
    
    # #read mixer lo ri_step values
    # ri_steps_tx    = np.frombuffer(r.mixer.read(f'tx_lo{p}_ri_step',4*nc),dtype='>u4')
    # ri_steps_rx    = np.frombuffer(r.mixer.read(f'rx_lo{p}_ri_step',4*nc),dtype='>u4')
    # ri_steps_tx    = uint2cplx(ri_steps_tx, r.mixer._n_ri_step_bits)
    # ri_steps_rx    = uint2cplx(ri_steps_rx, r.mixer._n_ri_step_bits)
    

    lo_control_values = read_from_current_control_buffer(r)

    phase_inc_tx = lo_control_values['tx']['phase_steps']
    phase_inc_rx = lo_control_values['rx']['phase_steps']
    ri_steps_tx = lo_control_values['tx']['ri_steps']
    ri_steps_rx = lo_control_values['rx']['ri_steps']

    #convert to ri_steps to phase angles
    phase_steps_tx = np.angle(ri_steps_tx)
    phase_steps_rx = np.angle(ri_steps_rx)

    #convert to offset frequencies
    offset_freqs_hz_tx    = phase_inc_tx * fft_rbw_hz / 2 / np.pi 
    offset_freqs_hz_rx    = phase_inc_rx * fft_rbw_hz / 2 / np.pi
    offset_freqs_hz_tx_ri    = phase_steps_tx * fft_rbw_hz / 2 / np.pi 
    offset_freqs_hz_rx_ri    = phase_steps_rx * fft_rbw_hz / 2 / np.pi
    
    #get the filterbank channels
    chanmap_psb  = psb_chanselect_get_channel_outmap(r)
    chanmap_pfb  = chanselect_get_channel_outmap(r)
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
    if dac_nyquist_zone == 1:
        duc_freqs = dbb_freqs_tx + 1e6*duc_settings.get('Freq',0)
    elif dac_nyquist_zone == 2:
        duc_freqs = dbb_freqs_tx - 1e6*duc_settings.get('Freq',0)
    if adc_nyquist_zone == 1:
        ddc_freqs = dbb_freqs_rx - 1e6*ddc_settings.get('Freq',0)
    elif adc_nyquist_zone == 2:
        ddc_freqs = dbb_freqs_rx + 1e6*ddc_settings.get('Freq',0)

    dac_out_freqs = np.abs(duc_freqs)
    adc_in_freqs = np.abs(ddc_freqs)

    #get the rf frequencies given any analog up/down conversion
    if udc_connected:
        udc_freqs_tx = udc_lo_frequency + udc_sideband * dac_out_freqs 
        udc_freqs_rx = udc_lo_frequency + udc_sideband * adc_in_freqs    
    else:
        udc_freqs_tx = dac_out_freqs
        udc_freqs_rx = adc_in_freqs
    
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
    duc_frequency = duc_settings['Freq']*1e6
    ddc_frequency = ddc_settings['Freq']*1e6

    dac_nyquist_zone = r.rfdc.core.get_nyquist_zone(dac_tile,dac_block,r.rfdc.core.DAC_TILE)
    adc_nyquist_zone = r.rfdc.core.get_nyquist_zone(adc_tile,adc_block,r.rfdc.core.ADC_TILE)
    if dac_nyquist_zone is None:
        print(bcolors.FAIL+'CRITICAL WARNING, misconfigured DAC tile/block, nyquist zone not found, assuming zone 1'+bcolors.ENDC)
        dac_nyquist_zone = 1
    if adc_nyquist_zone is None:
        print(bcolors.FAIL+'CRITICAL WARNING, misconfigured ADC tile/block, nyquist zone not found, assuming zone 1'+bcolors.ENDC)
        adc_nyquist_zone = 1

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

    duc_freqs = dac_out_freqs
    ddc_freqs = adc_in_freqs

    #get the DAC/ADC digitial frequencies given the Nyquist zone
    if dac_nyquist_zone == 1:
        dbb_freqs_tx = duc_freqs - duc_frequency
    elif dac_nyquist_zone == 2:
        dbb_freqs_tx = duc_freqs + duc_frequency
    else:
        raise ValueError(f'Invalid DAC nyquist zone ({dac_nyquist_zone})')

    if adc_nyquist_zone == 1:
        dbb_freqs_rx = ddc_freqs + ddc_frequency
    elif adc_nyquist_zone == 2:
        dbb_freqs_rx = ddc_freqs - ddc_frequency
    else:
        raise ValueError(f'Invalid ADC nyquist zone ({adc_nyquist_zone})')

    #check all tones are in within the baseband bandwidth
    txbbmin=np.min(all_tx_bin_centers_hz)
    txbbmax=np.max(all_tx_bin_centers_hz)+fft_rbw_hz
    rxbbmin=np.min(all_rx_bin_centers_hz)
    rxbbmax=np.max(all_rx_bin_centers_hz)+fft_rbw_hz

    if (dbb_freqs_tx > txbbmax).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth: dbb_freqs_tx={dbb_freqs_tx}')
    if (dbb_freqs_tx < txbbmin).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth: dbb_freqs_tx={dbb_freqs_tx}')
    if (dbb_freqs_rx > rxbbmax).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth: dbb_freqs_rx={dbb_freqs_rx}')
    if (dbb_freqs_rx < rxbbmin).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth: dbb_freqs_rx={dbb_freqs_rx}')
    
    #get the nearest filterbank center frequencies for each tone
    tx_nearest_bins = get_closest_bin_indices(dbb_freqs_tx, all_tx_bin_centers_hz)
    rx_nearest_bins = get_closest_bin_indices(dbb_freqs_rx, all_rx_bin_centers_hz)

    #get the frequency offsets for each tone
    tx_freq_offsets_hz = dbb_freqs_tx - all_tx_bin_centers_hz[tx_nearest_bins]
    rx_freq_offsets_hz = dbb_freqs_rx - all_rx_bin_centers_hz[rx_nearest_bins]
    
    #get the phase increments and ri steps for the mixer LOs
    phase_incs_tx = tx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    phase_incs_rx = rx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    ri_steps_tx = np.cos(phase_incs_tx) + 1j*np.sin(phase_incs_tx)
    ri_steps_rx = np.cos(phase_incs_rx) + 1j*np.sin(phase_incs_rx)
    
    #prepare the formatted lo control buffer values
    buf = get_next_buffer_idx(r)
    v = prepare_control_buffer_data(r,buf,{'tx':{'phase_steps':phase_incs_tx,
                                            'ri_steps':ri_steps_tx},
                                      'rx':{'phase_steps':phase_incs_rx,
                                            'ri_steps':ri_steps_rx}})
    
    # phase_incs_tx_formatted = _format_phase_steps(phase_incs_tx,r.mixer._phase_bp)
    # phase_incs_rx_formatted = _format_phase_steps(phase_incs_rx,r.mixer._phase_bp)
    # ri_steps_tx_formatted = cplx2uint(ri_steps_tx, r.mixer._n_ri_step_bits)
    # ri_steps_rx_formatted = cplx2uint(ri_steps_rx, r.mixer._n_ri_step_bits)

    #set the filterbank channel maps
    chanmap_psb[tx_nearest_bins] = channels
    chanmap_pfb[channels] = rx_nearest_bins

    # tone_settings_dict = {'phase_incs_tx_formatted':phase_incs_tx_formatted,
    #                       'phase_incs_rx_formatted':phase_incs_rx_formatted,
    #                       'ri_steps_tx_formatted':ri_steps_tx_formatted,
    #                       'ri_steps_rx_formatted':ri_steps_rx_formatted,
    #                       'chanmap_psb':chanmap_psb,
    #                       'chanmap_pfb':chanmap_pfb,
    #                       'num_tones':num_tones}
    
    tone_settings_dict = {'control_buffer_data_values':v,
                          'control_buffer_index':buf,
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
    # phase_incs_tx = tone_settings_dict.get('phase_incs_tx_formatted')
    # phase_incs_rx = tone_settings_dict.get('phase_incs_rx_formatted')
    # ri_steps_tx   = tone_settings_dict.get('ri_steps_tx_formatted')
    # ri_steps_rx   = tone_settings_dict.get('ri_steps_rx_formatted')
    # phase_offsets_tx = tone_settings_dict.get('phase_offsets_tx_formatted')
    # phase_offsets_rx = tone_settings_dict.get('phase_offsets_rx_formatted')
    # scaling_tx = tone_settings_dict.get('scaling_tx_formatted')
    # scaling_rx = tone_settings_dict.get('scaling_rx_formatted')

    v = tone_settings_dict.get('control_buffer_data_values')
    buf = tone_settings_dict.get('control_buffer_index')
    chanmap_psb   = tone_settings_dict.get('chanmap_psb')
    chanmap_pfb   = tone_settings_dict.get('chanmap_pfb')
    num_tones     = tone_settings_dict.get('num_tones')

    if num_tones is None:
        # num_tones = max((len(phase_incs_tx),len(phase_incs_tx),len(ri_steps_tx),len(ri_steps_tx),))
        num_tones = r.mixer.n_chans

    if not chanmap_psb is None:
        psb_chanselect_set_channel_outmap(r,np.copy(chanmap_psb))
    if not chanmap_pfb is None:
        chanselect_set_channel_outmap(r,np.copy(chanmap_pfb))

    # for i in range(min(r.mixer._n_parallel_chans, num_tones)):   
    #     if not phase_incs_tx is None:
    #         r.mixer.write(f'tx_lo{i}_phase_inc', phase_incs_tx[i::r.mixer._n_parallel_chans].tobytes())
    #     if not ri_steps_tx is None:
    #         r.mixer.write(f'tx_lo{i}_ri_step',     ri_steps_tx[i::r.mixer._n_parallel_chans].tobytes())
    #     if not phase_incs_rx is None:
    #         r.mixer.write(f'rx_lo{i}_phase_inc', phase_incs_rx[i::r.mixer._n_parallel_chans].tobytes())
    #     if not ri_steps_rx is None:
    #         r.mixer.write(f'rx_lo{i}_ri_step',     ri_steps_rx[i::r.mixer._n_parallel_chans].tobytes())
    
    # if autosync:
    #     # time.sleep(autosync_time_delay)
    #     r.sync.arm_sync(wait=False)
    #     time.sleep(autosync_time_delay)
    #     r.sync.sw_sync()

    write_control_buffer_data(r,buf,v)
    set_control_buffer_idx(r,buf)
    r.sync.arm_sync(wait=False)
    r.sync.sw_sync()
    return

def prepare_tone_frequency_settings_fast(r, config_dict, tone_frequencies, detailed_output=False):
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

    duc_freqs = dac_out_freqs
    ddc_freqs = adc_in_freqs

    #get the DAC/ADC digitial frequencies given the Nyquist zone
    if dac_nyquist_zone == 1:
        dbb_freqs_tx = duc_freqs - duc_frequency
    elif dac_nyquist_zone == 2:
        dbb_freqs_tx = duc_freqs + duc_frequency
    else:
        raise ValueError(f'Invalid DAC nyquist zone ({dac_nyquist_zone})')

    if adc_nyquist_zone == 1:
        dbb_freqs_rx = ddc_freqs + ddc_frequency
    elif adc_nyquist_zone == 2:
        dbb_freqs_rx = ddc_freqs - ddc_frequency
    else:
        raise ValueError(f'Invalid ADC nyquist zone ({adc_nyquist_zone})')


    #check all tones are in within the baseband bandwidth
    txbbmin=np.min(all_tx_bin_centers_hz)
    txbbmax=np.max(all_tx_bin_centers_hz)+fft_rbw_hz
    rxbbmin=np.min(all_rx_bin_centers_hz)
    rxbbmax=np.max(all_rx_bin_centers_hz)+fft_rbw_hz

    if (dbb_freqs_tx > txbbmax).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth: dbb_freqs_tx={dbb_freqs_tx}')
    if (dbb_freqs_tx < txbbmin).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth: dbb_freqs_tx={dbb_freqs_tx}')
    if (dbb_freqs_rx > rxbbmax).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth: dbb_freqs_rx={dbb_freqs_rx}')
    if (dbb_freqs_rx < rxbbmin).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth: dbb_freqs_rx={dbb_freqs_rx}')

    #get the nearest filterbank center frequencies for each tone
    tx_nearest_bins = get_closest_bin_indices(dbb_freqs_tx, all_tx_bin_centers_hz)
    rx_nearest_bins = get_closest_bin_indices(dbb_freqs_rx, all_rx_bin_centers_hz)

    #get the frequency offsets for each tone
    tx_freq_offsets_hz = dbb_freqs_tx - all_tx_bin_centers_hz[tx_nearest_bins]
    rx_freq_offsets_hz = dbb_freqs_rx - all_rx_bin_centers_hz[rx_nearest_bins]
    
    #get the phase increments and ri steps for the mixer LOs
    phase_incs_tx = tx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    phase_incs_rx = rx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    ri_steps_tx = np.cos(phase_incs_tx) + 1j*np.sin(phase_incs_tx)
    ri_steps_rx = np.cos(phase_incs_rx) + 1j*np.sin(phase_incs_rx)
    
    # #zero pad out to nchans
    # phase_incs_tx = np.pad(phase_incs_tx, (0,nc-len(phase_incs_tx)), 'constant', constant_values=(0,0))
    # phase_incs_rx = np.pad(phase_incs_rx, (0,nc-len(phase_incs_rx)), 'constant', constant_values=(0,0))
    # ri_steps_tx = np.pad(ri_steps_tx, (0,nc-len(ri_steps_tx)), 'constant', constant_values=(0,0))
    # ri_steps_rx = np.pad(ri_steps_rx, (0,nc-len(ri_steps_rx)), 'constant', constant_values=(0,0))

    v,i = prepare_control_buffer_data_fast(r,0,{'tx':{'phase_steps':phase_incs_tx,
                                            'ri_steps':ri_steps_tx},
                                      'rx':{'phase_steps':phase_incs_rx,
                                            'ri_steps':ri_steps_rx}})
    # #format the phase increments and ri steps for the mixer LOs
    # phase_incs_tx_formatted = _format_phase_steps(phase_incs_tx,r.mixer._phase_bp,fmt='<i4')
    # phase_incs_rx_formatted = _format_phase_steps(phase_incs_rx,r.mixer._phase_bp,fmt='<i4')
    # ri_steps_tx_formatted = cplx2uint(ri_steps_tx, r.mixer._n_ri_step_bits,fmt='<u4')
    # ri_steps_rx_formatted = cplx2uint(ri_steps_rx, r.mixer._n_ri_step_bits,fmt='<u4')

    #set the filterbank channel maps
    chanmap_psb[tx_nearest_bins] = channels
    chanmap_pfb[channels] = rx_nearest_bins

    # tone_settings_dict = {'phase_incs_tx_formatted':phase_incs_tx_formatted,
    #                      'phase_incs_rx_formatted':phase_incs_rx_formatted,
    #                      'ri_steps_tx_formatted':ri_steps_tx_formatted,
    #                      'ri_steps_rx_formatted':ri_steps_rx_formatted,
    #                      'chanmap_psb':chanmap_psb,
    #                      'chanmap_pfb':chanmap_pfb,
    #                      'num_tones':num_tones}
    tone_settings_dict = {'control_buffer_data_values':v,
                            'control_buffer_data_indices':i,
                            'control_buffer_index':0,
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


def prepare_sweep_settings_fast(r_fast, config_dict, sweep_frequencies, detailed_output=False):
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
    nc = r_fast.mixer.n_chans
    fft_period_s = r_fast.mixer._n_upstream_chans / r_fast.mixer._upstream_oversample_factor / r_fast.adc_clk_hz
    fft_rbw_hz = 1./fft_period_s
    fft_tx_nbins = 2 * N_TX_FFT
    fft_rx_nbins = N_RX_FFT
    all_tx_bin_centers_hz = np.fft.fftfreq(fft_tx_nbins, 1. / r_fast.adc_clk_hz)
    all_rx_bin_centers_hz = np.fft.fftfreq(fft_rx_nbins, 1. / r_fast.adc_clk_hz)

    chanmap_psb = np.full((num_points,r_fast.psb_chanselect.n_chans_out), -1, dtype=int)
    chanmap_pfb  = np.full((num_points,r_fast.chanselect.n_chans_out), -1, dtype=int)

    skip_chanmap_psb=np.zeros(num_points,dtype=bool)
    skip_chanmap_pfb=np.zeros(num_points,dtype=bool)

    # phase_incs_tx_formatted_padded = np.zeros((num_points,nc),dtype='<i4')+32767
    # phase_incs_rx_formatted_padded = np.zeros((num_points,nc),dtype='<i4')+32767
    # ri_steps_tx_formatted_padded = np.zeros((num_points,nc),dtype='<u4')+65535
    # ri_steps_rx_formatted_padded = np.zeros((num_points,nc),dtype='<u4')+65535
        

    #get the DAC/ADC analog frequencies given any analog up/down conversion
    #get the DAC/ADC analog frequencies given any analog up/down conversion
    if udc_connected:
        dac_out_freqs = (sweep_frequencies - udc_lo_frequency) / udc_sideband
        adc_in_freqs = (sweep_frequencies - udc_lo_frequency) / udc_sideband
    else:
        dac_out_freqs = sweep_frequencies
        adc_in_freqs = sweep_frequencies

    duc_freqs = dac_out_freqs
    ddc_freqs = adc_in_freqs

    #get the DAC/ADC digitial frequencies given the Nyquist zone
    if dac_nyquist_zone == 1:
        dbb_freqs_tx = duc_freqs - duc_frequency
    elif dac_nyquist_zone == 2:
        dbb_freqs_tx = duc_freqs + duc_frequency
    else:
        raise ValueError(f'Invalid DAC nyquist zone ({dac_nyquist_zone})')

    if adc_nyquist_zone == 1:
        dbb_freqs_rx = ddc_freqs + ddc_frequency
    elif adc_nyquist_zone == 2:
        dbb_freqs_rx = ddc_freqs - ddc_frequency
    else:
        raise ValueError(f'Invalid ADC nyquist zone ({adc_nyquist_zone})')


    #check all tones are in within the baseband bandwidth
    txbbmin=np.min(all_tx_bin_centers_hz)
    txbbmax=np.max(all_tx_bin_centers_hz)+fft_rbw_hz
    rxbbmin=np.min(all_rx_bin_centers_hz)
    rxbbmax=np.max(all_rx_bin_centers_hz)+fft_rbw_hz

    if (dbb_freqs_tx > txbbmax).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth: dbb_freqs_tx={dbb_freqs_tx}')
    if (dbb_freqs_tx < txbbmin).any():
        raise ValueError(f'TX frequencies exceed baseband bandwidth: dbb_freqs_tx={dbb_freqs_tx}')
    if (dbb_freqs_rx > rxbbmax).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth: dbb_freqs_rx={dbb_freqs_rx}')
    if (dbb_freqs_rx < rxbbmin).any():
        raise ValueError(f'RX frequencies exceed baseband bandwidth: dbb_freqs_rx={dbb_freqs_rx}')
    
    #get the nearest filterbank center frequencies for each tone
    tx_nearest_bins = get_closest_bin_indices(dbb_freqs_tx, all_tx_bin_centers_hz)
    rx_nearest_bins = get_closest_bin_indices(dbb_freqs_rx, all_rx_bin_centers_hz)

    #get the frequency offsets for each tone
    tx_freq_offsets_hz = dbb_freqs_tx - all_tx_bin_centers_hz[tx_nearest_bins]
    rx_freq_offsets_hz = dbb_freqs_rx - all_rx_bin_centers_hz[rx_nearest_bins]
        
    #get the phase increments and ri steps for the mixer LOs
    phase_incs_tx = tx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    phase_incs_rx = rx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    ri_steps_tx = np.cos(phase_incs_tx) + 1j*np.sin(phase_incs_tx)
    ri_steps_rx = np.cos(phase_incs_rx) + 1j*np.sin(phase_incs_rx)
    # ri_steps_tx = np.exp(1j*phase_incs_tx)
    # ri_steps_rx = np.exp(1j*phase_incs_rx)
    
    print('phase_incs_tx',phase_incs_tx.shape,'\n',phase_incs_tx)
    print('phase_incs_rx',phase_incs_rx.shape,'\n',phase_incs_rx)
    print('ri_steps_tx',ri_steps_tx.shape,'\n',ri_steps_tx)
    print('ri_steps_rx',ri_steps_rx.shape,'\n',ri_steps_rx)

    # #format the phase increments and ri steps for the mixer LOs
    # phase_incs_tx_formatted = _format_phase_steps(phase_incs_tx,r.mixer._phase_bp,fmt='<i4')
    # phase_incs_rx_formatted = _format_phase_steps(phase_incs_rx,r.mixer._phase_bp,fmt='<i4')
    # ri_steps_tx_formatted = cplx2uint(ri_steps_tx, r.mixer._n_ri_step_bits,fmt='<u4')
    # ri_steps_rx_formatted = cplx2uint(ri_steps_rx, r.mixer._n_ri_step_bits,fmt='<u4')

    
    # v0 = prepare_control_buffer_data_fast(r,0,{'tx':{'phase_steps':phase_incs_tx[0],
    #                                         'ri_steps':ri_steps_tx[0]},
    #                                   'rx':{'phase_steps':phase_incs_rx[0],
    #                                         'ri_steps':ri_steps_rx[0]}})
    
    # allv = np.zeros((num_points, len(v0)),dtype=v0.dtype)
    allv={}
    alli={}
    allbuf = np.zeros((num_points,),dtype=int)
    allbuf[1::2] = 1

    for p in points:
        # #zero pad out to nchans for the fast write
        # phase_incs_tx_formatted_padded[p,:len(phase_incs_tx_formatted[p])] = phase_incs_tx_formatted[p]
        # phase_incs_rx_formatted_padded[p,:len(phase_incs_rx_formatted[p])] = phase_incs_rx_formatted[p]
        # ri_steps_tx_formatted_padded[p,:len(ri_steps_tx_formatted[p])] = ri_steps_tx_formatted[p]
        # ri_steps_rx_formatted_padded[p,:len(ri_steps_rx_formatted[p])] = ri_steps_rx_formatted[p]
        print('prep_sweep, prep_buf',p)
        allv[p], alli[p] = prepare_control_buffer_data_fast(r_fast,allbuf[p],
                                                            {'tx':{'phase_steps':phase_incs_tx[p],
                                                                   'ri_steps':ri_steps_tx[p]},
                                                             'rx':{'phase_steps':phase_incs_rx[p],
                                                                   'ri_steps':ri_steps_rx[p]}})
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

    # sweep_settings_dict = {'phase_incs_tx_formatted':phase_incs_tx_formatted_padded,
    #                      'phase_incs_rx_formatted':phase_incs_rx_formatted_padded,
    #                      'ri_steps_tx_formatted':ri_steps_tx_formatted_padded,
    #                      'ri_steps_rx_formatted':ri_steps_rx_formatted_padded,
    #                      'chanmap_psb':chanmap_psb,
    #                      'chanmap_pfb':chanmap_pfb,
    #                      'skip_chanmap_psb':skip_chanmap_psb,
    #                      'skip_chanmap_pfb':skip_chanmap_pfb,
    #                      'num_tones':num_tones}

    sweep_settings_dict = {'control_buffer_data_values':allv,
                            'control_buffer_data_indices':alli,
                            'control_buffer_index':allbuf,
                            'chanmap_psb':chanmap_psb,
                            'chanmap_pfb':chanmap_pfb,
                            'skip_chanmap_psb':skip_chanmap_psb,
                            'skip_chanmap_pfb':skip_chanmap_pfb,
                            'num_tones':num_tones}

    return sweep_settings_dict

def apply_sweep_step_fast(r, r_fast, sweep_settings, step_index, autosync=True):
    # phase_incs_tx_formatted = sweep_settings.get('phase_incs_tx_formatted')
    # phase_incs_rx_formatted = sweep_settings.get('phase_incs_rx_formatted')
    # ri_steps_tx_formatted   = sweep_settings.get('ri_steps_tx_formatted')
    # ri_steps_rx_formatted   = sweep_settings.get('ri_steps_rx_formatted')
    print('apply_step', step_index)
    allv= sweep_settings.get('control_buffer_data_values')
    alli= sweep_settings.get('control_buffer_data_indices')
    allbuf = sweep_settings.get('control_buffer_index')
    chanmap_psb   = sweep_settings.get('chanmap_psb')
    chanmap_pfb   = sweep_settings.get('chanmap_pfb')
    skip_chanmap_psb = sweep_settings.get('skip_chanmap_psb')
    skip_chanmap_pfb = sweep_settings.get('skip_chanmap_pfb')
    num_tones     = sweep_settings.get('num_tones')
    
    c1=not skip_chanmap_psb[step_index]
    c2=not skip_chanmap_pfb[step_index]
    if c1:
        print('set chanmap 1 (out)')
        # r_fast.psb_chanselect.set_channel_outmap(np.copy(chanmap_psb[step_index]))
        psb_chanselect_set_channel_outmap(r_fast,chanmap_psb[step_index])
        
        # while not (r.psb_chanselect.get_channel_outmap()==chanmap_psb[step_index]).all():
        #     print('waiting for psb chanmap to update')
        #     time.sleep(0.001)
        # print('psb chanmap updated')
    if c2:
        print('set chanmap 2 (in)')
        # r_fast.chanselect.set_channel_outmap(np.copy(chanmap_pfb[step_index]))
        chanselect_set_channel_outmap(r_fast,chanmap_pfb[step_index])
        # while not (r.chanselect.get_channel_outmap()==chanmap_pfb[step_index]).all():
        #     print('waiting for pfb chanmap to update')
        #     time.sleep(0.001)
        # print('pfb chanmap updated')

    print('apply_step, write_buf',step_index, allbuf[step_index])
    write_control_buffer_data_fast(r_fast,allbuf[step_index],allv[step_index],alli[step_index])
    
    print('apply_step, set_buf',step_index,allbuf[step_index])
    set_control_buffer_idx_fast(r_fast,allbuf[step_index])

    force_sync_fast(r_fast,0.00001)
    
    # if c1 or c2:
    #     _wait_for_acc(r_fast,0,0.0001)


    # fast_write_mixer(r_fast, 
    #                   phase_incs_tx_formatted[step_index],
    #                     phase_incs_rx_formatted[step_index],
    #                       ri_steps_tx_formatted[step_index],
    #                         ri_steps_rx_formatted[step_index])

    # if autosync:
    #     # time.sleep(autosync_time_delay)
    #     r_fast.sync.arm_sync(wait=False)
    #     time.sleep(autosync_time_delay)
    #     r_fast.sync.sw_sync()

    return

# def get_bram_addresses_mixer(r_fast):
#     phase_addrs_tx = []
#     phase_addrs_rx = []
#     ri_step_addrs_tx = []
#     ri_step_addrs_rx = []
#     nbytes = r_fast.mixer._n_serial_chans * 4 # phases in 4 byte words
#     for i in range(r_fast.mixer._n_parallel_chans):
#         ramname = f'{r_fast.mixer.prefix}tx_lo{i}_phase_inc'
#         phase_addrs_tx += [r_fast.mixer.host.transport._get_device_address(ramname)]
#         ramname = f'{r_fast.mixer.prefix}rx_lo{i}_phase_inc'
#         phase_addrs_rx += [r_fast.mixer.host.transport._get_device_address(ramname)]
#         ramname = f'{r_fast.mixer.prefix}tx_lo{i}_ri_step'
#         ri_step_addrs_tx += [r_fast.mixer.host.transport._get_device_address(ramname)]
#         ramname = f'{r_fast.mixer.prefix}rx_lo{i}_ri_step'
#         ri_step_addrs_rx += [r_fast.mixer.host.transport._get_device_address(ramname)]
#     bram_addresses_mixer = {'phase_addrs_tx':phase_addrs_tx,
#                             'phase_addrs_rx':phase_addrs_rx,
#                             'ri_step_addrs_tx':ri_step_addrs_tx,
#                             'ri_step_addrs_rx':ri_step_addrs_rx,
#                             'nbytes':nbytes}
#     return bram_addresses_mixer

# def fast_write_mixer(r_fast, phase_incs_tx_formatted,phase_incs_rx_formatted,ri_steps_tx_formatted,ri_steps_rx_formatted):

#     if not hasattr(r_fast,'bram_addresses_mixer'):
#         r_fast.bram_addresses_mixer = get_bram_addresses_mixer(r_fast)

#     phase_addrs_tx = r_fast.bram_addresses_mixer['phase_addrs_tx']
#     phase_addrs_rx = r_fast.bram_addresses_mixer['phase_addrs_rx']
#     ri_step_addrs_tx = r_fast.bram_addresses_mixer['ri_step_addrs_tx']
#     ri_step_addrs_rx = r_fast.bram_addresses_mixer['ri_step_addrs_rx']
#     nbytes = r_fast.bram_addresses_mixer['nbytes']
    
#     phase_incs_tx_formatted=phase_incs_tx_formatted.reshape(r_fast.mixer._n_parallel_chans, r_fast.mixer._n_serial_chans)
#     phase_incs_rx_formatted=phase_incs_rx_formatted.reshape(r_fast.mixer._n_parallel_chans, r_fast.mixer._n_serial_chans)
#     ri_steps_tx_formatted=ri_steps_tx_formatted.reshape(r_fast.mixer._n_parallel_chans, r_fast.mixer._n_serial_chans)
#     ri_steps_rx_formatted=ri_steps_rx_formatted.reshape(r_fast.mixer._n_parallel_chans, r_fast.mixer._n_serial_chans)
    
#     # Seemingly can't write more than 512 bytes in one go.
#     # Assume nbytes is a multiple of 512
#     # n_write = (nbytes // 512)
#     maxwrite=512
#     n_write = (nbytes // maxwrite)
#     write_idxs = np.arange(n_write)
#     readback_delay = 0.00001
#     max_retries = 1000
#     for i in range(len(phase_addrs_tx)):
#         phase_incs_tx_bytes = phase_incs_tx_formatted[i].tobytes()
#         phase_incs_rx_bytes = phase_incs_rx_formatted[i].tobytes()
#         ri_steps_tx_bytes = ri_steps_tx_formatted[i].tobytes()
#         ri_steps_rx_bytes = ri_steps_rx_formatted[i].tobytes()
#         for j in write_idxs:
#             raw = phase_incs_tx_bytes[j*maxwrite:(j+1)*maxwrite]
#             r_fast.mixer.host.transport.axil_mm[phase_addrs_tx[i]+j*maxwrite:phase_addrs_tx[i] +(j+1)*maxwrite] = raw
#             time.sleep(readback_delay)
#             ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_tx[i]+j*maxwrite:phase_addrs_tx[i] +(j+1)*maxwrite]
#             retry_count=0
#             while ret!=raw:
#                 #retry write
#                 retry_count+=1
#                 r_fast.mixer.host.transport.axil_mm[phase_addrs_tx[i]+j*maxwrite:phase_addrs_tx[i] +(j+1)*maxwrite] = raw
#                 time.sleep(readback_delay*retry_count)
#                 ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_tx[i]+j*maxwrite:phase_addrs_tx[i] +(j+1)*maxwrite]
#                 if retry_count>max_retries:
#                     raise IOError(f'Failed to write phase_incs_tx {j} to BRAM after {max_retries} tries')
#         for j in write_idxs:
#             raw = phase_incs_rx_bytes[j*maxwrite:(j+1)*maxwrite]
#             r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite] = raw
#             time.sleep(readback_delay)
#             ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite]
#             retry_count=0
#             while ret!=raw:
#                 #retry write
#                 retry_count+=1
#                 r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite] = raw
#                 time.sleep(readback_delay*retry_count)
#                 ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite]
#                 if retry_count>max_retries:
#                     raise IOError(f'Failed to write phase_incs_rx {j} to BRAM after {max_retries} tries')
#         for j in write_idxs:
#             raw = ri_steps_tx_bytes[j*maxwrite:(j+1)*maxwrite]
#             r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite] = raw
#             time.sleep(readback_delay)
#             ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite]
#             retry_count=0
#             while ret!=raw:
#                 #retry write
#                 retry_count+=1
#                 r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite] = raw
#                 time.sleep(readback_delay*retry_count)
#                 ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite]
#                 if retry_count>max_retries:
#                     raise IOError(f'Failed to write ri_steps_tx {j} to BRAM after {max_retries} tries')
#         for j in write_idxs:
#             raw = ri_steps_rx_bytes[j*maxwrite:(j+1)*maxwrite]
#             r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite] = raw
#             time.sleep(readback_delay)
#             ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite]
#             retry_count=0
#             while ret!=raw:
#                 #retry write
#                 retry_count+=1
#                 r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite] = raw
#                 time.sleep(readback_delay*retry_count)
#                 ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite]
#                 if retry_count>max_retries:
#                     raise IOError(f'Failed to write ri_steps_rx {j} to BRAM after {max_retries} tries')
            
#     #     for j in write_idxs:
#     #         raw = phase_incs_rx_bytes[j*maxwrite:(j+1)*maxwrite]
#     #         r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite] = raw
#     #         time.sleep(0.00001)
#     #         ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite]
#     #         if ret==raw:
#     #             pass #print(f'phase_incs_rx {j:2d} write successful')
#     #         else:
#     #             # print(f'phase_incs_rx {j:2d} write failed')
#     #             for xx in range(10):
#     #                 # print('retrying write', xx)
#     #                 r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite] = raw
#     #                 time.sleep(0.00001)
#     #                 ret = r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*maxwrite:phase_addrs_rx[i] +(j+1)*maxwrite]
#     #                 if ret==raw:
#     #                     # print('retry successful')
#     #                     break
#     #             if xx==9:
#     #                 print('\t\t\t\tretry failed')
            
#     #         # while r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*512:phase_addrs_rx[i] +(j+1)*512] != raw:
#     #         #     time.sleep(0.00001)
#     #         #     r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*512:phase_addrs_rx[i] +(j+1)*512]=raw
                
#     #         # r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*512:phase_addrs_rx[i] +(j+1)*512] = phase_incs_rx_bytes[j*512:(j+1)*512]
#     #         # # while not (np.frombuffer(r_fast.mixer.host.transport.axil_mm[phase_addrs_rx[i]+j*512:phase_addrs_rx[i] +(j+1)*512],dtype='<i4').copy() == np.frombuffer(phase_incs_rx_bytes[j*512:(j+1)*512],dtype='<i4').copy()).all():
#     #         # #     print('waiting for phase_incs_rx to update')
#     #         # #     time.sleep(0.001)
#     #         # r_fast.mv_as_int[(ri_step_addrs_tx[i]+j*512)//4:(ri_step_addrs_tx[i] +(j+1)*512)//4] = memoryview(ri_steps_tx_bytes[(j*512):((j+1)*512)]).cast('I')
#     #     for j in write_idxs:
#     #         raw = ri_steps_tx_bytes[j*maxwrite:(j+1)*maxwrite]
#     #         r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite] = raw
#     #         time.sleep(0.00001)
#     #         ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite]
#     #         if ret==raw:
#     #             pass #print(f'ri_steps_tx   {j:2d} write successful')
#     #         else:
#     #             # print(f'ri_steps_tx   {j:2d} write failed')
#     #             for xx in range(10):
#     #                 # print('retrying write', xx)
#     #                 r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite] = raw
#     #                 time.sleep(0.00001)
#     #                 ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*maxwrite:ri_step_addrs_tx[i] +(j+1)*maxwrite]
#     #                 if ret==raw:
#     #                     # print('retry successful')
#     #                     break
#     #             if xx==9:
#     #                 print('\t\t\t\tretry failed')
#     #         # while r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*512:ri_step_addrs_tx[i] +(j+1)*512] != raw:
#     #         #     time.sleep(0.00001)
#     #         #     r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*512:ri_step_addrs_tx[i] +(j+1)*512]=raw
                
#     #         # r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*512:ri_step_addrs_tx[i] +(j+1)*512] = ri_steps_tx_bytes[j*512:(j+1)*512]
#     #         # # while not (np.frombuffer(r_fast.mixer.host.transport.axil_mm[ri_step_addrs_tx[i]+j*512:ri_step_addrs_tx[i] +(j+1)*512],dtype='<i4').copy() == np.frombuffer(ri_steps_tx_bytes[j*512:(j+1)*512],dtype='<i4').copy()).all():
#     #         # #     print('waiting for ri_steps_tx to update')
#     #         # #     time.sleep(0.001)
#     #         # r_fast.mv_as_int[(ri_step_addrs_rx[i]+j*512)//4:(ri_step_addrs_rx[i] +(j+1)*512)//4] = memoryview(ri_steps_rx_bytes[(j*512):((j+1)*512)]).cast('I')
#     #     for j in write_idxs:
#     #         raw = ri_steps_rx_bytes[j*maxwrite:(j+1)*maxwrite]
#     #         r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite] = raw
#     #         time.sleep(0.00001)
#     #         ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite]
#     #         if ret==raw:
#     #             pass #print(f'ri_steps_rx   {j:2d} write successful')
#     #         else:
#     #             # print(f'ri_steps_rx   {j:2d} write failed')
#     #             for xx in range(10):
#     #                 # print('retrying write', xx)
#     #                 r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite] = raw
#     #                 time.sleep(0.00001)
#     #                 ret = r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*maxwrite:ri_step_addrs_rx[i] +(j+1)*maxwrite]
#     #                 if ret==raw:
#     #                     # print('retry successful')
#     #                     break
#     #             if xx==9:
#     #                 print('\t\t\t\tretry failed')
#     #         # while r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*512:ri_step_addrs_rx[i] +(j+1)*512] != raw:
#     #         #     time.sleep(0.00001)
#     #         #     r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*512:ri_step_addrs_rx[i] +(j+1)*512]=raw
                
#     #         # r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*512:ri_step_addrs_rx[i] +(j+1)*512] = ri_steps_rx_bytes[j*512:(j+1)*512]
#     #         # # while not (np.frombuffer(r_fast.mixer.host.transport.axil_mm[ri_step_addrs_rx[i]+j*512:ri_step_addrs_rx[i] +(j+1)*512],dtype='<i4').copy() == np.frombuffer(ri_steps_rx_bytes[j*512:(j+1)*512],dtype='<i4').copy()).all():
#     #         # #     print('waiting for ri_steps_rx to update')
#     #         # #     time.sleep(0.001)
#     # # r_fast.mixer.host.transport.axil_mm.flush()



def apply_tone_frequency_settings_fast(r, r_fast, fast_tone_frequency_settings, autosync=True):
    
    v=fast_tone_frequency_settings.get('control_buffer_data_values')
    i=fast_tone_frequency_settings.get('control_buffer_data_indices')
    buf = fast_tone_frequency_settings.get('control_buffer_index')
    # phase_incs_tx_formatted = fast_tone_frequency_settings.get('phase_incs_tx_formatted')
    # phase_incs_rx_formatted = fast_tone_frequency_settings.get('phase_incs_rx_formatted')
    # ri_steps_tx_formatted   = fast_tone_frequency_settings.get('ri_steps_tx_formatted')
    # ri_steps_rx_formatted   = fast_tone_frequency_settings.get('ri_steps_rx_formatted')
    chanmap_psb   = fast_tone_frequency_settings.get('chanmap_psb')
    chanmap_pfb   = fast_tone_frequency_settings.get('chanmap_pfb')
    # num_tones     = fast_tone_frequency_settings.get('num_tones')
    c1 = chanmap_psb is not None
    c2 = chanmap_pfb is not None

    if c1 is not None:
        # print('chanmap_psb set')
        psb_chanselect_set_channel_outmap(r_fast,chanmap_psb)
    if c2 is not None:
        # print('chanmap_pfb set')
        chanselect_set_channel_outmap(r_fast,chanmap_pfb)
    

    write_control_buffer_data_fast(r_fast,buf,v,i)
    set_control_buffer_idx_fast(r_fast,buf)
    if c1 or c2:
        _wait_for_acc(r_fast,0,0.0001)

    # fast_write_mixer(r_fast,
    #                   phase_incs_tx_formatted,
    #                     phase_incs_rx_formatted,
    #                       ri_steps_tx_formatted,
    #                         ri_steps_rx_formatted)

    # if autosync:
    #     # time.sleep(autosync_time_delay)
    #     r_fast.sync.arm_sync(wait=False)
    #     time.sleep(autosync_time_delay)
    #     r_fast.sync.sw_sync()


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
   
    r.sync.arm_sync(wait=False)
    time.sleep(0.001)
    r.sync.sw_sync()

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
        chanmap_psb  = psb_chanselect_get_channel_outmap(r)
        chanmap_pfb  = chanselect_get_channel_outmap(r)
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
    
    control_buffer = read_from_current_control_buffer(r)
    scaling_tx = control_buffer['tx']['scaling']
    scaling_rx = control_buffer['rx']['scaling']
    # scaling_tx = np.zeros(num_tones_tx,dtype='>u4')
    # scaling_rx = np.zeros(num_tones_rx,dtype='>u4')
    # for i in range(min(r.mixer._n_parallel_chans, num_tones)):   
    #     scaling_tx[i::r.mixer._n_parallel_chans] = np.frombuffer(r.mixer.read(f'tx_lo{i}_scale',4*num_tones_tx),dtype='>u4')
    #     scaling_rx[i::r.mixer._n_parallel_chans] = np.frombuffer(r.mixer.read(f'rx_lo{i}_scale',4*num_tones_rx),dtype='>u4')
    # scaling_tx = _invert_format_amp_scale(scaling_tx, r.mixer._n_scale_bits)
    # scaling_rx = _invert_format_amp_scale(scaling_rx, r.mixer._n_scale_bits)
    return scaling_tx[:num_tones]

def set_tone_amplitudes(r, config_dict, tone_amplitudes,autosync=True):
    """
    Set the tone amplitude scale factors in the RFSOC.

    Currently sets both halfs of the double buffer to the same value.
    This is not ideal, but for now it will 
    """
    tone_amplitudes = np.atleast_1d(tone_amplitudes)
    num_tones = len(tone_amplitudes)

    # buf = get_control_buffer_idx(r)
    for buf in [0,1]:
        v = prepare_control_buffer_data(r,buf,{'tx':{'scaling':tone_amplitudes},
                                    'rx':{'scaling':tone_amplitudes}})
    
        write_control_buffer_data(r,buf,v)


    # scaling = _format_amp_scale(tone_amplitudes, r.mixer._n_scale_bits)
    # for i in range(min(r.mixer._n_parallel_chans, num_tones)):   
    #     r.mixer.write(f'tx_lo{i}_scale', scaling[i::r.mixer._n_parallel_chans].tobytes())
    #     r.mixer.write(f'rx_lo{i}_scale', scaling[i::r.mixer._n_parallel_chans].tobytes())

    # if autosync:
    #     # time.sleep(autosync_time_delay)
    #     r.sync.arm_sync(wait=False)
    #     time.sleep(autosync_time_delay)
    #     r.sync.sw_sync()

    return

def get_tone_phases(r, config_dict, num_tones=None):
    """
    Query the RFSOC for the current tone phase offsets.
    Note that the returned values are in the range [-pi,pi] regardless of how they were set.
    """
    if num_tones is None:
        #get the number of active filterbanck channels (assumes anything not -1 is a channel)
        chanmap_psb  = psb_chanselect_get_channel_outmap(r)
        chanmap_pfb  = chanselect_get_channel_outmap(r)
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
    
    control_buffer = read_from_current_control_buffer(r)
    phase_offsets_tx = control_buffer['tx']['phase_offsets']
    phase_offsets_rx = control_buffer['rx']['phase_offsets']

    # phase_offsets_tx = np.zeros(num_tones_tx,dtype='>i4')
    # phase_offsets_rx = np.zeros(num_tones_rx,dtype='>i4')
    # for i in range(min(r.mixer._n_parallel_chans, num_tones)):   
    #     phase_offsets_tx[i::r.mixer._n_parallel_chans] = np.frombuffer(r.mixer.read(f'tx_lo{i}_phase_offset',4*num_tones_tx),dtype='>i4')
    #     phase_offsets_rx[i::r.mixer._n_parallel_chans] = np.frombuffer(r.mixer.read(f'rx_lo{i}_phase_offset',4*num_tones_rx),dtype='>i4')
    # phase_offsets_tx = _invert_format_phase_offsets(phase_offsets_tx, r.mixer._phase_offset_bp)
    # phase_offsets_rx = _invert_format_phase_offsets(phase_offsets_rx, r.mixer._phase_offset_bp)
    return phase_offsets_tx[:num_tones]

def set_tone_phases(r, config_dict, tone_phases, autosync=True):
    """
    Set the tone phase offsets in the RFSOC.
    """
    tone_phases = np.atleast_1d(tone_phases)
    num_tones = len(tone_phases)
    buf = get_control_buffer_idx(r)
    v = prepare_control_buffer_data(r,buf,{'tx':{'phase_offsets':tone_phases},
                                    'rx':{'phase_offsets':tone_phases}})
    
    write_control_buffer_data(r,buf,v)
    
    # phase_offsets = _format_phase_offsets(tone_phases,r.mixer._phase_offset_bp)
    # for i in range(min(r.mixer._n_parallel_chans, num_tones)):
    #     r.mixer.write(f'tx_lo{i}_phase_offset', phase_offsets[i::r.mixer._n_parallel_chans].tobytes())
    #     r.mixer.write(f'rx_lo{i}_phase_offset', phase_offsets[i::r.mixer._n_parallel_chans].tobytes())
    # if autosync:
    #     # time.sleep(autosync_time_delay)
    #     r.sync.arm_sync(wait=False)
    #     time.sleep(autosync_time_delay)
    #     r.sync.sw_sync()
    return





def psb_chanselect_set_channel_outmap_pre79(r, outmap):
    """
    *** vectorised version of set_channel_outmap for psb_chanselect***
    
    Remap the channels such that the channel outmap[i]
    emerges out of the reorder map in position i.

    The provided map must be `r.psb_chanselect.n_chans_out` elements long, else
    `ValueError` is raised

    :param outmap: The outmap to which data should be mapped. I.e., if
        `outmap[0] = 16`, then the first channel out of the reorder block
        will be channel 16. 
    :type outmap: list of int

    """
    # default to outputting last input
    # serial_maps = (r.psb_chanselect.n_chans_in - 1) * np.ones([r.psb_chanselect._expansion_factor, r.psb_chanselect._reorder_depth])
    if not hasattr(r.psb_chanselect,'_serial_maps_convenience'):
        r.psb_chanselect._serial_maps_convenience = (r.psb_chanselect.n_chans_in - 1) * np.ones([r.psb_chanselect._expansion_factor, r.psb_chanselect._reorder_depth])
    serial_maps = r.psb_chanselect._serial_maps_convenience

    outmap = np.array(outmap, dtype=int)
    nout = len(outmap)

    # outchans = np.arange(r.psb_chanselect.n_chans_out)
    if not hasattr(r.psb_chanselect,'_outchans_convenience'):
        r.psb_chanselect._outchans_convenience = np.arange(r.psb_chanselect.n_chans_out)
    outchans = r.psb_chanselect._outchans_convenience
    # Which parallel path does a given output channel map to
    # block_id = (outchans // r.psb_chanselect.n_parallel_samples) % r.psb_chanselect._expansion_factor
    if not hasattr(r.psb_chanselect,'_block_id_convenience'):
        r.psb_chanselect._block_id_convenience = (outchans // r.psb_chanselect.n_parallel_samples) % r.psb_chanselect._expansion_factor
    block_id = r.psb_chanselect._block_id_convenience
    # Which serial position in this path does a channel map to
    # block_s_offset = (outchans // r.psb_chanselect.n_parallel_chans_out)
    if not hasattr(r.psb_chanselect,'_block_s_offset_convenience'):
        r.psb_chanselect._block_s_offset_convenience = (outchans // r.psb_chanselect.n_parallel_chans_out)
    block_s_offset = r.psb_chanselect._block_s_offset_convenience
    
    # Which parallel position in this word in this path
    # block_p_offset = (outchans % r.psb_chanselect.n_parallel_samples)
    if not hasattr(r.psb_chanselect,'_block_p_offset_convenience'):
        r.psb_chanselect._block_p_offset_convenience = (outchans % r.psb_chanselect.n_parallel_samples)
    block_p_offset = r.psb_chanselect._block_p_offset_convenience

    # Combined position in a block
    # block_offset = block_s_offset * r.psb_chanselect.n_parallel_samples + block_p_offset
    if not hasattr(r.psb_chanselect,'_block_offset_convenience'):
        r.psb_chanselect._block_offset_convenience = block_s_offset * r.psb_chanselect.n_parallel_samples + block_p_offset
    block_offset = r.psb_chanselect._block_offset_convenience

    # We want the user-select channel to end up in position `block_offset` of the block `block_id`
    # for i in range(nout):
    #     serial_maps[block_id[i], block_offset[i]] = outmap[i]
    serial_maps[block_id[:nout], block_offset[:nout]] = outmap[:nout]
    serial_maps = np.array(serial_maps, dtype=r.psb_chanselect._map_format)
    
    for i in range(r.psb_chanselect._expansion_factor):
        try:
            # if using fast firmware interface
            offset=r.psb_chanselect.host.transport._get_device_address(f'{r.psb_chanselect.prefix}map{i}_{r.psb_chanselect._map_reg}')
            r.psb_chanselect.host.transport.axil_mm[offset:offset+len(serial_maps[i].tobytes())]=serial_maps[i].astype('<i4').tobytes()
        except AttributeError:
            r.psb_chanselect.write(f'map{i}_{r.psb_chanselect._map_reg}', serial_maps[i].tobytes())



def psb_chanselect_get_channel_outmap_pre79(r):
        """
        *** vectorised version of get_channel_outmap for psb_chanselect***

        Read the currently loaded reorder map.

        :return: The reorder map currently loaded. Entry `i` in this map is the
            channel number which emerges in the `i`th output position.
        :rtype: list
        """
        nbytes = r.psb_chanselect._reorder_depth * np.dtype(r.psb_chanselect._map_format).itemsize
        serial_maps = np.zeros([r.psb_chanselect._expansion_factor, r.psb_chanselect._reorder_depth])
        for i in range(r.psb_chanselect._expansion_factor):
            serial_maps[i] = np.frombuffer(r.psb_chanselect.read(f'map{i}_{r.psb_chanselect._map_reg}', nbytes), dtype=r.psb_chanselect._map_format)

        ##not used:
        ## # Which serial position in each path does a channel map to
        ## block_s_offset = serial_maps // r.psb_chanselect.n_parallel_samples
        ## # Which parallel position in this word in this path
        ## block_p_offset = serial_maps % r.psb_chanselect.n_parallel_samples


        # outmap = np.zeros(r.psb_chanselect.n_chans_out, dtype=int)
        # for i in range(r.psb_chanselect._expansion_factor):
        #     for j in range(r.psb_chanselect._reorder_depth):
        #         s_off = j // r.psb_chanselect.n_parallel_samples
        #         p_off = j % r.psb_chanselect.n_parallel_samples
        #         outmap[i * r.psb_chanselect.n_parallel_samples + s_off*r.psb_chanselect.n_parallel_chans_out + p_off] = serial_maps[i, j]

        #i, j = np.indices((r.psb_chanselect.expansion_factor, r.psb_chanselect.reorder_depth))
        if not hasattr(r.psb_chanselect,'_i_j_convenience'):
            r.psb_chanselect._i_j_convenience = np.indices((r.psb_chanselect._expansion_factor, r.psb_chanselect._reorder_depth))
        i, j = r.psb_chanselect._i_j_convenience

        # s_off = j // r.psb_chanselect.n_parallel_samples
        if not hasattr(r.psb_chanselect,'_s_off_convenience'):
            r.psb_chanselect._s_off_convenience = j // r.psb_chanselect.n_parallel_samples
        s_off = r.psb_chanselect._s_off_convenience

        # p_off = j % r.psb_chanselect.n_parallel_samples
        if not hasattr(r.psb_chanselect,'_p_off_convenience'):
            r.psb_chanselect._p_off_convenience = j % r.psb_chanselect.n_parallel_samples
        p_off = r.psb_chanselect._p_off_convenience
        
        #indices = i * r.psb_chanselect.n_parallel_samples + s_off * r.psb_chanselect.n_parallel_chans_out + p_off
        if not hasattr(r.psb_chanselect,'_indices_convenience'):
            r.psb_chanselect._indices_convenience = i * r.psb_chanselect.n_parallel_samples + s_off * r.psb_chanselect.n_parallel_chans_out + p_off
        indices = r.psb_chanselect._indices_convenience

        outmap = np.zeros(r.psb_chanselect.n_chans_out, dtype=int)
        outmap[indices.ravel()] = serial_maps.ravel()


        return outmap




def psb_chanselect_set_channel_outmap_post79(r, outmap):
    """
    for now we just call the built in function

    plan to vectorise it for speed later

    """
    r.psb_chanselect.set_channel_outmap(outmap)


def psb_chanselect_get_channel_outmap_post79(r):
        """
        for now we just call the built in function
        plan to vectorise it for speed later
        """
        return r.psb_chanselect.get_channel_outmap()

def psb_chanselect_set_channel_outmap(r, outmap, descramble_input=None):
    """
    Wrapper to select pre-v7.9 or post-v7.9 version of psb_chanselect_set_channel_outmap
    """
    #are we pre v7.9?
    pre79 = type(r.psb_chanselect) == souk_mkid_readout.blocks.chanreorder.ChanReorderMultiSampleIn

    # #are we post 79?
    # post79 = type(r.psb_chanselect) == souk_mkid_readout.blocks.chanreorder.VaccReorderMultiSampleIn

    # #could also check the firmware version in the fpga but that would require a register read:
    # pre79 = r.fpga.get_firmware_version() < ('7','9','0','0')
    
    if pre79:
        psb_chanselect_set_channel_outmap_pre79(r, outmap)
    else:
        psb_chanselect_set_channel_outmap_post79(r, outmap)

def psb_chanselect_get_channel_outmap(r, descramble_input=None):
    """
    Wrapper to select pre-79 or post-79 version of psb_chanselect_get_channel_outmap
    """
    #are we pre v7.9?
    pre79 = type(r.psb_chanselect) == souk_mkid_readout.blocks.chanreorder.ChanReorderMultiSampleIn

    # #are we post 79?
    # post79 = type(r.psb_chanselect) == souk_mkid_readout.blocks.chanreorder.VaccReorderMultiSampleIn

    # #could also check the firmware version in the fpga but that would require a register read
    # pre79 = r.fpga.get_firmware_version() < ('7','9','0','0')

    if pre79:
        return psb_chanselect_get_channel_outmap_pre79(r)
    else:
        return psb_chanselect_get_channel_outmap_post79(r)
    

def chanselect_set_channel_outmap(r, outmap, descramble_input=None):
    """
    *** vectorised version of set_channel_outmap for chanselect***

    Remap the channels such that the channel outmap[i]
    emerges out of the reorder map in position i.

    The provided map must be `r.chanselect.n_chans_out` elements long, else
    `ValueError` is raised

    :param outmap: The outmap to which data should be mapped. I.e., if
        `outmap[0] = 16`, then the first channel out of the reorder block
        will be channel 16. 
    :type outmap: list of int

    :param descramble_input: If True, descramble the provided channel map.
        If not provided, descramble if the _descramble_default attribute is True.
    :type descramble_input: bool

    """

    outmap = np.array(outmap, dtype=int)

    serial_map = np.zeros(r.chanselect._reorder_depth)

    #  parallel_map = (r.chanselect._reduction_factor + 1) * np.ones(r.chanselect._reorder_depth)
    if not hasattr(r.chanselect,'_parallel_map_convenience'):
        r.chanselect._parallel_map_convenience = (r.chanselect._reduction_factor + 1) * np.ones(r.chanselect._reorder_depth)
    parallel_map = r.chanselect._parallel_map_convenience.copy()
    
    nout = len(outmap)
    outmap_isnt_n1 = outmap != -1
    if descramble_input or (descramble_input is None and r.chanselect._descramble_default):
        #for i in range(nout):
        #    if outmap[i] == -1:
        #        continue
        #    outmap[i] = r.chanselect._descramble_order[outmap[i]]
        outmap[outmap_isnt_n1] = r.chanselect._descramble_order[outmap[outmap_isnt_n1]]

    # block_id = np.zeros(nout)
    # block_s_offset = np.zeros(nout)
    # block_p_offset = np.zeros(nout)

    # block_id[:] = outmap // r.chanselect.n_parallel_chans_in
    # block_s_offset[:] = (outmap % r.chanselect.n_parallel_chans_in) % r.chanselect.n_parallel_samples
    # block_p_offset[:] = (outmap % r.chanselect.n_parallel_chans_in) // r.chanselect.n_parallel_samples

    block_id = outmap // r.chanselect.n_parallel_chans_in
    opp      = outmap % r.chanselect.n_parallel_chans_in
    block_s_offset = (opp) % r.chanselect.n_parallel_samples
    block_p_offset = (opp) // r.chanselect.n_parallel_samples

    serial_map[0:nout] = (block_id * r.chanselect.n_parallel_samples) + block_s_offset

    # parallel_map[0:nout] = block_p_offset
    # parallel_map[0:nout][outmap == -1] = r.chanselect._reduction_factor + 1

    parallel_map[:nout] = np.where(outmap_isnt_n1, block_p_offset, r.chanselect._reduction_factor + 1)


    try:
        # if using fast firmware interface
        addr = r.chanselect.host.transport._get_device_address(f'{r.chanselect.prefix}map0_{r.chanselect._map_reg}')
        r.chanselect.host.transport.axil_mm[addr:addr+len(serial_map)*4]= serial_map.astype('<i4').tobytes()
        addr = r.chanselect.host.transport._get_device_address(f'{r.chanselect.prefix}pmap')
        r.chanselect.host.transport.axil_mm[addr:addr+len(parallel_map)*4]= parallel_map.astype('<i4').tobytes()
    except AttributeError:
        r.chanselect.write(f'map0_{r.chanselect._map_reg}', serial_map.astype(r.chanselect._map_format).tobytes())
        r.chanselect.write('pmap', parallel_map.astype(r.chanselect._pmap_format).tobytes())



def chanselect_get_channel_outmap(r, descramble_input=None):
    """
    Read the currently loaded reorder map.

    :param descramble_input: If True, descramble the recovered channel map.
        If not provided, descramble if the _descramble_default attribute is True.
    :type descramble_input: bool

    :return: The reorder map currently loaded. Entry `i` in this map is the
        channel number which emerges in the `i`th output position.
    :rtype: list
    """

    nbytes = r.chanselect._reorder_depth * np.dtype(r.chanselect._map_format).itemsize
    serial_map = np.frombuffer(r.chanselect.read(f'map0_{r.chanselect._map_reg}', nbytes), dtype=r.chanselect._map_format)
    nbytes = r.chanselect._reorder_depth * np.dtype(r.chanselect._pmap_format).itemsize
    parallel_map = np.frombuffer(r.chanselect.read('pmap', nbytes), dtype=r.chanselect._pmap_format)

    block_id = serial_map // r.chanselect.n_parallel_samples
    block_s_offset = serial_map % r.chanselect.n_parallel_samples
    block_p_offset = parallel_map

    outmap = r.chanselect.n_parallel_chans_in * block_id + block_s_offset + (r.chanselect.n_parallel_samples * block_p_offset)
    outmap[parallel_map == r.chanselect._reduction_factor + 1] = -1
    if descramble_input or (descramble_input is None and r.chanselect._descramble_default):
        # for i in range(len(outmap)):
        #     if outmap[i] == -1:
        #         continue
        #     outmap[i] = r.chanselect._scramble_order[outmap[i]]
        
        outmap_isnt_n1 = outmap != -1
        outmap[outmap_isnt_n1] = r.chanselect._scramble_order[outmap[outmap_isnt_n1]]
        
    return outmap





def check_input_saturation(r,iterations=1,saturation_bits=adc_saturation_bits,threshold=0.95):
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
    i_over = imax >= 1.0*threshold
    i_under = imin <= -1.0*threshold
    q_over = qmax >= 1.0*threshold
    q_under = qmin <= -1.0*threshold
    any_saturation = bool(i_over|i_under|q_over|q_under)
    integration_time = ss.size/r.adc_clk_hz
    details = {'imax_fs':imax,'imin_fs':imin,'qmax_fs':qmax,'qmin_fs':qmin,
               'integration_time':integration_time,
               'threshold':threshold}
    return any_saturation, details

def check_output_saturation(r,iterations=1,saturation_bits=dac_saturation_bits,threshold = 0.95):
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
    i0_over,i1_over = i0max >= 1.0*threshold, i1max >= 1.0*threshold
    i0_under,i1_under = i0min <= -1.0*threshold, i1min <= -1.0*threshold
    q0_over,q1_over = q0max >= 1.0*threshold, q1max >= 1.0*threshold
    q0_under,q1_under = q0min <= -1.0*threshold, q1min <= -1.0*threshold
    any0_saturation = i0_over|i0_under|q0_over|q0_under
    any1_saturation = i1_over|i1_under|q1_over|q1_under
    any_saturation = bool(any0_saturation|any1_saturation)
    integration_time = ss0.size/r.adc_clk_hz
    details = {'i0max_fs':i0max,'i0min_fs':i0min,'q0max_fs':q0max,'q0min_fs':q0min,
               'i1max_fs':i1max,'i1min_fs':i1min,'q1max_fs':q1max,'q1min_fs':q1min,
               'integration_time':integration_time,
               'threshold':threshold}
    return any_saturation, details

def check_adc_over_range(r,detailed_output=False):
    """
    Check to see if the ADC block has gone over range.
    """

    ovr_flag_dict = r.rfdc.get_rts_flags()
    ovr_range = ovr_flag_dict['rts_over_range']
    ovr_volt  = ovr_flag_dict['rts_over_voltage']
    ovr_cm_over = ovr_flag_dict['rts_over_cm_over_voltage']
    ovr_cm_under = ovr_flag_dict['rts_over_cm_under_voltage']
    any_over = ovr_range | ovr_volt | ovr_cm_over | ovr_cm_under
    if any_over:
        print('ADC Over Range Flag Detected')
        print(r.rfdc.get_status())
    
    
    return any_over, ovr_flag_dict if detailed_output else any_over

def check_adc_threshold(r,detailed_output=False):
    """
    Check to see if the ADC block has exceeded the threshold.
    """

    ovr_flag_dict = r.rfdc.get_rts_flags()
    ovr_thresh1 = ovr_flag_dict['rts_over_threshold1']
    ovr_thresh2  = ovr_flag_dict['rts_over_threshold2']
    thresh_exceeded = ovr_thresh1 | ovr_thresh2
    if thresh_exceeded:
        print('ADC Over Threshold Flag Detected')
        print(r.rfdc.get_status())
    return thresh_exceeded, ovr_flag_dict if detailed_output else thresh_exceeded

def reset_adc_over_range(r):
    """
    Reset the ADC over voltage / over range flags.
    """
    r.rfdc.reset_rts_flags(over_range=True, over_voltage=True)
    ovr,flags = r.rfdc.check_adc_over_range(r,detailed_output=True)
    if ovr:
        raise RuntimeError('Failed to reset ADC over range flags, check signal level.')
    return 
    
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


def maximise_tx_power(r,config_dict=None):
    """
    Maximise the tx amplitudes, psb fft shift and psb scale, avoiding saturation.
    """
    init_dac_saturation, init_dac_levels = check_output_saturation(r,iterations=50)
    if init_dac_saturation:
        raise ValueError('DAC saturation detected prior to optimisation')
    init_amps = get_tone_amplitudes(r,config_dict)
    init_psb_scale = r.psbscale.get_scale()
    init_psb_fftshift = r.psb.get_fftshift()
    # init_tone_powers,init_tone_powers_details = get_tone_powers(r,config_dict,detailed_output=True)

    #maximise amps 
    max_amp = 1-2**-12
    amps_max = np.max(init_amps)
    if amps_max == 0:
        raise ValueError('Tone powers are all zero')
    amps_gain = max_amp/amps_max
    amps = init_amps*amps_gain
    print('Maximise amplitudes:',amps)
    set_tone_amplitudes(r,config_dict,amps)
    time.sleep(0.01)

    #adjust fft shift to avoid overflow
    psb_fft_overflow = []
    psb_fftshifts = (2**np.arange(14)-1).astype(int)
    for i,shift in enumerate(psb_fftshifts):
        r.psb.set_fftshift(shift)
        time.sleep(0.01)
        dsp_overflow, dsp_overflow_details = check_dsp_overflow(r,0.5)
        psb_ovf = dsp_overflow_details['psb_ovf_delta']
        psb_fft_overflow.append(psb_ovf)
        print('Maximise fftshift:',format(shift,'#016b'),psb_ovf)

    best_fftshift = int(psb_fftshifts[np.argmin(psb_fft_overflow)]) # assume higher power from lower fftshift 
    fftshift_gain = (best_fftshift+1) /(init_psb_fftshift+1) #assume shift stages are all ones.
    r.psb.set_fftshift(best_fftshift)
    print('Best FFT Shift set:',format(best_fftshift,'#016b'))

    time.sleep(0.01)

    #adjust psb scale to up until just before overflow
    scalemax=255
    scalemin=1/256
    tolerance = 0.1 #10% of lower bound
    check = check_dsp_overflow(r,0.5)[1]['psbscale_ovf_delta']
    high=scalemax
    low=scalemin
    while (high-low) > tolerance*low:
        mid = (high+low)/2
        r.psbscale.set_scale(mid)
        time.sleep(0.01)
        check = check_dsp_overflow(r,0.5)[1]['psbscale_ovf_delta']
        print('Maximise psb_scale:',mid,check)
        if check:
            high=mid
        else:
            low=mid
    psb_scale = low*0.90
    r.psbscale.set_scale(psb_scale)

    #check for DAC saturation and reduce scale if so
    dac_saturation = check_output_saturation(r,iterations=50)[0]
    high = psb_scale
    low = 1/256
    tolerance = 0.1
    if dac_saturation:
        while high-low >tolerance*low:
            mid = (high+low)/2
            r.psbscale.set_scale(mid)
            time.sleep(0.01)
            check = check_output_saturation(r,iterations=10)[0]
            print('DAC saturation... Reduce psb_scale:',mid,check)
            if check:
                high=mid
            else:
                low=mid
        psb_scale = low*0.90
        r.psbscale.set_scale(psb_scale)

    return amps, best_fftshift, psb_scale, check_dsp_overflow(r,0.5)[1], check_output_saturation(r,iterations=50)[1]

def fix_dac_saturation(r,config_dict=None):
    init_amps = get_tone_amplitudes(r,config_dict)
    init_psb_fftshift = r.psb.get_fftshift()
    init_psb_scale = r.psbscale.get_scale()
    min_value = 1/256
    max_value = 255
    lower_bound = min_value
    upper_bound = init_psb_scale
    current_value = init_psb_scale
    precision = 0.05

    # Check if the current value causes saturation
    check,levels=check_output_saturation(r,iterations=50)
    if check:
        # Exponential Reduction Phase
        last_saturated_value = current_value
        while True:
            # Reduce current_value exponentially
            print('DAC Saturation... Reduce psb_scale: ', current_value,levels)
            current_value /= 2
            if current_value < min_value:
                current_value = min_value
                r.psbscale.set_scale(current_value)
                time.sleep(0.1)
                check,levels=check_output_saturation(r,iterations=50)
                if check:
                    # Saturation cannot be avoided
                    return min_value
                else:
                    lower_bound = min_value
                    break
            r.psbscale.set_scale(current_value)
            time.sleep(0.1)
            check,levels=check_output_saturation(r,iterations=50)
            if check:
                last_saturated_value = current_value
            else:
                lower_bound = current_value
                upper_bound = last_saturated_value
                break
    else:
        pass

    # Binary Search Phase
    while upper_bound - lower_bound > precision*lower_bound:
        
        mid_value = (lower_bound + upper_bound) / 2
        r.psbscale.set_scale(mid_value)
        time.sleep(0.1)
        check,levels=check_output_saturation(r,iterations=50)
        print('Increase psb_scale with Binary Search: ', mid_value,levels)
        if check:
            upper_bound = mid_value
        else:
            lower_bound = mid_value

    # Set the parameter to the highest non-saturating value
    psb_scale = lower_bound *0.90
    r.psbscale.set_scale(psb_scale)
    time.sleep(0.1)
    check,levels=check_output_saturation(r,iterations=50)

    return psb_scale, check_dsp_overflow(r,0.5)[1], levels
    


def optimise_tx_snr(r,config_dict=None):
    """
    Optimise the tx amplitudes, psb fft shift and psb scale. Maintain the same power but maximise the SNR.

    Rescale the tx amplitudes to the maximum possible and then the psb fft shift to get the highest gain.
    Then adjust psb scale to return to the original poers.
    """
    #get initial levels and settings
    init_dac_saturation, init_dac_levels = check_output_saturation(r,iterations=50)
    if init_dac_saturation:
        raise ValueError('DAC saturation detected prior to optimisation')
    init_amps = get_tone_amplitudes(r,config_dict)
    init_psb_scale = r.psbscale.get_scale()
    init_psb_fftshift = r.psb.get_fftshift()

    #rescale amps 
    max_amp = 1-2**-12
    amps_max = np.max(init_amps)
    if amps_max == 0:
        raise ValueError('Tone powers are all zero')
    amps_gain = max_amp/amps_max
    amps = init_amps*amps_gain
    print('Maximise amplitudes:',amps)
    set_tone_amplitudes(r,config_dict,amps)

    #adjust fft shift
    psb_fft_overflow = []
    psb_fftshifts = (2**np.arange(14)-1).astype(int)
    for i,shift in enumerate(psb_fftshifts):
        r.psb.set_fftshift(shift)
        time.sleep(0.01)
        dsp_overflow, dsp_overflow_details = check_dsp_overflow(r,0.5)
        psb_ovf = dsp_overflow_details['psb_ovf_delta']
        psb_fft_overflow.append(psb_ovf)
        print('Maximise fftshift:',format(shift,'#016b'),psb_ovf)
    best_fftshift = int(psb_fftshifts[np.argmin(psb_fft_overflow)]) # assume higher power from lower fftshift 
    fftshift_gain = (best_fftshift+1) /(init_psb_fftshift+1) #assume shift stages are all ones.
    r.psb.set_fftshift(best_fftshift)
    print('Best FFT Shift set:',format(best_fftshift,'#016b'))

    time.sleep(0.01)
    
    #adjust psb scale to compensate for change in amps and fftshift
    psb_gain = 1/fftshift_gain * 1/amps_gain
    psb_scale = init_psb_scale * psb_gain
    r.psbscale.set_scale(psb_scale)
    time.sleep(0.01)
    
    #check not overflowing and revert if so.
    dsp_overflow, dsp_overflow_details = check_dsp_overflow(r,0.5)
    dac_saturation, dac_levels = check_output_saturation(r,iterations=50)
    if dac_saturation or dsp_overflow_details['psb_ovf_delta'] or dsp_overflow_details['psbscale_ovf_delta']:
        print('DAC or DSP Saturated... reverting to initial settings')
        set_tone_amplitudes(r,config_dict,init_amps)
        r.psb.set_fftshift(init_psb_fftshift)
        r.psbscale.set_scale(init_psb_scale)
        raise ValueError('TX DSP overflow detected')
    
    
    return amps, best_fftshift, psb_scale, dsp_overflow_details, dac_levels


def maximise_rx_power(r,config_dict,headroom_db = 0.5):
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    init_adc_saturation, init_adc_levels = check_input_saturation(r,iterations=50)
    if init_adc_saturation:
        raise ValueError('ADC saturation detected prior to optimisation')
    init_pfb_fftshift = r.pfb.get_fftshift()

    init_dsa = float(r.rfdc.core.get_dsa(adc_tile,adc_block)['dsa'])

    #adjust dsa to down until just before saturation
    dsamax=27
    dsamin=0
    step_size = 0.25

    # Generate array of possible attenuation values
    attenuations = np.arange(dsamin, dsamax + step_size, step_size)
    num_steps = len(attenuations)
    
    # Initialize binary search indices
    low_idx = 0
    high_idx = num_steps - 1
    best_dsa = dsamin  # Initialize best attenuation
    
    # Function to set DSA and check saturation
    def set_and_check(att_value):
        r.rfdc.core.set_dsa(adc_tile, adc_block, att_value)
        time.sleep(0.1)  # Allow system to stabilize
        check, levels = check_input_saturation(r, iterations=50)
        print('Set DSA:',att_value,'Saturated:',check, levels)
        return check, levels
    
    # First, check if minimum attenuation causes saturation
    check, levels = set_and_check(dsamin)
    if not check:
        # No saturation at minimum attenuation; no need to increase attenuation
        print(f"No saturation detected at minimum attenuation: {dsamin} dB")
        best_dsa = dsamin
    else:
        # Binary Search Phase
        while low_idx <= high_idx:
            mid_idx = (low_idx + high_idx) // 2
            mid_att = attenuations[mid_idx]
            
            # Set attenuation to mid_att and check saturation
            check, levels = set_and_check(mid_att)
            print(f"Checking attenuation: {mid_att} dB, Saturated: {check}")
            
            if check:
                # Saturated: Need to increase attenuation
                low_idx = mid_idx + 1
            else:
                # Not saturated: Record this as best_dsa and try to find a higher attenuation
                best_dsa = mid_att
                high_idx = mid_idx - 1
        
        # After binary search, best_dsa holds the maximum attenuation without saturation
        print(f"Optimal attenuation found: {best_dsa} dB")
    
        best_dsa = min(best_dsa + headroom_db, dsamax)
        print(f"Setting DSA to: {best_dsa} dB, to give headroom of {headroom_db} dB")
        r.rfdc.core.set_dsa(adc_tile, adc_block, best_dsa)
        time.sleep(0.1)

    check, levels = check_input_saturation(r, iterations=50)
    maxlevel = np.max(np.abs([levels['imax_fs'],levels['imin_fs'],levels['qmax_fs'],levels['qmin_fs']]))
    print(f'Current ADC Headroom = {20*np.log10(maxlevel)} dB')

    #adjust fft shift to maximise the gain
    pfb_fft_overflow = []
    pfb_fftshifts = (2**np.arange(14)-1).astype(int)
    for i,shift in enumerate(pfb_fftshifts):
        r.pfb.set_fftshift(shift)
        time.sleep(0.01)
        dsp_overflow, dsp_overflow_details = check_dsp_overflow(r,0.5)
        pfb_ovf = dsp_overflow_details['pfb_ovf_delta']
        pfb_fft_overflow.append(pfb_ovf)
        print('Maximise fftshift:',format(shift,'#016b'),pfb_ovf)

    best_fftshift = int(pfb_fftshifts[np.argmin(pfb_fft_overflow)]) # assume higher power from lower fftshift 
    fftshift_gain = (best_fftshift+1) /(init_pfb_fftshift+1) #assume shift stages are all ones.
    r.pfb.set_fftshift(best_fftshift)
    print('Best FFT Shift set:',format(best_fftshift,'#016b'))
    time.sleep(0.01)
    return best_dsa,best_fftshift, check_dsp_overflow(r,0.5)[1], levels

def fix_adc_saturation(r,config_dict):
    best_dsa,best_fftshift, dsp_ovf, adc_levels = maximise_rx_power(r,config_dict)
    check,levels = check_input_saturation(r,iterations=50)
    if check:
        raise ValueError('ADC saturation detected at maximum attenuation')
    return best_dsa,best_fftshift, dsp_ovf, levels

def optimise_rx_snr(r,config_dict=None):
    init_adc_saturation, init_adc_levels = check_input_saturation(r,iterations=50)
    if init_adc_saturation:
        raise ValueError('ADC saturation detected prior to optimisation')
    
    #adjust fft shift
    init_pfb_fftshift = r.pfb.get_fftshift()

    pfb_fft_overflow = []
    pfb_fftshifts = (2**np.arange(14)-1).astype(int)
    for i,shift in enumerate(pfb_fftshifts):
        r.pfb.set_fftshift(shift)
        time.sleep(0.01)
        dsp_overflow, dsp_overflow_details = check_dsp_overflow(r,0.5)
        pfb_ovf = dsp_overflow_details['pfb_ovf_delta']
        pfb_fft_overflow.append(pfb_ovf)
        print(i,shift,pfb_ovf)
    best_fftshift = int(pfb_fftshifts[np.argmin(pfb_fft_overflow)]) # assume higher power from lower fftshift 
    fftshift_gain = (best_fftshift+1) /(init_pfb_fftshift+1) #assume shift stages are all ones.
    r.pfb.set_fftshift(best_fftshift)
    print('Best FFT Shift set:',format(best_fftshift,'#016b'))
    time.sleep(0.01)
    check,levels = check_input_saturation(r,iterations=50)
    return best_fftshift, check_dsp_overflow(r,0.5)[1],levels




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
    sweep_sems = np.std(sweep_data.real,axis=1)/np.sqrt(samples_per_point) + 1j*np.std(sweep_data.imag,axis=1)/np.sqrt(samples_per_point)

    results = {
        'sweep_frequencies': sweepfreqs,
        'sweep_responses': sweep_responses,
        'sweep_stds': sweep_stds,
        'sweep_sems': sweep_sems,
        'samples_per_point': samples_per_point,
        'samples_per_second': get_sample_rate(r_fast),
        'accumulation_counts': acc_counts,
        'accumulation_errors': acc_errs
        }
    return results 

def perform_retune(r, r_fast,config_dict, centers, spans, points, samples_per_point, direction, method,smooth_len=3,freq_offsets=None):
    """
    A blocking call to perform a frequency retune of the RFSOC.
    SImply performs a sweep and then retunes to the frequencies of maximum gradient or minimum magnitude.
    An asynchronous version of this function is available in the readout_server code.
    if freq_offsets is given, it is added to the retune frequencies before setting them.
    """
    if freq_offsets is None:
        freq_offsets = np.zeros_like(centers)
    elif np.isscalar(freq_offsets):
        freq_offsets = np.full_like(centers,freq_offsets)
    elif freq_offsets.shape != np.atleast_1d(centers).shape:
            raise ValueError("freq_offsets must be None, a scalar, or have the same shape as centers")
    
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
            retune_freqs[t] = freqs[max_grad] + freq_offsets[t]
    elif method == 'min_mag':
        retune_freqs = np.zeros_like(results['sweep_frequencies'])
        for t in range(len(centers)):
            freqs = results['sweep_frequencies'][t]
            mags = np.abs(results['sweep_responses'][t])
            min_mag = np.argmin(mags)
            retune_freqs[t] = freqs[min_mag] + freq_offsets[t]

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
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,dac_dbfs_to_dbm),ndmin=2).T
        dac_dbfs_to_dbm = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(dac_dbfs_to_dbm):
        #perform nearest neighbour interpolation on calibration data if arrays stored in config file directly
        cal_f,cal_db = np.array(dac_dbfs_to_dbm,ndmin=2).T
        dac_dbfs_to_dbm = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_combiner_loss_db is None:
        tx_combiner_loss_db = 0
    elif isinstance(tx_combiner_loss_db,str):
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,tx_combiner_loss_db),ndmin=2).T
        tx_combiner_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_combiner_loss_db):
        cal_f,cal_db = np.array(tx_combiner_loss_db,ndmin=2).T
        tx_combiner_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_attenuator_value_db is None:
        tx_attenuator_value_db = 0

    if tx_if_s21_db is None:
        tx_if_s21_db = 0
    elif isinstance(tx_if_s21_db,str):
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,tx_if_s21_db),ndmin=2).T
        tx_if_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_if_s21_db):
        cal_f,cal_db = np.array(tx_if_s21_db,ndmin=2).T
        tx_if_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_mixer_conversion_loss_db is None:
        tx_mixer_conversion_loss_db = 0
    elif isinstance(tx_mixer_conversion_loss_db,str):
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,tx_mixer_conversion_loss_db),ndmin=2).T
        tx_mixer_conversion_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_mixer_conversion_loss_db):
        cal_f,cal_db = np.array(tx_mixer_conversion_loss_db,ndmin=2).T
        tx_mixer_conversion_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_rf_s21_db is None:
        tx_rf_s21_db = 0
    elif isinstance(tx_rf_s21_db,str):
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,tx_rf_s21_db),ndmin=2).T
        tx_rf_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])
    elif not np.isscalar(tx_rf_s21_db):
        cal_f,cal_db = np.array(tx_rf_s21_db,ndmin=2).T
        tx_rf_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])
    
    if cryostat_input_s21_db is None:
        cryostat_input_s21_db = 0
    elif isinstance(cryostat_input_s21_db,str):
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,cryostat_input_s21_db),ndmin=2).T
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
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,dac_dbfs_to_dbm),ndmin=2).T
        dac_dbfs_to_dbm = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(dac_dbfs_to_dbm):
        #perform nearest neighbour interpolation on calibration data if arrays stored in config file directly
        cal_f,cal_db = np.array(dac_dbfs_to_dbm,ndmin=2).T
        dac_dbfs_to_dbm = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_combiner_loss_db is None:
        tx_combiner_loss_db = 0
    elif isinstance(tx_combiner_loss_db,str):
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,tx_combiner_loss_db),ndmin=2).T
        tx_combiner_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_combiner_loss_db):
        cal_f,cal_db = np.array(tx_combiner_loss_db,ndmin=2).T
        tx_combiner_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_attenuator_value_db is None:
        tx_attenuator_value_db = 0

    if tx_if_s21_db is None:
        tx_if_s21_db = 0
    elif isinstance(tx_if_s21_db,str):
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,tx_if_s21_db),ndmin=2).T
        tx_if_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_if_s21_db):
        cal_f,cal_db = np.array(tx_if_s21_db,ndmin=2).T
        tx_if_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_mixer_conversion_loss_db is None:
        tx_mixer_conversion_loss_db = 0
    elif isinstance(tx_mixer_conversion_loss_db,str):
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,tx_mixer_conversion_loss_db),ndmin=2).T
        tx_mixer_conversion_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    elif not np.isscalar(tx_mixer_conversion_loss_db):
        cal_f,cal_db = np.array(tx_mixer_conversion_loss_db,ndmin=2).T
        tx_mixer_conversion_loss_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['analog_output_freq']])
    
    if tx_rf_s21_db is None:
        tx_rf_s21_db = 0
    elif isinstance(tx_rf_s21_db,str):
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,tx_rf_s21_db),ndmin=2).T
        tx_rf_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])
    elif not np.isscalar(tx_rf_s21_db):
        cal_f,cal_db = np.array(tx_rf_s21_db,ndmin=2).T
        tx_rf_s21_db = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_details['tx']['rf_output_freq']])
    
    if cryostat_input_s21_db is None:
        cryostat_input_s21_db = 0
    elif isinstance(cryostat_input_s21_db,str):
        cal_f,cal_db = np.loadtxt(os.path.join(USER_DIR,cryostat_input_s21_db),ndmin=2).T
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


def force_sync_fast(r_fast,wait_s=0.0001):
    regname = 'p0_sync_ctrl'
    if not hasattr(r_fast,'_p0_sync_ctrl_addr'):
        setattr(r_fast, '_p0_sync_ctrl_addr',r_fast.sync.host.transport._get_device_address(regname))
        setattr(r_fast, '_p0_sync_ctrl_arm_bit', 1<<r_fast.sync.OFFSET_ARM_SYNC_OUT)
        setattr(r_fast, '_p0_sync_ctrl_sync_bit', 1<<r_fast.sync.OFFSET_MAN_SYNC)
                
    #arm_sync
    addr = r_fast._p0_sync_ctrl_addr
    mm = r_fast.sync.host.transport.axil_mm
    (value,) = struct.unpack('<I',mm[addr:addr+4])
    
    #set 0
    value &= ~r_fast._p0_sync_ctrl_arm_bit
    mm[addr:addr+4] = struct.pack('<I',value)
    #set 1
    value |= r_fast._p0_sync_ctrl_arm_bit
    mm[addr:addr+4] = struct.pack('<I',value)
    #set 0
    value &= ~r_fast._p0_sync_ctrl_arm_bit
    mm[addr:addr+4] = struct.pack('<I',value)

    # change_reg_bits_fast(addr, 0, r_fast.sync.OFFSET_ARM_SYNC_OUT)
    # change_reg_bits(addr, 1, r_fast.sync.OFFSET_ARM_SYNC_OUT)
    # change_reg_bits(addr, 0, r_fast.sync.OFFSET_ARM_SYNC_OUT)

    #wait
    time.sleep(wait_s)
    
    #manual sync
    #set0
    value &= ~r_fast._p0_sync_ctrl_sync_bit
    mm[addr:addr+4] = struct.pack('<I',value)
    #set 1
    value |= r_fast._p0_sync_ctrl_sync_bit
    mm[addr:addr+4] = struct.pack('<I',value)
    #set 0
    value &= ~r_fast._p0_sync_ctrl_sync_bit
    mm[addr:addr+4] = struct.pack('<I',value)
    
    # change_reg_bits('ctrl', 0, r_fast.sync.OFFSET_MAN_SYNC)
    # change_reg_bits('ctrl', 1, r_fast.sync.OFFSET_MAN_SYNC)
    # change_reg_bits('ctrl', 0, r_fast.sync.OFFSET_MAN_SYNC)

    return 

def get_closest_bin_indices(freqs_hz, bin_centers_hz):
    """
    Efficiently find the closest bin index in `bin_centers_hz` for each frequency in `freqs_hz`.

    :param freqs_hz: Scalar, 1D or 2D array of frequencies [Hz]
    :param bin_centers_hz: 1D array of bin center frequencies [Hz]

    :return: Closest bin index or array of indices (matching input shape)
    :rtype: int or np.ndarray of int
    """
    freqs = np.asarray(freqs_hz)
    input_shape = freqs.shape
    flat_freqs = freqs.ravel()

    # Ensure bin centers are sorted
    sort_idx = np.argsort(bin_centers_hz)
    sorted_bins = bin_centers_hz[sort_idx]

    # Vectorized nearest neighbor search
    idx_right = np.searchsorted(sorted_bins, flat_freqs, side='right')
    idx_left = np.clip(idx_right - 1, 0, len(sorted_bins) - 1)
    idx_right = np.clip(idx_right, 0, len(sorted_bins) - 1)

    dist_left = np.abs(flat_freqs - sorted_bins[idx_left])
    dist_right = np.abs(flat_freqs - sorted_bins[idx_right])
    closer_on_right = dist_right < dist_left

    closest_sorted = np.where(closer_on_right, idx_right, idx_left)
    closest = sort_idx[closest_sorted]
    closest = closest.reshape(input_shape)

    # Return a scalar if input was a scalar
    if np.isscalar(freqs_hz) or freqs.ndim == 0:
        return int(closest)
    return closest


def get_burst(r):
    """
    Get a burst of data on a channel
    In practice the bursts are taken from the accunmulator in the same way as regular samples
    """
    return r.accumulators[0].get_new_burst()

def get_burst_mode(r):
    """
    Get the burst mode setting
    """
    burst_mode = r.accumulators[0].get_burst_mode()
    return burst_mode
    
def set_burst_mode(r,mode):
    """
    Set the burst mode setting
    """
    r.accumulators[0].set_burst_mode(mode)
    return



#include private functions when import * for debugging, to be removed later
__all__ = list(globals().keys())
