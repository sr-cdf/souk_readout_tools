"""
This module provides a set of functions that are used by the readout server to interact with the SOUK firmware.


"""

import warnings
import numpy as np
import time
import os
import yaml
import struct
import subprocess
import threading

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


def create_standard_readout_interface(fw_config_file,pipeline_id=0):
    try:
        r = SoukMkidReadout('localhost',configfile=fw_config_file,pipeline_id=pipeline_id)
    except NameError:
        raise RuntimeError('souk_mkid_readout module not imported, cannot create readout interface')
    return r

def create_fast_readout_interface(fw_config_file,pipeline_id=0):
    try:
        r_fast = SoukMkidReadout('localhost',configfile=fw_config_file,local=True,pipeline_id=pipeline_id)
    except NameError:
        raise RuntimeError('souk_mkid_readout module not imported, cannot create readout interface')
    return r_fast

def needs_programming(r,config_dict):

    if r is None:
        print('************************************************')
        print('needs_programming?')
        print('yes, readout interface is None')
        print('************************************************')
        return True

    currentfpg = r.fpgfile
    try:
        currentfpg = os.readlink(currentfpg)
    except OSError:
        pass
    with open(config_dict['firmware']['fw_config_file'],'r') as file:
        newfpg = yaml.safe_load(file)['fpgfile']
    newfpg = newfpg.replace('../','').replace('./','/home/casper/souk-firmware/')
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

    if not r.fpga.is_programmed():
        print('yes, FPGA is not programmed')
        return True

    if newfpg != currentfpg:
        print('yes, current fpg is not the requested one')
        return True

    #if not hasattr(r, 'accumulators'):
    #    # catches certain rare cases
    #    print('yes, accumulators not found')
    #    return True

    print('no')
    return False

def needs_shared_resource_initialising(r, config_dict):
    """
    True if shared resources need initialising.

    checks autocorr acc_len
      - r.autocorr.get_acc_len() == 0 => not initialised
    """
    print('************************************************')
    print('needs_shared_resource_initialising?')
    print('************************************************')

    if r is None or not hasattr(r, "autocorr"):
        print('yes, autocorr block missing (or interface is None)')
        return True

    autocorr_acc_len = r.autocorr.get_acc_len()
    if autocorr_acc_len==0:
        print('yes, autocorr acc_len is zero')
        return True

    print('no')
    return False

def needs_pipeline_initialising(r, config_dict):
    """
    True if pipeline resources need initialising.

    checks pipeline accumulator acc_len
      - r.accumulators[0].get_acc_len() == 0 => not initialised
    """
    print('************************************************')
    print('needs_pipeline_initialising?')
    print('************************************************')

    if r is None or not hasattr(r, "accumulators") or len(r.accumulators) == 0:
        print('yes, accumulators missing (or interface is None)')
        return True

    acc_len = r.accumulators[0].get_acc_len()
    if acc_len == 0:
        print('yes, pipeline acc_len is zero')
        return True

    print('no')
    return False

def needs_initialising(r,config_dict):
    """
    Deprecated function, use needs_shared_resource_initialising and needs_pipeline_initialising instead.
    """
    print('************************************************')
    print('needs_initialising?')
    print(bcolors.FAIL+'This function is deprecated, use needs_shared_resource_initialising and needs_pipeline_initialising instead'+bcolors.ENDC)
    print('************************************************')
    needs_initialising_shared = needs_shared_resource_initialising(r, config_dict)
    needs_initialising_pipeline = needs_pipeline_initialising(r, config_dict)
    return needs_initialising_shared or needs_initialising_pipeline


def reload_firmware(config_dict):
    """
    Program/reprogram and return interfaces.

    IMPORTANT: This and any second pipeline will need initialising. Do not initialise shared or pipeline resources here.
    """
    print(bcolors.WARNING+'Reloading firmware: all shared/pipeline resources will need re-initialising'+bcolors.ENDC)
    fw_config_file = config_dict['firmware']['fw_config_file']
    pipeline_id = config_dict['firmware']['pipeline_id']
    r = create_standard_readout_interface(fw_config_file,pipeline_id=pipeline_id)
    r_fast = create_fast_readout_interface(fw_config_file,pipeline_id=pipeline_id)
    r.program()

    fw_type = r.fpga.get_firmware_type()
    if fw_type==2:
        if pipeline_id!=0:
            raise ValueError(f'Pipeline ID {pipeline_id} does not exist in type 2 single pipeline firmware: {fw_config_file}')
    elif fw_type==3:
        if pipeline_id not in [0,1]:
            raise ValueError(f'Pipeline ID {pipeline_id} does not exist in type 3 dual pipeline firmware: {fw_config_file}')

    return r, r_fast



def initialise_shared_resources(r,config_dict):
    _shared_block_names = ['common', 'adc_snapshot', 'dac_snapshot', 'zoomfft', 'zoomacc', 'gen_cordic', 'gen_lut', 'autocorr']

    #read from config
    ## no common block configurations in use right now

    #initialise and setup blocks
    r.initialize_shared_blocks()

    #nothing to setup right now
    return

def initialise_pipeline_resources(r,r_fast,config_dict):
    _pipeline_block_names = ['sync', 'input', 'pfb', 'pfbtvg', 'chanselect', 'mixer', 'psb_chanselect', 'psb', 'psbscale', 'accumulator0', 'accumulator1', 'output', 'out_delay']

    #read config
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

    #initialise and setup blocks
    r.initialize_pipeline_blocks()

    r.output.use_psb()

    if sync_delay is not None:
        r.sync.set_delay(sync_delay)
        r.sync.arm_sync(wait=False)
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
    # Ensure config has current mixer frequencies (set_nyquist_zone may have updated them)
    config_dict['firmware']['defaults']['dac_duc_mixer_frequency_hz'] = float(r.rfdc.core.get_mixer_settings(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE)['Freq'])*1e6
    config_dict['firmware']['defaults']['adc_ddc_mixer_frequency_hz'] = float(r.rfdc.core.get_mixer_settings(adc_tile,adc_block,r.rfdc.core.ADC_TILE)['Freq'])*1e6
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
        r.rfdc.core.set_dsa(adc_tile, adc_block, int(dsa))
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

    # #check signal levels -- does not work anymore during init because r_fast has no blocks yet
    # dac_saturation = check_output_saturation(r_fast,iterations=25,saturation_bits=dac_saturation_bits)
    # adc_saturation = check_input_saturation(r,r_fast,iterations=25,saturation_bits=adc_saturation_bits)
    # dsp_overflow = check_dsp_overflow(r)
    # print(f'DAC levels: {dac_saturation}')
    # print(f'ADC levels: {adc_saturation}')
    # print(f'DSP overflow: {dsp_overflow}')

    return

def _get_git_commit(repo_path):
    """Get the short git commit hash and tag (if any) for a repository path, or None."""
    try:
        env = os.environ.copy()
        env['GIT_DIR'] = os.path.join(repo_path, '.git')
        env['GIT_WORK_TREE'] = repo_path
        git_safe = ['git', '-c', f'safe.directory={repo_path}']
        commit = subprocess.check_output(
            git_safe + ['rev-parse', '--short', 'HEAD'],
            cwd=repo_path, stderr=subprocess.DEVNULL, env=env
        ).decode().strip()
        try:
            describe = subprocess.check_output(
                git_safe + ['describe', '--tags', '--exact-match', 'HEAD'],
                cwd=repo_path, stderr=subprocess.DEVNULL, env=env
            ).decode().strip()
            return f'{describe} ({commit})'
        except subprocess.CalledProcessError:
            return commit
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def get_system_information(r,config_dict):

    pipeline_id = config_dict['firmware']['pipeline_id']
    dac0_tile = config_dict['firmware']['dac0_tile']
    dac0_block = config_dict['firmware']['dac0_block']
    dac1_tile = config_dict['firmware']['dac1_tile']
    dac1_block = config_dict['firmware']['dac1_block']
    adc_tile = config_dict['firmware']['adc_tile']
    adc_block = config_dict['firmware']['adc_block']

    info = {}

    # Software version information
    try:
        import importlib.metadata
        info['souk_readout_tools_version'] = importlib.metadata.version('souk_readout_tools')
    except Exception:
        info['souk_readout_tools_version'] = None

    try:
        info['souk_mkid_readout_sw_version'] = souk_mkid_readout.__version__
    except (NameError, AttributeError):
        info['souk_mkid_readout_sw_version'] = None

    # Supported firmware version from the souk_mkid_readout package
    try:
        info['souk_mkid_readout_fw_version'] = souk_mkid_readout.__fwversion__
    except (NameError, AttributeError):
        info['souk_mkid_readout_fw_version'] = None

    # Git repository commit IDs (source repos on the RFSoC)
    info['souk_readout_tools_commit'] = _get_git_commit('/home/casper/souk_readout_tools')
    info['souk_firmware_commit'] = _get_git_commit('/home/casper/souk-firmware')
    info['souk_peripherals_commit'] = _get_git_commit('/home/casper/souk_readout_tools/src/souk_readout_tools/server/souk-peripherals-control')

    info['fpga_status'] = r.fpga.get_status()[0]
    info['fpg_file'] = r.fpgfile
    info['pipeline_id'] = r.pipeline_id
    info['adc_clk_hz'] = r.adc_clk_hz

    # Determine initialisation level from attribute availability
    # (the needs_*() checks are done by the caller / get_server_status)
    # Shared blocks: common, adc_snapshot, dac_snapshot, zoomfft, zoomacc, gen_cordic, gen_lut, autocorr
    # Pipeline blocks: sync, input, pfb, pfbtvg, chanselect, mixer, psb_chanselect, psb, psbscale, accumulator0, accumulator1, output, out_delay
    programmed = r.fpga.is_programmed()
    shared_ready = programmed and hasattr(r, 'autocorr')
    pipeline_ready = shared_ready and hasattr(r, 'output') and hasattr(r, 'accumulators') and len(r.accumulators) > 0 and r.accumulators[0].get_acc_len() > 0
    if pipeline_ready:
        info['initialisation_level'] = 'pipeline'
    elif shared_ready:
        info['initialisation_level'] = 'shared'
    elif programmed:
        info['initialisation_level'] = 'programmed'
    else:
        info['initialisation_level'] = 'not_programmed'

    # Pipeline block parameters (output, sync, input, pfb, psb, psbscale, mixer, accumulators)
    if pipeline_ready:
        info['output_mode'] = r.output.get_status()[0]['mode']
        info['sync_delay'] = r.sync.get_delay()
        info['internal_loopback'] = r.input.loopback_enabled()
        info['psb_scale'] = r.psbscale.get_scale()
        info['psb_fftshift'] = r.psb.get_fftshift()
        info['pfb_fftshift'] = r.pfb.get_fftshift()
        info['acc_len'] = r.accumulators[0].get_acc_len()
        info['acc_freq'] = get_sample_rate(r)
    else:
        info['output_mode'] = None
        info['sync_delay'] = None
        info['internal_loopback'] = None
        info['psb_scale'] = None
        info['psb_fftshift'] = None
        info['pfb_fftshift'] = None
        info['acc_len'] = None
        info['acc_freq'] = None

    #rfdc info will be missing keys if the dac and adc tiles/blocks are not set to match those in the firmware
    #lets check we can read dsa on the adc and vop on each dac before trying to read them

    has_rfdc = hasattr(r, 'rfdc')
    correct_adc = has_rfdc and r.rfdc.core.get_dsa(adc_tile,adc_block).get('dsa',None) is not None
    correct_dac0 = has_rfdc and r.rfdc.core.get_output_current(dac0_tile,dac0_block).get('current',None) is not None
    correct_dac1 = has_rfdc and r.rfdc.core.get_output_current(dac1_tile,dac1_block).get('current',None) is not None
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
        info['adc_cal_frozen'] = get_cal_freeze(r,config_dict)
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
        info['adc_cal_frozen'] = None
        if not has_rfdc:
            print(bcolors.FAIL+'CRITICAL WARNING - RFDC block not found, the FPGA may not be programmed'+bcolors.ENDC)
        else:
            print(bcolors.FAIL+'CRITICAL WARNING - RFDC settings not found, check that the DAC and ADC tiles/blocks are set correctly in the config to match the firmware'+bcolors.ENDC)
            print('Continuing regardless but the system will not work.')
    # RTS overvoltage flags
    rts_event, rts_details = check_rfdc_rts_events(r, clear=False)
    info['rts_events'] = rts_details

    if pipeline_ready:
        freqs_detailed = get_tone_frequencies(r,config_dict,detailed_output=True)
        info['tone_frequencies'] = freqs_detailed[0].tolist()
        info['tone_amplitudes'] = get_tone_amplitudes(r,config_dict).tolist()
        info['tone_phases'] = get_tone_phases(r,config_dict).tolist()
        info['tone_indices'] = freqs_detailed[1]['rx']['tone_indices']
    else:
        info['tone_frequencies'] = None
        info['tone_amplitudes'] = None
        info['tone_phases'] = None
        info['tone_indices'] = None
    print('system information:')
    for key, value in info.items():
        print(f'{key}: {value}\n')

    return info


def apply_config(new_config_dict, r, r_fast=None, prev_config_dict=None):
    """
    Apply configuration changes to hardware.

    If prev_config_dict is None, applies all values from new_config_dict.
    If prev_config_dict is provided, only applies values that have changed.

    Parameters
    ----------
    new_config_dict : dict
        The new configuration dictionary to apply
    r : object
        The readout object with hardware access
    prev_config_dict : dict, optional
        The previous configuration dictionary for comparison
    """
    fwconf = new_config_dict['firmware']
    defaults = fwconf.get('defaults', {})

    # Treat empty dict same as None
    if not prev_config_dict:
        prev_config_dict = None

    # Get previous config for comparison
    if prev_config_dict is not None:
        prev_fwconf = prev_config_dict.get('firmware', {})
        prev_defaults = prev_fwconf.get('defaults', {})
    else:
        prev_fwconf = {}
        prev_defaults = {}

    # Check for tile/block changes that require re-initialisation
    reinit_keys = ['dac0_tile', 'dac0_block', 'dac1_tile', 'dac1_block', 'adc_tile', 'adc_block', 'fw_config_file', 'pipeline_id']
    if prev_config_dict is not None:
        for key in reinit_keys:
            if fwconf.get(key) != prev_fwconf.get(key):
                print(bcolors.WARNING + f'WARNING: firmware config "{key}" changed ({prev_fwconf.get(key)} -> {fwconf.get(key)}). '
                      f'This requires re-initialisation of firmware resources.' + bcolors.ENDC)

    # Check for rfsoc_host parameter changes
    if prev_config_dict is not None:
        new_rfsoc_host = new_config_dict.get('rfsoc_host', {})
        prev_rfsoc_host = prev_config_dict.get('rfsoc_host', {})
        if new_rfsoc_host != prev_rfsoc_host:
            print(bcolors.WARNING + 'WARNING: rfsoc_host configuration changed. These parameters cannot be applied remotely. '
                  'Log into the RFSoC directly to make these changes.' + bcolors.ENDC)

    # Helper to check if a value changed
    def changed(key):
        if prev_config_dict is None:
            return key in defaults
        return defaults.get(key) != prev_defaults.get(key)

    # Get tile/block config for RFDC operations
    dac0_tile = fwconf['dac0_tile']
    dac0_block = fwconf['dac0_block']
    dac1_tile = fwconf['dac1_tile']
    dac1_block = fwconf['dac1_block']
    adc_tile = fwconf['adc_tile']
    adc_block = fwconf['adc_block']

    # Apply changed parameters
    if changed('sync_delay'):
        sync_delay = defaults.get('sync_delay')
        if sync_delay is not None:
            print(f'apply_config: setting sync_delay = {sync_delay}')
            r.sync.set_delay(sync_delay)
            r.sync.arm_sync(wait=False)
            r.sync.sw_sync()

    if changed('acc_len'):
        acc_len = defaults.get('acc_len')
        if acc_len is not None:
            print(f'apply_config: setting acc_len = {acc_len}')
            r.accumulators[0].set_acc_len(acc_len)

    if changed('dac_duc_mixer_frequency_hz'):
        dac_duc_mixer_frequency_hz = defaults.get('dac_duc_mixer_frequency_hz')
        if dac_duc_mixer_frequency_hz is not None:
            print(f'apply_config: setting dac_duc_mixer_frequency_hz = {dac_duc_mixer_frequency_hz}')
            r.rfdc.core.set_fine_mixer_freq(dac0_tile, dac0_block, r.rfdc.core.DAC_TILE, dac_duc_mixer_frequency_hz/1e6)
            r.rfdc.core.set_fine_mixer_freq(dac1_tile, dac1_block, r.rfdc.core.DAC_TILE, dac_duc_mixer_frequency_hz/1e6)

    if changed('adc_ddc_mix_frequency_hz'):
        adc_ddc_mix_frequency_hz = defaults.get('adc_ddc_mix_frequency_hz')
        if adc_ddc_mix_frequency_hz is not None:
            print(f'apply_config: setting adc_ddc_mix_frequency_hz = {adc_ddc_mix_frequency_hz}')
            r.rfdc.core.set_fine_mixer_freq(adc_tile, adc_block, r.rfdc.core.ADC_TILE, adc_ddc_mix_frequency_hz/1e6)

    if changed('nyquist_zone'):
        nyquist_zone = defaults.get('nyquist_zone')
        if nyquist_zone is not None:
            print(f'apply_config: setting nyquist_zone = {nyquist_zone}')
            set_nyquist_zone(r, new_config_dict, nyquist_zone, inv_sinc=False)

    # Ensure config has current mixer frequencies after any changes
    if changed('dac_duc_mixer_frequency_hz') or changed('adc_ddc_mix_frequency_hz') or changed('nyquist_zone'):
        new_config_dict['firmware']['defaults']['dac_duc_mixer_frequency_hz'] = float(r.rfdc.core.get_mixer_settings(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE)['Freq'])*1e6
        new_config_dict['firmware']['defaults']['adc_ddc_mixer_frequency_hz'] = float(r.rfdc.core.get_mixer_settings(adc_tile,adc_block,r.rfdc.core.ADC_TILE)['Freq'])*1e6

    if changed('dac_mixer_scale_1p0'):
        dac_mixer_scale_1p0 = defaults.get('dac_mixer_scale_1p0')
        if dac_mixer_scale_1p0 is not None:
            print(f'apply_config: setting dac_mixer_scale_1p0 = {dac_mixer_scale_1p0}')
            if dac_mixer_scale_1p0:
                r.rfdc.core.set_mixer_scale(dac0_tile, dac0_block, r.rfdc.core.DAC_TILE, r.rfdc.core.MIX_SCALE_1P0)
                r.rfdc.core.set_mixer_scale(dac1_tile, dac1_block, r.rfdc.core.DAC_TILE, r.rfdc.core.MIX_SCALE_1P0)
            else:
                r.rfdc.core.set_mixer_scale(dac0_tile, dac0_block, r.rfdc.core.DAC_TILE, r.rfdc.core.MIX_SCALE_AUTO)
                r.rfdc.core.set_mixer_scale(dac1_tile, dac1_block, r.rfdc.core.DAC_TILE, r.rfdc.core.MIX_SCALE_AUTO)

    if changed('adc_mixer_scale_1p0'):
        adc_mixer_scale_1p0 = defaults.get('adc_mixer_scale_1p0')
        if adc_mixer_scale_1p0 is not None:
            print(f'apply_config: setting adc_mixer_scale_1p0 = {adc_mixer_scale_1p0}')
            if adc_mixer_scale_1p0:
                r.rfdc.core.set_mixer_scale(adc_tile, adc_block, r.rfdc.core.ADC_TILE, r.rfdc.core.MIX_SCALE_1P0)
            else:
                r.rfdc.core.set_mixer_scale(adc_tile, adc_block, r.rfdc.core.ADC_TILE, r.rfdc.core.MIX_SCALE_AUTO)

    if changed('dsa'):
        dsa = defaults.get('dsa')
        if dsa is not None:
            print(f'apply_config: setting dsa = {dsa}')
            r.rfdc.core.set_dsa(adc_tile, adc_block, int(dsa))

    if changed('vop'):
        vop = defaults.get('vop')
        if vop is not None:
            print(f'apply_config: setting vop = {vop}')
            r.rfdc.core.set_vop(dac0_tile, dac0_block, vop)
            r.rfdc.core.set_vop(dac1_tile, dac1_block, vop)

    if changed('internal_loopback'):
        internal_loopback = defaults.get('internal_loopback')
        if internal_loopback is not None:
            print(f'apply_config: setting internal_loopback = {internal_loopback}')
            if internal_loopback:
                r.input.enable_loopback()
            else:
                r.input.disable_loopback()

    if changed('psb_scale'):
        psb_scale = defaults.get('psb_scale')
        if psb_scale is not None:
            print(f'apply_config: setting psb_scale = {psb_scale}')
            r.psbscale.set_scale(psb_scale)

    if changed('psb_fftshift'):
        psb_fftshift = defaults.get('psb_fftshift')
        if psb_fftshift is not None:
            print(f'apply_config: setting psb_fftshift = {psb_fftshift}')
            r.psb.set_fftshift(psb_fftshift)

    if changed('pfb_fftshift'):
        pfb_fftshift = defaults.get('pfb_fftshift')
        if pfb_fftshift is not None:
            print(f'apply_config: setting pfb_fftshift = {pfb_fftshift}')
            r.pfb.set_fftshift(pfb_fftshift)

    if changed('frequencies'):
        frequencies = defaults.get('frequencies', [])
        if frequencies:
            print(f'apply_config: setting frequencies ({len(frequencies)} tones)')
            set_tone_frequencies(r, new_config_dict, frequencies)

    if changed('amplitudes'):
        amplitudes = defaults.get('amplitudes', [])
        if amplitudes:
            print(f'apply_config: setting amplitudes ({len(amplitudes)} values)')
            set_tone_amplitudes(r, new_config_dict, amplitudes)

    if changed('phases'):
        phases = defaults.get('phases', [])
        if phases:
            print(f'apply_config: setting phases ({len(phases)} values)')
            set_tone_phases(r, new_config_dict, phases)

    #check signal levels
    dac_saturation = check_output_saturation(r_fast,iterations=25,saturation_bits=dac_saturation_bits)
    adc_saturation = check_input_saturation(r,r_fast,iterations=25,saturation_bits=adc_saturation_bits)
    dsp_overflow = check_dsp_overflow(r)
    print(f'DAC levels: {dac_saturation}')
    print(f'ADC levels: {adc_saturation}')
    print(f'DSP overflow: {dsp_overflow}')




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

    # Store actual mixer frequencies in config so fast functions can access them without RFDC
    config_dict['firmware']['defaults']['dac_duc_mixer_frequency_hz'] = float(r.rfdc.core.get_mixer_settings(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE)['Freq'])*1e6
    config_dict['firmware']['defaults']['adc_ddc_mixer_frequency_hz'] = float(r.rfdc.core.get_mixer_settings(adc_tile,adc_block,r.rfdc.core.ADC_TILE)['Freq'])*1e6

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
        # #   phase_steps = (freq / fft_rbw_hz) * 2pi
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


def _get_control_buffer_addresses(r_fast):
    """
    Get (and cache) the base addresses of the TX and RX control buffers.

    Returns a dict with 'tx' and 'rx' keys containing the base byte addresses.
    """
    if not hasattr(r_fast.mixer, '_control_buffer_addrs'):
        tx_addr = r_fast.mixer.host.transport._get_device_address(
            f'{r_fast.mixer.prefix}tx_lo0_control')
        rx_addr = r_fast.mixer.host.transport._get_device_address(
            f'{r_fast.mixer.prefix}rx_lo0_control')
        r_fast.mixer._control_buffer_addrs = {'tx': tx_addr, 'rx': rx_addr}
    return r_fast.mixer._control_buffer_addrs

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

    # Get the dynamically-looked-up base addresses for the control buffers
    addrs = _get_control_buffer_addresses(r_fast)
    # Buffer size: n_serial_chans * _CONTROL_N_WORDS * 4 bytes
    buf_size = r_fast.mixer._n_serial_chans * r_fast.mixer._CONTROL_N_WORDS * 4

    for lo in los:
        if lo not in ['tx','rx']:
            raise ValueError(f"Only LOs 'rx' and 'tx' are understood. Not {lo}.")

        offset = addrs[lo] + buf_size * buf
        length = buf_size
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

def prepare_control_buffer_data_fast(r_fast, buf, lo_control_values, tone_indices=None):
    """
    Faster version of prepare_control_buffer

    Does not prepare the full buffer, only returns the given formatted values and their indices

    Parameters:
    r_fast: readout object (fast interface)
    buf: int, index of buffer to write to, 0 or 1.
    lo_control_values: dictionary with keys 'tx' and 'rx'
     - 'tx': dictionary with optional keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
     - 'rx': dictionary with optional keys 'phase_steps', 'ri_steps', 'phase_offsets', 'scaling'
    tone_indices: array of LO indices for the tones. If None, assumes contiguous indices starting from 0.
                  With VACC, these may be non-contiguous.

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

        # Use tone_indices if provided, otherwise fall back to contiguous indices
        # With VACC, tone_indices may be non-contiguous (e.g., [0, 5, 10] instead of [0, 1, 2])
        if tone_indices is not None:
            idx_phase_steps = np.asarray(tone_indices[:n_phase_steps]) if n_phase_steps > 0 else np.array([], dtype=int)
            idx_phase_offsets = np.asarray(tone_indices[:n_phase_offsets]) if n_phase_offsets > 0 else np.array([], dtype=int)
            idx_ri_steps = np.asarray(tone_indices[:n_ri_steps]) if n_ri_steps > 0 else np.array([], dtype=int)
            idx_scaling = np.asarray(tone_indices[:n_scaling]) if n_scaling > 0 else np.array([], dtype=int)
        else:
            idx_phase_steps = np.arange(n_phase_steps)
            idx_phase_offsets = np.arange(n_phase_offsets)
            idx_ri_steps = np.arange(n_ri_steps)
            idx_scaling = np.arange(n_scaling)

        i[lo] = np.empty(n_phase_steps+n_phase_offsets+n_ri_steps+n_scaling, dtype=int)
        i[lo][0:n_phase_steps] = idx_phase_steps*r_fast.mixer._CONTROL_N_WORDS+r_fast.mixer._PHASE_INC_WORD_OFFSET
        i[lo][n_phase_steps:n_phase_steps+n_phase_offsets] = idx_phase_offsets*r_fast.mixer._CONTROL_N_WORDS+r_fast.mixer._PHASE_OFFSET_WORD_OFFSET
        i[lo][n_phase_steps+n_phase_offsets:n_phase_steps+n_phase_offsets+n_ri_steps] = idx_ri_steps*r_fast.mixer._CONTROL_N_WORDS+r_fast.mixer._RI_STEP_WORD_OFFSET
        i[lo][n_phase_steps+n_phase_offsets+n_ri_steps:n_phase_steps+n_phase_offsets+n_ri_steps+n_scaling] = idx_scaling*r_fast.mixer._CONTROL_N_WORDS+r_fast.mixer._SCALE_WORD_OFFSET
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

    # Get the dynamically-looked-up base addresses for the control buffers
    addrs = _get_control_buffer_addresses(r_fast)
    # Buffer size in bytes: n_serial_chans * _CONTROL_N_WORDS * 4 bytes
    buf_size = r_fast.mixer._n_serial_chans * r_fast.mixer._CONTROL_N_WORDS * 4

    if indices is None:
        for lo in ['tx','rx']:
            start = addrs[lo] + buf_size * buf
            length = buf_size
            r_fast.mixer.host.transport.axil_mm[int(start):int(start+length)] = v[lo].astype('<u4').tobytes()
    else:
        # Convert byte addresses to word offsets (divide by 4)
        start_tx = addrs['tx'] // 4 + (buf_size * buf) // 4
        start_rx = addrs['rx'] // 4 + (buf_size * buf) // 4
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

    # moved from outmap to inmap in the v7.9 psb_chanselect
    # #get the filterbank channels
    # chanmap_psb  = psb_chanselect_get_channel_outmap(r)
    # chanmap_pfb  = chanselect_get_channel_outmap(r)
    # psb_chans_active = np.nonzero(chanmap_psb+1)[0]
    # pfb_chans_active = np.nonzero(chanmap_pfb+1)[0]
    # psb_channels = psb_chans_active[np.argsort(chanmap_psb[psb_chans_active])]
    # pfb_channels = chanmap_pfb[pfb_chans_active]
    # if np.all(chanmap_psb == 2047):
    #     warnings.warn('Possibly attempting to get frequencies when none are set.')
    #     psb_channels = np.copy(pfb_channels)

    #get the filterbank channels
    chanmap_psb_inmap = psb_chanselect_get_channel_inmap(r)
    chanmap_pfb = chanselect_get_channel_outmap(r)

    # For inmap: find active input channels (tones) - those not mapping to discard bin
    # For outmap: find active output channels (tones) - those not mapping to discard chan
    psb_discard_bit = r.psb_chanselect.DISCARD_BIT
    pfb_discard_chan = -1
    psb_tones_active = np.nonzero((chanmap_psb_inmap & psb_discard_bit) == 0)[0]
    pfb_chans_active = np.nonzero(chanmap_pfb != pfb_discard_chan)[0]

    # psb_channels are the FFT bins that active tones map to (in tone order)
    psb_channels = chanmap_psb_inmap[psb_tones_active]
    pfb_channels = chanmap_pfb[pfb_chans_active]



    #get the number of active channels (assumes anything not -1 is a channel)
    num_tones_tx = len(psb_channels)
    num_tones_rx = len(pfb_channels)
    if num_tones_tx != num_tones_rx:
        warnings.warn(f'Number of tones in tx ({num_tones_tx}) and rx ({num_tones_rx}) do not match.')

    #get filterbank center frequencies
    tx_bin_centers_hz = all_tx_bin_centers_hz[psb_channels]
    rx_bin_centers_hz = all_rx_bin_centers_hz[pfb_channels]

    #get the digital baseband frequencies
    # index by psb_tones_active (not :num_tones_tx) since tone indices may be non-contiguous with VACC
    dbb_freqs_tx = tx_bin_centers_hz + offset_freqs_hz_tx[psb_tones_active]
    dbb_freqs_rx = rx_bin_centers_hz + offset_freqs_hz_rx[pfb_chans_active]

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
        details['tx']['tone_indices'] = psb_tones_active.tolist()
        details['tx']['filterbank_bins'] = psb_channels.tolist()
        details['tx']['mixer_lo_phase_increment'] = phase_inc_tx[psb_tones_active].tolist()
        details['tx']['mixer_lo_ri_step'] = [(i,q) for i,q in zip(ri_steps_tx[psb_tones_active].real.tolist(),ri_steps_tx[psb_tones_active].imag.tolist())]
        details['tx']['mixer_lo_phase_step'] = phase_steps_tx[psb_tones_active].tolist()
        details['tx']['mixer_lo_offset_freq'] = offset_freqs_hz_tx[psb_tones_active].tolist()
        details['tx']['filterbank_center_freq'] = tx_bin_centers_hz.tolist()
        details['tx']['digital_baseband_freq'] = dbb_freqs_tx.tolist()
        details['tx']['analog_output_freq'] = dac_out_freqs.tolist()
        details['tx']['rf_output_freq'] = udc_freqs_tx.tolist()
        details['rx']['tone_indices'] = pfb_chans_active.tolist()
        details['rx']['filterbank_bins'] = pfb_channels.tolist()
        details['rx']['mixer_lo_phase_increment'] = phase_inc_rx[pfb_chans_active].tolist()
        details['rx']['mixer_lo_ri_step'] = [(i,q) for i,q in zip(ri_steps_rx[pfb_chans_active].real.tolist(),ri_steps_rx[pfb_chans_active].imag.tolist())]
        details['rx']['mixer_lo_phase_step'] = phase_steps_rx[pfb_chans_active].tolist()
        details['rx']['mixer_lo_offset_freq'] = offset_freqs_hz_rx[pfb_chans_active].tolist()
        details['rx']['filterbank_center_freq'] = rx_bin_centers_hz.tolist()
        details['rx']['digital_baseband_freq'] = dbb_freqs_rx.tolist()
        details['rx']['analog_input_freq'] = adc_in_freqs.tolist()
        details['rx']['rf_input_freq'] = udc_freqs_rx.tolist()
        return output_freqs,details
    else:
        return output_freqs


def compute_vacc_tone_indices(tx_nearest_bins, n_lo, min_tone_separation=6):
    """
    Compute LO indices for tones in user order, respecting VACC constraints.

    When multiple tones map to the same TX PSB bin, their LO indices must be
    separated by at least min_tone_separation due to VACC dual-port RAM timing.
    """
    tx_nearest_bins = np.atleast_1d(tx_nearest_bins)
    num_tones = len(tx_nearest_bins)

    if num_tones == 0:
        return np.array([], dtype=int)

    # Fast path: all bins unique -> no VACC constraints apply
    if np.unique(tx_nearest_bins).size == num_tones:
        return np.arange(num_tones, dtype=int)

    # Precompute: for each tone, index of previous tone with same bin (-1 if none)
    prev_same_bin = np.full(num_tones, -1, dtype=int)
    bin_last_seen = {}
    for i, b in enumerate(tx_nearest_bins.tolist()):
        if b in bin_last_seen:
            prev_same_bin[i] = bin_last_seen[b]
        bin_last_seen[b] = i

    # Assign LO indices
    tone_indices = np.empty(num_tones, dtype=int)
    next_lo = 0

    for i in range(num_tones):
        if prev_same_bin[i] >= 0:
            # Must maintain separation from previous same-bin tone
            next_lo = max(next_lo, tone_indices[prev_same_bin[i]] + min_tone_separation)

        if next_lo >= n_lo:
            raise ValueError(f'Exceeded {n_lo} LO channels with min_tone_separation={min_tone_separation}')

        tone_indices[i] = next_lo
        next_lo += 1

    return tone_indices



def prepare_tone_frequency_settings(r, config_dict, tone_frequencies, tone_indices=None, min_tone_separation=6):
    """
    Prepare the tone frequency settings for applying to the RFSOC.

    Given a set of desired RF tone frequencies, the function calculates the
    required DAC/ADC analog frequencies given any analog up/down conversion.
    The required DAC/ADC digital frequencies are then calculated given
    the selected Nyquist zone and the digital baseband frequencies are calculated given
    the RFDC DUC/DDC setting. Finally the filterbank center frequencies and the mixer LO
    offsets are translated to the formatted channel maps and phase accumulator increments
    and returned for setting in the RFSOC firmware.

    Parameters:
    r: readout object
    config_dict: configuration dictionary
    tone_frequencies: array of tone frequencies in Hz
    tone_indices: array of LO indices for the tones. If None, automatically computes optimal
                  indices using compute_vacc_tone_indices() to handle VACC constraints.
    min_tone_separation: minimum separation between LO indices feeding the same FFT bin
                         (only used when tone_indices is None). Default is 6.
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

    chanmap_pfb  = np.full(r.chanselect.n_chans_out, -1, dtype=int)
    num_tones = len(tone_frequencies)


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

    # Compute optimal tone indices if not provided
    # This handles VACC constraints where tones in the same FFT bin need separated LO indices
    if tone_indices is None:
        tone_indices = compute_vacc_tone_indices(tx_nearest_bins, r.mixer.n_chans, min_tone_separation)
    else:
        tone_indices = np.asarray(tone_indices)
        if len(tone_indices) != num_tones:
            raise ValueError(f'Number of tone_indices ({len(tone_indices)}) must match number of tone_frequencies ({num_tones})')


    #get the frequency offsets for each tone
    tx_freq_offsets_hz = dbb_freqs_tx - all_tx_bin_centers_hz[tx_nearest_bins]
    rx_freq_offsets_hz = dbb_freqs_rx - all_rx_bin_centers_hz[rx_nearest_bins]

    #get the phase increments and ri steps for the mixer LOs
    phase_incs_tx = tx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    phase_incs_rx = rx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
    ri_steps_tx = np.cos(phase_incs_tx) + 1j*np.sin(phase_incs_tx)
    ri_steps_rx = np.cos(phase_incs_rx) + 1j*np.sin(phase_incs_rx)

    # Build full-sized arrays with values at the correct tone_indices positions
    # This is required because prepare_control_buffer_data expects full arrays
    n_chans = r.mixer.n_chans
    phase_incs_tx_full = np.zeros(n_chans)
    phase_incs_rx_full = np.zeros(n_chans)
    ri_steps_tx_full = np.zeros(n_chans, dtype=complex)
    ri_steps_rx_full = np.zeros(n_chans, dtype=complex)

    phase_incs_tx_full[tone_indices] = phase_incs_tx
    phase_incs_rx_full[tone_indices] = phase_incs_rx
    ri_steps_tx_full[tone_indices] = ri_steps_tx
    ri_steps_rx_full[tone_indices] = ri_steps_rx

    #prepare the formatted lo control buffer values
    buf = get_next_buffer_idx(r)
    v = prepare_control_buffer_data(r,buf,{'tx':{'phase_steps':phase_incs_tx_full,
                                            'ri_steps':ri_steps_tx_full},
                                      'rx':{'phase_steps':phase_incs_rx_full,
                                            'ri_steps':ri_steps_rx_full}})

    # phase_incs_tx_formatted = _format_phase_steps(phase_incs_tx,r.mixer._phase_bp)
    # phase_incs_rx_formatted = _format_phase_steps(phase_incs_rx,r.mixer._phase_bp)
    # ri_steps_tx_formatted = cplx2uint(ri_steps_tx, r.mixer._n_ri_step_bits)
    # ri_steps_rx_formatted = cplx2uint(ri_steps_rx, r.mixer._n_ri_step_bits)

    #set the filterbank channel maps
    # v7.9: use inmap for psb_chanselect (chanmap_psb_inmap[lo_index] = fft_bin)
    chanmap_psb_inmap = np.full(r.psb_chanselect.n_chans_in, r.psb_chanselect.DISCARD_BIN, dtype=np.uint32)  # default to discard bin
    chanmap_psb_inmap[tone_indices] = tx_nearest_bins
    # chanmap_pfb uses outmap: outmap[output_slot] = fft_bin
    # Use tone_indices so RX output slots match TX LO indices
    chanmap_pfb[tone_indices] = rx_nearest_bins

    # tone_settings_dict = {'phase_incs_tx_formatted':phase_incs_tx_formatted,
    #                       'phase_incs_rx_formatted':phase_incs_rx_formatted,
    #                       'ri_steps_tx_formatted':ri_steps_tx_formatted,
    #                       'ri_steps_rx_formatted':ri_steps_rx_formatted,
    #                       'chanmap_psb':chanmap_psb,
    #                       'chanmap_pfb':chanmap_pfb,
    #                       'num_tones':num_tones}

    tone_settings_dict = {'control_buffer_data_values':v,
                          'control_buffer_index':buf,
                          'chanmap_psb_inmap':chanmap_psb_inmap,
                          'chanmap_pfb':chanmap_pfb,
                          'tone_indices':tone_indices,
                          'num_tones':num_tones}


    details = {'tx':{},'rx':{},'num_tones':num_tones, 'tone_indices':tone_indices.tolist()}
    details['tx']['digital_baseband_freq'] = dbb_freqs_tx.tolist()
    details['tx']['filterbank_center_freq'] = all_tx_bin_centers_hz[tx_nearest_bins].tolist()
    details['tx']['filterbank_channel_inmap'] = chanmap_psb_inmap.tolist()
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
    'control_buffer_data_values', 'control_buffer_index', 'chanmap_psb_inmap', 'chanmap_pfb', 'num_tones
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
    chanmap_psb_inmap = tone_settings_dict.get('chanmap_psb_inmap')
    chanmap_pfb   = tone_settings_dict.get('chanmap_pfb')
    num_tones     = tone_settings_dict.get('num_tones')

    if num_tones is None:
        # num_tones = max((len(phase_incs_tx),len(phase_incs_tx),len(ri_steps_tx),len(ri_steps_tx),))
        num_tones = r.mixer.n_chans

    # v7.9: use inmap setter for psb_chanselect
    if not chanmap_psb_inmap is None:
        psb_chanselect_set_channel_inmap(r,np.copy(chanmap_psb_inmap))
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

def prepare_tone_frequency_settings_fast(r, config_dict, tone_frequencies, tone_indices=None, min_tone_separation=6, detailed_output=False):
    """
    Prepare the tone frequency settings for applying to the RFSOC.

    Given a set of desired RF tone frequencies, the function calculates the
    required DAC/ADC analog frequencies given any analog up/down conversion.
    The required DAC/ADC digital frequencies are then calculated given
    the selected Nyquist zone and the digital baseband frequencies are calculated given
    the RFDC DUC/DDC setting. Finally the filterbank center frequencies and the mixer LO
    offsets are translated to the formatted channel maps and phase accumulator increments
    and returned for setting in the RFSOC firmware.

    Parameters:
    r: readout object
    config_dict: configuration dictionary
    tone_frequencies: array of tone frequencies in Hz
    tone_indices: array of LO indices for the tones. If None, automatically computes optimal
                  indices using compute_vacc_tone_indices() to handle VACC constraints.
    min_tone_separation: minimum separation between LO indices feeding the same FFT bin
                         (only used when tone_indices is None). Default is 6.
    detailed_output: if True, return detailed output dictionary
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
    defaults = config_dict['firmware']['defaults']
    dac_nyquist_zone = defaults['nyquist_zone']
    adc_nyquist_zone = defaults['nyquist_zone']
    # DUC/DDC mixer frequencies: use stored values if available, otherwise derive from nyquist zone
    _fs = 2 * r.adc_clk_hz  # RFDC sampling frequency
    duc_frequency = defaults.get('dac_duc_mixer_frequency_hz',
        _fs / 4 if dac_nyquist_zone == 1 else -_fs * 3 / 4)
    ddc_frequency = defaults.get('adc_ddc_mixer_frequency_hz',
        -_fs / 4 if adc_nyquist_zone == 1 else _fs * 3 / 4)

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
    chanmap_pfb  = np.full(r.chanselect.n_chans_out, -1, dtype=int)
    num_tones = len(tone_frequencies)


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

    # Compute optimal tone indices if not provided
    # This handles VACC constraints where tones in the same FFT bin need separated LO indices
    if tone_indices is None:
        tone_indices = compute_vacc_tone_indices(tx_nearest_bins, r.mixer.n_chans, min_tone_separation)
    else:
        tone_indices = np.asarray(tone_indices)
        if len(tone_indices) != num_tones:
            raise ValueError(f'Number of tone_indices ({len(tone_indices)}) must match number of tone_frequencies ({num_tones})')


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

    # Pass tone_indices to prepare_control_buffer_data_fast for correct sparse indexing
    v,i = prepare_control_buffer_data_fast(r,0,{'tx':{'phase_steps':phase_incs_tx,
                                            'ri_steps':ri_steps_tx},
                                      'rx':{'phase_steps':phase_incs_rx,
                                            'ri_steps':ri_steps_rx}},
                                      tone_indices=tone_indices)
    # #format the phase increments and ri steps for the mixer LOs
    # phase_incs_tx_formatted = _format_phase_steps(phase_incs_tx,r.mixer._phase_bp,fmt='<i4')
    # phase_incs_rx_formatted = _format_phase_steps(phase_incs_rx,r.mixer._phase_bp,fmt='<i4')
    # ri_steps_tx_formatted = cplx2uint(ri_steps_tx, r.mixer._n_ri_step_bits,fmt='<u4')
    # ri_steps_rx_formatted = cplx2uint(ri_steps_rx, r.mixer._n_ri_step_bits,fmt='<u4')

    #set the filterbank channel maps
    # v7.9: use inmap for psb_chanselect (chanmap_psb_inmap[lo_index] = fft_bin)
    chanmap_psb_inmap = np.full(r.psb_chanselect.n_chans_in, r.psb_chanselect.DISCARD_BIN, dtype=np.uint32)  # default to discard bin
    chanmap_psb_inmap[tone_indices] = tx_nearest_bins
    # chanmap_pfb uses outmap: outmap[output_slot] = fft_bin
    # Use tone_indices so RX output slots match TX LO indices
    chanmap_pfb[tone_indices] = rx_nearest_bins

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
                            'chanmap_psb_inmap':chanmap_psb_inmap,
                            'chanmap_pfb':chanmap_pfb,
                            'tone_indices':tone_indices,
                            'num_tones':num_tones}

    if detailed_output:
        details = {'tx':{},'rx':{},'num_tones':num_tones,'tone_indices':tone_indices.tolist()}
        details['tx']['digital_baseband_freq'] = dbb_freqs_tx.tolist()
        details['tx']['filterbank_center_freq'] = all_tx_bin_centers_hz[tx_nearest_bins].tolist()
        details['tx']['filterbank_channel_inmap'] = chanmap_psb_inmap.tolist()
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


def prepare_sweep_settings_fast(r_fast, config_dict, sweep_frequencies, min_tone_separation=6, detailed_output=False):
    """
    Prepare sweep step settings with VACC-aware tone index assignment.

    sweep_freqs: 2D array (num_points, num_tones)
    """
    sweep_frequencies = np.atleast_2d(sweep_frequencies)
    num_points,num_tones = sweep_frequencies.shape
    channels = np.arange(num_tones)
    points = np.arange(num_points)
    n_lo = r_fast.mixer.n_chans  # 2048

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
    defaults = config_dict['firmware']['defaults']
    dac_nyquist_zone = defaults['nyquist_zone']
    adc_nyquist_zone = defaults['nyquist_zone']
    # DUC/DDC mixer frequencies: use stored values if available, otherwise derive from nyquist zone
    _fs = 2 * r_fast.adc_clk_hz  # RFDC sampling frequency
    duc_frequency = defaults.get('dac_duc_mixer_frequency_hz',
        _fs / 4 if dac_nyquist_zone == 1 else -_fs * 3 / 4)
    ddc_frequency = defaults.get('adc_ddc_mixer_frequency_hz',
        -_fs / 4 if adc_nyquist_zone == 1 else _fs * 3 / 4)

    #constants
    nc = r_fast.mixer.n_chans
    fft_period_s = r_fast.mixer._n_upstream_chans / r_fast.mixer._upstream_oversample_factor / r_fast.adc_clk_hz
    fft_rbw_hz = 1./fft_period_s
    fft_tx_nbins = 2 * N_TX_FFT
    fft_rx_nbins = N_RX_FFT
    all_tx_bin_centers_hz = np.fft.fftfreq(fft_tx_nbins, 1. / r_fast.adc_clk_hz)
    all_rx_bin_centers_hz = np.fft.fftfreq(fft_rx_nbins, 1. / r_fast.adc_clk_hz)

    # v7.9: use inmap for psb_chanselect - size is n_chans_in (LO indices), default to discard bin
    psb_discard_bin = r_fast.psb_chanselect.DISCARD_BIN
    chanmap_psb_inmap = np.full((num_points, r_fast.psb_chanselect.n_chans_in), psb_discard_bin, dtype=np.uint32)
    chanmap_pfb  = np.full((num_points,r_fast.chanselect.n_chans_out), -1, dtype=int)

    skip_chanmap_psb_inmap=np.zeros(num_points,dtype=bool)
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


    # Check if TX bin assignments are stable across all sweep points
    # This is common for narrow sweeps where frequency shift < bin bandwidth
    tx_bins_stable = np.all(tx_nearest_bins == tx_nearest_bins[0:1, :])

    if tx_bins_stable:
        # Optimization: compute tone_indices once, tile for all points
        tone_indices = compute_vacc_tone_indices(
            tx_nearest_bins[0], n_lo, min_tone_separation
        )
        tone_indices_arr = np.tile(tone_indices, (num_points, 1))  # (num_points, num_tones)
    else:
        # TX bins change during sweep - must compute per point
        # This is slower but necessary for wide sweeps
        tone_indices_arr = np.zeros((num_points, num_tones), dtype=int)
        for p in range(num_points):
            tone_indices_arr[p] = compute_vacc_tone_indices(
                tx_nearest_bins[p], n_lo, min_tone_separation
            )



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
                                                                   'ri_steps':ri_steps_rx[p]}},
                                                            tone_indices=tone_indices_arr[p])
        #set the filterbank channel maps
        # v7.9: use inmap for psb_chanselect (chanmap_psb_inmap[lo_index] = fft_bin)
        chanmap_psb_inmap[p, tone_indices_arr[p]] = tx_nearest_bins[p]
        # chanmap_pfb uses outmap: outmap[output_slot] = fft_bin
        # Use tone_indices so RX output slots match TX LO indices
        chanmap_pfb[p, tone_indices_arr[p]] = rx_nearest_bins[p]

    for p in points:
        if p==0:
            # continue, not pass!
            continue
        if (chanmap_psb_inmap[p] == chanmap_psb_inmap[p-1]).all():
            skip_chanmap_psb_inmap[p]=True
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
                            'chanmap_psb_inmap':chanmap_psb_inmap,
                            'chanmap_pfb':chanmap_pfb,
                            'skip_chanmap_psb_inmap':skip_chanmap_psb_inmap,
                            'skip_chanmap_pfb':skip_chanmap_pfb,
                            'tone_indices':tone_indices_arr,
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
    chanmap_psb_inmap   = sweep_settings.get('chanmap_psb_inmap')
    chanmap_pfb   = sweep_settings.get('chanmap_pfb')
    skip_chanmap_psb_inmap = sweep_settings.get('skip_chanmap_psb_inmap')
    skip_chanmap_pfb = sweep_settings.get('skip_chanmap_pfb')
    num_tones     = sweep_settings.get('num_tones')

    c1=not skip_chanmap_psb_inmap[step_index]
    c2=not skip_chanmap_pfb[step_index]
    if c1:
        print('set chanmap 1 (psb inmap)')
        # v7.9: use inmap setter for psb_chanselect
        psb_chanselect_set_channel_inmap(r_fast, chanmap_psb_inmap[step_index])

        # while not (r.psb_chanselect.get_channel_outmap()==chanmap_psb[step_index]).all():
        #     print('waiting for psb chanmap to update')
        #     time.sleep(0.001)
        # print('psb chanmap updated')
    if c2:
        print('set chanmap 2 (pfb outmap)')
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
    chanmap_psb_inmap = fast_tone_frequency_settings.get('chanmap_psb_inmap')
    chanmap_pfb   = fast_tone_frequency_settings.get('chanmap_pfb')
    # num_tones     = fast_tone_frequency_settings.get('num_tones')
    c1 = chanmap_psb_inmap is not None
    c2 = chanmap_pfb is not None

    # v7.9: use inmap setter for psb_chanselect
    if c1:
        # print('chanmap_psb_inmap set')
        psb_chanselect_set_channel_inmap(r_fast,chanmap_psb_inmap)
    if c2:
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


def set_tone_frequencies(r, config_dict, tone_frequencies, tone_indices=None, min_tone_separation=6, autosync=True, detailed_output=False):
    """
    Set the tone frequencies in the RFSOC.

    Given a set of desired RF tone frequencies, the function calculates the
    required DAC/ADC analog frequencies given any analog up/down conversion.
    The required DAC/ADC digital frequencies are then calculated given
    the selected Nyquist zone and the digital baseband frequencies are calculated given
    the RFDC DUC/DDC setting. Finally the filterbank center frequencies and the mixer LO
    offsets are translated to the formatted channel maps and phase accumulator increments
    and written to the RFSOC firmware.

    Parameters:
    r: readout object
    config_dict: configuration dictionary
    tone_frequencies: array of tone frequencies in Hz
    tone_indices: array of LO indices for the tones. If None, automatically computes optimal
                  indices using compute_vacc_tone_indices() to handle VACC constraints.
    min_tone_separation: minimum separation between LO indices feeding the same FFT bin
                         (only used when tone_indices is None). Default is 6.
    autosync: if True, sync after setting tones
    detailed_output: if True, return detailed output dictionary

    TODO: account for dual dac mode, for now assume all on dac 0

    """

    tone_frequency_settings, details = prepare_tone_frequency_settings(r, config_dict, tone_frequencies,
                                                                        tone_indices=tone_indices,
                                                                        min_tone_separation=min_tone_separation)
    apply_tone_frequency_settings(r, tone_frequency_settings, autosync=autosync)

    r.sync.arm_sync(wait=False)
    time.sleep(0.001)
    r.sync.sw_sync()

    if detailed_output:
        return details
    else:
        return

def set_tone_frequencies_fast(r, r_fast, config_dict, tone_frequencies, tone_indices=None, min_tone_separation=6, autosync=True):
    """
    Set the tone frequencies in the RFSOC using the fast firmware interface.

    Given a set of desired RF tone frequencies, the function calculates the
    required DAC/ADC analog frequencies given any analog up/down conversion.
    The required DAC/ADC digital frequencies are then calculated given
    the selected Nyquist zone and the digital baseband frequencies are calculated given
    the RFDC DUC/DDC setting. Finally the filterbank center frequencies and the mixer LO
    offsets are translated to the formatted channel maps and phase accumulator increments
    and written to the fast firmware interface.

    Parameters:
    r: readout object
    r_fast: fast firmware interface object
    config_dict: configuration dictionary
    tone_frequencies: array of tone frequencies in Hz
    tone_indices: array of LO indices for the tones. If None, automatically computes optimal
                  indices using compute_vacc_tone_indices() to handle VACC constraints.
    min_tone_separation: minimum separation between LO indices feeding the same FFT bin
                         (only used when tone_indices is None). Default is 6.
    autosync: if True, sync after setting tones
    """

    tone_frequency_settings = prepare_tone_frequency_settings_fast(r, config_dict, tone_frequencies,
                                                                    tone_indices=tone_indices,
                                                                    min_tone_separation=min_tone_separation)
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



def get_tone_amplitudes(r,config_dict):
    """
    Query the RFSOC for the current tone amplitude scale factors.
    """
    # moved from outmap to inmap in the v7.9 psb_chanselect
    chanmap_psb_inmap = psb_chanselect_get_channel_inmap(r)
    chanmap_pfb = chanselect_get_channel_outmap(r)

    psb_discard_bit = r.psb_chanselect.DISCARD_BIT
    pfb_discard_chan = -1
    psb_tones_active = np.nonzero((chanmap_psb_inmap & psb_discard_bit) == 0)[0]
    pfb_chans_active = np.nonzero(chanmap_pfb != pfb_discard_chan)[0]

    if len(psb_tones_active) == 0:
        warnings.warn('Possibly attempting to get amplitudes when no tones are set.')
        return np.array([],dtype=float)

    num_tones_tx = len(psb_tones_active)
    num_tones_rx = len(pfb_chans_active)

    if num_tones_tx != num_tones_rx:
        warnings.warn(f'Number of tones in tx ({num_tones_tx}) and rx ({num_tones_rx}) do not match.')

    control_buffer = read_from_current_control_buffer(r)
    scaling_tx = control_buffer['tx']['scaling']
    scaling_rx = control_buffer['rx']['scaling']
    # index by psb_tones_active (not :num_tones) since tone indices may be non-contiguous with VACC
    return scaling_tx[psb_tones_active]

def set_tone_amplitudes(r, config_dict, tone_amplitudes,autosync=True):
    """
    Set the tone amplitude scale factors in the RFSOC.

    Currently sets both halfs of the double buffer to the same value.
    This is not ideal, but for now it will
    """
    tone_amplitudes = np.atleast_1d(tone_amplitudes)

    # Get active tone indices - with VACC these may be non-contiguous
    chanmap_psb_inmap = psb_chanselect_get_channel_inmap(r)
    psb_discard_bit = r.psb_chanselect.DISCARD_BIT
    psb_tones_active = np.nonzero((chanmap_psb_inmap & psb_discard_bit) == 0)[0]

    if len(tone_amplitudes) != len(psb_tones_active):
        raise ValueError(f'Number of amplitudes ({len(tone_amplitudes)}) does not match number of active tones ({len(psb_tones_active)})')

    # Create full-sized array and place values at correct LO indices
    scaling_full = np.zeros(r.mixer.n_chans, dtype=float)
    scaling_full[psb_tones_active] = tone_amplitudes

    # buf = get_control_buffer_idx(r)
    for buf in [0,1]:
        v = prepare_control_buffer_data(r,buf,{'tx':{'scaling':scaling_full},
                                    'rx':{'scaling':scaling_full}})

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

def get_tone_phases(r, config_dict):
    """
    Query the RFSOC for the current tone phase offsets.
    Note that the returned values are in the range [-pi,pi] regardless of how they were set.
    """
    # moved from outmap to inmap in the v7.9 psb_chanselect
    chanmap_psb_inmap = psb_chanselect_get_channel_inmap(r)
    chanmap_pfb = chanselect_get_channel_outmap(r)

    psb_discard_bit = r.psb_chanselect.DISCARD_BIT
    pfb_discard_chan = -1
    psb_tones_active = np.nonzero((chanmap_psb_inmap & psb_discard_bit) == 0)[0]
    pfb_chans_active = np.nonzero(chanmap_pfb != pfb_discard_chan)[0]

    if len(psb_tones_active) == 0:
        warnings.warn('Possibly attempting to get phases when no tones are set.')
        return np.array([],dtype=float)

    num_tones_tx = len(psb_tones_active)
    num_tones_rx = len(pfb_chans_active)

    if num_tones_tx != num_tones_rx:
        warnings.warn(f'Number of tones in tx ({num_tones_tx}) and rx ({num_tones_rx}) do not match.')

    control_buffer = read_from_current_control_buffer(r)
    phase_offsets_tx = control_buffer['tx']['phase_offsets']
    phase_offsets_rx = control_buffer['rx']['phase_offsets']
    # index by psb_tones_active (not :num_tones) since tone indices may be non-contiguous with VACC
    return phase_offsets_tx[psb_tones_active]

def set_tone_phases(r, config_dict, tone_phases, autosync=True):
    """
    Set the tone phase offsets in the RFSOC.
    """
    tone_phases = np.atleast_1d(tone_phases)

    # Get active tone indices - with VACC these may be non-contiguous
    chanmap_psb_inmap = psb_chanselect_get_channel_inmap(r)
    psb_discard_bit = r.psb_chanselect.DISCARD_BIT
    psb_tones_active = np.nonzero((chanmap_psb_inmap & psb_discard_bit) == 0)[0]

    if len(tone_phases) != len(psb_tones_active):
        raise ValueError(f'Number of phases ({len(tone_phases)}) does not match number of active tones ({len(psb_tones_active)})')

    # Create full-sized array and place values at correct LO indices
    phase_offsets_full = np.zeros(r.mixer.n_chans, dtype=float)
    phase_offsets_full[psb_tones_active] = tone_phases

    # buf = get_control_buffer_idx(r)
    for buf in [0,1]:
        v = prepare_control_buffer_data(r,buf,{'tx':{'phase_offsets':phase_offsets_full},
                                    'rx':{'phase_offsets':phase_offsets_full}})

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





def psb_chanselect_set_channel_outmap(r, outmap):
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



def psb_chanselect_get_channel_outmap(r):
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


# New function to set channel in-map for the VACC-based PSB reorder
def psb_chanselect_set_channel_inmap(r, inmap):
    """
    Remap the channels such that input channel `i`
    contributes to output channel `inmap[i]`

    :param inmap: The mapping of input to output data. I.e.,
        if `inmap[16] = 0` then input channel 16 will contribute to
        output channel 0.
    :type inmap: list
    """

    r.psb_chanselect.set_channel_inmap(inmap)
    if False:
            
        if not hasattr(r.psb_chanselect, '_cached_block_id'):
            n_exp = r.psb_chanselect._expansion_factor
            n_par_samp = r.psb_chanselect.n_parallel_samples
            n_par_chans = r.psb_chanselect.n_parallel_chans_out

            r.psb_chanselect._c_n_exp = n_exp
            r.psb_chanselect._c_n_chans_in = r.psb_chanselect.n_chans_in
            r.psb_chanselect._c_n_chans_out = r.psb_chanselect.n_chans_out
            r.psb_chanselect._c_discard_bin = r.psb_chanselect.DISCARD_BIN
            r.psb_chanselect._c_discard_bit = r.psb_chanselect.DISCARD_BIT
            r.psb_chanselect._c_addr_mask = r.psb_chanselect.ADDR_MASK

            outchans = np.arange(r.psb_chanselect._c_n_chans_out)
            r.psb_chanselect._cached_block_id = (outchans // n_par_samp) % n_exp
            r.psb_chanselect._cached_block_offset = (outchans // n_par_chans) * n_par_samp + (outchans % n_par_samp)

            r.psb_chanselect._cached_lookup = np.full((n_exp, r.psb_chanselect._reorder_depth), r.psb_chanselect._c_discard_bin, dtype=int)
            r.psb_chanselect._cached_lookup[r.psb_chanselect._cached_block_id, r.psb_chanselect._cached_block_offset] = outchans

        # Initialize with DISCARD_BIN (has DISCARD_BIT set)
        serial_maps = np.full((r.psb_chanselect._c_n_exp, r.psb_chanselect._reorder_depth), r.psb_chanselect._c_discard_bin, dtype=np.uint32)

        inmap = np.asarray(inmap, dtype=np.uint32)
        # Filter out discarded entries (those with DISCARD_BIT set)
        valid_mask = (inmap & r.psb_chanselect._c_discard_bit) == 0
        valid_inmap = inmap[valid_mask]
        nin = len(valid_inmap)

        # Vectorized assignment
        input_indices = np.where(valid_mask)[0][:nin]
        serial_maps[r.psb_chanselect._cached_block_id[valid_inmap], input_indices] = r.psb_chanselect._cached_block_offset[valid_inmap]

        # Write to hardware
        for i in range(r.psb_chanselect._c_n_exp):
            r.psb_chanselect.write(f'map{i}_{r.psb_chanselect._map_reg}', serial_maps[i].tobytes())

    


def psb_chanselect_get_channel_inmap(r):
    """
    Get the currently loaded reorder map.
    :return: The reorder map currently loaded. Entry `i` in this map
        corresponds to the output channel to which input `i` contributes.
    :rtype: list
    """
    im = r.psb_chanselect.get_channel_inmap()
    return im
    if False:
            
        if not hasattr(r.psb_chanselect, '_cached_block_id'):
            n_exp = r.psb_chanselect._expansion_factor
            n_par_samp = r.psb_chanselect.n_parallel_samples
            n_par_chans = r.psb_chanselect.n_parallel_chans_out

            r.psb_chanselect._c_n_exp = n_exp
            r.psb_chanselect._c_n_chans_in = r.psb_chanselect.n_chans_in
            r.psb_chanselect._c_n_chans_out = r.psb_chanselect.n_chans_out
            r.psb_chanselect._c_discard_bin = r.psb_chanselect.DISCARD_BIN
            r.psb_chanselect._c_discard_bit = r.psb_chanselect.DISCARD_BIT
            r.psb_chanselect._c_addr_mask = r.psb_chanselect.ADDR_MASK

            outchans = np.arange(r.psb_chanselect._c_n_chans_out)
            r.psb_chanselect._cached_block_id = (outchans // n_par_samp) % n_exp
            r.psb_chanselect._cached_block_offset = (outchans // n_par_chans) * n_par_samp + (outchans % n_par_samp)

            r.psb_chanselect._cached_lookup = np.full((n_exp, r.psb_chanselect._reorder_depth), r.psb_chanselect._c_discard_bin, dtype=int)
            r.psb_chanselect._cached_lookup[r.psb_chanselect._cached_block_id, r.psb_chanselect._cached_block_offset] = outchans

        # Read the reorder memory contents
        nbytes = r.psb_chanselect._reorder_depth * np.dtype(np.uint32).itemsize
        serial_maps = np.full((r.psb_chanselect._c_n_exp, r.psb_chanselect._reorder_depth), r.psb_chanselect._c_discard_bin, dtype=np.uint32)
        for i in range(r.psb_chanselect._c_n_exp):
            serial_maps[i] = np.frombuffer(r.psb_chanselect.read(f'map{i}_{r.psb_chanselect._map_reg}', nbytes), dtype=np.uint32).view(np.uint32)

        # Check for valid mappings: entries without DISCARD_BIT set
        is_valid = (serial_maps[:, :r.psb_chanselect._c_n_chans_in] & r.psb_chanselect._c_discard_bit) == 0
        first_exp = np.argmax(is_valid, axis=0)
        has_mapping = np.any(is_valid, axis=0)

        # Gather stored values and lookup
        input_idx = np.arange(r.psb_chanselect._c_n_chans_in)
        # Mask out the discard bit to get actual offset values
        stored = (serial_maps[first_exp, input_idx] & r.psb_chanselect._c_addr_mask).astype(int)

        # Build result
        inmap = np.full(r.psb_chanselect._c_n_chans_in, r.psb_chanselect._c_discard_bin, dtype=int)
        inmap[has_mapping] = r.psb_chanselect._cached_lookup[first_exp[has_mapping], stored[has_mapping]]

        return inmap




# # not tested



# def prepare_tone_frequency_settings_vacc(r, config_dict, tone_frequencies):
#     """
#     Prepare tone frequency settings for VACC-enabled firmware.

#     Unlike prepare_tone_frequency_settings(), this uses inmap semantics and
#     supports multiple tones per FFT bin with VACC constraint (min 2 LO separation).

#     Supports both standard and fast firmware interfaces.

#     :param r: Firmware interface (standard or fast)
#     :param config_dict: Configuration dictionary
#     :param tone_frequencies: Array of tone frequencies in Hz
#     :return: Dictionary with prepared settings
#     """
#     # Get config parameters
#     udc_connected = config_dict.get('rf_frontend', {}).get('connected', False)
#     udc_lo_frequency = float(config_dict.get('rf_frontend', {}).get('tx_mixer_lo_frequency_hz', 0))
#     udc_sideband = int(config_dict.get('rf_frontend', {}).get('tx_mixer_sideband', 1))
#     dac_tile = int(config_dict['firmware']['dac0_tile'])
#     dac_block = int(config_dict['firmware']['dac0_block'])
#     adc_tile = int(config_dict['firmware']['adc_tile'])
#     adc_block = int(config_dict['firmware']['adc_block'])

#     tone_frequencies = np.atleast_1d(tone_frequencies)
#     num_tones = len(tone_frequencies)
#     n_chans_in = r.psb_chanselect.n_chans_in  # 2048 LOs
#     n_chans_out = r.psb_chanselect.n_chans_out  # 8192 FFT bins

#     # Get FFT bin centers
#     N_TX_FFT = n_chans_out // 2  # 4096
#     N_RX_FFT = r.chanselect.n_chans_in
#     fft_period_s = r.mixer._n_upstream_chans / r.mixer._upstream_oversample_factor / r.adc_clk_hz
#     fft_rbw_hz = 1. / fft_period_s
#     all_tx_bin_centers_hz = np.fft.fftfreq(2 * N_TX_FFT, 1. / r.adc_clk_hz)
#     all_rx_bin_centers_hz = np.fft.fftfreq(N_RX_FFT, 1. / r.adc_clk_hz)

#     # Get Nyquist zones and mixer settings
#     duc_settings = r.rfdc.core.get_mixer_settings(dac_tile, dac_block, r.rfdc.core.DAC_TILE)
#     ddc_settings = r.rfdc.core.get_mixer_settings(adc_tile, adc_block, r.rfdc.core.ADC_TILE)
#     dac_nyquist_zone = r.rfdc.core.get_nyquist_zone(dac_tile, dac_block, r.rfdc.core.DAC_TILE)
#     adc_nyquist_zone = r.rfdc.core.get_nyquist_zone(adc_tile, adc_block, r.rfdc.core.ADC_TILE)

#     # Convert to DAC/ADC frequencies
#     if udc_connected:
#         dac_out_freqs = (tone_frequencies - udc_lo_frequency) / udc_sideband
#         adc_in_freqs = (tone_frequencies - udc_lo_frequency) / udc_sideband
#     else:
#         dac_out_freqs = tone_frequencies.copy()
#         adc_in_freqs = tone_frequencies.copy()

#     # Apply Nyquist zone correction
#     if dac_nyquist_zone == 1:
#         duc_freqs = dac_out_freqs
#     elif dac_nyquist_zone == 2:
#         duc_freqs = 2 * r.adc_clk_hz - dac_out_freqs
#     else:
#         raise ValueError(f'Invalid DAC nyquist zone ({dac_nyquist_zone})')

#     if adc_nyquist_zone == 1:
#         ddc_freqs = adc_in_freqs
#     elif adc_nyquist_zone == 2:
#         ddc_freqs = 2 * r.adc_clk_hz - adc_in_freqs
#     else:
#         raise ValueError(f'Invalid ADC nyquist zone ({adc_nyquist_zone})')

#     # Get digital baseband frequencies
#     dbb_freqs_tx = duc_freqs - 1e6 * duc_settings['Freq']
#     dbb_freqs_rx = ddc_freqs + 1e6 * ddc_settings['Freq']

#     # Find nearest FFT bins
#     diff_tx = dbb_freqs_tx[:, np.newaxis] - all_tx_bin_centers_hz
#     diff_rx = dbb_freqs_rx[:, np.newaxis] - all_rx_bin_centers_hz
#     tx_nearest_bins = np.argmin(diff_tx ** 2, axis=-1)
#     rx_nearest_bins = np.argmin(diff_rx ** 2, axis=-1)

#     # Get frequency offsets
#     tx_freq_offsets_hz = diff_tx[np.arange(num_tones), tx_nearest_bins]
#     rx_freq_offsets_hz = diff_rx[np.arange(num_tones), rx_nearest_bins]

#     # VACC-aware LO assignment with gap-filling
#     # Group tones by their target FFT bin
#     bin_to_tones = {}
#     for tone_idx, bin_idx in enumerate(tx_nearest_bins):
#         bin_to_tones.setdefault(bin_idx, []).append(tone_idx)

#     # Assign LOs with VACC constraint
#     inmap_psb = np.full(n_chans_in, -1, dtype=int)  # inmap[lo] = fft_bin
#     lo_assignments = np.full(num_tones, -1, dtype=int)  # lo_assignments[tone_idx] = lo_idx
#     used_los = set()

#     def find_available_lo(preferred, used, n_los, min_sep=2):
#         """Find an available LO index, respecting VACC constraint."""
#         if preferred not in used:
#             # Check VACC constraint
#             conflict = any(abs(preferred - u) < min_sep and u != preferred for u in used)
#             if not conflict:
#                 return preferred
#         # Search outward from preferred
#         for offset in range(1, n_los):
#             for candidate in [preferred + offset, preferred - offset]:
#                 if 0 <= candidate < n_los and candidate not in used:
#                     conflict = any(abs(candidate - u) < min_sep for u in used)
#                     if not conflict:
#                         return candidate
#         return None

#     for bin_idx in sorted(bin_to_tones.keys()):
#         tone_indices = bin_to_tones[bin_idx]
#         for i, tone_idx in enumerate(tone_indices):
#             if i == 0:
#                 # First tone for this bin: try to use LO = bin_idx % n_chans_in
#                 preferred_lo = bin_idx % n_chans_in
#             else:
#                 # Subsequent tones: find next available LO with VACC separation
#                 prev_lo = lo_assignments[tone_indices[i-1]]
#                 preferred_lo = prev_lo + 2  # Start searching from prev + min_separation

#             lo = find_available_lo(preferred_lo, used_los, n_chans_in, min_sep=2)
#             if lo is None:
#                 raise ValueError(f"Could not find available LO for tone {tone_idx} (bin {bin_idx})")

#             lo_assignments[tone_idx] = lo
#             used_los.add(lo)
#             inmap_psb[lo] = bin_idx

#     # Build PFB chanmap (outmap semantics: chanmap[lo] = rx_bin)
#     chanmap_pfb = np.full(r.chanselect.n_chans_out, -1, dtype=int)
#     for tone_idx, lo in enumerate(lo_assignments):
#         if lo >= 0 and lo < r.chanselect.n_chans_out:
#             chanmap_pfb[lo] = rx_nearest_bins[tone_idx]

#     # Prepare phase increments and RI steps
#     phase_incs_tx = tx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
#     phase_incs_rx = rx_freq_offsets_hz / fft_rbw_hz * 2 * np.pi
#     ri_steps_tx = np.cos(phase_incs_tx) + 1j * np.sin(phase_incs_tx)
#     ri_steps_rx = np.cos(phase_incs_rx) + 1j * np.sin(phase_incs_rx)

#     # Build full mixer arrays (indexed by LO)
#     full_phase_incs_tx = np.zeros(n_chans_in)
#     full_phase_incs_rx = np.zeros(n_chans_in)
#     full_ri_steps_tx = np.ones(n_chans_in, dtype=complex)
#     full_ri_steps_rx = np.ones(n_chans_in, dtype=complex)

#     for tone_idx, lo in enumerate(lo_assignments):
#         full_phase_incs_tx[lo] = phase_incs_tx[tone_idx]
#         full_phase_incs_rx[lo] = phase_incs_rx[tone_idx]
#         full_ri_steps_tx[lo] = ri_steps_tx[tone_idx]
#         full_ri_steps_rx[lo] = ri_steps_rx[tone_idx]

#     # Format for firmware
#     phase_incs_tx_formatted = _format_phase_steps(full_phase_incs_tx, r.mixer._phase_bp)
#     phase_incs_rx_formatted = _format_phase_steps(full_phase_incs_rx, r.mixer._phase_bp)
#     ri_steps_tx_formatted = cplx2uint(full_ri_steps_tx, r.mixer._n_ri_step_bits)
#     ri_steps_rx_formatted = cplx2uint(full_ri_steps_rx, r.mixer._n_ri_step_bits)

#     return {
#         'inmap_psb': inmap_psb,
#         'chanmap_pfb': chanmap_pfb,
#         'lo_assignments': lo_assignments,
#         'phase_incs_tx': phase_incs_tx_formatted,
#         'phase_incs_rx': phase_incs_rx_formatted,
#         'ri_steps_tx': ri_steps_tx_formatted,
#         'ri_steps_rx': ri_steps_rx_formatted,
#         'tx_nearest_bins': tx_nearest_bins,
#         'rx_nearest_bins': rx_nearest_bins,
#         'tx_freq_offsets_hz': tx_freq_offsets_hz,
#         'rx_freq_offsets_hz': rx_freq_offsets_hz,
#     }


# def apply_tone_frequency_settings_vacc(r, tone_settings_dict, autosync=True):
#     """
#     Apply VACC-aware tone frequency settings using inmap semantics.

#     Supports both standard and fast firmware interfaces.

#     :param r: Firmware interface (standard or fast)
#     :param tone_settings_dict: Dictionary from prepare_tone_frequency_settings_vacc()
#     :param autosync: If True, trigger a sync after applying settings
#     """
#     # Get next buffer index
#     buf = get_next_buffer_idx(r)

#     # Prepare and write control buffer data
#     lo_control_values = {
#         'tx': {
#             'phase_incs': tone_settings_dict['phase_incs_tx'],
#             'ri_steps': tone_settings_dict['ri_steps_tx'],
#         },
#         'rx': {
#             'phase_incs': tone_settings_dict['phase_incs_rx'],
#             'ri_steps': tone_settings_dict['ri_steps_rx'],
#         }
#     }

#     try:
#         # Fast interface
#         formatted = prepare_control_buffer_data_fast(r, buf, lo_control_values)
#         indices = np.arange(r.mixer.n_chans)
#         write_control_buffer_data_fast(r, buf, formatted, indices)
#     except AttributeError:
#         # Standard interface
#         formatted = prepare_control_buffer_data(r, buf, lo_control_values)
#         write_control_buffer_data(r, buf, formatted)

#     # Set PSB chanselect using inmap
#     psb_chanselect_set_channel_inmap(r, tone_settings_dict['inmap_psb'])

#     # Set PFB chanselect using outmap
#     chanselect_set_channel_outmap(r, tone_settings_dict['chanmap_pfb'])

#     # Switch to new buffer and sync
#     try:
#         set_control_buffer_idx_fast(r, buf)
#     except AttributeError:
#         set_control_buffer_idx(r, buf)

#     if autosync:
#         try:
#             force_sync_fast(r)
#         except:
#             r.sync.arm_sync()
#             r.sync.sw_sync()


















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





def check_input_saturation(r,r_fast,iterations=25,saturation_bits=adc_saturation_bits,threshold=0.45,check_rts=True,verbose=True):
    """
    Check to see if the input ADC is saturating.

    Checks ADC snapshot levels and (if available) the RFDC RTS hardware
    flags.  RTS over_range is treated as saturation (warning), while RTS
    over_voltage indicates the signal far exceeded the input range (error).

    Full scale is +/-0.5 so threshold is 0.45.

    Parameters
    ----------
    r_fast : fast readout interface
    iterations : int
        Number of snapshot captures to check.
    saturation_bits : int
        Number of bits used for full-scale normalisation.
    threshold : float
        Fraction of full-scale to consider saturated.
    check_rts : bool
        If True and r is provided, also check the RFDC RTS sticky flags.
    verbose : bool
        If True (default), print status. If False, return silently.
    r : readout interface, optional
        Katcp readout interface, needed for RFDC RTS checks.
        If None, RTS checks are skipped.
    """
    if r is None:
        check_rts = False

    # Clear stale RTS sticky flags *before* capturing snapshots so that
    # any flag that re-asserts during the snapshot window reflects a
    # current condition rather than a past transient.  The snapshot
    # iterations themselves provide a natural observation window (much
    # longer than a fixed sleep) for intermittent spikes to trigger the
    # hardware flags.
    rts_available = False
    if check_rts:
        _, rts_stale = check_rfdc_rts_events(r, clear=True)
        rts_available = rts_stale.get('rts_available', False)

    ss_0 = get_adc_snapshot_fast(r_fast) / 2**(saturation_bits-1)
    ss=np.zeros((iterations,ss_0.size),dtype=ss_0.dtype)
    ss[0]=ss_0
    for i in range(1,iterations):
        ss[i]=get_adc_snapshot_fast(r_fast) / 2**(saturation_bits-1)
    imax = np.max(ss.real)
    imin = np.min(ss.real)
    qmax = np.max(ss.imag)
    qmin = np.min(ss.imag)
    i_over = imax >= 1.0*threshold
    i_under = imin <= -1.0*threshold
    q_over = qmax >= 1.0*threshold
    q_under = qmin <= -1.0*threshold
    any_saturation = bool(i_over|i_under|q_over|q_under)
    integration_time = ss.size/r_fast.adc_clk_hz
    details = {'imax_fs':imax,'imin_fs':imin,'qmax_fs':qmax,'qmin_fs':qmin,
               'integration_time':integration_time,
               'threshold':threshold}

    # Read RTS flags *after* the snapshot window — any flag that latched
    # during the captures indicates the condition is still active.
    if check_rts and rts_available:
        rts_event, rts_details = check_rfdc_rts_events(r)
        details.update(rts_details)
        if rts_details.get('rts_over_voltage', False):
            if verbose:
                print('ERROR: ADC RTS over-voltage flag set — signal far exceeded input range')
            any_saturation = True
        elif rts_details.get('rts_over_range', False):
            if verbose:
                print('WARNING: ADC RTS over-range flag set — signal exceeded full-scale input')
            any_saturation = True

    if verbose:
        status = 'SATURATING' if any_saturation else 'OK'
        print(f'ADC input saturation check: {status}')
        print(f'  I range: [{imin:.3f}, {imax:.3f}] FS  |  Q range: [{qmin:.3f}, {qmax:.3f}] FS  (threshold: {threshold:.0%})')
        if check_rts and rts_available:
            print(f'  RTS Over-Range: {rts_details.get("rts_over_range", False)}, RTS Over-Voltage: {rts_details.get("rts_over_voltage", False)}')

    return any_saturation, details

def check_output_saturation(r_fast,iterations=25,saturation_bits=dac_saturation_bits,threshold=0.90,verbose=True):
    """
    Check to see if the output DACs are saturating.

    TODO: extend this to check for amplifier saturation
    """
    scale = 2**(saturation_bits-1)
    ss0_0,ss1_0 = get_dac_snapshot_fast(r_fast)
    ss0_0 /= scale
    ss1_0 /= scale
    ss0=np.zeros((iterations,ss0_0.size),dtype=ss0_0.dtype)
    ss1=np.zeros((iterations,ss1_0.size),dtype=ss1_0.dtype)
    ss0[0]=ss0_0
    ss1[0]=ss1_0
    for i in range(1,iterations):
        ss0[i],ss1[i] = get_dac_snapshot_fast(r_fast)
        ss0[i] /= scale
        ss1[i] /= scale
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
    integration_time = ss0.size/r_fast.adc_clk_hz
    details = {'i0max_fs':i0max,'i0min_fs':i0min,'q0max_fs':q0max,'q0min_fs':q0min,
               'i1max_fs':i1max,'i1min_fs':i1min,'q1max_fs':q1max,'q1min_fs':q1min,
               'integration_time':integration_time,
               'threshold':threshold}

    if verbose:
        status = 'SATURATING' if any_saturation else 'OK'
        dac0_status = 'SATURATING' if any0_saturation else 'OK'
        dac1_status = 'SATURATING' if any1_saturation else 'OK'
        print(f'DAC output saturation check: {status}')
        print(f'  DAC0 ({dac0_status}): I range: [{i0min:.3f}, {i0max:.3f}] FS  |  Q range: [{q0min:.3f}, {q0max:.3f}] FS')
        print(f'  DAC1 ({dac1_status}): I range: [{i1min:.3f}, {i1max:.3f}] FS  |  Q range: [{q1min:.3f}, {q1max:.3f}] FS')
        print(f'  Threshold: {threshold:.0%}')

    return any_saturation, details


def check_rfdc_rts_events(r, clear=True):
    """
    Check the RFDC Real-Time Status (RTS) sticky event flags.

    These hardware-level flags latch when an overvoltage or overrange event
    occurs at the ADC and persist until explicitly cleared.  This is
    separate from the DSP overflow counters (PSB/PFB) which track overflow
    in the FPGA signal processing chain.

    The RTS flags are per-pipeline and are accessed via the Rfdc block's
    ``get_rts_flags()`` / ``reset_rts_flags()`` methods (souk_mkid_readout
    commit 14ff5d2, Oct 2025).  Older firmware versions will return
    rts_available=False gracefully.

    RTS flag definitions (see PG269):
        rts_over_range       : signal exceeded full-scale ADC input (sticky)
        rts_over_threshold1  : signal above programmable threshold 1
        rts_over_threshold2  : signal above programmable threshold 2
        rts_over_voltage     : signal "far exceeded" input range (sticky)
        rts_over_cm_over_voltage  : common-mode voltage too high
        rts_over_cm_under_voltage : common-mode voltage too low

    Parameters
    ----------
    r : readout interface
    clear : bool
        If True, reset the sticky over_range and over_voltage flags
        after reading them.

    Returns
    -------
    any_event : bool
        True if any RTS event flag was set.
    details : dict
        Flag values and availability status.
    """
    details = {'rts_available': False}
    try:
        flags = r.rfdc.get_rts_flags().copy()
    except AttributeError:
        # get_rts_flags not available in this version of souk_mkid_readout
        return False, details

    details['rts_available'] = True
    details.update(flags)
    any_event = any(flags.values())

    if clear:
        try:
            r.rfdc.reset_rts_flags(over_range=True, over_voltage=True)
        except AttributeError:
            pass

    return any_event, details


def check_dsp_overflow(r, duration_s=0.1, verbose=True):
    """
    Check to see if any of the digital signal processing blocks have overflowed.

    Checks the PSB scale, PSB filterbank and PFB filterbank overflow counters.
    ADC-level checks (RFDC RTS flags) are handled by check_input_saturation().
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

    # Unsigned 32-bit wrap-safe subtraction — FPGA counters are unsigned and
    # can wrap from 2^32-1 to 0 between reads.
    _OVF_MOD = 2**32
    psbscale_delta = (psbscale_overflow1 - psbscale_overflow0) % _OVF_MOD
    psb_delta = (psb_overflow1 - psb_overflow0) % _OVF_MOD
    pfb_delta = (pfb_overflow1 - pfb_overflow0) % _OVF_MOD

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

    if verbose:
        status = 'OVERFLOW DETECTED' if any_overflow else 'OK'
        print(f'DSP overflow check ({duration_s:.1f}s window): {status}')
        print(f'  PSB scale overflow delta: {psbscale_delta}')
        print(f'  PSB filterbank overflow delta: {psb_delta}')
        print(f'  PFB filterbank overflow delta: {pfb_delta}')

    return any_overflow, details


def _resolve_cal_value(value, freq_axis):
    """Resolve a calibration parameter to per-tone values.

    Supports:
      None           -> 0
      scalar         -> returned as-is
      str (filename) -> loaded from USER_DIR, nearest-neighbour interpolated
      array-like     -> [[freq, dB], ...] nearest-neighbour interpolated
    """
    if value is None:
        return 0
    if isinstance(value, str):
        cal_f, cal_db = np.loadtxt(os.path.join(USER_DIR, value), ndmin=2).T
        return np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_axis])
    if not np.isscalar(value):
        cal_f, cal_db = np.array(value, ndmin=2).T
        return np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in freq_axis])
    return value


def _gather_tx_chain_params(r, config_dict, rf_peripherals=None):
    """Read firmware state and resolve all TX chain calibration parameters.

    Returns a dict with everything needed by calibration.calc_tone_powers /
    calc_tone_amplitudes, plus metadata (freqs, freq_details, connectivity).

    When rf_peripherals is provided, live attenuator/amp values are read
    from hardware, overriding any None values in config_dict.
    """
    dac_tile = int(config_dict['firmware']['dac0_tile'])
    dac_block = int(config_dict['firmware']['dac0_block'])

    freqs, freq_details = get_tone_frequencies(r, config_dict, detailed_output=True)
    analog_freq = freq_details['tx']['analog_output_freq']
    rf_freq = freq_details['tx']['rf_output_freq']

    # Live firmware state
    amps = get_tone_amplitudes(r, config_dict)
    psb_fftshift = r.psb.get_fftshift()
    psb_scale = r.psbscale.get_scale()
    mixer_settings = r.rfdc.core.get_mixer_settings(dac_tile, dac_block, r.rfdc.core.DAC_TILE)
    mixer_scale_is_1p0 = mixer_settings['FineMixerScale'] == r.rfdc.core.MIX_SCALE_1P0
    qmc_settings = r.rfdc.core.get_qmc_settings(dac_tile, dac_block, r.rfdc.core.DAC_TILE)
    mixer_qmc_gain = qmc_settings['GainCorrectionFactor'] if qmc_settings['EnableGain'] else 1.0
    vop_current = int(r.rfdc.core.get_output_current(dac_tile, dac_block)['current'])

    # Fixed config params
    dac_fs_bits = config_dict['firmware']['dac_fullscale_bits']
    vop_current_fs = config_dict['firmware']['vop_current_fullscale']

    has_rf = rf_peripherals is not None and rf_peripherals.enabled

    # Resolve calibration values (scalar / file / array → per-tone)
    dac_dbfs_to_dbm = _resolve_cal_value(config_dict['firmware']['dac0_dbfs_to_dbm'], analog_freq)
    tx_combiner_loss_db = _resolve_cal_value(config_dict['rf_frontend']['tx_combiner_loss_db'], analog_freq)

    # TX attenuator: read from hardware if available, else config, else 0
    tx_attenuator_value_db = config_dict['rf_frontend']['tx_attenuator_value_db']
    if tx_attenuator_value_db is None and has_rf:
        tx_attenuator_value_db = rf_peripherals.get_tx_attenuation()
    if tx_attenuator_value_db is None:
        tx_attenuator_value_db = 0

    tx_if_s21_db = _resolve_cal_value(config_dict['rf_frontend']['tx_if_s21_db'], analog_freq)
    tx_mixer_conversion_loss_db = _resolve_cal_value(config_dict['rf_frontend']['tx_mixer_conversion_loss_db'], analog_freq)
    tx_rf_s21_db = _resolve_cal_value(config_dict['rf_frontend']['tx_rf_s21_db'], rf_freq)

    # TX amp S21: use bypass or enabled value from hardware if config is default
    tx_bypass_amp_s21_db = _resolve_cal_value(config_dict['rf_frontend'].get('tx_bypass_amp_s21_db', 0), rf_freq)

    cryostat_input_s21_db = _resolve_cal_value(config_dict['cryostat']['input_s21_db'], rf_freq)

    rf_connected = config_dict['rf_frontend']['connected']
    cryo_connected = config_dict['cryostat']['connected']

    if not rf_connected:
        tx_combiner_loss_db = 0
        tx_attenuator_value_db = 0
        tx_if_s21_db = 0
        tx_mixer_conversion_loss_db = 0
        tx_rf_s21_db = 0
        tx_bypass_amp_s21_db = 0
    if not cryo_connected:
        cryostat_input_s21_db = 0

    return {
        'freqs': freqs,
        'freq_details': freq_details,
        'amps': amps,
        'psb_fftshift': psb_fftshift,
        'psb_scale': psb_scale,
        'mixer_scale_is_1p0': mixer_scale_is_1p0,
        'mixer_qmc_gain': mixer_qmc_gain,
        'vop_current': vop_current,
        'vop_current_fs': vop_current_fs,
        'dac_fs_bits': dac_fs_bits,
        'dac_dbfs_to_dbm': dac_dbfs_to_dbm,
        'tx_combiner_loss_db': tx_combiner_loss_db,
        'tx_attenuator_value_db': tx_attenuator_value_db,
        'tx_if_s21_db': tx_if_s21_db,
        'tx_mixer_conversion_loss_db': tx_mixer_conversion_loss_db,
        'tx_rf_s21_db': tx_rf_s21_db,
        'tx_bypass_amp_s21_db': tx_bypass_amp_s21_db,
        'cryostat_input_s21_db': cryostat_input_s21_db,
        'rf_connected': rf_connected,
        'cryo_connected': cryo_connected,
    }


def _mask_cal_for_reference_plane(cal_params, reference_plane):
    """Zero out calibration stages beyond the reference plane.

    Returns a new dict with the same keys, masking stages that are
    downstream of the chosen reference plane so that
    calc_tone_amplitudes computes amplitudes for power at that plane.
    """
    masked = dict(cal_params)
    if reference_plane == 'dac':
        for key in ('tx_combiner_loss_db', 'tx_attenuator_value_db',
                     'tx_if_s21_db', 'tx_mixer_conversion_loss_db',
                     'tx_rf_s21_db', 'tx_bypass_amp_s21_db',
                     'cryostat_input_s21_db'):
            masked[key] = 0
    elif reference_plane == 'rf_output':
        masked['cryostat_input_s21_db'] = 0
    return masked


def _find_best_psb_fftshift(r, overflow_check_duration=0.1):
    """Find the best PSB FFT shift without overflow, testing on live hardware.

    Mutes PSB output (psb_scale → 0) before searching to avoid sending
    transient spikes to the DAC.  Tone amplitudes should already be set
    to their target values before calling, since PSB overflow depends on
    amplitudes + fftshift (psb_scale is downstream).

    After finding the best shift, if psb_scale was non-zero on entry,
    this function restores it with compensation for the fftshift gain
    change so that total DAC power is preserved.  If psb_scale was
    already muted (0) on entry, it remains muted for the caller to set.

    Iterates from most-attenuating (highest popcount) to highest-gain
    (lowest popcount) fftshift, stopping at the first overflow.
    Steps back one extra level as a safety margin against intermittent
    overflows.

    Returns (best_fftshift, best_fftshift_idx, fftshifts_array).
    """
    psb_fftshifts = (2**np.arange(14) - 1).astype(int)[::-1]  # 8191, 4095, ..., 1, 0
    best_fftshift = int(psb_fftshifts[0])  # start with safest (lowest gain)

    # Save entry state so we can restore/compensate afterwards
    entry_psb_scale = r.psbscale.get_scale()
    entry_fftshift = r.psb.get_fftshift()
    was_muted = (entry_psb_scale == 0)

    # Mute output during search — psb_scale is downstream of the PSB
    # filterbank so it doesn't affect overflow detection.
    r.psbscale.set_scale(0)
    time.sleep(0.01)

    for shift in psb_fftshifts:
        r.psb.set_fftshift(shift)
        time.sleep(0.01)
        _, ovf_details = check_dsp_overflow(r, overflow_check_duration, verbose=False)
        psb_ovf = ovf_details['psb_ovf_delta']
        popcount = bin(shift).count('1')
        status = f'OVERFLOW ({psb_ovf})' if psb_ovf else 'ok'
        print(f'    fftshift {format(shift, "#016b")} (popcount {popcount:2d}): {status}')
        if psb_ovf:
            break  # overflow at this shift — use the previous safe value
        best_fftshift = int(shift)

    # Step back one extra level as safety margin against intermittent overflows
    best_idx = list(psb_fftshifts).index(best_fftshift)
    if best_idx > 0:
        best_fftshift = int(psb_fftshifts[best_idx - 1])
        print(f'    safety margin: stepped back to {format(best_fftshift, "#016b")}')

    best_fftshift_idx = list(psb_fftshifts).index(best_fftshift)

    # Set the best fftshift
    r.psb.set_fftshift(best_fftshift)
    time.sleep(0.01)

    # Restore psb_scale with compensation for the fftshift gain change
    if not was_muted:
        entry_popcount = bin(entry_fftshift).count('1')
        best_popcount = bin(best_fftshift).count('1')
        fftshift_gain_ratio = 2.0 ** (entry_popcount - best_popcount)
        compensated_scale = entry_psb_scale / fftshift_gain_ratio
        compensated_scale = float(np.clip(compensated_scale, 1/256, 255))
        r.psbscale.set_scale(compensated_scale)
        time.sleep(0.01)
        print(f'    restored psb_scale={compensated_scale:.6f} '
              f'(compensated x{fftshift_gain_ratio:.2f})')

    best_popcount = bin(best_fftshift).count('1')
    print(f'  best PSB fftshift: {format(best_fftshift, "#016b")} '
          f'(popcount {best_popcount}, {"muted" if was_muted else "restored"})')

    return best_fftshift, best_fftshift_idx, psb_fftshifts


def _find_best_pfb_fftshift(r, overflow_check_duration=0.1):
    """Find the best PFB FFT shift without overflow, testing on live hardware.

    The PFB is an analysis filterbank (inverse of PSB synthesis).  More bits
    set = more divide-by-2 stages = more attenuation.  shift=0 is maximum
    gain, shift=8191 is maximum attenuation.

    Iterates from most-attenuating (highest popcount) to highest-gain
    (lowest popcount) fftshift, stopping at the first overflow.
    Steps back two levels from the overflow point as a safety margin
    against intermittent overflows.

    Returns (best_fftshift, pfb_fftshifts_array).
    """
    pfb_fftshifts = (2**np.arange(14) - 1).astype(int)[::-1]  # 8191, 4095, ..., 1, 0
    best_fftshift = int(pfb_fftshifts[0])  # start with safest (most attenuation)

    for shift in pfb_fftshifts:
        r.pfb.set_fftshift(shift)
        time.sleep(0.01)
        _, ovf_details = check_dsp_overflow(r, overflow_check_duration, verbose=False)
        pfb_ovf = ovf_details['pfb_ovf_delta']
        popcount = bin(shift).count('1')
        status = f'OVERFLOW ({pfb_ovf})' if pfb_ovf else 'ok'
        print(f'    fftshift {format(shift, "#016b")} (popcount {popcount:2d}): {status}')
        if pfb_ovf:
            break  # overflow at this shift — use the previous safe value
        best_fftshift = int(shift)

    # Step back one extra level as safety margin against intermittent overflows
    best_idx = list(pfb_fftshifts).index(best_fftshift)
    if best_idx > 0:
        best_fftshift = int(pfb_fftshifts[best_idx - 1])
        print(f'    safety margin: stepped back to {format(best_fftshift, "#016b")}')

    r.pfb.set_fftshift(best_fftshift)
    time.sleep(0.01)
    best_popcount = bin(best_fftshift).count('1')
    print(f'  best PFB fftshift: {format(best_fftshift, "#016b")} (popcount {best_popcount})')

    return best_fftshift, pfb_fftshifts


def _apply_per_bin_scaling(r, config_dict, amps):
    """Scale all amplitudes to account for worst-case coherent addition in shared FFT bins.

    Each tone has a unique LO index, but multiple tones can map to the same
    FFT bin.  When that happens, the vector accumulator that feeds the bin
    sums their amplitudes coherently.  If the sum exceeds 1.0 the VACC
    overflows.  To avoid this while preserving relative powers across ALL
    tones, we scale everything down by the worst-case bin overlap factor.
    """
    _, freq_details = get_tone_frequencies(r, config_dict, detailed_output=True)
    bin_indices = np.array(freq_details['tx']['filterbank_bins'])
    _, counts = np.unique(bin_indices, return_counts=True)
    max_tones_per_bin = int(np.max(counts))
    if max_tones_per_bin > 1:
        amps = amps / max_tones_per_bin
        print(f'  WARNING: up to {max_tones_per_bin} tones share an FFT bin, '
              f'scaling all amplitudes by 1/{max_tones_per_bin}')
    return amps


# ---- RX policy helper ----
# Used by maximise_tx_power and set_tone_powers to manage the RX path
# when TX power changes risk saturating the ADC.

RX_POLICIES = ('protect', 'compensate', 'raise', 'none')


def _apply_rx_policy(r, r_fast, config_dict, rf_peripherals, rx_policy,
                     tx_power_change_db=None):
    """Check and optionally protect the RX path after a TX power change.

    Called after each incremental TX power change (attenuator step,
    psb_scale change, amp enable, etc.) to ensure the ADC is not
    saturating.  The action taken depends on the rx_policy:

    Parameters
    ----------
    r : readout interface
    config_dict : dict
        Live config dict (used for ADC tile/block lookup).
    rf_peripherals : RFPeripherals or None
        RF peripheral controller.  May be None — the ADC DSA is always
        available regardless.
    rx_policy : str
        One of:

        ``'protect'`` (default)
            If ADC saturation is detected, increase RX attenuation (if
            available and not already at max), otherwise increase the
            ADC DSA.  Logs the intervention but does not attempt to
            preserve the RX signal level.  Goal: prevent ADC damage /
            clipping artefacts.

        ``'compensate'``
            Track the cumulative TX power change (``tx_power_change_db``)
            and mirror it on the RX path — prefer RX attenuator, fall
            back to ADC DSA.  This keeps the round-trip power at the
            ADC approximately constant.  If the RX attenuator and DSA
            ranges are exhausted, falls back to ``'protect'`` behaviour
            (i.e. best-effort compensation, then reactive protection).

        ``'raise'``
            Check for ADC saturation after the TX change.  If detected,
            raise ``RuntimeError`` immediately.  The caller is
            responsible for reverting any TX changes.  Useful for
            scripted workflows where the caller manages the RX path
            independently.

        ``'none'``
            Do nothing.  The caller takes full responsibility for the
            RX path.  No saturation check is performed.

    tx_power_change_db : float or None
        The estimated TX power change in dB from this step (positive
        means TX power increased).  Required for ``'compensate'`` mode;
        ignored by other modes.  If None in ``'compensate'`` mode, falls
        back to ``'protect'`` behaviour for this call.

    Returns
    -------
    dict or None
        None if no intervention was needed (or policy is ``'none'``).
        Otherwise a dict describing what changed::

            {
                'policy': str,           # the policy that was applied
                'saturated': bool,       # True if saturation was detected
                'action': str,           # human-readable description
                'rx_atten_change_db': float or None,
                'dsa_change_db': float or None,
            }

    Raises
    ------
    RuntimeError
        If ``rx_policy='raise'`` and ADC saturation is detected.
    ValueError
        If ``rx_policy`` is not one of the valid policy strings.
    """
    if rx_policy not in RX_POLICIES:
        raise ValueError(
            f"rx_policy must be one of {RX_POLICIES}, got {rx_policy!r}")

    if rx_policy == 'none':
        return None

    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    DSA_MAX = 27
    has_rf = rf_peripherals is not None and rf_peripherals.enabled

    # --- Compensate mode: proactively mirror TX change onto RX path ---
    if rx_policy == 'compensate' and tx_power_change_db is not None:
        # Positive tx_power_change_db means TX got louder → increase RX
        # attenuation by the same amount to keep round-trip constant.
        delta = tx_power_change_db
        rx_atten_change = 0.0
        dsa_change = 0.0

        if delta > 0.1:
            # TX power increased — add attenuation on RX side
            if has_rf:
                atten_step = rf_peripherals.ATTEN_STEP
                atten_max = rf_peripherals.ATTEN_MAX
                current_atten = rf_peripherals.get_rx_attenuation()
                add_atten = min(delta, atten_max - current_atten)
                if add_atten >= atten_step:
                    new_atten = round((current_atten + add_atten) / atten_step) * atten_step
                    new_atten = float(np.clip(new_atten, current_atten, atten_max))
                    rf_peripherals.set_rx_attenuation(new_atten)
                    time.sleep(0.1)
                    rx_atten_change = new_atten - current_atten
                    delta -= rx_atten_change

            if delta > 0.5:
                # Remaining delta absorbed by DSA
                current_dsa = int(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])
                add_dsa = min(int(round(delta)), DSA_MAX - current_dsa)
                if add_dsa >= 1:
                    new_dsa = current_dsa + add_dsa
                    r.rfdc.core.set_dsa(adc_tile, adc_block, int(new_dsa))
                    time.sleep(0.1)
                    dsa_change = float(new_dsa - current_dsa)
                    delta -= dsa_change

        elif delta < -0.1:
            # TX power decreased — reduce RX attenuation to recover signal
            recover = -delta
            if has_rf:
                atten_step = rf_peripherals.ATTEN_STEP
                atten_min = rf_peripherals.ATTEN_MIN
                current_atten = rf_peripherals.get_rx_attenuation()
                remove_atten = min(recover, current_atten - atten_min)
                if remove_atten >= atten_step:
                    new_atten = round((current_atten - remove_atten) / atten_step) * atten_step
                    new_atten = float(np.clip(new_atten, atten_min, current_atten))
                    rf_peripherals.set_rx_attenuation(new_atten)
                    time.sleep(0.1)
                    rx_atten_change = new_atten - current_atten  # negative
                    recover += rx_atten_change  # rx_atten_change is negative

            if recover > 0.5:
                current_dsa = int(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])
                remove_dsa = min(int(round(recover)), current_dsa)
                if remove_dsa >= 1:
                    new_dsa = current_dsa - remove_dsa
                    r.rfdc.core.set_dsa(adc_tile, adc_block, int(new_dsa))
                    time.sleep(0.1)
                    dsa_change = float(new_dsa - current_dsa)  # negative

        if rx_atten_change != 0.0 or dsa_change != 0.0:
            parts = []
            if rx_atten_change != 0.0:
                parts.append(f'RX atten {rx_atten_change:+.1f} dB')
            if dsa_change != 0.0:
                parts.append(f'DSA {dsa_change:+.0f} dB')
            action = f'compensate: {", ".join(parts)} (TX changed {tx_power_change_db:+.1f} dB)'
            print(f'    rx_policy: {action}')
            return {
                'policy': 'compensate',
                'saturated': False,
                'action': action,
                'rx_atten_change_db': rx_atten_change if rx_atten_change != 0.0 else None,
                'dsa_change_db': dsa_change if dsa_change != 0.0 else None,
            }

        # Even in compensate mode, fall through to saturation check
        # in case the compensation was insufficient or rounding left
        # a residual.

    # --- Saturation check (protect, raise, or compensate fallback) ---
    saturated, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
    if not saturated:
        return None

    if rx_policy == 'raise':
        raise RuntimeError(
            'ADC saturation detected after TX power change. '
            'Revert TX settings or switch to rx_policy="protect".')

    # protect (or compensate fallback): increase RX attenuation / DSA
    init_rx_atten = rf_peripherals.get_rx_attenuation() if has_rf else None
    init_dsa = int(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])

    if has_rf:
        atten_step = rf_peripherals.ATTEN_STEP
        atten_max = rf_peripherals.ATTEN_MAX
        current_atten = init_rx_atten
        while saturated and current_atten < atten_max:
            current_atten = min(
                round((current_atten + 3.0) / atten_step) * atten_step,
                atten_max)
            rf_peripherals.set_rx_attenuation(current_atten)
            time.sleep(0.1)
            saturated, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)

    if saturated:
        current_dsa = init_dsa
        while saturated and current_dsa < DSA_MAX:
            current_dsa = min(current_dsa + 2, DSA_MAX)
            r.rfdc.core.set_dsa(adc_tile, adc_block, int(current_dsa))
            time.sleep(0.1)
            saturated, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)

    final_rx_atten = rf_peripherals.get_rx_attenuation() if has_rf else None
    final_dsa = int(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])
    rx_atten_change = (final_rx_atten - init_rx_atten) if has_rf else 0.0
    dsa_change = float(final_dsa - init_dsa)

    parts = []
    if rx_atten_change != 0.0:
        parts.append(f'RX atten -> {final_rx_atten:.1f} dB')
    if dsa_change != 0.0:
        parts.append(f'DSA -> {final_dsa} dB')

    if saturated:
        action = f'WARNING: ADC still saturating after max RX attenuation'
        print(f'    rx_policy: {action}')
    else:
        action = f'protect: {", ".join(parts)}' if parts else 'protect: no action needed'
        print(f'    rx_policy: {action}')

    return {
        'policy': rx_policy,
        'saturated': saturated,
        'action': action,
        'rx_atten_change_db': rx_atten_change if rx_atten_change != 0.0 else None,
        'dsa_change_db': dsa_change if dsa_change != 0.0 else None,
    }


def maximise_tx_power(r, r_fast=None, config_dict=None, headroom_db=1.0,
                      reference_plane='dac', rf_peripherals=None,
                      power_limit_dbm=None, rx_policy='protect'):
    """Maximise the TX output power at the chosen reference plane.

    Scales tone amplitudes to near-max, finds the highest PSB FFT shift
    without PSB overflow, then ramps psb_scale to just below DAC
    saturation using a multi-resolution approach (6 dB → 3 dB → 1 dB →
    0.5 dB → 0.1 dB steps).

    For reference planes beyond the DAC ('rf_output' or 'detector'),
    also minimises the TX attenuator and enables the TX amplifier
    (if rf_peripherals is provided).

    Parameters
    ----------
    headroom_db : float
        Safety margin in dB below the saturation point (default 1.0).
    reference_plane : str
        'dac' (default), 'rf_output', or 'detector'.
    rf_peripherals : RFPeripherals or None
        RF peripheral controller.  Required for reference planes beyond
        'dac'.  If None for 'rf_output'/'detector', the digital chain
        is still maximised but a warning is printed.
    power_limit_dbm : float or None
        Maximum allowed tone power (strongest tone) in dBm at the
        reference plane.  If the achieved power exceeds this limit,
        TX attenuation is increased (or psb_scale reduced) to bring it
        down.  Requires config_dict for power computation.  If None
        (default), no limit is applied.
    rx_policy : str
        How to manage the RX path when TX power changes risk saturating
        the ADC.  Checked after each incremental TX power step (psb_scale
        ramp, attenuator ramp, amp enable).  One of:

        - ``'protect'`` (default) — if ADC saturates, increase RX
          attenuation or DSA just enough to clear it and warn.
        - ``'compensate'`` — mirror each TX power change onto the RX
          path (prefer RX attenuator, fall back to DSA) to keep
          round-trip power constant.  Falls back to ``'protect'`` if
          range is exhausted.
        - ``'raise'`` — raise ``RuntimeError`` if ADC saturates.
        - ``'none'`` — do not check or touch the RX path.

        See :func:`_apply_rx_policy` for full details.
    """
    if rx_policy not in RX_POLICIES:
        raise ValueError(
            f"rx_policy must be one of {RX_POLICIES}, got {rx_policy!r}")
    print(f'maximise_tx_power: starting (reference_plane={reference_plane})')
    if reference_plane not in ('dac', 'rf_output', 'detector'):
        raise ValueError(f"reference_plane must be 'dac', 'rf_output', or 'detector', "
                         f"got '{reference_plane}'")

    has_rf = rf_peripherals is not None and rf_peripherals.enabled

    init_dac_saturation, _ = check_output_saturation(r_fast, iterations=250, verbose=False)
    if init_dac_saturation:
        print('  DAC saturating — fixing first')
        fix_dac_saturation(r, r_fast, config_dict)
    init_amps = get_tone_amplitudes(r, config_dict)
    init_psb_scale = r.psbscale.get_scale()
    init_psb_fftshift = r.psb.get_fftshift()
    print(f'  init: psb_scale={init_psb_scale:.4f}, '
          f'fftshift={format(init_psb_fftshift, "#016b")}, '
          f'max_amp={np.max(init_amps):.4f}')

    # --- Step 1: Mute, maximise amplitudes ---
    max_amp = 1 - 2**-12
    scalemin = 1 / 256
    scalemax = 255
    amps_max = np.max(init_amps)
    if amps_max == 0:
        raise ValueError('Tone powers are all zero')
    amps_gain = max_amp / amps_max
    r.psbscale.set_scale(0)
    time.sleep(0.01)
    amps = init_amps * amps_gain
    amps = _apply_per_bin_scaling(r, config_dict, amps)
    set_tone_amplitudes(r, config_dict, amps)
    time.sleep(0.01)
    print(f'  step 1: amplitudes maximised (x{amps_gain:.4f})')

    # --- Step 2: Find best PSB FFT shift (output muted) ---
    print(f'  step 2: PSB fftshift search')
    best_fftshift, _, _ = _find_best_psb_fftshift(r)

    # --- Step 3: Multi-resolution psb_scale ramp ---
    print(f'  step 3: psb_scale ramp')
    headroom_linear = 10**(-headroom_db / 20)

    def _is_ok(scale):
        """Check if a psb_scale value is safe (no overflow, no saturation)."""
        scale = float(np.clip(scale, scalemin, scalemax))
        r.psbscale.set_scale(scale)
        time.sleep(0.01)
        _, ovf_details = check_dsp_overflow(r, 0.1, verbose=False)
        ovf = ovf_details['psbscale_ovf_delta'] or ovf_details['psb_ovf_delta']
        sat = check_output_saturation(r_fast, iterations=250, verbose=False)[0]
        return not (ovf or sat)

    # Estimate starting psb_scale from initial conditions.
    init_popcount = bin(init_psb_fftshift).count('1')
    best_popcount = bin(best_fftshift).count('1')
    fftshift_gain_ratio = 2.0 ** (init_popcount - best_popcount)
    if init_psb_scale > 0:
        estimated_scale = init_psb_scale / (amps_gain * fftshift_gain_ratio)
        estimated_scale = float(np.clip(estimated_scale, scalemin, scalemax))
    else:
        estimated_scale = scalemin
    print(f'    estimate from init: {estimated_scale:.4f} '
          f'(amps x{amps_gain:.2f}, fftshift x{fftshift_gain_ratio:.2f})')

    # Multi-resolution ramp: 6dB, 3dB, 1dB, 0.5dB, 0.1dB steps.
    step_factors = [2.0, 2**0.5, 10**(1/20), 10**(0.5/20), 10**(0.1/20)]
    safe_scale = scalemin
    failed_scale = float('inf')
    for step_db, factor in zip([6, 3, 1, 0.5, 0.1], step_factors):
        scale = safe_scale
        if step_db == 6 and estimated_scale > scale:
            if estimated_scale < failed_scale and _is_ok(estimated_scale):
                safe_scale = estimated_scale
                scale = estimated_scale
                print(f'    {step_db:4.1f} dB: estimate {estimated_scale:.4f} ok')
            else:
                failed_scale = min(failed_scale, estimated_scale)
                print(f'    {step_db:4.1f} dB: estimate {estimated_scale:.4f} saturates')
        next_scale = min(scale * factor, scalemax)
        while next_scale > scale:
            if next_scale >= failed_scale:
                print(f'    {step_db:4.1f} dB: {next_scale:.4f} skip (already failed)')
                break
            if _is_ok(next_scale):
                safe_scale = next_scale
                print(f'    {step_db:4.1f} dB: {next_scale:.4f} ok')
                if next_scale >= scalemax:
                    break
                scale = next_scale
                next_scale = min(scale * factor, scalemax)
            else:
                failed_scale = next_scale
                print(f'    {step_db:4.1f} dB: {next_scale:.4f} LIMIT')
                break

    psb_scale = safe_scale * headroom_linear
    psb_scale = float(np.clip(psb_scale, scalemin, scalemax))
    r.psbscale.set_scale(psb_scale)
    time.sleep(0.01)
    print(f'  psb_scale: {psb_scale:.4f} ({headroom_db} dB headroom)')

    # Estimate total TX power change at the DAC from the combined effect
    # of amplitude scaling, fftshift change, and psb_scale change.
    # All three were applied while muted, so the RX path saw nothing
    # until this unmute.
    if init_psb_scale > 0:
        total_gain = amps_gain * fftshift_gain_ratio * (psb_scale / init_psb_scale)
        tx_change_db = float(20 * np.log10(total_gain))
    else:
        tx_change_db = None  # can't estimate from zero
    _apply_rx_policy(r, r_fast, config_dict, rf_peripherals, rx_policy,
                     tx_power_change_db=tx_change_db)

    # --- Step 4: Maximise analog chain (if reference plane beyond DAC) ---
    tx_atten_db = None
    tx_amp_bypass = None
    if reference_plane != 'dac':
        if has_rf:
            print(f'  step 4: maximise analog chain')
            # Enable TX amplifier
            current_bypass = rf_peripherals.get_tx_amp_bypass()
            if current_bypass:
                # Check model S21 in both states to see if amp has any effect
                s21_bypassed = rf_peripherals._get_amp_s21('transmit_atten')
                rf_peripherals.set_tx_amp_bypass(False)
                time.sleep(0.1)
                s21_enabled = rf_peripherals._get_amp_s21('transmit_atten')
                expected_gain_db = s21_enabled - s21_bypassed
                if abs(expected_gain_db) < 0.5:
                    # No gain difference — no bypass amp connected
                    rf_peripherals.set_tx_amp_bypass(True)
                    time.sleep(0.1)
                    print(f'    TX amp has no effect in model '
                          f'(S21 bypass={s21_bypassed:.1f}, enabled={s21_enabled:.1f} dB) '
                          f'— skipping')
                    tx_amp_bypass = True
                else:
                    print(f'    TX amp: enabled ({expected_gain_db:+.1f} dB expected gain)')
                    _apply_rx_policy(r, r_fast, config_dict, rf_peripherals, rx_policy,
                                     tx_power_change_db=expected_gain_db)
                    tx_amp_bypass = False
            else:
                tx_amp_bypass = False

            # Reduce TX attenuator gradually (3 dB steps), checking the
            # RX path after each step via _apply_rx_policy.
            current_atten = rf_peripherals.get_tx_attenuation()
            min_atten = rf_peripherals.ATTEN_MIN
            atten_step = rf_peripherals.ATTEN_STEP
            if current_atten > min_atten:
                atten = current_atten
                ramp_step = max(3.0, atten_step)
                while atten > min_atten:
                    prev_atten = atten
                    atten = max(atten - ramp_step, min_atten)
                    atten = round(atten / atten_step) * atten_step
                    atten = max(atten, min_atten)
                    rf_peripherals.set_tx_attenuation(atten)
                    time.sleep(0.1)
                    step_change_db = prev_atten - atten  # positive = TX power increased
                    _apply_rx_policy(r, r_fast, config_dict, rf_peripherals, rx_policy,
                                     tx_power_change_db=step_change_db)
                print(f'    TX atten: {current_atten:.1f} -> {atten:.1f} dB')
            else:
                print(f'    TX atten: already at minimum ({current_atten:.1f} dB)')
            tx_atten_db = min_atten
        else:
            print(f'  step 4: WARNING — no rf_peripherals, digital only')

    # --- Step 5: Enforce power limit ---
    if power_limit_dbm is not None and config_dict is not None:
        achieved_powers = get_tone_powers(r, config_dict, reference_plane=reference_plane,
                                          rf_peripherals=rf_peripherals)
        max_achieved = float(np.max(achieved_powers))
        excess_db = max_achieved - power_limit_dbm
        print(f'  step 5: power limit {power_limit_dbm} dBm — '
              f'achieved {max_achieved:.1f} dBm, excess {excess_db:+.1f} dB')
        if excess_db > 0.1:
            if has_rf:
                current_atten = rf_peripherals.get_tx_attenuation()
                atten_step = rf_peripherals.ATTEN_STEP
                atten_max = rf_peripherals.ATTEN_MAX
                needed_atten = current_atten + excess_db
                new_atten = min(
                    round(needed_atten / atten_step) * atten_step,
                    atten_max)
                rf_peripherals.set_tx_attenuation(new_atten)
                time.sleep(0.1)
                print(f'    TX atten: {current_atten:.1f} → {new_atten:.1f} dB')
                remaining_db = needed_atten - new_atten
                if remaining_db > 0.1:
                    psb_scale = r.psbscale.get_scale()
                    psb_scale *= 10**(-remaining_db / 20)
                    psb_scale = float(np.clip(psb_scale, scalemin, scalemax))
                    r.psbscale.set_scale(psb_scale)
                    time.sleep(0.01)
                    print(f'    psb_scale reduced to {psb_scale:.4f} '
                          f'({remaining_db:.1f} dB remaining)')
            else:
                psb_scale = r.psbscale.get_scale()
                psb_scale *= 10**(-excess_db / 20)
                psb_scale = float(np.clip(psb_scale, scalemin, scalemax))
                r.psbscale.set_scale(psb_scale)
                time.sleep(0.01)
                print(f'    psb_scale reduced to {psb_scale:.4f} (no RF peripherals)')

    _, dsp_overflow_details = check_dsp_overflow(r, 0.1, verbose=False)
    _, dac_saturation_details = check_output_saturation(r_fast, iterations=250, verbose=False)
    print(f'maximise_tx_power: done')

    return amps, best_fftshift, psb_scale, dsp_overflow_details, dac_saturation_details

def fix_dac_saturation(r, r_fast=None, config_dict=None):
    """Reduce psb_scale until DAC saturation clears.

    Halves psb_scale one step at a time until the DAC snapshot shows no
    saturation, then applies a 0.9 headroom factor.  If the DAC is not
    currently saturating (intermittent case), applies a 3 dB safety
    reduction and returns.
    """
    print(f'fix_dac_saturation (psb_scale={r.psbscale.get_scale():.6f})')
    init_psb_scale = r.psbscale.get_scale()
    min_value = 1 / 256

    check, levels = check_output_saturation(r_fast, iterations=250, verbose=False)
    if not check:
        # Intermittent saturation — apply 3 dB safety reduction
        psb_scale = init_psb_scale * 0.707
        r.psbscale.set_scale(psb_scale)
        time.sleep(0.01)
        check, levels = check_output_saturation(r_fast, iterations=250, verbose=False)
        print(f'  intermittent saturation — applied 3 dB reduction: psb_scale={psb_scale:.4f}')
        return psb_scale, check_dsp_overflow(r, 0.1, verbose=False)[1], levels

    # Halve psb_scale until not saturating
    psb_scale = init_psb_scale
    while check and psb_scale > min_value:
        psb_scale = max(psb_scale / 2, min_value)
        r.psbscale.set_scale(psb_scale)
        time.sleep(0.01)
        check, levels = check_output_saturation(r_fast, iterations=250, verbose=False)
        print(f'  psb_scale={psb_scale:.6f}, saturated={check}')

    if check:
        print('  WARNING: saturation persists at minimum psb_scale')
        return min_value, check_dsp_overflow(r, 0.1, verbose=False)[1], levels

    # Apply headroom
    psb_scale *= 0.90
    r.psbscale.set_scale(psb_scale)
    time.sleep(0.01)
    _, levels = check_output_saturation(r_fast, iterations=250, verbose=False)
    print(f'  resolved at psb_scale={psb_scale:.6f}')
    return psb_scale, check_dsp_overflow(r, 0.1, verbose=False)[1], levels


def fix_dsp_overflow(r, duration_s=0.5, max_iterations=10):
    """
    Fix DSP overflow by targeting the specific block(s) that are overflowing.

    Uses _find_best_psb_fftshift / _find_best_pfb_fftshift to sweep for the
    optimal fftshift in one pass rather than incrementing one step at a time.

    - PFB filterbank overflow → sweep for best PFB fftshift
    - PSB filterbank overflow → sweep for best PSB fftshift (mutes output
      during sweep), then compensate psb_scale for the fftshift change
    - PSB scale overflow → halve psb_scale

    Repeats until no overflow is detected or max_iterations is reached.

    Returns
    -------
    changed : bool
        True if any settings were modified.
    details : dict
        Final overflow check details.
    """
    changed = False

    for iteration in range(max_iterations):
        any_overflow, details = check_dsp_overflow(r, duration_s)
        if not any_overflow:
            if iteration == 0:
                print('fix_dsp_overflow: no overflow detected')
            else:
                print(f'fix_dsp_overflow: resolved after {iteration} iteration(s)')
            return changed, details

        psbscale_ovf = details['psbscale_ovf_delta']
        psb_ovf = details['psb_ovf_delta']
        pfb_ovf = details['pfb_ovf_delta']

        if pfb_ovf:
            # PFB filterbank overflowing — sweep for best fftshift
            init_pfb_shift = r.pfb.get_fftshift()
            best_pfb_shift, _ = _find_best_pfb_fftshift(r, duration_s)
            print(f'fix_dsp_overflow: PFB overflow — fftshift '
                  f'{format(init_pfb_shift, "#016b")} -> {format(best_pfb_shift, "#016b")}')
            changed = True

        if psb_ovf:
            # PSB filterbank overflowing — sweep for best fftshift.
            # _find_best_psb_fftshift saves/restores psb_scale with
            # compensation for the fftshift gain change.
            init_psb_shift = r.psb.get_fftshift()
            init_psb_scale = r.psbscale.get_scale()
            best_psb_shift, _, _ = _find_best_psb_fftshift(r, duration_s)
            new_psb_scale = r.psbscale.get_scale()

            print(f'fix_dsp_overflow: PSB overflow — fftshift '
                  f'{format(init_psb_shift, "#016b")} -> {format(best_psb_shift, "#016b")}, '
                  f'psb_scale {init_psb_scale:.4f} -> {new_psb_scale:.4f} (compensated)')
            changed = True

        if psbscale_ovf:
            # PSB scale block overflowing — reduce psb_scale
            psb_scale = r.psbscale.get_scale()
            new_scale = psb_scale / 2
            new_scale = max(new_scale, 1/256)
            r.psbscale.set_scale(new_scale)
            print(f'fix_dsp_overflow: PSB scale overflow — psb_scale '
                  f'{psb_scale:.4f} -> {new_scale:.4f}')
            time.sleep(0.01)
            changed = True

    # Ran out of iterations
    _, details = check_dsp_overflow(r, duration_s)
    print(f'fix_dsp_overflow: WARNING — overflow persists after {max_iterations} iterations')
    return changed, details


def optimise_tx_snr(r, r_fast=None, config_dict=None, reference_plane='detector',
                    rf_peripherals=None, headroom_db=1.0):
    """Optimise the TX digital dynamic range while preserving output power.

    Maximises digital gain (amplitudes near max, best fftshift, highest
    psb_scale) and compensates with TX attenuation so that tone power at
    the reference plane is unchanged.

    When rf_peripherals is available, psb_scale is ramped to just below
    DAC saturation and the power increase is absorbed by the TX
    attenuator.  When rf_peripherals is not available, psb_scale is
    computed analytically to preserve DAC-level power (digital-only
    rebalance).

    Parameters
    ----------
    reference_plane : str
        'dac', 'rf_output', or 'detector' (default).
    rf_peripherals : RFPeripherals or None
        RF peripheral controller for analog compensation.
    headroom_db : float
        DAC headroom in dB below saturation (default 1.0).
    """
    print(f'optimise_tx_snr (reference_plane={reference_plane})')
    if reference_plane not in ('dac', 'rf_output', 'detector'):
        raise ValueError(f"reference_plane must be 'dac', 'rf_output', or 'detector', "
                         f"got '{reference_plane}'")
    has_rf = rf_peripherals is not None and rf_peripherals.enabled

    init_dac_saturation, _ = check_output_saturation(r_fast, iterations=250, verbose=False)
    if init_dac_saturation:
        print('  DAC saturation detected — fixing first')
        fix_dac_saturation(r, r_fast, config_dict)
    init_amps = get_tone_amplitudes(r, config_dict)
    init_psb_scale = r.psbscale.get_scale()
    init_psb_fftshift = r.psb.get_fftshift()
    init_tx_atten = rf_peripherals.get_tx_attenuation() if has_rf else None
    init_powers = get_tone_powers(r, config_dict, reference_plane=reference_plane,
                                          rf_peripherals=rf_peripherals)
    scalemin = 1 / 256
    scalemax = 255
    print(f'  init: psb_scale={init_psb_scale:.6f}, '
          f'fftshift={format(init_psb_fftshift, "#016b")}, '
          f'max_amp={np.max(init_amps):.6f}'
          + (f', tx_atten={init_tx_atten:.1f} dB' if has_rf else ''))

    # --- Step 1: Mute and maximise amplitudes ---
    print('  step 1: mute and maximise amplitudes')
    max_amp = 1 - 2**-12
    amps_max = np.max(init_amps)
    if amps_max == 0:
        raise ValueError('Tone powers are all zero')
    amps_gain = max_amp / amps_max
    r.psbscale.set_scale(0)
    time.sleep(0.01)
    amps = init_amps * amps_gain
    amps = _apply_per_bin_scaling(r, config_dict, amps)
    set_tone_amplitudes(r, config_dict, amps)
    time.sleep(0.01)
    print(f'    amplitudes scaled by {amps_gain:.4f}')

    # --- Step 2: Find best PSB FFT shift (output muted) ---
    print('  step 2: find best PSB fftshift')
    best_fftshift, _, _ = _find_best_psb_fftshift(r)

    # --- Step 3: Maximise psb_scale ---
    if has_rf:
        print('  step 3: ramp psb_scale to max (analog compensation)')
        headroom_linear = 10**(-headroom_db / 20)

        def _is_ok(scale):
            scale = float(np.clip(scale, scalemin, scalemax))
            r.psbscale.set_scale(scale)
            time.sleep(0.01)
            ovf_details = check_dsp_overflow(r, 0.1, verbose=False)[1]
            ovf = ovf_details['psbscale_ovf_delta'] or ovf_details['psb_ovf_delta']
            sat = check_output_saturation(r_fast, iterations=250, verbose=False)[0]
            return not (ovf or sat)

        # Estimate starting psb_scale from initial conditions
        init_popcount = bin(init_psb_fftshift).count('1')
        best_popcount = bin(best_fftshift).count('1')
        fftshift_gain_ratio = 2.0 ** (init_popcount - best_popcount)
        if init_psb_scale > 0:
            estimated_scale = init_psb_scale / (amps_gain * fftshift_gain_ratio)
            estimated_scale = float(np.clip(estimated_scale, scalemin, scalemax))
            print(f'    analytical estimate: {estimated_scale:.6f} '
                  f'(amps_gain={amps_gain:.4f}, fftshift_gain={fftshift_gain_ratio:.4f})')
        else:
            estimated_scale = scalemin
            print(f'    init psb_scale was 0, starting from scalemin')

        # Multi-resolution ramp
        step_factors = [2.0, 2**0.5, 10**(1/20), 10**(0.5/20), 10**(0.1/20)]
        safe_scale = scalemin
        failed_scale = float('inf')
        for step_db, factor in zip([6, 3, 1, 0.5, 0.1], step_factors):
            scale = safe_scale
            if step_db == 6 and estimated_scale > scale:
                if estimated_scale < failed_scale and _is_ok(estimated_scale):
                    print(f'    ramp ({step_db} dB): jumped to estimate {estimated_scale:.6f} OK')
                    safe_scale = estimated_scale
                    scale = estimated_scale
                else:
                    failed_scale = min(failed_scale, estimated_scale)
                    print(f'    ramp ({step_db} dB): estimate {estimated_scale:.6f} saturates, '
                          f'starting from {scale:.6f}')
            next_scale = min(scale * factor, scalemax)
            while next_scale > scale:
                if next_scale >= failed_scale:
                    print(f'    ramp ({step_db} dB): {next_scale:.6f} skip (already failed)')
                    break
                if _is_ok(next_scale):
                    safe_scale = next_scale
                    print(f'    ramp ({step_db} dB): {next_scale:.6f} OK')
                    if next_scale >= scalemax:
                        break
                    scale = next_scale
                    next_scale = min(scale * factor, scalemax)
                else:
                    failed_scale = next_scale
                    print(f'    ramp ({step_db} dB): {next_scale:.6f} LIMIT')
                    break

        psb_scale = safe_scale * headroom_linear
        psb_scale = float(np.clip(psb_scale, scalemin, scalemax))
        r.psbscale.set_scale(psb_scale)
        time.sleep(0.01)
        print(f'    psb_scale={psb_scale:.6f} (headroom={headroom_db} dB)')

        # --- Step 4: Compensate with TX attenuator ---
        print('  step 4: compensate with TX attenuator')
        new_powers = get_tone_powers(r, config_dict, reference_plane=reference_plane,
                                          rf_peripherals=rf_peripherals)
        power_increase_db = float(np.max(new_powers)) - float(np.max(init_powers))
        print(f'    power change from digital maximisation: {power_increase_db:+.1f} dB')

        if power_increase_db > 0.1:
            atten_step = rf_peripherals.ATTEN_STEP
            atten_max = rf_peripherals.ATTEN_MAX
            min_atten = rf_peripherals.ATTEN_MIN
            needed_atten = init_tx_atten + power_increase_db
            new_atten = round(needed_atten / atten_step) * atten_step
            new_atten = float(np.clip(new_atten, min_atten, atten_max))

            # Gradual ramp to target attenuation
            current_atten = rf_peripherals.get_tx_attenuation()
            ramp_step = max(3.0, atten_step)
            if abs(new_atten - current_atten) > ramp_step:
                direction = 1.0 if new_atten > current_atten else -1.0
                step_atten = current_atten
                while abs(new_atten - step_atten) > ramp_step:
                    step_atten += direction * ramp_step
                    step_atten = round(step_atten / atten_step) * atten_step
                    step_atten = float(np.clip(step_atten, min_atten, atten_max))
                    rf_peripherals.set_tx_attenuation(step_atten)
                    time.sleep(0.1)
            rf_peripherals.set_tx_attenuation(new_atten)
            time.sleep(0.1)
            print(f'    TX attenuator {init_tx_atten:.1f} -> {new_atten:.1f} dB')

            # If attenuator can't absorb all excess, reduce psb_scale
            remaining_db = needed_atten - new_atten
            if remaining_db > 0.1:
                psb_scale = r.psbscale.get_scale()
                psb_scale *= 10**(-remaining_db / 20)
                psb_scale = float(np.clip(psb_scale, scalemin, scalemax))
                r.psbscale.set_scale(psb_scale)
                time.sleep(0.01)
                print(f'    psb_scale reduced to {psb_scale:.6f} '
                      f'(remaining {remaining_db:.1f} dB)')
        elif power_increase_db < -0.1:
            decrease = -power_increase_db
            min_atten = rf_peripherals.ATTEN_MIN
            atten_step = rf_peripherals.ATTEN_STEP
            new_atten = max(init_tx_atten - decrease, min_atten)
            new_atten = round(new_atten / atten_step) * atten_step
            new_atten = float(np.clip(new_atten, min_atten, rf_peripherals.ATTEN_MAX))
            rf_peripherals.set_tx_attenuation(new_atten)
            time.sleep(0.1)
            print(f'    TX attenuator {init_tx_atten:.1f} -> {new_atten:.1f} dB '
                  f'(recovering {decrease:.1f} dB)')
        else:
            print('    power unchanged, no attenuator adjustment needed')

    else:
        print('  step 3: compute psb_scale analytically (no RF peripherals)')
        init_popcount = bin(init_psb_fftshift).count('1')
        best_popcount = bin(best_fftshift).count('1')
        fftshift_gain = 2**(init_popcount - best_popcount)
        psb_scale = init_psb_scale / (fftshift_gain * amps_gain)
        psb_scale = float(np.clip(psb_scale, scalemin, scalemax))
        print(f'    fftshift_gain={fftshift_gain:.4f}, amps_gain={amps_gain:.4f}, '
              f'psb_scale={psb_scale:.6f}')

        r.psbscale.set_scale(psb_scale)
        time.sleep(0.01)

    print(f'  result: fftshift={format(best_fftshift, "#016b")}, '
          f'psb_scale={r.psbscale.get_scale():.6f}')

    # Verify: no overflow or saturation
    dsp_overflow_details = check_dsp_overflow(r, 0.1, verbose=False)[1]
    dac_saturation, dac_levels = check_output_saturation(r_fast, iterations=250, verbose=False)
    if (dsp_overflow_details['psbscale_ovf_delta'] or
            dsp_overflow_details['psb_ovf_delta'] or dac_saturation):
        print('  WARNING: overflow/saturation detected — reverting')
        set_tone_amplitudes(r, config_dict, init_amps)
        r.psb.set_fftshift(init_psb_fftshift)
        r.psbscale.set_scale(init_psb_scale)
        if has_rf:
            rf_peripherals.set_tx_attenuation(init_tx_atten)
        raise ValueError('TX DSP overflow detected after optimisation')

    # Verify output power is preserved
    achieved_powers = get_tone_powers(r, config_dict, reference_plane=reference_plane,
                                          rf_peripherals=rf_peripherals)
    power_error = float(np.max(np.abs(achieved_powers - init_powers)))
    if power_error > 0.5:
        print(f'  WARNING: output power changed by {power_error:.1f} dB')
    else:
        print(f'  power preserved (max error {power_error:.2f} dB)')

    return amps, best_fftshift, r.psbscale.get_scale(), dsp_overflow_details, dac_levels


def maximise_rx_power(r, r_fast, config_dict, headroom_db=1.0, rf_peripherals=None):
    """Maximise RX signal power into the ADC without clipping.

    Reduces attenuation to bring ADC levels as close to full-scale as
    possible while maintaining *headroom_db* of margin.

    Strategy:
      1. If saturated: fix with fix_adc_saturation first.
      2. Estimate available headroom from ADC snapshot.
      3. Reduce DSA by headroom estimate, check, step back if saturated.
      4. Reduce RX attenuator similarly, check, step back if saturated.
      5. Try enabling RX amp if available.
      6. Apply headroom.
      7. Find best PFB FFT shift.

    Parameters
    ----------
    headroom_db : float
        Safety margin in dB below the saturation point (default 1.0).
    """
    print('maximise_rx_power')
    DSA_MAX = 27
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    has_rf = rf_peripherals is not None and rf_peripherals.enabled

    def _get_dsa():
        return int(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])

    def _set_dsa(val):
        val = int(np.clip(round(val), 0, DSA_MAX))
        r.rfdc.core.set_dsa(adc_tile, adc_block, val)
        time.sleep(0.1)
        return val

    def _peak_dbfs(levels):
        peak = np.max(np.abs([levels['imax_fs'], levels['imin_fs'],
                              levels['qmax_fs'], levels['qmin_fs']]))
        if peak <= 0:
            return -100.0
        return float(20 * np.log10(peak))

    init_dsa = _get_dsa()
    init_rx_atten = rf_peripherals.get_rx_attenuation() if has_rf else None
    init_rx_amp_bypass = rf_peripherals.get_rx_amp_bypass() if has_rf else None
    print(f'  init: DSA={init_dsa} dB'
          + (f', RX atten={init_rx_atten:.1f} dB, RX amp bypass={init_rx_amp_bypass}' if has_rf else ''))

    # --- Handle RTS over-voltage (hidden firmware DSA) ---
    rts_event, rts_details = check_rfdc_rts_events(r, clear=False)
    if rts_details.get('rts_over_voltage', False):
        print('  step 0: RTS over-voltage — clearing hidden firmware DSA')
        _set_dsa(DSA_MAX)
        check_rfdc_rts_events(r, clear=True)
        time.sleep(0.1)
    elif rts_details.get('rts_over_range', False):
        print('  step 0: RTS over-range — clearing')
        check_rfdc_rts_events(r, clear=True)

    # --- If saturated, fix first ---
    saturated, levels = check_input_saturation(r, r_fast, iterations=250, verbose=False)
    peak_db = _peak_dbfs(levels)
    print(f'  step 1: initial check — saturated={saturated}, peak={peak_db:.1f} dBFS')
    if saturated:
        print('    ADC saturated — fixing first')
        fix_adc_saturation(r, r_fast, config_dict, rf_peripherals=rf_peripherals)

    # --- Reduce DSA towards zero ---
    print('  step 2: reduce DSA')
    current_dsa = _get_dsa()
    if current_dsa > 0:
        _, levels = check_input_saturation(r, r_fast, iterations=250, verbose=False)
        peak_db = _peak_dbfs(levels)
        headroom_available = -peak_db - headroom_db
        decrease = min(headroom_available, current_dsa)
        if decrease >= 1.0:
            new_dsa = _set_dsa(current_dsa - decrease)
            print(f'    DSA {current_dsa} -> {new_dsa} dB')
            sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
            while sat and new_dsa < current_dsa:
                new_dsa = _set_dsa(new_dsa + 1)
                print(f'    DSA step back to {new_dsa} dB')
                sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)

    # --- Reduce RX attenuator ---
    if has_rf:
        print('  step 3: reduce RX attenuator')
        current_atten = rf_peripherals.get_rx_attenuation()
        atten_min = rf_peripherals.ATTEN_MIN
        atten_step = rf_peripherals.ATTEN_STEP
        if current_atten > atten_min:
            _, levels = check_input_saturation(r, r_fast, iterations=250, verbose=False)
            peak_db = _peak_dbfs(levels)
            headroom_available = -peak_db - headroom_db
            decrease = min(headroom_available, current_atten - atten_min)
            if decrease >= atten_step:
                new_atten = max(
                    round((current_atten - decrease) / atten_step) * atten_step,
                    atten_min)
                # Gradual ramp down in 3 dB steps
                ramp_step = max(3.0, atten_step)
                step_atten = current_atten
                while step_atten - new_atten > ramp_step:
                    step_atten = max(
                        round((step_atten - ramp_step) / atten_step) * atten_step,
                        new_atten)
                    rf_peripherals.set_rx_attenuation(step_atten)
                    time.sleep(0.1)
                rf_peripherals.set_rx_attenuation(new_atten)
                time.sleep(0.1)
                print(f'    RX atten {current_atten:.1f} -> {new_atten:.1f} dB')
                # Check and step back if saturated
                sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
                while sat and new_atten < current_atten:
                    new_atten = min(
                        round((new_atten + 3.0) / atten_step) * atten_step,
                        rf_peripherals.ATTEN_MAX)
                    rf_peripherals.set_rx_attenuation(new_atten)
                    time.sleep(0.1)
                    print(f'    RX atten step back to {new_atten:.1f} dB')
                    sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)

        # --- Try enabling RX amp ---
        print('  step 4: try enabling RX amp')
        if rf_peripherals.get_rx_amp_bypass():
            # Check model S21 in both states to see if amp has any effect
            s21_bypassed = rf_peripherals._get_amp_s21('recv_atten')
            rf_peripherals.set_rx_amp_bypass(False)
            time.sleep(0.1)
            s21_enabled = rf_peripherals._get_amp_s21('recv_atten')
            expected_gain_db = s21_enabled - s21_bypassed
            if abs(expected_gain_db) < 0.5:
                # No gain difference — no bypass amp connected
                rf_peripherals.set_rx_amp_bypass(True)
                time.sleep(0.1)
                print(f'    RX amp has no effect in model '
                      f'(S21 bypass={s21_bypassed:.1f}, enabled={s21_enabled:.1f} dB) '
                      f'— skipping')
            else:
                sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
                if sat:
                    rf_peripherals.set_rx_amp_bypass(True)
                    time.sleep(0.1)
                    print('    RX amp causes saturation — keeping bypassed')
                else:
                    print(f'    RX amplifier enabled ({expected_gain_db:+.1f} dB expected gain)')

    # --- Final assessment ---
    print('  step 5: final assessment and PFB fftshift')
    saturated, levels = check_input_saturation(r, r_fast, iterations=250, verbose=False)
    best_dsa = _get_dsa()
    best_rx_atten = rf_peripherals.get_rx_attenuation() if has_rf else None
    peak_db = _peak_dbfs(levels)
    print(f'  done: peak={peak_db:.1f} dBFS, DSA={best_dsa} dB, saturated={saturated}')

    # --- Optimise PFB FFT shift ---
    best_fftshift, _ = _find_best_pfb_fftshift(r)

    return best_dsa, best_fftshift, check_dsp_overflow(r, 0.1, verbose=False)[1], levels, best_rx_atten

def fix_adc_saturation(r, r_fast, config_dict, rf_peripherals=None):
    """Attempt to clear ADC saturation using RF peripherals and ADC DSA.

    Steps through controls in order of preference:
      1. Bypass the RX amplifier.
      2. Step RX attenuator up in 3 dB increments until clear.
      3. Step ADC DSA up in 2 dB increments until clear.

    Returns
    -------
    result : dict
        'dsa', 'adc_levels', 'rx_attenuation_db', 'rx_amp_bypass', 'saturation'
    """
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    DSA_MAX = 27
    has_rf = rf_peripherals is not None and rf_peripherals.enabled

    def _make_result():
        check_rfdc_rts_events(r, clear=True)
        time.sleep(0.1)
        sat, lvls = check_input_saturation(r, r_fast, iterations=500, verbose=False)
        d = float(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])
        ra = rf_peripherals.get_rx_attenuation() if has_rf else None
        ab = rf_peripherals.get_rx_amp_bypass() if has_rf else None
        return {'dsa': d, 'adc_levels': lvls, 'rx_attenuation_db': ra,
                'rx_amp_bypass': ab, 'saturation': sat}

    def _check_saturated():
        check_rfdc_rts_events(r, clear=True)
        time.sleep(0.1)
        sat, _ = check_input_saturation(r, r_fast, iterations=500, verbose=False)
        return sat

    # --- Handle RTS over-voltage (hidden firmware DSA) ---
    rts_event, rts_details = check_rfdc_rts_events(r, clear=False)
    if rts_details.get('rts_over_voltage', False):
        print('fix_adc_saturation: RTS over-voltage — clearing hidden firmware DSA')
        r.rfdc.core.set_dsa(adc_tile, adc_block, int(DSA_MAX))
        time.sleep(0.1)
        check_rfdc_rts_events(r, clear=True)
        time.sleep(0.1)
    else:
        check_rfdc_rts_events(r, clear=True)
        time.sleep(0.1)

    # Check if saturation is present
    if not _check_saturated():
        print('fix_adc_saturation: no saturation detected')
        return _make_result()

    print('fix_adc_saturation: ADC saturation detected')

    # --- Step 1: Bypass RX amplifier ---
    if has_rf and not rf_peripherals.get_rx_amp_bypass():
        # Check model S21 to see if bypassing would reduce gain
        s21_enabled = rf_peripherals._get_amp_s21('recv_atten')
        rf_peripherals.set_rx_amp_bypass(True)
        time.sleep(0.1)
        s21_bypassed = rf_peripherals._get_amp_s21('recv_atten')
        expected_reduction_db = s21_enabled - s21_bypassed
        if abs(expected_reduction_db) < 0.5:
            # No gain difference — no bypass amp connected, revert
            rf_peripherals.set_rx_amp_bypass(False)
            time.sleep(0.1)
            print(f'  Bypass amp has no effect in model '
                  f'(S21 enabled={s21_enabled:.1f}, bypass={s21_bypassed:.1f} dB) '
                  f'— skipping')
        else:
            print('  Bypassing RX amplifier...')
            if not _check_saturated():
                print('  Resolved by bypassing RX amplifier')
                return _make_result()
            print('  Still saturated after bypassing RX amplifier')

    # --- Step 2: Step RX attenuator up in 3 dB increments ---
    if has_rf:
        atten_step = 3.0
        atten_max = rf_peripherals.ATTEN_MAX
        hw_step = rf_peripherals.ATTEN_STEP
        current_atten = rf_peripherals.get_rx_attenuation()
        while current_atten < atten_max:
            current_atten = min(
                round((current_atten + atten_step) / hw_step) * hw_step,
                atten_max)
            rf_peripherals.set_rx_attenuation(current_atten)
            time.sleep(0.1)
            sat = _check_saturated()
            print(f'  RX atten: {current_atten:.1f} dB, saturated: {sat}')
            if not sat:
                print(f'  Resolved at RX attenuation = {current_atten:.1f} dB')
                return _make_result()
        print(f'  RX attenuator at max ({atten_max:.1f} dB), still saturated')

    # --- Step 3: Step ADC DSA up in 2 dB increments ---
    dsa_step = 2
    current_dsa = int(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])
    while current_dsa < DSA_MAX:
        current_dsa = min(current_dsa + dsa_step, DSA_MAX)
        r.rfdc.core.set_dsa(adc_tile, adc_block, int(current_dsa))
        time.sleep(0.1)
        sat = _check_saturated()
        print(f'  DSA: {current_dsa} dB, saturated: {sat}')
        if not sat:
            print(f'  Resolved at ADC DSA = {current_dsa} dB')
            return _make_result()

    print('  WARNING: ADC saturation persists at maximum DSA and attenuation')
    return _make_result()

def optimise_rx_snr(r, r_fast=None, config_dict=None, headroom_db=1.0, rf_peripherals=None):
    """Optimise the RX signal-to-noise ratio.

    Maximises the analog signal into the ADC by preferring the RX
    attenuator over the DSA for any required gain control (the analog
    attenuator has better noise performance than the digital DSA).

    Steps:
      1. Fix any existing ADC saturation.
      2. Set DSA to 0 (transfer all attenuation to RX attenuator).
         If saturated, increase RX attenuator until clear.
      3. If not saturated, decrease RX attenuator until just before
         saturation, then add headroom.
      4. Find best PFB FFT shift.

    Parameters
    ----------
    r : readout interface
    config_dict : dict
    headroom_db : float
        Safety margin in dB (default 1.0).
    rf_peripherals : RFPeripheralController or None
    """
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    has_rf = rf_peripherals is not None and rf_peripherals.enabled

    print('optimise_rx_snr')

    # --- Fix existing saturation ---
    sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
    if sat:
        print('  ADC saturated — fixing first')
        fix_adc_saturation(r, r_fast, config_dict, rf_peripherals=rf_peripherals)

    # --- Step 1: Set DSA to 0 ---
    current_dsa = float(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])
    if current_dsa > 0:
        print(f'  step 1: set DSA to 0 (was {current_dsa:.0f} dB)')
        r.rfdc.core.set_dsa(adc_tile, adc_block, 0)
        time.sleep(0.1)

        sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
        if sat and has_rf:
            atten_step = rf_peripherals.ATTEN_STEP
            atten_max = rf_peripherals.ATTEN_MAX
            current_atten = rf_peripherals.get_rx_attenuation()
            new_atten = min(
                round((current_atten + current_dsa) / atten_step) * atten_step,
                atten_max)
            rf_peripherals.set_rx_attenuation(new_atten)
            time.sleep(0.1)
            print(f'    transferred DSA to RX atten: {current_atten:.1f} -> {new_atten:.1f} dB')

            sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
            while sat and new_atten < atten_max:
                new_atten = min(
                    round((new_atten + 3.0) / atten_step) * atten_step,
                    atten_max)
                rf_peripherals.set_rx_attenuation(new_atten)
                time.sleep(0.1)
                sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
                print(f'    RX atten: {new_atten:.1f} dB, saturated={sat}')

            if sat:
                dsa = 0
                while sat and dsa < 27:
                    dsa = min(dsa + 2, 27)
                    r.rfdc.core.set_dsa(adc_tile, adc_block, int(dsa))
                    time.sleep(0.1)
                    sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
                    print(f'    DSA: {dsa} dB, saturated={sat}')
        elif sat:
            dsa = 0
            while sat and dsa < 27:
                dsa = min(dsa + 1, 27)
                r.rfdc.core.set_dsa(adc_tile, adc_block, int(dsa))
                time.sleep(0.1)
                sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
                print(f'    DSA: {dsa} dB, saturated={sat}')
        else:
            print(f'    no saturation at DSA=0')

    # --- Step 2: Reduce RX attenuation to maximise signal ---
    if has_rf:
        atten_step = rf_peripherals.ATTEN_STEP
        atten_min = rf_peripherals.ATTEN_MIN
        current_atten = rf_peripherals.get_rx_attenuation()
        if current_atten > atten_min:
            print(f'  step 2: reduce RX atten from {current_atten:.1f} dB')
            new_atten = current_atten
            while new_atten > atten_min:
                test_atten = max(
                    round((new_atten - atten_step) / atten_step) * atten_step,
                    atten_min)
                rf_peripherals.set_rx_attenuation(test_atten)
                time.sleep(0.1)
                sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
                if sat:
                    new_atten = min(
                        round((test_atten + headroom_db) / atten_step) * atten_step,
                        rf_peripherals.ATTEN_MAX)
                    rf_peripherals.set_rx_attenuation(new_atten)
                    time.sleep(0.1)
                    print(f'    optimal RX atten: {new_atten:.1f} dB '
                          f'({headroom_db} dB headroom)')
                    break
                new_atten = test_atten
            else:
                print(f'    no saturation at minimum RX atten: {atten_min:.1f} dB')

    # --- Step 3: Find best PFB FFT shift ---
    print('  step 3: find best PFB fftshift')
    best_fftshift, _ = _find_best_pfb_fftshift(r)

    _, levels = check_input_saturation(r, r_fast, iterations=250, verbose=False)
    print(f'  done: PFB fftshift={format(best_fftshift, "#016b")}')
    return best_fftshift, check_dsp_overflow(r, 0.1, verbose=False)[1], levels




def read_accumulated_data(r, num_tones=None, tone_indices=None):
    """
    Read one sample of accumulated data from the RFSOC using the slower CASPER interface.

    :param r: Readout object
    :param num_tones: Number of tones (deprecated, use tone_indices instead)
    :param tone_indices: Array of output channel indices to read. With VACC, these may be
                        non-contiguous (e.g., [0, 6, 12] instead of [0, 1, 2]).
                        If None and num_tones is given, assumes contiguous indices [0..num_tones-1].
    :return: Complex data array for the specified tones
    """
    data = np.asarray(r.accumulators[0].get_new_spectra())
    if tone_indices is not None:
        # Extract data at specific output channel indices
        return data[tone_indices]
    elif num_tones is None:
        return data
    else:
        # Legacy behavior: assume contiguous indices
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
    tt_msb_addr = acc.host.transport._get_device_address(f'{acc.prefix}acc_tt_msb')
    tt_lsb_addr = acc.host.transport._get_device_address(f'{acc.prefix}acc_tt_lsb')
    params = {'acc':acc,
              'addrs':addrs,
              'nbytes':nbytes,
              'nbranch':nbranch,
              'base_addr':addrs[0],
              'tt_msb_addr':tt_msb_addr,
              'tt_lsb_addr':tt_lsb_addr}

    return params


def read_tt_fast(fast_read_params):
    """
    Read the PTP telescope time from the accumulator using the fast local memory transport.

    :param fast_read_params: Parameters from get_fast_read_params()
    :return: 64-bit telescope time as a Python int
    """
    acc = fast_read_params['acc']
    mm = acc.host.transport.axil_mm
    tt_msb_addr = fast_read_params['tt_msb_addr']
    tt_lsb_addr = fast_read_params['tt_lsb_addr']
    (msb,) = struct.unpack('<I', mm[tt_msb_addr:tt_msb_addr+4])
    (lsb,) = struct.unpack('<I', mm[tt_lsb_addr:tt_lsb_addr+4])
    return (msb << 32) + lsb


def get_accumulator_snapshot(r, config_dict, tone_index):
    """
    Grab a single pre-accumulation snapshot for a given tone.

    Translates the user-facing tone index (0, 1, 2, ...) to the firmware
    accumulator channel index, then acquires and returns 1024 complex samples
    at full rate (before accumulation).

    :param r: Readout object
    :param config_dict: Configuration dictionary (needed for tone index mapping)
    :param tone_index: User-facing tone index (0-based)
    :return: Complex numpy array of 1024 samples
    """
    details = get_tone_frequencies(r, config_dict, detailed_output=True)[1]
    firmware_indices = details['rx']['tone_indices']
    if tone_index >= len(firmware_indices):
        raise ValueError(f'Tone index {tone_index} out of range '
                         f'(only {len(firmware_indices)} tones active)')
    fw_chan = firmware_indices[tone_index]
    acc = r.accumulators[0]
    acc.set_snapshot_chan(fw_chan)
    return acc.get_new_snapshot()


def _read_accumulator_snapshot_fast(r_fast, fw_chan):
    """
    Low-level fast accumulator snapshot read for a single firmware channel.

    :param r_fast: Fast readout object (local=True)
    :param fw_chan: Firmware accumulator channel index
    :return: Complex numpy array of snapshot samples
    :rtype: numpy.ndarray
    """
    acc = r_fast.accumulators[0]
    mm = acc.host.transport.axil_mm

    # Cache addresses on first call
    if not hasattr(acc, '_fast_snap_addrs'):
        snap_name = f'{acc.prefix}snapshot'
        acc._fast_snap_addrs = {
            'snapshot_chan': acc.host.transport._get_device_address(f'{acc.prefix}snapshot_chan'),
            'ctrl': acc.host.transport._get_device_address(f'{snap_name}_ctrl'),
            'status': acc.host.transport._get_device_address(f'{snap_name}_status'),
            'bram': acc.host.transport._get_device_address(f'{snap_name}_bram'),
        }
        snap_obj = acc.host.snapshots[snap_name]
        acc._fast_snap_nbytes = snap_obj.length_bytes

    addrs = acc._fast_snap_addrs
    nbytes = acc._fast_snap_nbytes

    # Set snapshot channel
    mm[addrs['snapshot_chan']:addrs['snapshot_chan']+4] = struct.pack('<I', fw_chan)

    # Arm snapshot: man_trig=True, man_valid=False
    # ctrl = 0 + (1<<1) = 2, then ctrl = 1 + (1<<1) = 3
    mm[addrs['ctrl']:addrs['ctrl']+4] = struct.pack('<I', 2)
    mm[addrs['ctrl']:addrs['ctrl']+4] = struct.pack('<I', 3)

    # Poll status until done (bit 31 clear)
    while True:
        (status,) = struct.unpack('<I', mm[addrs['status']:addrs['status']+4])
        if not (status & 0x80000000):
            break

    # Read BRAM data
    raw = bytes(mm[addrs['bram']:addrs['bram']+nbytes])
    dc = np.frombuffer(raw, dtype='<i4')
    return dc[0::2] + 1j*dc[1::2]


def get_accumulator_snapshot_fast(r, r_fast, config_dict, tone_index, firmware_indices=None):
    """
    Grab a single pre-accumulation snapshot for a given tone using the
    fast local memory transport (devmem).

    Translates the user-facing tone index (0, 1, 2, ...) to the firmware
    accumulator channel index, then acquires and returns 1024 complex samples
    at full rate (before accumulation).

    :param r_fast: Fast readout object (local=True)
    :param r: Standard katcp readout object (needed for tone frequency lookup via RFDC)
    :param config_dict: Configuration dictionary (needed for tone index mapping)
    :param tone_index: User-facing tone index (0-based)
    :param firmware_indices: Pre-computed firmware channel indices from
        get_tone_frequencies(). If None, will be looked up via r.
        Pass this when calling in a loop to avoid repeated lookups.
    :return: Complex numpy array of 1024 samples
    :rtype: numpy.ndarray
    """
    if firmware_indices is None:
        details = get_tone_frequencies(r, config_dict, detailed_output=True)[1]
        firmware_indices = details['rx']['tone_indices']
    if tone_index >= len(firmware_indices):
        raise ValueError(f'Tone index {tone_index} out of range '
                         f'(only {len(firmware_indices)} tones active)')
    fw_chan = firmware_indices[tone_index]
    return _read_accumulator_snapshot_fast(r_fast, fw_chan)


def get_adc_snapshot(r):
    """
    Capture a single ADC snapshot (4096 complex128 samples).

    :param r: Readout object
    :return: Complex numpy array of ADC samples
    """
    return np.asarray(r.adc_snapshot.get_snapshot(), dtype=np.complex128)


def get_dac_snapshot(r):
    """
    Capture a single DAC snapshot (4096 complex128 samples per DAC).

    :param r: Readout object
    :return: Tuple of (dac0, dac1) complex numpy arrays
    """
    dac0, dac1 = r.dac_snapshot.get_snapshot()
    return (np.asarray(dac0, dtype=np.complex128),
            np.asarray(dac1, dtype=np.complex128))


# Lock protecting the shared common input mux + snapshot hardware.
# The ADC/DAC snapshot blocks are shared across pipelines and selected
# via common_sel, so set_input + trigger + read must be atomic.
_snapshot_lock = threading.Lock()


def _set_common_input_fast(r_fast):
    """
    Set the common block input mux to this pipeline via devmem.
    Caches the register address on the common block for subsequent calls.
    """
    common = r_fast.common
    mm = common.host.transport.axil_mm
    if not hasattr(common, '_fast_sel_addr'):
        common._fast_sel_addr = common.host.transport._get_device_address(f'{common.prefix}sel')
    addr = common._fast_sel_addr
    mm[addr:addr+4] = struct.pack('<I', r_fast.pipeline_id)


def get_adc_snapshot_fast(r_fast):
    """
    Capture a single ADC snapshot using the fast local memory transport (devmem).

    Selects this pipeline via the common input mux before triggering.
    Holds ``_snapshot_lock`` for the full set_input + trigger + read sequence
    to prevent races between pipelines sharing the snapshot hardware.

    :param r_fast: Fast readout object (local=True)
    :return: Complex numpy array of ADC samples (complex128)
    :rtype: numpy.ndarray
    """
    ss = r_fast.adc_snapshot
    mm = ss.host.transport.axil_mm

    # Cache register addresses on first call
    if not hasattr(ss, '_fast_addrs'):
        ss._fast_addrs = {
            'ctrl': ss.host.transport._get_device_address(f'{ss.prefix}ctrl'),
            'n_bytes': ss.host.transport._get_device_address(f'{ss.prefix}n_bytes'),
            'i': ss.host.transport._get_device_address(f'{ss.prefix}i'),
            'q': ss.host.transport._get_device_address(f'{ss.prefix}q'),
        }
        ss._fast_trig_bit = 1 << ss.ADC_SS_TRIG_OFFSET

    addrs = ss._fast_addrs
    trig_bit = ss._fast_trig_bit

    with _snapshot_lock:
        _set_common_input_fast(r_fast)

        # Trigger snapshot: clear bit, set bit, clear bit
        (ctrl_val,) = struct.unpack('<I', mm[addrs['ctrl']:addrs['ctrl']+4])
        ctrl_val &= ~trig_bit
        mm[addrs['ctrl']:addrs['ctrl']+4] = struct.pack('<I', ctrl_val)
        ctrl_val |= trig_bit
        mm[addrs['ctrl']:addrs['ctrl']+4] = struct.pack('<I', ctrl_val)
        ctrl_val &= ~trig_bit
        mm[addrs['ctrl']:addrs['ctrl']+4] = struct.pack('<I', ctrl_val)

        # Read n_bytes
        (nbyte,) = struct.unpack('<I', mm[addrs['n_bytes']:addrs['n_bytes']+4])

        # Read I and Q data buffers
        di = bytes(mm[addrs['i']:addrs['i']+nbyte])
        dq = bytes(mm[addrs['q']:addrs['q']+nbyte])

    i = np.frombuffer(di, dtype='<h')
    q = np.frombuffer(dq, dtype='<h')
    return np.asarray(i + 1j*q, dtype=np.complex128)


def get_dac_snapshot_fast(r_fast):
    """
    Capture a single DAC snapshot using the fast local memory transport (devmem).

    Selects this pipeline via the common input mux before triggering.
    Holds ``_snapshot_lock`` for the full set_input + trigger + read sequence
    to prevent races between pipelines sharing the snapshot hardware.

    :param r_fast: Fast readout object (local=True)
    :return: Tuple of (dac0, dac1) complex numpy arrays (complex128)
    :rtype: tuple of numpy.ndarray
    """
    ss = r_fast.dac_snapshot
    mm = ss.host.transport.axil_mm

    # Cache register addresses on first call
    if not hasattr(ss, '_fast_addrs'):
        ss._fast_addrs = {
            'ctrl': ss.host.transport._get_device_address(f'{ss.prefix}ctrl'),
            'n_bytes': ss.host.transport._get_device_address(f'{ss.prefix}n_bytes'),
            '0': ss.host.transport._get_device_address(f'{ss.prefix}0'),
            '1': ss.host.transport._get_device_address(f'{ss.prefix}1'),
        }
        ss._fast_trig_bit = 1 << ss.ADC_SS_TRIG_OFFSET

    addrs = ss._fast_addrs
    trig_bit = ss._fast_trig_bit

    with _snapshot_lock:
        _set_common_input_fast(r_fast)

        # Trigger snapshot
        (ctrl_val,) = struct.unpack('<I', mm[addrs['ctrl']:addrs['ctrl']+4])
        ctrl_val &= ~trig_bit
        mm[addrs['ctrl']:addrs['ctrl']+4] = struct.pack('<I', ctrl_val)
        ctrl_val |= trig_bit
        mm[addrs['ctrl']:addrs['ctrl']+4] = struct.pack('<I', ctrl_val)
        ctrl_val &= ~trig_bit
        mm[addrs['ctrl']:addrs['ctrl']+4] = struct.pack('<I', ctrl_val)

        # Read n_bytes
        (nbyte,) = struct.unpack('<I', mm[addrs['n_bytes']:addrs['n_bytes']+4])

        # Read DAC0 and DAC1 data buffers (interleaved I/Q)
        d0_raw = bytes(mm[addrs['0']:addrs['0']+nbyte])
        d1_raw = bytes(mm[addrs['1']:addrs['1']+nbyte])

    d0iq = np.frombuffer(d0_raw, dtype='<h')
    d1iq = np.frombuffer(d1_raw, dtype='<h')
    d0 = d0iq[0::2] + 1j*d0iq[1::2]
    d1 = d1iq[0::2] + 1j*d1iq[1::2]
    return (np.asarray(d0, dtype=np.complex128),
            np.asarray(d1, dtype=np.complex128))


def read_accumulated_data_fast(fast_read_params, num_tones=None, tone_indices=None):
    """
    Read one sample of accumulated data from the RFSOC
    utilising the faster katcp local memory transport.

    :param fast_read_params: Parameters from get_fast_read_params()
    :param num_tones: Number of tones (deprecated, use tone_indices instead)
    :param tone_indices: Array of output channel indices to read. With VACC, these may be
                        non-contiguous (e.g., [0, 6, 12] instead of [0, 1, 2]).
                        If None and num_tones is given, assumes contiguous indices [0..num_tones-1].
    :return: (acc_cnt, data, error_flag, telescope_time) where data is complex values at specified tone indices
             and telescope_time is the 64-bit PTP timestamp.
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
    tt = read_tt_fast(fast_read_params)
    stop_acc_cnt = acc.get_acc_cnt()
    if start_acc_cnt != stop_acc_cnt:
        acc.logger.warning('Accumulation counter changed while reading data!')
        err=True

    if tone_indices is not None:
        # Extract real and imaginary parts at specific output channel indices
        # Data is interleaved as [real0, imag0, real1, imag1, ...]
        tone_indices = np.asarray(tone_indices)
        real_indices = 2 * tone_indices
        imag_indices = 2 * tone_indices + 1
        # Interleave back to [real0, imag0, real1, imag1, ...]
        result = np.empty(2 * len(tone_indices), dtype=dout.dtype)
        result[0::2] = dout[real_indices]
        result[1::2] = dout[imag_indices]
        return start_acc_cnt, result, err, tt
    elif num_tones is None:
        return start_acc_cnt, dout, err, tt
    else:
        # Legacy behavior: assume contiguous indices
        return start_acc_cnt, dout[:2*num_tones], err, tt


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

    # Prepare sweep settings - this computes tone_indices for each point
    # as they may change when tones cross FFT bin boundaries
    fast_sweep_params = prepare_sweep_settings_fast(r_fast, config_dict, sweepfreqs.T)  # transpose to (num_points, num_tones)
    tone_indices_arr = fast_sweep_params.get('tone_indices')  # shape: (num_points, num_tones)

    for p in range(num_points):
        set_tone_frequencies(r,
                             config_dict,
                             sweepfreqs[:,p],
                             autosync=True)

        # Get tone_indices for this sweep point
        tone_indices_p = tone_indices_arr[p] if tone_indices_arr is not None else np.arange(num_tones)

        for s in range(samples_per_point):
            cnt,data,err,_tt = read_accumulated_data_fast(fast_read_params,
                                                      tone_indices=tone_indices_p)
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
    results = perform_sweep(r,r_fast,config_dict,centers, spans, points, samples_per_point, direction)

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

def get_tone_powers(r, config_dict, detailed_output=False, reference_plane='detector',
                    rf_peripherals=None):
    """
    Get current tone powers at the specified reference plane.

    Covers the full signal chain from DAC through to the accumulator.
    When detailed_output is True, returns power at every intermediate stage
    in both the TX and RX chains.

    Parameters
    ----------
    reference_plane : str
        TX chain (DAC -> detector):
            'dac'              - DAC output (after VOP, before analog frontend)
            'rf_output'        - RF frontend output (after amp, before cryostat)
            'detector'         - cryogenic focal plane (default)
        RX chain (detector -> accumulator):
            'cryostat_output'  - cryostat output (before RX frontend)
            'adc_input'        - ADC input (after RX frontend)
            'accumulator'      - raw accumulated IQ magnitude in dB
    detailed_output : bool
        If True, return (powers, details) where details is a dict of
        per-stage values across the full TX and RX chain.
    """
    TX_PLANES = ('dac', 'rf_output', 'detector')
    RX_PLANES = ('cryostat_output', 'adc_input', 'accumulator')
    VALID_PLANES = TX_PLANES + RX_PLANES
    if reference_plane not in VALID_PLANES:
        raise ValueError(f'reference_plane must be one of {VALID_PLANES}, got {reference_plane!r}')

    need_rx = reference_plane in RX_PLANES or detailed_output

    has_rf = rf_peripherals is not None and rf_peripherals.enabled

    # ---- TX chain ----
    p = _gather_tx_chain_params(r, config_dict, rf_peripherals=rf_peripherals)
    freqs = p['freqs']
    freq_details = p['freq_details']

    details = {}

    tx_powers, tx_details = calibration.calc_tone_powers(
        p['amps'], p['psb_fftshift'], p['psb_scale'],
        p['mixer_scale_is_1p0'], p['mixer_qmc_gain'], p['vop_current'],
        p['vop_current_fs'], p['dac_dbfs_to_dbm'],
        p['tx_combiner_loss_db'], p['tx_attenuator_value_db'],
        p['tx_if_s21_db'], p['tx_mixer_conversion_loss_db'],
        p['tx_rf_s21_db'], p['tx_bypass_amp_s21_db'],
        p['cryostat_input_s21_db'], p['dac_fs_bits'],
        detailed_output=True)
    details.update(tx_details)

    # ---- RX chain (forward computation from detector power) ----
    if need_rx:
        adc_tile = int(config_dict['firmware']['adc_tile'])
        adc_block = int(config_dict['firmware']['adc_block'])

        adc_mixer_settings = r.rfdc.core.get_mixer_settings(adc_tile, adc_block, r.rfdc.core.ADC_TILE)
        adc_mixer_scale_is_1p0 = adc_mixer_settings['FineMixerScale'] == r.rfdc.core.MIX_SCALE_1P0
        adc_qmc_settings = r.rfdc.core.get_qmc_settings(adc_tile, adc_block, r.rfdc.core.ADC_TILE)
        adc_mixer_qmc_gain = adc_qmc_settings['GainCorrectionFactor'] if adc_qmc_settings['EnableGain'] else 1.0

        adc_bits = config_dict['firmware']['adc_fullscale_bits']
        adc_dbm_to_dbfs = config_dict['firmware'].get('adc_dbm_to_dbfs', 12.0)
        pfb_fftshift = r.pfb.get_fftshift()
        acc_len = r.accumulators[0].get_acc_len()
        rx_mix_scale = config_dict['firmware'].get('rx_mix_scale', 1.0)

        # ADC DSA (RFDC digital step attenuator, before ADC)
        adc_dsa_db = float(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])

        # RX frontend parameters
        rx_combiner_loss_db = config_dict['rf_frontend'].get('rx_combiner_loss_db', 0) or 0
        rx_attenuator_value_db = config_dict['rf_frontend'].get('rx_attenuator_value_db', None)
        if rx_attenuator_value_db is None and has_rf:
            rx_attenuator_value_db = rf_peripherals.get_rx_attenuation()
        if rx_attenuator_value_db is None:
            rx_attenuator_value_db = 0
        rx_if_s21_db = config_dict['rf_frontend'].get('rx_if_s21_db', 0) or 0
        rx_mixer_conversion_loss_db = config_dict['rf_frontend'].get('rx_mixer_conversion_loss_db', 0) or 0
        rx_rf_s21_db = config_dict['rf_frontend'].get('rx_rf_s21_db', 0) or 0
        rx_bypass_amp_s21_db = config_dict['rf_frontend'].get('rx_bypass_amp_s21_db', 0) or 0
        cryostat_output_s21_db = config_dict['cryostat'].get('output_s21_db', 0) or 0

        if not p['rf_connected']:
            rx_combiner_loss_db = 0
            rx_attenuator_value_db = 0
            rx_if_s21_db = 0
            rx_mixer_conversion_loss_db = 0
            rx_rf_s21_db = 0
            rx_bypass_amp_s21_db = 0
        if not p['cryo_connected']:
            cryostat_output_s21_db = 0

        # Forward computation: use detector power from TX chain as input
        detector_power_dbm = tx_powers

        rx_iq, rx_details = calibration.calc_accumulated_iq_level(
            detector_power_dbm, adc_dbm_to_dbfs, adc_mixer_qmc_gain, adc_mixer_scale_is_1p0,
            adc_bits, pfb_fftshift, rx_mix_scale, acc_len,
            rx_combiner_loss_db=rx_combiner_loss_db,
            rx_attenuator_value_db=rx_attenuator_value_db,
            rx_if_s21_db=rx_if_s21_db,
            rx_mixer_conversion_loss_db=rx_mixer_conversion_loss_db,
            rx_rf_s21_db=rx_rf_s21_db,
            rx_bypass_amp_s21_db=rx_bypass_amp_s21_db,
            cryostat_output_s21_db=cryostat_output_s21_db,
            adc_dsa_db=adc_dsa_db,
            detailed_output=True)
        details.update(rx_details)

    # Select power at requested reference plane
    if reference_plane == 'dac':
        powers = np.array(details['dac_dbm'])
    elif reference_plane == 'rf_output':
        powers = np.array(details['tx_amp_dbm'])
    elif reference_plane == 'detector':
        powers = tx_powers
    elif reference_plane == 'accumulator':
        powers = np.array(details['accumulator_db'])
    elif reference_plane == 'adc_input':
        powers = np.array(details['adc_dbm'])
    elif reference_plane == 'cryostat_output':
        powers = np.array(details['cryostat_output_dbm'])

    if detailed_output:
        return powers,details
    else:
        return powers



def set_tone_powers(r, r_fast, config_dict, powers_dbm, reference_plane='detector',
                    optimise_dynamic_range=False, rf_peripherals=None,
                    rx_policy='protect'):
    """Set tone powers to specified levels in dBm at the chosen reference plane.

    Always preserves relative tone powers.  When optimise_dynamic_range is True,
    maximises DAC bit utilisation (amplitudes near max, best fftshift, highest
    psb_scale) and uses analog attenuation to reach the target level.
    DAC VOP is never modified.

    The function operates as a compute-then-apply pipeline:
      1. GATHER  — read firmware state and calibration
      2. PLAN    — compute optimal settings, check achievability
      3. APPLY   — write to hardware in safe transient-free order
      4. VERIFY  — read back and report error

    Parameters
    ----------
    r : readout interface
    config_dict : dict
        Live config dict (may be modified in-place for analog settings).
    powers_dbm : float or array_like
        Target tone power(s) in dBm at the reference plane.
    reference_plane : str
        'dac', 'rf_output', or 'detector'.
    optimise_dynamic_range : bool
        If True, maximise DAC bit utilisation and adjust analog chain.
    rf_peripherals : RFPeripheralController or None
        Required for analog adjustment during optimisation.
    rx_policy : str
        How to manage the RX path when the TX power change risks
        saturating the ADC.  One of:

        - ``'protect'`` (default) — if ADC saturates after the TX
          settings are applied, increase RX attenuation or DSA just
          enough to clear it and warn.
        - ``'compensate'`` — mirror the TX power change onto the RX
          path to keep round-trip power constant.
        - ``'raise'`` — raise ``RuntimeError`` if ADC saturates.
        - ``'none'`` — do not check or touch the RX path.

        See :func:`_apply_rx_policy` for full details.

    Returns
    -------
    dict with keys:
        target_powers_dbm, reference_plane, achieved_powers_dbm, power_error_db,
        amplitudes, psb_fftshift, psb_scale, tx_attenuation_db, tx_amp_bypass,
        tx_bypass_amp_s21_db, optimised, effective_bits_per_tone,
        amplitude_resolution_bits, dac_headroom_db, warnings
    """
    if rx_policy not in RX_POLICIES:
        raise ValueError(
            f"rx_policy must be one of {RX_POLICIES}, got {rx_policy!r}")
    VALID_PLANES = ('dac', 'rf_output', 'detector')
    if reference_plane not in VALID_PLANES:
        raise ValueError(f'reference_plane must be one of {VALID_PLANES}, got {reference_plane!r}')

    powers_dbm = np.atleast_1d(powers_dbm).astype(float)

    # ---- Phase 1: GATHER ----
    p = _gather_tx_chain_params(r, config_dict, rf_peripherals=rf_peripherals)
    cal = _mask_cal_for_reference_plane(p, reference_plane)

    # Bin sharing factor
    bin_indices = np.array(p['freq_details']['tx']['filterbank_bins'])
    _, counts = np.unique(bin_indices, return_counts=True)
    max_tones_per_bin = int(np.max(counts))

    # Broadcast scalar power to all tones
    n_tones = len(bin_indices)
    if powers_dbm.size == 1 and n_tones > 1:
        powers_dbm = np.full(n_tones, powers_dbm[0])

    # ---- Gather RF peripheral info ----
    has_rf = (rf_peripherals is not None and rf_peripherals.enabled
              and reference_plane != 'dac')
    s21_enabled = 0.0
    s21_bypassed = 0.0
    if has_rf:
        current_bypass = rf_peripherals.get_tx_amp_bypass()
        if current_bypass:
            s21_bypassed = float(rf_peripherals._get_amp_s21('transmit_atten'))
            rf_peripherals.set_tx_amp_bypass(False)
            s21_enabled = float(rf_peripherals._get_amp_s21('transmit_atten'))
            rf_peripherals.set_tx_amp_bypass(True)  # restore
        else:
            s21_enabled = float(rf_peripherals._get_amp_s21('transmit_atten'))
            rf_peripherals.set_tx_amp_bypass(True)
            s21_bypassed = float(rf_peripherals._get_amp_s21('transmit_atten'))
            rf_peripherals.set_tx_amp_bypass(False)  # restore

    if not optimise_dynamic_range:
        # Simple mode: just compute amplitudes with current settings
        max_amp = (1 - 2**-12) / max_tones_per_bin
        amps = calibration.calc_tone_amplitudes(
            powers_dbm, p['psb_fftshift'], p['psb_scale'],
            cal['mixer_scale_is_1p0'], cal['mixer_qmc_gain'], cal['vop_current'],
            cal['vop_current_fs'], cal['dac_dbfs_to_dbm'],
            cal['tx_combiner_loss_db'], cal['tx_attenuator_value_db'],
            cal['tx_if_s21_db'], cal['tx_mixer_conversion_loss_db'],
            cal['tx_rf_s21_db'], cal['tx_bypass_amp_s21_db'],
            cal['cryostat_input_s21_db'], cal['dac_fs_bits'])

        warnings_list = []
        if np.any(amps > max_amp):
            scale_factor = max_amp / float(np.max(amps))
            shortfall_db = -20 * np.log10(scale_factor)
            if shortfall_db < 0.1:
                # Marginal overshoot (< 0.1 dB) — clip to max_amp
                amps = np.clip(amps, 0, max_amp)
                msg = (f'Tone amplitudes clipped to max ({shortfall_db:.2f} dB overshoot)')
                print(f'  WARNING: {msg}')
                warnings_list.append(msg)
            else:
                n_tones = len(powers_dbm)
                raise ValueError(
                    f'Target power too high by {shortfall_db:.1f} dB for current '
                    f'gain settings with {n_tones} tones. '
                    f'Reduce tone_powers_dbm or num_tones, '
                    f'or use optimise_dynamic_range=True to auto-adjust.')
        if np.any((amps > 0) & (amps < 2**-12)):
            n_low = int(np.sum((amps > 0) & (amps < 2**-12)))
            msg = f'{n_low} tone(s) below minimum amplitude resolution'
            print(f'  WARNING: {msg}')
            warnings_list.append(msg)

        if max_tones_per_bin > 1:
            warnings_list.append(
                f'Up to {max_tones_per_bin} tones share an FFT bin, '
                f'amplitudes scaled by 1/{max_tones_per_bin}')

        # Dynamic range metrics for simple mode
        popcount = bin(p['psb_fftshift']).count('1')
        dac_amp_fs = np.abs(amps) / 2**(popcount + 1) * p['psb_scale']
        with np.errstate(divide='ignore'):
            eff_bits = 16 + np.log2(np.where(dac_amp_fs > 0, dac_amp_fs, np.nan))
            amp_res_bits = np.log2(np.where(
                np.abs(amps) > 0, np.abs(amps) / 2**-12, np.nan))

        print(f'set_tone_powers: setting {len(powers_dbm)} tones at '
              f'reference_plane={reference_plane!r}')
        init_amps_max = float(np.max(get_tone_amplitudes(r, config_dict)))
        set_tone_amplitudes(r, config_dict, amps)
        new_amps_max = float(np.max(np.abs(amps)))
        if init_amps_max > 0 and new_amps_max > 0:
            amp_change_db = float(20 * np.log10(new_amps_max / init_amps_max))
            _apply_rx_policy(r, r_fast, config_dict, rf_peripherals, rx_policy,
                             tx_power_change_db=amp_change_db)
        else:
            _apply_rx_policy(r, r_fast, config_dict, rf_peripherals, rx_policy,
                             tx_power_change_db=None)

        # DAC Saturation Check
        dac_saturation, dac_saturation_details = check_output_saturation(r_fast, iterations=250)
        if dac_saturation:
            print('  WARNING: DAC output is saturating!')
            warnings_list.append('DAC output is saturating!')

        # Measured DAC headroom from snapshot
        ss0, ss1 = get_dac_snapshot_fast(r_fast)
        ss0 = ss0 / 2**(dac_saturation_bits - 1)
        ss1 = ss1 / 2**(dac_saturation_bits - 1)
        dac_peak_measured = float(max(np.max(np.abs(ss0)), np.max(np.abs(ss1))))
        dac_headroom_db = float(-20 * np.log10(dac_peak_measured)) if dac_peak_measured > 0 else float('inf')
        print(f'  DAC headroom (measured): {dac_headroom_db:.1f} dB')

        achieved = get_tone_powers(r, config_dict, reference_plane=reference_plane,
                                          rf_peripherals=rf_peripherals)
        error = achieved - powers_dbm
        print(f'  Max power error: {np.max(np.abs(error)):.2f} dB')

        return {
            'target_powers_dbm': powers_dbm.tolist(),
            'reference_plane': reference_plane,
            'achieved_powers_dbm': achieved.tolist(),
            'power_error_db': error.tolist(),
            'amplitudes': amps.tolist(),
            'psb_fftshift': int(p['psb_fftshift']),
            'psb_scale': float(p['psb_scale']),
            'tx_attenuation_db': float(cal['tx_attenuator_value_db']) if np.isscalar(cal['tx_attenuator_value_db']) else float(np.mean(cal['tx_attenuator_value_db'])),
            'tx_amp_bypass': None,
            'tx_bypass_amp_s21_db': float(cal['tx_bypass_amp_s21_db']) if np.isscalar(cal['tx_bypass_amp_s21_db']) else float(np.mean(cal['tx_bypass_amp_s21_db'])),
            'optimised': False,
            'effective_bits_per_tone': eff_bits.tolist(),
            'amplitude_resolution_bits': amp_res_bits.tolist(),
            'dac_headroom_db': dac_headroom_db,
            'dac_saturation': dac_saturation,
            'dac_saturation_details': dac_saturation_details,
            'warnings': warnings_list,
        }

    # ---- Optimised mode ----
    # Strategy: maximise digital first, then attenuate to hit target.
    #   1. Maximise amplitudes (preserving relative ratios), find best
    #      fftshift, ramp psb_scale to just below DAC saturation.
    #   2. Measure achieved power at reference plane with CURRENT analog
    #      settings (don't touch attenuator/amp yet).
    #   3. Compute delta = achieved - target.  Use analog controls to
    #      absorb the excess: attenuator > amp bypass > reduce psb_scale.
    print(f'set_tone_powers: optimising for {len(powers_dbm)} tones '
          f'at reference_plane={reference_plane!r}')
    warnings_list = []

    max_amp = (1 - 2**-12) / max_tones_per_bin
    scalemin = 1 / 256
    scalemax = 255
    headroom_db = 1.0
    headroom_linear = 10**(-headroom_db / 20)

    # --- Step 1: Maximise digital gain ---
    # Compute amplitudes that preserve relative tone powers with max = max_amp.
    ref_amps = calibration.calc_tone_amplitudes(
        powers_dbm, p['psb_fftshift'], psb_scale=1.0,
        mixer_scale_is_1p0=cal['mixer_scale_is_1p0'],
        mixer_qmc_gain=cal['mixer_qmc_gain'],
        vop_current=cal['vop_current'],
        vop_current_fs=cal['vop_current_fs'],
        dac_dbfs_to_dbm=cal['dac_dbfs_to_dbm'],
        tx_combiner_loss_db=cal['tx_combiner_loss_db'],
        tx_attenuator_value_db=cal['tx_attenuator_value_db'],
        tx_if_s21_db=cal['tx_if_s21_db'],
        tx_mixer_conversion_loss_db=cal['tx_mixer_conversion_loss_db'],
        tx_rf_s21_db=cal['tx_rf_s21_db'],
        tx_bypass_amp_s21_db=cal['tx_bypass_amp_s21_db'],
        cryostat_input_s21_db=cal['cryostat_input_s21_db'],
        dac_fs_bits=cal['dac_fs_bits'])
    max_ref = float(np.max(np.abs(ref_amps)))
    if max_ref <= 0:
        raise ValueError('Target powers result in zero amplitudes')
    amps = ref_amps * (max_amp / max_ref)

    # Mute, set amplitudes, find best fftshift
    r.psbscale.set_scale(0)
    time.sleep(0.01)
    set_tone_amplitudes(r, config_dict, amps)
    time.sleep(0.01)
    print(f'  step 1: maximise digital — amplitudes at max')

    best_fftshift, _, _ = _find_best_psb_fftshift(r)

    # Ramp psb_scale to just below DAC saturation (same as maximise_tx_power)
    print(f'  step 2: ramp psb_scale')

    def _is_ok(scale):
        scale = float(np.clip(scale, scalemin, scalemax))
        r.psbscale.set_scale(scale)
        time.sleep(0.01)
        _, ovf_details = check_dsp_overflow(r, 0.1, verbose=False)
        ovf = ovf_details['psbscale_ovf_delta'] or ovf_details['psb_ovf_delta']
        sat = check_output_saturation(r_fast, iterations=250, verbose=False)[0]
        return not (ovf or sat)

    # Estimate starting psb_scale from amplitudes and fftshift
    popcount = bin(best_fftshift).count('1')
    amp_sum = float(np.sum(np.abs(amps)))
    if amp_sum > 0:
        estimated_scale = 2.0 ** (popcount + 1) / amp_sum
        estimated_scale = float(np.clip(estimated_scale, scalemin, scalemax))
        print(f'    analytical estimate: {estimated_scale:.4f} '
              f'(popcount={popcount}, amp_sum={amp_sum:.4f})')
    else:
        estimated_scale = scalemin

    step_factors = [2.0, 2**0.5, 10**(1/20), 10**(0.5/20), 10**(0.1/20)]
    safe_scale = scalemin
    failed_scale = float('inf')
    for step_db, factor in zip([6, 3, 1, 0.5, 0.1], step_factors):
        scale = safe_scale
        if step_db == 6 and estimated_scale > scale:
            if estimated_scale < failed_scale and _is_ok(estimated_scale):
                safe_scale = estimated_scale
                scale = estimated_scale
                print(f'    {step_db:4.1f} dB: jumped to estimate {estimated_scale:.4f} ok')
            else:
                failed_scale = min(failed_scale, estimated_scale)
                print(f'    {step_db:4.1f} dB: estimate {estimated_scale:.4f} saturates')
        next_scale = min(scale * factor, scalemax)
        while next_scale > scale:
            if next_scale >= failed_scale:
                print(f'    {step_db:4.1f} dB: {next_scale:.4f} skip (already failed)')
                break
            if _is_ok(next_scale):
                safe_scale = next_scale
                print(f'    {step_db:4.1f} dB: {next_scale:.4f} ok')
                if next_scale >= scalemax:
                    break
                scale = next_scale
                next_scale = min(scale * factor, scalemax)
            else:
                failed_scale = next_scale
                print(f'    {step_db:4.1f} dB: {next_scale:.4f} LIMIT')
                break

    optimal_psb_scale = safe_scale * headroom_linear
    optimal_psb_scale = float(np.clip(optimal_psb_scale, scalemin, scalemax))
    r.psbscale.set_scale(optimal_psb_scale)
    time.sleep(0.01)
    print(f'    psb_scale: {optimal_psb_scale:.4f} ({headroom_db} dB headroom)')
    ramp_psb_scale = optimal_psb_scale  # before any step-4c reduction

    # --- Step 3: Measure achieved power at reference plane ---
    # Digital is now maximised.  Analog settings are unchanged from entry.
    # Measure what power we're actually producing.
    max_digital_powers = get_tone_powers(r, config_dict,
                                         reference_plane=reference_plane,
                                         rf_peripherals=rf_peripherals)
    max_achieved = float(np.max(max_digital_powers))
    target_max = float(np.max(powers_dbm))
    delta_db = max_achieved - target_max
    print(f'  step 3: achieved {max_achieved:.1f} dBm, '
          f'target {target_max:.1f} dBm, delta {delta_db:+.1f} dB')

    # --- Step 4: Adjust analog controls to hit target ---
    # delta > 0: achieved > target → need to reduce power (add attenuation)
    # delta < 0: achieved < target → need to increase power (reduce attenuation / enable amp)
    tx_atten_db = float(cal['tx_attenuator_value_db'] if np.isscalar(
        cal['tx_attenuator_value_db']) else np.mean(cal['tx_attenuator_value_db']))
    tx_amp_bypass = rf_peripherals.get_tx_amp_bypass() if has_rf else None
    tx_amp_s21 = cal['tx_bypass_amp_s21_db']

    if abs(delta_db) > 0.5:
        print(f'  step 4: adjust analog ({delta_db:+.1f} dB)')
        remaining = delta_db  # positive = need to reduce, negative = need to increase

        if has_rf:
            atten_step = rf_peripherals.ATTEN_STEP
            atten_max = rf_peripherals.ATTEN_MAX
            min_atten = rf_peripherals.ATTEN_MIN

            if remaining < -0.1:
                # --- Need MORE power: reduce attenuation, enable amp ---

                # 4a: Enable TX amp if bypassed
                if tx_amp_bypass:
                    amp_gain = s21_enabled - s21_bypassed
                    if amp_gain > 0.5:
                        rf_peripherals.set_tx_amp_bypass(False)
                        time.sleep(0.1)
                        tx_amp_bypass = False
                        tx_amp_s21 = s21_enabled
                        remaining += amp_gain  # remaining gets less negative
                        print(f'    TX amp: enabled (+{amp_gain:.1f} dB)')

                # 4b: Reduce TX attenuator
                if remaining < -0.1 and tx_atten_db > min_atten:
                    needed = -remaining  # how much more power we need
                    reduce = min(needed, tx_atten_db - min_atten)
                    new_atten = round((tx_atten_db - reduce) / atten_step) * atten_step
                    new_atten = float(np.clip(new_atten, min_atten, atten_max))
                    rf_peripherals.set_tx_attenuation(new_atten)
                    time.sleep(0.1)
                    gained = tx_atten_db - new_atten
                    remaining += gained  # remaining gets less negative
                    print(f'    TX atten: {tx_atten_db:.1f} -> {new_atten:.1f} dB '
                          f'(+{gained:.1f} dB)')
                    tx_atten_db = new_atten

                # If still short after exhausting analog, error
                if remaining < -0.5:
                    raise ValueError(
                        f'Target power exceeds maximum achievable by '
                        f'{-remaining:.1f} dB at '
                        f'reference_plane={reference_plane!r}. '
                        f'Reduce tone_powers_dbm or num_tones.')

            elif remaining > 0.1:
                # --- Need LESS power: add attenuation, bypass amp ---

                # 4a: Increase TX attenuator
                atten_headroom = atten_max - tx_atten_db
                if remaining > 0.1 and atten_headroom >= atten_step:
                    add_atten = min(remaining, atten_headroom)
                    new_atten = round((tx_atten_db + add_atten) / atten_step) * atten_step
                    new_atten = float(np.clip(new_atten, min_atten, atten_max))
                    rf_peripherals.set_tx_attenuation(new_atten)
                    time.sleep(0.1)
                    absorbed = new_atten - tx_atten_db
                    remaining -= absorbed
                    print(f'    TX atten: {tx_atten_db:.1f} -> {new_atten:.1f} dB '
                          f'(-{absorbed:.1f} dB)')
                    tx_atten_db = new_atten

                # 4b: Bypass TX amp if still excess
                if remaining > 0.5 and not tx_amp_bypass:
                    amp_gain = s21_enabled - s21_bypassed
                    if amp_gain > 0.5:
                        rf_peripherals.set_tx_amp_bypass(True)
                        time.sleep(0.1)
                        tx_amp_bypass = True
                        tx_amp_s21 = s21_bypassed
                        remaining -= amp_gain
                        print(f'    TX amp: bypassed (-{amp_gain:.1f} dB)')

                        # Bypassing may have overshot — recover with attenuator
                        if remaining < -0.1:
                            current_atten = rf_peripherals.get_tx_attenuation()
                            remove = -remaining
                            new_atten = round((current_atten - remove) / atten_step) * atten_step
                            new_atten = float(np.clip(new_atten, min_atten, atten_max))
                            rf_peripherals.set_tx_attenuation(new_atten)
                            time.sleep(0.1)
                            remaining += (current_atten - new_atten)
                            print(f'    TX atten: {current_atten:.1f} -> {new_atten:.1f} dB '
                                  f'(recovering after bypass)')
                            tx_atten_db = new_atten

        elif remaining < -0.5:
            # No RF peripherals and need more power than digital max
            raise ValueError(
                f'Target power exceeds maximum achievable by '
                f'{-remaining:.1f} dB at reference_plane={reference_plane!r} '
                f'with no RF peripherals. Reduce tone_powers_dbm or num_tones.')

        # 4c: Last resort — reduce psb_scale for any remaining excess
        if remaining > 0.1:
            optimal_psb_scale = r.psbscale.get_scale()
            optimal_psb_scale *= 10**(-remaining / 20)
            optimal_psb_scale = float(np.clip(optimal_psb_scale, scalemin, scalemax))
            r.psbscale.set_scale(optimal_psb_scale)
            time.sleep(0.01)
            print(f'    psb_scale reduced to {optimal_psb_scale:.4f} '
                  f'(remaining {remaining:.1f} dB)')

    # Now compute final amplitudes analytically for the chosen settings.
    # The amplitudes are still at max from step 1, preserving relative
    # ratios.  The psb_scale + attenuator + amp bypass together set the
    # absolute power level.  We just need to recompute the amplitudes
    # to exactly hit the target.
    final_amps = calibration.calc_tone_amplitudes(
        powers_dbm, best_fftshift, optimal_psb_scale,
        mixer_scale_is_1p0=cal['mixer_scale_is_1p0'],
        mixer_qmc_gain=cal['mixer_qmc_gain'],
        vop_current=cal['vop_current'],
        vop_current_fs=cal['vop_current_fs'],
        dac_dbfs_to_dbm=cal['dac_dbfs_to_dbm'],
        tx_combiner_loss_db=cal['tx_combiner_loss_db'],
        tx_attenuator_value_db=tx_atten_db,
        tx_if_s21_db=cal['tx_if_s21_db'],
        tx_mixer_conversion_loss_db=cal['tx_mixer_conversion_loss_db'],
        tx_rf_s21_db=cal['tx_rf_s21_db'],
        tx_bypass_amp_s21_db=tx_amp_s21,
        cryostat_input_s21_db=cal['cryostat_input_s21_db'],
        dac_fs_bits=cal['dac_fs_bits'])
    final_amps = np.clip(final_amps, 0, max_amp)
    set_tone_amplitudes(r, config_dict, final_amps)
    time.sleep(0.01)

    # RX policy check for the overall TX power change
    init_powers = get_tone_powers(r, config_dict, reference_plane=reference_plane,
                                 rf_peripherals=rf_peripherals)
    # init_powers is measured after all changes; compare to entry state
    # which we captured from the gather phase.
    entry_powers = calibration.calc_tone_powers(
        np.array(p['amps']), p['psb_fftshift'], p['psb_scale'],
        mixer_scale_is_1p0=cal['mixer_scale_is_1p0'],
        mixer_qmc_gain=cal['mixer_qmc_gain'],
        vop_current=cal['vop_current'],
        vop_current_fs=cal['vop_current_fs'],
        dac_dbfs_to_dbm=cal['dac_dbfs_to_dbm'],
        tx_combiner_loss_db=cal['tx_combiner_loss_db'],
        tx_attenuator_value_db=cal['tx_attenuator_value_db'],
        tx_if_s21_db=cal['tx_if_s21_db'],
        tx_mixer_conversion_loss_db=cal['tx_mixer_conversion_loss_db'],
        tx_rf_s21_db=cal['tx_rf_s21_db'],
        tx_bypass_amp_s21_db=cal['tx_bypass_amp_s21_db'],
        cryostat_input_s21_db=cal['cryostat_input_s21_db'],
        dac_fs_bits=cal['dac_fs_bits'])
    tx_power_change_db = float(np.max(init_powers)) - float(np.max(entry_powers))
    _apply_rx_policy(r, r_fast, config_dict, rf_peripherals, rx_policy,
                     tx_power_change_db=tx_power_change_db)

    # --- Dynamic range metrics ---
    popcount = bin(best_fftshift).count('1')

    # Per-tone DAC amplitude (for effective bits calculation)
    dac_amp_per_tone = np.abs(amps) / 2**(popcount + 1) * ramp_psb_scale
    with np.errstate(divide='ignore'):
        eff_bits = 16 + np.log2(np.where(dac_amp_per_tone > 0, dac_amp_per_tone, np.nan))
        amp_res_bits = np.log2(np.where(
            np.abs(final_amps) > 0, np.abs(final_amps) / 2**-12, np.nan))

    # Measured DAC headroom from snapshot
    ss0, ss1 = get_dac_snapshot_fast(r_fast)
    ss0 = ss0 / 2**(dac_saturation_bits - 1)
    ss1 = ss1 / 2**(dac_saturation_bits - 1)
    dac_peak_measured = float(max(np.max(np.abs(ss0)), np.max(np.abs(ss1))))
    dac_headroom_db = float(-20 * np.log10(dac_peak_measured)) if dac_peak_measured > 0 else float('inf')

    print(f'  result:')
    print(f'    fftshift: {format(best_fftshift, "#016b")} (popcount {popcount})')
    print(f'    psb_scale: {optimal_psb_scale:.4f}')
    if has_rf:
        print(f'    TX atten: {tx_atten_db:.1f} dB, TX amp bypass: {tx_amp_bypass}')
    print(f'    effective DAC bits (worst): {float(np.nanmin(eff_bits)):.1f}')
    print(f'    DAC headroom (measured): {dac_headroom_db:.1f} dB')

    if max_tones_per_bin > 1:
        warnings_list.append(
            f'Up to {max_tones_per_bin} tones share an FFT bin, '
            f'amplitudes scaled by 1/{max_tones_per_bin}')

    # --- Verify ---
    achieved = get_tone_powers(r, config_dict, reference_plane=reference_plane,
                               rf_peripherals=rf_peripherals)
    final_amps_read = get_tone_amplitudes(r, config_dict)
    error = achieved - powers_dbm
    max_error = float(np.max(np.abs(error)))
    print(f'    verification: max power error = {max_error:.2f} dB')

    if max_error > 1.0:
        msg = (f'Achieved power differs from target by up to {max_error:.1f} dB '
               f'(tone {int(np.argmax(np.abs(error)))}). '
               f'The requested power may not be achievable at this reference plane.')
        print(f'    WARNING: {msg}')
        warnings_list.append(msg)

    dac_saturation, dac_saturation_details = check_output_saturation(
        r_fast, iterations=250, verbose=False)
    if dac_saturation:
        print('    WARNING: DAC output is saturating!')
        warnings_list.append('DAC output is saturating!')

    result = {
        'target_powers_dbm': powers_dbm.tolist(),
        'reference_plane': reference_plane,
        'achieved_powers_dbm': achieved.tolist(),
        'power_error_db': error.tolist(),
        'amplitudes': final_amps_read.tolist(),
        'psb_fftshift': int(best_fftshift),
        'psb_scale': float(optimal_psb_scale),
        'tx_attenuation_db': float(tx_atten_db),
        'tx_amp_bypass': tx_amp_bypass,
        'tx_bypass_amp_s21_db': float(tx_amp_s21) if np.isscalar(tx_amp_s21) else float(np.mean(tx_amp_s21)),
        'optimised': True,
        'effective_bits_per_tone': eff_bits.tolist(),
        'amplitude_resolution_bits': amp_res_bits.tolist(),
        'dac_headroom_db': dac_headroom_db,
        'dac_saturation': dac_saturation,
        'dac_saturation_details': dac_saturation_details,
        'warnings': warnings_list,
    }

    if warnings_list:
        print(f'  done with {len(warnings_list)} warning(s)')
    else:
        print(f'  done — all targets achieved')

    return result


def force_sync_fast(r_fast,wait_s=0.0001):
    pid = r_fast.pipeline_id
    regname = f'p{pid}_sync_ctrl'
    cache_prefix = f'_p{pid}_sync_ctrl'
    if not hasattr(r_fast, f'{cache_prefix}_addr'):
        setattr(r_fast, f'{cache_prefix}_addr', r_fast.sync.host.transport._get_device_address(regname))
        setattr(r_fast, f'{cache_prefix}_arm_bit', 1<<r_fast.sync.OFFSET_ARM_SYNC_OUT)
        setattr(r_fast, f'{cache_prefix}_sync_bit', 1<<r_fast.sync.OFFSET_MAN_SYNC)

    #arm_sync
    addr = getattr(r_fast, f'{cache_prefix}_addr')
    arm_bit = getattr(r_fast, f'{cache_prefix}_arm_bit')
    sync_bit = getattr(r_fast, f'{cache_prefix}_sync_bit')
    mm = r_fast.sync.host.transport.axil_mm
    (value,) = struct.unpack('<I',mm[addr:addr+4])

    #set 0
    value &= ~arm_bit
    mm[addr:addr+4] = struct.pack('<I',value)
    #set 1
    value |= arm_bit
    mm[addr:addr+4] = struct.pack('<I',value)
    #set 0
    value &= ~arm_bit
    mm[addr:addr+4] = struct.pack('<I',value)

    # change_reg_bits_fast(addr, 0, r_fast.sync.OFFSET_ARM_SYNC_OUT)
    # change_reg_bits(addr, 1, r_fast.sync.OFFSET_ARM_SYNC_OUT)
    # change_reg_bits(addr, 0, r_fast.sync.OFFSET_ARM_SYNC_OUT)

    #wait
    time.sleep(wait_s)

    #manual sync
    #set0
    value &= ~sync_bit
    mm[addr:addr+4] = struct.pack('<I',value)
    #set 1
    value |= sync_bit
    mm[addr:addr+4] = struct.pack('<I',value)
    #set 0
    value &= ~sync_bit
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


def estimate_papr_db(freqs, amps, phases, sample_rate, duration_s=0.001, chunk_size=65536, verbose=True):
    """
    Estimate the time-domain PAPR (peak-to-average power ratio, dB) for a sum of tones over a simulated duration.
    This version uses a for-loop over tones for each chunk (less vectorized, more memory-safe for some environments).

    Parameters
    ----------
    freqs : array_like
        Tone frequencies in Hz.
    amps : array_like
        Amplitudes of each tone (linear, not dB).
    phases : array_like
        Phase offsets for each tone (radians).
    sample_rate : float
        Sample rate in Hz (e.g., 2*adc_clk_hz).
    duration_s : float
        Duration to simulate in seconds (default 0.001).
    chunk_size : int
        Number of samples to process per chunk (default 65536).
    verbose : bool
        If True, print the simulated time and peak value for each chunk.

    Returns
    -------
    papr_db : float
        Peak-to-average power ratio in dB.
    """
    import numpy as np
    freqs = np.asarray(freqs)
    phases = np.asarray(phases)
    amps = np.asarray(amps)
    n_tones = len(amps)
    n_samples = int(np.round(duration_s * sample_rate))
    max_val = 0.0
    total_chunks = (n_samples + chunk_size - 1) // chunk_size
    for i in range(total_chunks):
        start = i * chunk_size
        end = min((i + 1) * chunk_size, n_samples)
        t = np.arange(start, end) / sample_rate
        block = np.zeros_like(t, dtype=np.complex128)
        for k in range(n_tones):
            block += amps[k] * np.exp(2j * np.pi * freqs[k] * t + 1j * phases[k])
        abs_block = np.abs(block)
        block_max = np.max(abs_block)
        if verbose:
            print(f"[FORLOOP {i}/{total_chunks}] Simulated time: {t[0]:.6f} to {t[-1]:.6f} s, chunk peak = {block_max:.6f}")
        max_val = max(max_val, block_max)
    avg_power = np.sum(amps ** 2)
    papr = (max_val ** 2) / avg_power
    papr_db = 10 * np.log10(papr)
    return papr_db


#include private functions when import * for debugging, to be removed later
__all__ = list(globals().keys())