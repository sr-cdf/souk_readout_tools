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
import tempfile
import threading

# pwd and fcntl are POSIX-only (absent on Windows). They are only used by
# on-hardware code paths (file locking, file ownership) that never run on
# Windows, so guard the imports to keep the module importable there.
try:
    import pwd
except ImportError:
    pwd = None
try:
    import fcntl
except ImportError:
    fcntl = None

try:
    import souk_mkid_readout
    from souk_mkid_readout.souk_mkid_readout import *
except ImportError:
    print("firmware_lib.py: Warning: Importing firmware lib without souk_mkid_readout support.")

from souk_readout_tools import calibration
from souk_readout_tools import config_utils
from souk_readout_tools.timing import unix_to_iso, DEFAULT_ALIGN_TOL_S

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


def _firmware_log(message, source='general'):
    print(f'firmware:{source}: {message}', flush=True)


# Persistent data (configs, calibration files) lives in the target user's home.
# The server runs under sudo, so this is resolved via config_utils rather than
# expanduser('~') (which would point at /root). On a Windows client it falls
# back to the user's home.
USER_DIR = config_utils.get_user_dir()

# Cross-pipeline coordination locks must resolve to the same path for every
# pipeline server on the board, regardless of which user starts it. The server
# is launched under sudo, so a home-relative path lands in /root rather than the
# target user's home; use the system temp dir instead (matches the rudat and
# lna_controller lock handling). USER_DIR is left home-relative on purpose - it
# also resolves calibration files, which live in the real user home.
_LOCK_DIR = os.path.join(tempfile.gettempdir(), 'souk_readout_tools')
FPGA_PROGRAM_LOCK = os.path.join(_LOCK_DIR, '.fpga_program.lock')
SHARED_INIT_LOCK = os.path.join(_LOCK_DIR, '.shared_init.lock')

autosync_time_delay = 0.001 #seconds

# v7.10 timed-sync: master-reset (MRST) is now an independent ctrl bit, decoupled from the
# sync pulse. Per-step (``autosync``) syncs default to mrst=False (a light re-reference that
# rides continuous accumulation across a buffer flip); init/config-time syncs use mrst=True
# (full reset+start, the old welded behaviour). The per-call ``mrst`` flags are plumbed
# through so reset can be toggled independently for bench testing.
INIT_SYNC_MRST = True  # master-reset on init/config-time syncs (startup, sync_delay change)

adc_saturation_bits = 16 # note the adc gives 16 bit data but is a 12 or 14 bit converter
dac_saturation_bits = 16 # note the dac takes 16 bit data but is a 12 or 14 bit converter

def _sync_if_requested(r, autosync=False, mrst=False):
    if not autosync:
        return
    # v7.10: sw_sync no longer needs arming; ``mrst`` toggles the (now independent) reset.
    r.sync.sw_sync(mrst=mrst)

def _warn_sync_delay_deprecated(value):
    """Warn that the ``sync_delay`` config parameter is deprecated under firmware v7.11.

    The RX/TX LO delay used to live in the sync block (``r.sync.set_delay``). From
    v7.11 it is owned by the mixer and configured automatically by
    ``initialize_pipeline_blocks()``: ``mixer.set_rx_sync_delay(SYNC_DELAY=5752)``
    followed by an automatic RX/TX skew match. This knob is therefore ignored.

    To repoint/override the RX sync delay manually after init (rarely needed)::

        r.mixer.set_rx_sync_delay(n)                              # set
        r.mixer.read_uint('sync_delay')                           # read back
        r.sync.sw_sync(mrst=True)                                 # re-reference
        r.mixer.set_buffer_switch_skew(r.mixer.get_tx_rx_skew())  # re-match skew
    """
    print(f'WARNING: sync_delay={value} is deprecated and ignored under firmware '
          'v7.11 (the mixer owns RX/TX delay and sets it during init; '
          'see _warn_sync_delay_deprecated for the manual-override recipe).')

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

# Fabric clock the path-delay tick count is measured against: 307.2 MHz, the
# fabric/FFT clock = adc_clk_hz (4915.2 MHz) / 16 (4x parallel * 2x oversample of
# the upstream FFT). Used only to turn a tick count into a delay time; kept as a
# constant so the compensation does not depend on the firmware mixer geometry.
FABRIC_CLK_HZ = 307.2e6

# Fixed group delay between the RX and TX paths, in FABRIC_CLK_HZ ticks. It is
# normally hidden because every retune is followed by a software sync that
# re-aligns both paths. When you retune *without* that sync (e.g. fast
# sweep/modulation), the RX fine-mixer LO has run ~46.666 us (14336 ticks)
# further than the TX LO, so each tone picks up a per-tone phase error
# proportional to its RX baseband offset frequency. See _rx_phase_compensation.
TX_RX_PATH_DELAY_TICKS = 14336

def _rx_phase_compensation(phase_incs_rx, fft_rbw_hz, compensate_rx_ticks,
                           tick_clk_hz=FABRIC_CLK_HZ):
    """Per-tone RX phase offset (rad) that cancels the RX-vs-TX path delay when
    retuning without a sync.

    The delay acts on the RX fine-mixer LO, whose phase advances by
    ``phase_incs_rx`` radians per FFT period (the residual offset of the tone
    from its bin centre). Over a delay of ``Dt = compensate_rx_ticks /
    tick_clk_hz`` seconds that is

        phi = phase_incs_rx * (Dt / fft_period_s)
            = phase_incs_rx * compensate_rx_ticks * fft_rbw_hz / tick_clk_hz

    (since ``fft_period_s = 1 / fft_rbw_hz``). Equivalent to
    ``2*pi * rx_freq_offset_hz * Dt``. ``compensate_rx_ticks`` is counted in
    ``tick_clk_hz`` (307.2 MHz fabric clock) ticks, NOT adc_clk_hz. Added to the
    RX phase offsets only; the TX path is left untouched.

    Returns 0.0 (a scalar that broadcasts) when no compensation is requested, so
    callers can add it unconditionally without changing the zero-tick result.
    """
    if not compensate_rx_ticks:
        return 0.0
    return phase_incs_rx * compensate_rx_ticks * fft_rbw_hz / tick_clk_hz

def _wrap_bin_offsets_for_nco(bin_offsets):
    """Wrap bin offsets to the fine-mixer NCO's one-bin phase domain.

    The fine-mixer phase step is sampled once per FFT period, so offsets that
    differ by an integer FFT bin program the same NCO word. Keep diagnostics in
    the raw drift domain, but use this wrapped residual when generating control
    words and RX path-delay compensation.
    """
    return (np.asarray(bin_offsets, dtype=float) + 0.5) % 1.0 - 0.5

def _bin_spacing_hz(bin_centers_hz):
    """Return the uniform spacing of an FFT bin-centre grid in Hz."""
    centers = np.asarray(bin_centers_hz, dtype=float).ravel()
    if centers.size < 2:
        raise ValueError('Need at least two FFT bin centres to determine spacing')
    diffs = np.diff(np.sort(centers))
    diffs = diffs[diffs > 0]
    if diffs.size == 0:
        raise ValueError('FFT bin-centre grid has no positive spacing')
    return float(np.min(diffs))

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


# v7.11: the design has a single accumulator, at r.accumulators[0] (accnum defaults to 0).
def _wait_for_acc(r,accnum=0,poll_period_s=0.1):
    return r.accumulators[accnum]._wait_for_acc(poll_period_s)

def _blocking_sleep(duration, get_now=time.perf_counter):
    now = get_now()
    end = now + duration
    while now < end:
        now = get_now()


def _blocking_wait_for_acc(acc,poll_period_s=0.1,timeout=1):
    """
    Function to wait for the next accumulation.
    Uses a blocking sleep method to improve performance (by reducing context switches?).
    If no acc counts detected within timeout seconds, raises TimeoutError.
    """
    t0=0
    cnt0 = acc.get_acc_cnt()
    cnt1 = acc.get_acc_cnt()
    # Counter overflow protection
    if cnt1 < cnt0:
        cnt1 += 2**32
    while cnt1 < ((cnt0+1) % (2**32)):
        # #time.sleep(poll_period_s)
        _blocking_sleep(poll_period_s)
        t0+=poll_period_s
        if t0 > timeout:
            raise TimeoutError(f'Timeout waiting for accumulation (firmware_lib._blocking_wait_for_acc: {timeout}s)')
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

def needs_programming(r,config_dict, verbose=False):

    if r is None:
        if verbose:
            _firmware_log('programming required: readout interface is missing', source='ready')
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

    if verbose:
        _firmware_log(f'programming check: current={currentfpg}, requested={newfpg}', source='ready')

    if not r.fpga.is_programmed():
        if verbose:
            _firmware_log('programming required: FPGA is not programmed', source='ready')
        return True

    # The souk_mkid_readout constructor only builds its firmware block interfaces
    # (sync, autocorr, accumulators, ...) when the board is running supported
    # firmware. If that failed - e.g. the PL holds a stale/unsupported image even
    # though is_programmed() reports True - the readout object is left with an
    # empty blocks dict, and shared/pipeline init would later crash accessing
    # e.g. r.sync. Treat that as "needs (re)programming" so we reprogram first.
    if not getattr(r, 'blocks', None):
        if verbose:
            _firmware_log('programming required: firmware blocks not created (unprogrammed or unsupported firmware)', source='ready')
        return True

    if newfpg != currentfpg:
        if verbose:
            _firmware_log('programming required: current FPG does not match config', source='ready')
        return True

    #if not hasattr(r, 'accumulators'):
    #    # catches certain rare cases
    #    print('yes, accumulators not found')
    #    return True

    if verbose:
        _firmware_log('programming check: ready', source='ready')
    return False

def needs_shared_resource_initialising(r, config_dict, verbose=False):
    """
    True if shared resources need initialising.

    checks autocorr acc_len
      - r.autocorr.get_acc_len() == 0 => not initialised
    """
    if r is None or not hasattr(r, "autocorr"):
        if verbose:
            _firmware_log('shared resources require initialisation: autocorr block missing', source='ready')
        return True

    autocorr_acc_len = r.autocorr.get_acc_len()
    if autocorr_acc_len==0:
        if verbose:
            _firmware_log('shared resources require initialisation: autocorr acc_len is zero', source='ready')
        return True

    if verbose:
        _firmware_log('shared resources check: ready', source='ready')
    return False

def needs_pipeline_initialising(r, config_dict, verbose=False):
    """
    True if pipeline resources need initialising.

    checks pipeline accumulator acc_len
      - r.mixer.get_acc_len() == 0 => not initialised
    """
    if r is None or not hasattr(r, "accumulators") or len(r.accumulators) == 0:
        if verbose:
            _firmware_log('pipeline resources require initialisation: accumulators missing', source='ready')
        return True

    acc_len = r.mixer.get_acc_len()
    if acc_len == 0:
        if verbose:
            _firmware_log('pipeline resources require initialisation: accumulator acc_len is zero', source='ready')
        return True

    if verbose:
        _firmware_log('pipeline resources check: ready', source='ready')
    return False

def needs_initialising(r,config_dict, verbose=False):
    """
    Deprecated function, use needs_shared_resource_initialising and needs_pipeline_initialising instead.
    """
    if verbose:
        _firmware_log('needs_initialising is deprecated; use shared/pipeline checks instead', source='ready')
    needs_initialising_shared = needs_shared_resource_initialising(r, config_dict, verbose=verbose)
    needs_initialising_pipeline = needs_pipeline_initialising(r, config_dict, verbose=verbose)
    return needs_initialising_shared or needs_initialising_pipeline


class ClockFault(RuntimeError):
    """Raised when the PLL clock tree is not locked and firmware access must stop.

    Subclasses RuntimeError so existing ``except (ValueError, RuntimeError)``
    handlers still catch it. Carries the clock classification so callers can
    distinguish an LMK (reference) fault - which usually means the PL power rail
    is down and a power cycle is needed - from an LMX (downstream PLL) fault,
    which a deprogram + clock re-init + reprogram cycle can recover.
    """

    def __init__(self, message, fault=None, status=None):
        super().__init__(message)
        self.fault = fault      # 'lmk' | 'lmx' | None
        self.status = status    # raw classify_clock_status() dict


def classify_clock_status(status=None):
    """Classify PLL lock state, separating the LMK reference from the LMX PLLs.

    Pure read - shells out to ``krc-utils status`` (PS-side I2C/SPI) and never
    touches the PL fabric / AXI bus, so it is safe to call even when the fabric
    clock is dead.

    Parameters
    ----------
    status : dict, optional
        A pre-fetched :func:`get_clock_status` result. Fetched if omitted.

    Returns
    -------
    dict
        ``{'all_locked', 'lmk_locked', 'lmx_locked', 'fault', 'chips'}`` where
        ``fault`` is ``'lmk'`` (reference unlocked), ``'lmx'`` (a downstream PLL
        unlocked while the LMK is locked), or ``None`` (all locked). If no chips
        are reported (krc-utils unavailable/timed out) everything is treated as
        unlocked, i.e. ``fault == 'lmk'``.
    """
    if status is None:
        status = get_clock_status()
    chips = status.get('chips', [])
    lmk_chips = [c for c in chips if str(c.get('name', '')).lower().startswith('lmk')]
    lmx_chips = [c for c in chips if str(c.get('name', '')).lower().startswith('lmx')]
    lmk_locked = bool(lmk_chips) and all(c.get('status') == 'locked' for c in lmk_chips)
    lmx_locked = bool(lmx_chips) and all(c.get('status') == 'locked' for c in lmx_chips)
    all_locked = bool(status.get('all_locked', False)) and lmk_locked and lmx_locked
    if not lmk_locked:
        fault = 'lmk'
    elif not lmx_locked:
        fault = 'lmx'
    else:
        fault = None
    return {
        'all_locked': all_locked,
        'lmk_locked': lmk_locked,
        'lmx_locked': lmx_locked,
        'fault': fault,
        'chips': chips,
    }


def _configured_clock_source(config_dict):
    """Return the configured clock source, accepting the legacy location."""
    source = config_dict.get('firmware', {}).get(
        'clock_source',
        config_dict.get('rfsoc_host', {}).get('clock_source')
    )
    if source is None:
        return None
    return str(source).strip().lower()


def apply_clock_config(config_dict, max_attempts=3):
    """
    Check the clocks and re-init them only if necessary - MUST run on a blank PL.

    Reads the desired clock_source from the config (firmware section, falling
    back to rfsoc_host for older configs). If the selected source already
    matches and all PLLs are locked, returns immediately without running
    ``krc-utils init``. Otherwise it applies/re-applies the clock configuration
    up to ``max_attempts`` times.

    WARNING: running ``krc-utils init`` while the PL is programmed and loaded
    with tones collapses the PL power rail and hangs the PS (issue #14). This
    function is only safe to call once the PL has been deprogrammed - it is
    invoked from :func:`reload_firmware` immediately after :func:`deprogram_fpga`.

    Parameters
    ----------
    config_dict : dict
        The system configuration dictionary.
    max_attempts : int
        Maximum number of times to apply the clock config before giving up.

    Returns
    -------
    dict
        Clock status dict (see :func:`get_clock_status`).

    Raises
    ------
    ClockFault
        If clocks cannot be locked after retries.
    """
    max_attempts = int(max_attempts)
    if max_attempts < 1:
        raise ValueError('max_attempts must be >= 1')

    desired_clock = _configured_clock_source(config_dict)
    current_clock = get_clock_source()

    if desired_clock is not None and current_clock != desired_clock:
        _firmware_log(
            f'clock source mismatch: config={desired_clock}, '
            f'current={current_clock}; applying configured source',
            source='clock',
        )
    else:
        status = get_clock_status()
        if status.get('all_locked', False):
            _firmware_log('clocks locked', source='clock')
            return status

    reapply_clock = desired_clock or current_clock
    status = None
    last_error = None
    for attempt in range(max_attempts):
        _firmware_log(
            f'clocks not locked (attempt {attempt + 1}/{max_attempts}); '
            f're-applying clock source: {reapply_clock}',
            source='clock',
        )
        try:
            status = set_clock_source(reapply_clock)
        except (ValueError, FileNotFoundError) as exc:
            last_error = exc
            _firmware_log(f'failed to set clock source: {exc}', source='clock')
            continue
        if status.get('all_locked', False):
            _firmware_log('clocks locked after re-init', source='clock')
            return status

    # Still not locked
    suggestion = ""
    if desired_clock == 'external':
        suggestion = (" Config specifies 'external' clock — if no 10 MHz reference is "
                      "connected, change firmware.clock_source to 'internal'.")
    elif desired_clock == 'internal':
        suggestion = (" Config specifies 'internal' clock — if the on-board oscillator "
                      "is not functioning, check hardware.")
    error_note = f' Last error: {last_error}.' if last_error is not None else ''
    cls = classify_clock_status(status)
    raise ClockFault(
        f'Clock check failed: clocks are not locked after {max_attempts} '
        f'attempt(s) to set clock source {reapply_clock!r}. Server remains at '
        f'server level; firmware interfaces were not created. '
        f'Status: {status}.{error_note}{suggestion}',
        fault=cls['fault'],
        status=cls,
    )


# Kernel FPGA manager - used to reset the PL to its base bitstream from the PS
# side without constructing a casperfpga interface (and without needing the
# fabric clock). The firmware node takes a filename only (resolved against
# /lib/firmware); the base image is always tcpborphserver's locally stored copy.
FPGA_MANAGER_FIRMWARE_NODE = '/sys/class/fpga_manager/fpga0/firmware'
FPGA_MANAGER_BASE_BITSTREAM = 'tcpborphserver.bin'


def deprogram_fpga(r=None):
    """Reset the PL to its base image (tcpborphserver), clearing all tones.

    The base image holds no souk DSP, so it draws minimal current - the safe
    precondition for a clock re-init (issue #14: running ``krc-utils init`` while
    the souk pipeline is loaded with tones collapses the PL power rail and hangs
    the PS). It also keeps a PL image in place, which the clock-forwarding paths
    need to function.

    If a live readout interface ``r`` is supplied its casperfpga transport is
    used; otherwise - or if that path is unresponsive - the kernel FPGA manager
    is driven directly from the PS. The FPGA manager route avoids the AXI
    register traffic of the casperfpga path (which would hang if the fabric were
    unresponsive) and never constructs a :class:`SoukMkidReadout` (construction
    reads registers and would hang for the same reason).
    """
    if r is not None:
        try:
            _firmware_log('resetting PL to base image via casperfpga', source='program')
            r.fpga.host.deprogram()
            return
        except Exception as exc:
            _firmware_log(
                f'casperfpga deprogram failed ({exc}); falling back to FPGA manager',
                source='program',
            )

    _firmware_log(
        f'deprogramming FPGA via FPGA manager ({FPGA_MANAGER_BASE_BITSTREAM})',
        source='program',
    )
    with open(FPGA_MANAGER_FIRMWARE_NODE, 'w') as node:
        node.write(FPGA_MANAGER_BASE_BITSTREAM + '\n')


def reload_firmware(config_dict, r=None):
    """
    Program/reprogram and return interfaces.

    Safe sequence (issue #14): reset the PL to its base image FIRST so the clock
    re-init runs against a PL holding zero tones (minimal current), then check/
    re-init the clocks only if necessary, then program the souk design. Running
    ``krc-utils init`` while the souk pipeline is loaded with tones collapses the
    PL power rail and hangs the PS, so the reset-to-base must happen before any
    clock work.

    ``r`` is the current readout interface, if any: its casperfpga transport is
    used for the reset-to-base, falling back to the kernel FPGA manager (which
    avoids the AXI register traffic of the casperfpga path). When ``r`` is None
    (e.g. recovering from a latched clock fault) the FPGA manager is used
    directly, without constructing a SoukMkidReadout (construction reads
    registers and would hang if the fabric were unresponsive).

    IMPORTANT: This and any second pipeline will need initialising. Do not initialise shared or pipeline resources here.
    """
    _firmware_log('reloading firmware; shared and pipeline resources will need reinitialising', source='program')

    fw_config_file = config_dict['firmware']['fw_config_file']
    pipeline_id = config_dict['firmware']['pipeline_id']

    # Acquire cross-pipeline file lock to prevent concurrent programming.
    # fcntl.flock is automatically released if the process crashes.
    os.makedirs(os.path.dirname(FPGA_PROGRAM_LOCK), exist_ok=True)
    lock_fd = open(FPGA_PROGRAM_LOCK, 'w')
    try:
        _firmware_log(f'waiting for FPGA programming lock: {FPGA_PROGRAM_LOCK}', source='program')
        fcntl.flock(lock_fd, fcntl.LOCK_EX)
        _firmware_log('FPGA programming lock acquired', source='program')

        # 1) Reset the PL to its base image FIRST so it holds zero tones (minimal
        #    current) before we touch the clock tree.
        deprogram_fpga(r)
        time.sleep(1)

        # 2) Check clocks and re-init only if necessary - safe now the PL is blank.
        apply_clock_config(config_dict)

        # 3) Build interfaces and program the souk design.
        r = create_standard_readout_interface(fw_config_file,pipeline_id=pipeline_id)
        r_fast = create_fast_readout_interface(fw_config_file,pipeline_id=pipeline_id)
        _firmware_log('programming FPGA', source='program')
        r.program()
        time.sleep(1)
        fw_type = r.fpga.get_firmware_type()
        if fw_type==2:
            if pipeline_id!=0:
                raise ValueError(f'Pipeline ID {pipeline_id} does not exist in type 2 single pipeline firmware: {fw_config_file}')
        elif fw_type==3:
            if pipeline_id not in [0,1]:
                raise ValueError(f'Pipeline ID {pipeline_id} does not exist in type 3 dual pipeline firmware: {fw_config_file}')
    finally:
        fcntl.flock(lock_fd, fcntl.LOCK_UN)
        lock_fd.close()
        _firmware_log('FPGA programming lock released', source='program')

    return r, r_fast



def initialise_shared_resources(r,config_dict):
    """
    Initialise shared firmware blocks with cross-pipeline file lock.

    On dual-pipeline systems both servers may call this concurrently.
    The lock ensures only one pipeline performs the initialisation;
    the second re-checks and skips if it has already been done.
    """
    _shared_block_names = ['common', 'adc_snapshot', 'dac_snapshot', 'zoomfft', 'zoomacc', 'gen_cordic', 'gen_lut', 'autocorr']

    # Acquire cross-pipeline file lock to prevent concurrent shared resource init.
    # fcntl.flock is automatically released if the process crashes.
    os.makedirs(os.path.dirname(SHARED_INIT_LOCK), exist_ok=True)
    lock_fd = open(SHARED_INIT_LOCK, 'w')
    try:
        _firmware_log(f'waiting for shared resource init lock: {SHARED_INIT_LOCK}', source='shared')
        fcntl.flock(lock_fd, fcntl.LOCK_EX)
        _firmware_log('shared resource init lock acquired', source='shared')

        # Re-check inside the lock: another pipeline may have already initialised.
        if not needs_shared_resource_initialising(r, config_dict):
            _firmware_log('shared resources already initialised by another pipeline; skipping', source='shared')
            return

        #read from config
        ## no common block configurations in use right now

        #initialise and setup blocks
        _firmware_log('initialising shared resources', source='shared')
        r.initialize_shared_blocks()

        #nothing to setup right now
    finally:
        fcntl.flock(lock_fd, fcntl.LOCK_UN)
        lock_fd.close()
        _firmware_log('shared resource init lock released', source='shared')

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
    adc_ddc_mixer_frequency_hz = defaults.get(
        'adc_ddc_mixer_frequency_hz',
        defaults.get('adc_ddc_mix_frequency_hz', None),
    )
    nyquist_zone = defaults.get('nyquist_zone',None)
    dac_mixer_scale_1p0 = defaults.get('dac_mixer_scale_1p0',None)
    adc_mixer_scale_1p0 = defaults.get('adc_mixer_scale_1p0',None)
    dsa = defaults.get('dsa',None)
    vop = defaults.get('vop',None)
    internal_loopback = defaults.get('internal_loopback',None)
    psb_scale = defaults.get('psb_scale',None)
    psb_fftshift = defaults.get('psb_fftshift',None)
    pfb_fftshift = defaults.get('pfb_fftshift',None)
    tone_plan = get_configured_tone_plan(config_dict)
    frequencies = tone_plan['frequencies']
    amplitudes = tone_plan['amplitudes']
    phases = tone_plan['phases']

    #initialise and setup blocks
    _firmware_log(f'initialising pipeline resources for pipeline {pipeline_id}', source='pipeline')
    if not getattr(r, 'blocks', None):
        raise RuntimeError(
            'cannot initialise pipeline resources: firmware block interfaces were '
            'not created. The board is unprogrammed or running firmware unsupported '
            'by this souk_mkid_readout version; (re)program with matching firmware first.'
        )
    r.initialize_pipeline_blocks()

    r.output.use_psb()

    if sync_delay is not None:
        _warn_sync_delay_deprecated(sync_delay)  # v7.11: mixer owns RX/TX delay (set at init)
    if acc_len is not None:
        r.mixer.set_acc_len(acc_len)  # v7.11: acc_len lives in the mixer
        r.sync.sw_sync(mrst=True)     # acc_len change needs a master-reset + sync
    if dac_duc_mixer_frequency_hz is not None:
        r.rfdc.core.set_fine_mixer_freq(dac0_tile,dac0_block,r.rfdc.core.DAC_TILE,dac_duc_mixer_frequency_hz/1e6)
        r.rfdc.core.set_fine_mixer_freq(dac1_tile,dac1_block,r.rfdc.core.DAC_TILE,dac_duc_mixer_frequency_hz/1e6)
    if adc_ddc_mixer_frequency_hz is not None:
        r.rfdc.core.set_fine_mixer_freq(adc_tile,adc_block,r.rfdc.core.ADC_TILE,adc_ddc_mixer_frequency_hz/1e6)
    if nyquist_zone is not None:
        set_nyquist_zone(r,config_dict,nyquist_zone)
        set_dac_inverse_sinc_filter(r, config_dict, inv_sinc=None)
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
    if len(frequencies) > 0:
        try:
            r_fast.mixer.host.transport.axil_mm
            r_fast.mixer.host.transport._get_device_address
            set_tone_frequencies_fast(
                r, r_fast, config_dict, frequencies,
                tone_amplitudes=amplitudes, tone_phases=phases)
        except AttributeError:
            set_tone_frequencies(
                r, config_dict, frequencies,
                tone_amplitudes=amplitudes, tone_phases=phases)
    if amplitudes is not None and len(amplitudes) > 0:
        set_tone_amplitudes(r, config_dict, amplitudes)
    if phases is not None and len(phases) > 0:
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


# ---------------------------------------------------------------------------
# Structured info section helpers (used by server get_info dispatcher)
# ---------------------------------------------------------------------------

def info_versions():
    """Software versions and git commit IDs."""
    info = {}
    try:
        import importlib.metadata
        info['souk_readout_tools_version'] = importlib.metadata.version('souk_readout_tools')
    except Exception:
        info['souk_readout_tools_version'] = None
    try:
        info['souk_mkid_readout_sw_version'] = souk_mkid_readout.__version__
    except (NameError, AttributeError):
        info['souk_mkid_readout_sw_version'] = None
    try:
        info['souk_mkid_readout_fw_version'] = souk_mkid_readout.__fwversion__
    except (NameError, AttributeError):
        info['souk_mkid_readout_fw_version'] = None
    info['souk_readout_tools_commit'] = _get_git_commit('/home/casper/souk_readout_tools')
    info['souk_firmware_commit'] = _get_git_commit('/home/casper/souk-firmware')
    info['souk_peripherals_commit'] = _get_git_commit('/home/casper/souk_readout_tools/src/souk_readout_tools/server/souk-peripherals-control')
    return {'ready': True, **info}


def info_clock():
    """Clock source and PLL lock status, split into LMK reference vs LMX PLLs."""
    source = get_clock_source()
    cls = classify_clock_status()
    return {
        'ready': True,
        'source': source,
        'all_locked': cls['all_locked'],
        'lmk_locked': cls['lmk_locked'],
        'lmx_locked': cls['lmx_locked'],
        'fault': cls['fault'],
        'chips': cls['chips'],
    }


def info_fpga(r):
    """FPGA programming state and base parameters."""
    programmed = r is not None and r.fpga.is_programmed()
    if not programmed:
        return {'ready': False, 'fpga_status': None, 'fpg_file': None,
                'pipeline_id': None, 'adc_clk_hz': None}
    return {
        'ready': True,
        'fpga_status': r.fpga.get_status()[0],
        'fpg_file': r.fpgfile,
        'pipeline_id': r.pipeline_id,
        'adc_clk_hz': r.adc_clk_hz,
    }


def info_rfdc(r, config_dict):
    """RFDC settings — needs FPGA programmed."""
    dac0_tile = config_dict['firmware']['dac0_tile']
    dac0_block = config_dict['firmware']['dac0_block']
    dac1_tile = config_dict['firmware']['dac1_tile']
    dac1_block = config_dict['firmware']['dac1_block']
    adc_tile = config_dict['firmware']['adc_tile']
    adc_block = config_dict['firmware']['adc_block']

    has_rfdc = r is not None and hasattr(r, 'rfdc')
    correct_adc = has_rfdc and r.rfdc.core.get_dsa(adc_tile, adc_block).get('dsa', None) is not None
    correct_dac0 = has_rfdc and r.rfdc.core.get_output_current(dac0_tile, dac0_block).get('current', None) is not None
    correct_dac1 = has_rfdc and r.rfdc.core.get_output_current(dac1_tile, dac1_block).get('current', None) is not None

    if not (correct_adc and correct_dac0 and correct_dac1):
        if not has_rfdc:
            print(bcolors.FAIL + 'CRITICAL WARNING - RFDC block not found, the FPGA may not be programmed' + bcolors.ENDC)
        else:
            print(bcolors.FAIL + 'CRITICAL WARNING - RFDC settings not found, check that the DAC and ADC tiles/blocks are set correctly in the config to match the firmware' + bcolors.ENDC)
        return {
            'ready': False,
            'dsa': 0, 'vop_dac0': 0, 'vop_dac1': 0,
            'dac_duc_mixer_frequency_hz': 0, 'adc_ddc_mixer_frequency_hz': 0,
            'nyquist_zone_adc': 1, 'nyquist_zone_dac0': 1, 'nyquist_zone_dac1': 1,
            'inverse_sinc_fir_mode_dac0': None, 'inverse_sinc_fir_mode_dac1': None,
            'inverse_sinc_filter_enabled_dac0': None,
            'inverse_sinc_filter_enabled_dac1': None,
            'mixer_scale_1p0_dac0': None, 'mixer_scale_1p0_dac1': None, 'mixer_scale_1p0_adc': None,
            'qmc_settings_dac0': None, 'qmc_settings_dac1': None, 'qmc_settings_adc': None,
            'adc_cal_frozen': None, 'rts_events': {'rts_available': False},
        }

    rts_event, rts_details = check_rfdc_rts_events(r, clear=False)
    inverse_sinc_mode_dac0 = r.rfdc.core.get_invsinc_fir(
        dac0_tile, dac0_block)
    inverse_sinc_mode_dac1 = r.rfdc.core.get_invsinc_fir(
        dac1_tile, dac1_block)
    return {
        'ready': True,
        'dsa': r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'],
        'vop_dac0': r.rfdc.core.get_output_current(dac0_tile, dac0_block)['current'],
        'vop_dac1': r.rfdc.core.get_output_current(dac1_tile, dac1_block)['current'],
        'dac_duc_mixer_frequency_hz': float(r.rfdc.core.get_mixer_settings(dac0_tile, dac0_block, r.rfdc.core.DAC_TILE)['Freq']) * 1e6,
        'adc_ddc_mixer_frequency_hz': float(r.rfdc.core.get_mixer_settings(adc_tile, adc_block, r.rfdc.core.ADC_TILE)['Freq']) * 1e6,
        'nyquist_zone_adc': r.rfdc.core.get_nyquist_zone(adc_tile, adc_block, r.rfdc.core.ADC_TILE),
        'nyquist_zone_dac0': r.rfdc.core.get_nyquist_zone(dac0_tile, dac0_block, r.rfdc.core.DAC_TILE),
        'nyquist_zone_dac1': r.rfdc.core.get_nyquist_zone(dac1_tile, dac1_block, r.rfdc.core.DAC_TILE),
        'inverse_sinc_fir_mode_dac0': inverse_sinc_mode_dac0,
        'inverse_sinc_fir_mode_dac1': inverse_sinc_mode_dac1,
        'inverse_sinc_filter_enabled_dac0': (
            None if inverse_sinc_mode_dac0 is None
            else inverse_sinc_mode_dac0 != r.rfdc.core.INVSINC_FIR_DISABLED
        ),
        'inverse_sinc_filter_enabled_dac1': (
            None if inverse_sinc_mode_dac1 is None
            else inverse_sinc_mode_dac1 != r.rfdc.core.INVSINC_FIR_DISABLED
        ),
        'mixer_scale_1p0_dac0': r.rfdc.core.get_mixer_settings(dac0_tile, dac0_block, r.rfdc.core.DAC_TILE)['FineMixerScale'] == r.rfdc.core.MIX_SCALE_1P0,
        'mixer_scale_1p0_dac1': r.rfdc.core.get_mixer_settings(dac1_tile, dac1_block, r.rfdc.core.DAC_TILE)['FineMixerScale'] == r.rfdc.core.MIX_SCALE_1P0,
        'mixer_scale_1p0_adc': r.rfdc.core.get_mixer_settings(adc_tile, adc_block, r.rfdc.core.ADC_TILE)['FineMixerScale'] == r.rfdc.core.MIX_SCALE_1P0,
        'qmc_settings_dac0': r.rfdc.core.get_qmc_settings(dac0_tile, dac0_block, r.rfdc.core.DAC_TILE),
        'qmc_settings_dac1': r.rfdc.core.get_qmc_settings(dac1_tile, dac1_block, r.rfdc.core.DAC_TILE),
        'qmc_settings_adc': r.rfdc.core.get_qmc_settings(adc_tile, adc_block, r.rfdc.core.ADC_TILE),
        'adc_cal_frozen': get_cal_freeze(r, config_dict),
        'rts_events': rts_details,
    }


def info_pipeline(r):
    """DSP pipeline block parameters — needs pipeline init."""
    pipeline_ready = (r is not None and hasattr(r, 'accumulators')
                      and len(r.accumulators) > 0 and r.mixer.get_acc_len() > 0)
    if not pipeline_ready:
        return {
            'ready': False,
            'output_mode': None, 'sync_delay': None,
            'tx_rx_skew': None, 'buffer_switch_skew': None,
            'internal_loopback': None,
            'psb_scale': None, 'psb_fftshift': None, 'pfb_fftshift': None,
            'acc_len': None, 'acc_freq_hz': None,
        }
    return {
        'ready': True,
        'output_mode': r.output.get_status()[0]['mode'],
        # v7.11: the mixer manages the RX/TX sync timing. Report the RX sync delay
        # (sync_delay), the measured TX->RX skew, and the applied RX LO buffer-switch
        # skew (rx_delay) -- all in FPGA clock cycles.
        'sync_delay': r.mixer.read_uint('sync_delay'),
        'tx_rx_skew': r.mixer.get_tx_rx_skew(),
        'buffer_switch_skew': r.mixer.read_uint('rx_delay'),
        'internal_loopback': r.input.loopback_enabled(),
        'psb_scale': r.psbscale.get_scale(),
        'psb_fftshift': r.psb.get_fftshift(),
        'pfb_fftshift': r.pfb.get_fftshift(),
        'acc_len': r.mixer.get_acc_len(),
        'acc_freq_hz': get_sample_rate(r),
    }


def _is_present_config_value(value):
    if value is None:
        return False
    if isinstance(value, (list, tuple, np.ndarray)) and len(value) == 0:
        return False
    return True


def _as_1d_float_array(value, name):
    if not _is_present_config_value(value):
        return np.array([], dtype=float)
    try:
        return np.atleast_1d(value).astype(float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be numeric') from exc


def _normalise_config_tone_values(values, n_values, name, allow_scalar=True):
    arr = _as_1d_float_array(values, name)
    if arr.size == 0:
        return arr
    if arr.size == n_values:
        return arr
    if allow_scalar and arr.size == 1 and n_values > 1:
        return np.full(n_values, float(arr[0]), dtype=float)
    raise ValueError(
        f'Number of {name} values ({arr.size}) must match number of tones '
        f'({n_values})')


def _normalise_split_tone_values(values, blind_values, n_regular, n_blind,
                                 name, default_missing):
    """Combine regular/blind per-tone config values.

    ``values`` normally describes regular tones and ``blind_values`` describes
    blind tones.  For convenience, ``values`` may also already contain the full
    combined list when ``blind_values`` is absent.
    """
    total = n_regular + n_blind
    values_present = _is_present_config_value(values)
    blind_present = _is_present_config_value(blind_values)
    if total == 0 or not values_present and not blind_present:
        return None

    primary = _as_1d_float_array(values, name)
    if values_present and n_blind and not blind_present and primary.size == total:
        return primary

    if values_present:
        primary = _normalise_config_tone_values(primary, n_regular, name)
    elif n_regular:
        primary = np.full(n_regular, default_missing, dtype=float)
    else:
        primary = np.array([], dtype=float)

    if blind_present:
        blind = _normalise_config_tone_values(
            blind_values, n_blind, f'blind_{name}')
    elif n_blind:
        blind = np.full(n_blind, default_missing, dtype=float)
    else:
        blind = np.array([], dtype=float)

    return np.concatenate([primary, blind])


def get_configured_tone_plan(config_dict):
    """Return the config tone plan, including configured blind tones.

    The firmware still receives a single combined tone list.  The returned
    metadata records which user-facing indices are regular tones and which are
    blind tones so higher-level control loops can treat them differently.
    """
    defaults = (
        config_dict.get('firmware', {}).get('defaults', {}) or {}
    )
    regular_frequencies = _as_1d_float_array(
        defaults.get('frequencies', []), 'frequencies')
    blind_frequencies = _as_1d_float_array(
        defaults.get('blind_frequencies', []), 'blind_frequencies')

    n_regular = len(regular_frequencies)
    n_blind = len(blind_frequencies)
    frequencies = np.concatenate([regular_frequencies, blind_frequencies])

    amplitudes = _normalise_split_tone_values(
        defaults.get('amplitudes', []),
        defaults.get('blind_amplitudes', []),
        n_regular, n_blind, 'amplitudes', 1.0)
    phases = _normalise_split_tone_values(
        defaults.get('phases', []),
        defaults.get('blind_phases', []),
        n_regular, n_blind, 'phases', 0.0)

    blind_spans = _normalise_config_tone_values(
        defaults.get('blind_spans', []), n_blind, 'blind_spans')

    regular_indices = list(range(n_regular))
    blind_indices = list(range(n_regular, n_regular + n_blind))
    tone_types = ['regular'] * n_regular + ['blind'] * n_blind
    is_blind = [False] * n_regular + [True] * n_blind

    return {
        'frequencies': frequencies,
        'regular_frequencies': regular_frequencies,
        'blind_frequencies': blind_frequencies,
        'amplitudes': amplitudes,
        'phases': phases,
        'blind_spans': blind_spans,
        'tone_types': tone_types,
        'is_blind': is_blind,
        'regular_indices': regular_indices,
        'blind_indices': blind_indices,
        'num_regular_tones': n_regular,
        'num_blind_tones': n_blind,
        'num_tones': n_regular + n_blind,
    }


def get_configured_tone_metadata(config_dict, active_count=None):
    """Return user-facing tone metadata for the active tone list.

    If the live tone count no longer matches the configured regular+blind tone
    plan, the metadata falls back to treating all active tones as regular
    tones.  This avoids incorrectly freezing arbitrary user-set tones.
    """
    plan = get_configured_tone_plan(config_dict)
    configured_count = plan['num_tones']
    if active_count is None:
        active_count = configured_count
    active_count = int(active_count)

    if configured_count == active_count:
        tone_types = plan['tone_types']
        is_blind = plan['is_blind']
        regular_indices = plan['regular_indices']
        blind_indices = plan['blind_indices']
    else:
        tone_types = ['regular'] * active_count
        is_blind = [False] * active_count
        regular_indices = list(range(active_count))
        blind_indices = []

    return {
        'tone_types': tone_types,
        'is_blind': is_blind,
        'regular_indices': regular_indices,
        'blind_indices': blind_indices,
        'num_regular_tones': len(regular_indices),
        'num_blind_tones': len(blind_indices),
        'metadata_matches_config': configured_count == active_count,
        'configured_num_tones': configured_count,
    }


def info_tones(r, r_fast, config_dict, rf_peripherals=None, reference_plane='detector'):
    """Tone frequencies, amplitudes, phases, powers, and firmware indices."""
    pipeline_ready = (r is not None and hasattr(r, 'accumulators')
                      and len(r.accumulators) > 0 and r.mixer.get_acc_len() > 0)
    if not pipeline_ready:
        return {
            'ready': False,
            'count': 0, 'frequencies_hz': None, 'amplitudes': None,
            'phases_rad': None, 'powers_dbm': None, 'firmware_indices': None,
            'detailed_frequency_info': None,
        }
    freqs_detailed = get_tone_frequencies(r, r_fast, config_dict, detailed_output=True)
    freqs = freqs_detailed[0]
    details = freqs_detailed[1]
    amps = get_tone_amplitudes(r, r_fast, config_dict)
    phases = get_tone_phases(r, r_fast, config_dict)
    try:
        powers = get_tone_powers(
            r, r_fast, config_dict, reference_plane=reference_plane,
            rf_peripherals=rf_peripherals)
    except Exception:
        powers = np.array([])
    metadata = get_configured_tone_metadata(config_dict, active_count=len(freqs))
    tone_plan = get_configured_tone_plan(config_dict)
    blind_spans = (
        tone_plan['blind_spans'].tolist()
        if metadata['metadata_matches_config'] else []
    )
    return {
        'ready': True,
        'count': len(freqs),
        'frequencies_hz': freqs.tolist(),
        'amplitudes': amps.tolist(),
        'phases_rad': phases.tolist(),
        'powers_dbm': powers.tolist() if len(powers) > 0 else None,
        'powers_reference_plane': reference_plane,
        'firmware_indices': details['rx']['tone_indices'],
        'blind_spans': blind_spans,
        **metadata,
        'detailed_frequency_info': details,
    }


def info_diagnostics(r, r_fast, config_dict):
    """Saturation, overflow, and signal levels (expensive — captures snapshots)."""
    pipeline_ready = (r is not None and hasattr(r, 'accumulators')
                      and len(r.accumulators) > 0 and r.mixer.get_acc_len() > 0)
    if not pipeline_ready:
        return {'ready': False, 'adc_saturation': None, 'dac_saturation': None, 'dsp_overflow': None}

    adc_sat, adc_details = check_input_saturation(r, r_fast, iterations=25, verbose=False)
    dac_sat, dac_details = check_output_saturation(r_fast, iterations=25, verbose=False)
    dsp_ovf, dsp_details = check_dsp_overflow(r, duration_s=0.1, verbose=False)
    return {
        'ready': True,
        'adc_saturation': {'saturated': adc_sat, **adc_details},
        'dac_saturation': {'saturated': dac_sat, **dac_details},
        'dsp_overflow': {'overflow': dsp_ovf, **dsp_details},
    }


def info_rfsoc_sensors(sensor_path='/sys/bus/iio/devices/iio:device0/'):
    """RFSoC on-chip IIO sensor readings (temperatures in C, voltages in V).

    Reads every ``in_*_raw`` entry exposed by the PS/PL SYSMON via IIO and
    converts it using the accompanying ``_scale`` and ``_offset`` sysfs files
    (offset applies only to temperature sensors).  Sensor keys are the raw
    sysfs names (minus ``in_`` and ``_raw``) so duplicated short names like
    ``vccams`` or ``vccint`` on both PS and PL rails do not collide.
    """
    import glob
    info = {'ready': False, 'available': False, 'sensor_path': sensor_path,
            'temperatures_c': {}, 'voltages_v': {}, 'other': {}}
    if not os.path.isdir(sensor_path):
        return info

    def _read_float(fname):
        with open(fname, 'r') as fh:
            return float(fh.read())

    raw_files = sorted(glob.glob(os.path.join(sensor_path, 'in_*_raw')))
    if not raw_files:
        return info

    info['available'] = True
    for raw_file in raw_files:
        base = os.path.basename(raw_file)[3:-4]  # strip 'in_' and '_raw'
        try:
            raw = _read_float(raw_file)
            scale = _read_float(os.path.join(sensor_path, 'in_' + base + '_scale'))
            offset = 0.0
            if base.startswith('temp'):
                offset = _read_float(os.path.join(sensor_path, 'in_' + base + '_offset'))
            value = scale * (raw + offset) / 1000.0
        except Exception:
            continue
        if base.startswith('temp'):
            info['temperatures_c'][base] = value
        elif base.startswith('voltage'):
            info['voltages_v'][base] = value
        else:
            info['other'][base] = value

    info['ready'] = True
    return info


def info_calibrations(r, r_fast, config_dict):
    """Resolved calibration values currently in effect."""
    # Check if tones are set for per-tone interpolation
    pipeline_ready = (r is not None and hasattr(r, 'accumulators')
                      and len(r.accumulators) > 0 and r.mixer.get_acc_len() > 0)

    info = {
        'ready': True,
        'dac0_dbfs_to_dbm': config_dict['firmware'].get('dac0_dbfs_to_dbm'),
        'dac1_dbfs_to_dbm': config_dict['firmware'].get('dac1_dbfs_to_dbm'),
        'adc_dbm_to_dbfs': config_dict['firmware'].get('adc_dbm_to_dbfs'),
        'vop_current_fullscale': config_dict['firmware'].get('vop_current_fullscale'),
    }

    rf = config_dict.get('rf_frontend', {})
    cryo = config_dict.get('cryostat', {})
    cal_keys_rf = [
        'tx_combiner_loss_db', 'rx_combiner_loss_db',
        'tx_if_s21_db', 'rx_if_s21_db',
        'tx_rf_s21_db', 'rx_rf_s21_db',
        'tx_mixer_conversion_loss_db', 'rx_mixer_conversion_loss_db',
    ]

    if pipeline_ready:
        try:
            freqs = get_tone_frequencies(r, r_fast, config_dict)
            freq_axis = freqs
        except Exception:
            freq_axis = None
    else:
        freq_axis = None

    for key in cal_keys_rf:
        raw = rf.get(key)
        if freq_axis is not None and raw is not None:
            try:
                resolved = _resolve_cal_value(raw, freq_axis)
                info[key] = resolved.tolist() if hasattr(resolved, 'tolist') else resolved
            except Exception:
                info[key] = raw
        else:
            info[key] = raw

    info['cryostat_input_s21_db'] = cryo.get('input_s21_db')
    info['cryostat_output_s21_db'] = cryo.get('output_s21_db')

    mixerless = rf.get('mixerless_module', {}) or {}
    for key in (
        'tx_amp_enabled_s21_db', 'tx_amp_bypassed_s21_db',
        'tx_amp_bypass_delta_s21_db',
        'rx_amp_enabled_s21_db', 'rx_amp_bypassed_s21_db',
        'rx_amp_bypass_delta_s21_db',
        'tx_group_delay_ns', 'rx_group_delay_ns',
    ):
        raw = mixerless.get(key)
        if freq_axis is not None and raw is not None:
            try:
                resolved = _resolve_cal_value(raw, freq_axis)
                info[f'mixerless_module.{key}'] = (
                    resolved.tolist() if hasattr(resolved, 'tolist') else resolved
                )
            except Exception:
                info[f'mixerless_module.{key}'] = raw
        else:
            info[f'mixerless_module.{key}'] = raw
    info['mixerless_module.tx_input_1db_comp_dbm'] = mixerless.get(
        'tx_input_1db_comp_dbm')
    info['mixerless_module.rx_input_1db_comp_dbm'] = mixerless.get(
        'rx_input_1db_comp_dbm')

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
    new_rfsoc_host = new_config_dict.get('rfsoc_host', {})
    if prev_config_dict is not None:
        prev_rfsoc_host = prev_config_dict.get('rfsoc_host', {})
        non_clock_changed = {k for k in set(new_rfsoc_host) | set(prev_rfsoc_host)
                             if k != 'clock_source' and new_rfsoc_host.get(k) != prev_rfsoc_host.get(k)}
        if non_clock_changed:
            print(bcolors.WARNING + 'WARNING: rfsoc_host configuration changed. These parameters cannot be applied remotely. '
                  'Log into the RFSoC directly to make these changes.' + bcolors.ENDC)

    # Clock source is a board-level setting shared across pipelines. Do not
    # re-apply it here on every config push; the server gates firmware access on
    # a pure clock check (classify_clock_status) and only ever runs krc-utils
    # init from the deprogram-first reload_firmware sequence (issue #14).

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
            _warn_sync_delay_deprecated(sync_delay)  # v7.11: mixer owns RX/TX delay

    if changed('acc_len'):
        acc_len = defaults.get('acc_len')
        if acc_len is not None:
            print(f'apply_config: setting acc_len = {acc_len}')
            r.mixer.set_acc_len(acc_len)  # v7.11: acc_len lives in the mixer
            r.sync.sw_sync(mrst=True)     # acc_len change needs a master-reset + sync

    if changed('dac_duc_mixer_frequency_hz'):
        dac_duc_mixer_frequency_hz = defaults.get('dac_duc_mixer_frequency_hz')
        if dac_duc_mixer_frequency_hz is not None:
            print(f'apply_config: setting dac_duc_mixer_frequency_hz = {dac_duc_mixer_frequency_hz}')
            r.rfdc.core.set_fine_mixer_freq(dac0_tile, dac0_block, r.rfdc.core.DAC_TILE, dac_duc_mixer_frequency_hz/1e6)
            r.rfdc.core.set_fine_mixer_freq(dac1_tile, dac1_block, r.rfdc.core.DAC_TILE, dac_duc_mixer_frequency_hz/1e6)

    def changed_any(*keys):
        return any(changed(key) for key in keys)

    if changed_any('adc_ddc_mixer_frequency_hz', 'adc_ddc_mix_frequency_hz'):
        adc_ddc_mixer_frequency_hz = defaults.get(
            'adc_ddc_mixer_frequency_hz',
            defaults.get('adc_ddc_mix_frequency_hz'),
        )
        if adc_ddc_mixer_frequency_hz is not None:
            print(f'apply_config: setting adc_ddc_mixer_frequency_hz = {adc_ddc_mixer_frequency_hz}')
            r.rfdc.core.set_fine_mixer_freq(adc_tile, adc_block, r.rfdc.core.ADC_TILE, adc_ddc_mixer_frequency_hz/1e6)

    if changed('nyquist_zone'):
        nyquist_zone = defaults.get('nyquist_zone')
        if nyquist_zone is not None:
            print(f'apply_config: setting nyquist_zone = {nyquist_zone}')
            set_nyquist_zone(r, new_config_dict, nyquist_zone)

    if changed_any('nyquist_zone', 'dac_inverse_sinc_filter_enabled'):
        inv_sinc = defaults.get('dac_inverse_sinc_filter_enabled', True)
        print(f'apply_config: setting dac_inverse_sinc_filter_enabled = {inv_sinc}')
        set_dac_inverse_sinc_filter(r, new_config_dict, inv_sinc=None)

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

    tone_plan = get_configured_tone_plan(new_config_dict)
    tone_frequencies_changed = changed_any('frequencies', 'blind_frequencies')
    tone_amplitudes_changed = changed_any('amplitudes', 'blind_amplitudes')
    tone_phases_changed = changed_any('phases', 'blind_phases')

    if tone_frequencies_changed:
        frequencies = tone_plan['frequencies']
        if len(frequencies) > 0:
            print(f'apply_config: setting frequencies ({len(frequencies)} tones, '
                  f'{tone_plan["num_blind_tones"]} blind)')
            try:
                r_fast.mixer.host.transport.axil_mm
                r_fast.mixer.host.transport._get_device_address
                set_tone_frequencies_fast(
                    r, r_fast, new_config_dict, frequencies,
                    tone_amplitudes=tone_plan['amplitudes'],
                    tone_phases=tone_plan['phases'])
            except AttributeError:
                set_tone_frequencies(
                    r, new_config_dict, frequencies,
                    tone_amplitudes=tone_plan['amplitudes'],
                    tone_phases=tone_plan['phases'])

    if tone_amplitudes_changed and not tone_frequencies_changed:
        amplitudes = tone_plan['amplitudes']
        if amplitudes is not None and len(amplitudes) > 0:
            print(f'apply_config: setting amplitudes ({len(amplitudes)} values)')
            set_tone_amplitudes(r, new_config_dict, amplitudes)

    if tone_phases_changed and not tone_frequencies_changed:
        phases = tone_plan['phases']
        if phases is not None and len(phases) > 0:
            print(f'apply_config: setting phases ({len(phases)} values)')
            set_tone_phases(r, new_config_dict, phases)


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
    acc_len = r.mixer.get_acc_len()
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
    r.mixer.set_acc_len(acc_len)  # v7.11: acc_len lives in the mixer
    r.sync.sw_sync(mrst=True)     # acc_len change needs a master-reset + sync
    return acc_freq


def set_nyquist_zone(r,config_dict,nyquist_zone):
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

    return


def set_dac_inverse_sinc_filter(r, config_dict, inv_sinc=None):
    """
    Enable or disable the inverse-sinc FIR on both configured DACs.

    When ``inv_sinc`` is None, the setting is read from
    ``firmware.defaults.dac_inverse_sinc_filter_enabled`` and defaults to
    enabled when the key is absent. The enabled RFDC mode follows the
    configured Nyquist zone.
    """
    fwconf = config_dict['firmware']
    defaults = fwconf.get('defaults', {})
    dac0_tile = int(fwconf['dac0_tile'])
    dac0_block = int(fwconf['dac0_block'])
    dac1_tile = int(fwconf['dac1_tile'])
    dac1_block = int(fwconf['dac1_block'])
    nyquist_zone = defaults.get('nyquist_zone')

    if nyquist_zone not in [1, 2]:
        raise ValueError(
            f'Invalid or missing Nyquist zone {nyquist_zone} for DAC inverse-sinc filter')

    if inv_sinc is None:
        inv_sinc = defaults.get('dac_inverse_sinc_filter_enabled', True)
    if not isinstance(inv_sinc, (bool, np.bool_)):
        raise ValueError('dac_inverse_sinc_filter_enabled must be a boolean')

    if not inv_sinc:
        mode = r.rfdc.core.INVSINC_FIR_DISABLED
    elif nyquist_zone == 1:
        mode = r.rfdc.core.INVSINC_FIR_NYQUIST1
    else:
        mode = r.rfdc.core.INVSINC_FIR_NYQUIST2

    r.rfdc.core.set_invsinc_fir(dac0_tile, dac0_block, mode)
    r.rfdc.core.set_invsinc_fir(dac1_tile, dac1_block, mode)
    return bool(inv_sinc)


def read_raw_control_buffer_data(r,buf,los=['tx','rx'],slot=0):
    """
    Read all tone parameters from one mixer LO control buffer in a single chunk.

    The ping-pong buffer layout changed in firmware v7.11:
      - pre-v7.11: a single register ``{lo}_lo{i}_control`` held *both* ping-pong
        buffers, selected by a ``buf * _n_serial_chans`` word offset.
      - v7.11+: each ping-pong buffer is its *own* register
        (``{lo}_lo{i}_control0`` / ``{lo}_lo{i}_control1``), each holding
        ``n_slots`` LO slots. The buffer is selected by register name (``buf``)
        and the slot is the in-register offset. ``slot=0`` reproduces the
        pre-v7.11 single-LO behaviour.

    There are still separate buffers for tx and rx settings.

    Parameters:
    r: readout object
    buf: int, index of ping-pong buffer to read from, 0 or 1.
    los: list of strings, either 'tx' or 'rx' to read the control values for the respective LO
    slot: int, LO slot to read (0..n_slots-1). Default 0.

    Returns a dictionary of the lo control values with the following keys:
    - 'tx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'
    - 'rx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'
    Each of these keys contains a numpy array with the values for each tone in firmware format

    """
    formatted_lo_control_values={'tx':{},'rx':{}}

    if buf not in [0,1]:
        raise ValueError(f"Buffer index must be 0 or 1. Not {buf}.")
    if not 0 <= slot < r.mixer.n_slots:
        raise ValueError(f"Slot must be in 0..{r.mixer.n_slots-1}. Not {slot}.")

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

        # v7.11: each parallel stream slice lives in register f'{lo}_lo{i}_control{buf}'
        # at an offset of: 4 * _CONTROL_N_WORDS * (slot * _n_serial_chans + i)
        # and with a length of: 4 * _CONTROL_N_WORDS * Ns bytes.
        slice_len_bytes = 4 * r.mixer._CONTROL_N_WORDS * Ns

        for i in range(Np):
            offset = 4 * r.mixer._CONTROL_N_WORDS * (slot * r.mixer._n_serial_chans + i)
            reg = f'{lo}_lo{i}_control{buf}'
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
    Get (and cache) the base byte addresses of the TX/RX mixer control buffers.

    v7.11: the two ping-pong buffers are *separate registers*
    (``{lo}_lo0_control0`` / ``{lo}_lo0_control1``), so an address is cached per
    ``(lo, buf)``. The single-parallel-stream design (``_n_parallel_chans == 1``)
    means one register per ``(lo, buf)`` holds every tone for all slots.

    Returns a dict keyed by ``(lo, buf)`` -> base byte address.
    """
    if not hasattr(r_fast.mixer, '_control_buffer_addrs'):
        # _n_parallel_chans is a firmware channel-ordering optimisation (kept at 1
        # here); it is independent of pipeline count -- dual pipeline scales via
        # separate p0_/p1_ prefixed mixers, not by growing this. The fast path's
        # single-lo0-register assumption only holds while it is 1.
        assert r_fast.mixer._n_parallel_chans == 1, (
            'fast control-buffer path assumes _n_parallel_chans == 1; '
            'generalise to loop over lo{i} registers if this changes')
        addrs = {}
        for lo in ('tx', 'rx'):
            for buf in (0, 1):
                addrs[(lo, buf)] = r_fast.mixer.host.transport._get_device_address(
                    f'{r_fast.mixer.prefix}{lo}_lo0_control{buf}')
        r_fast.mixer._control_buffer_addrs = addrs
    return r_fast.mixer._control_buffer_addrs

def read_raw_control_buffer_data_fast(r_fast,buf,los=['tx','rx'],slot=0):
    """
    Memory-mapped (devmem) read of all tone parameters from one mixer LO control
    buffer in a single chunk.

    v7.11: the ping-pong buffer is selected by register (``control0``/``control1``)
    and the slot is an offset within that register. ``slot=0`` reproduces the
    pre-v7.11 single-LO behaviour.

    There are still separate buffers for tx and rx settings.

    Parameters:
    r: readout object
    buf: int, index of ping-pong buffer to read from, 0 or 1.
    los: list of strings, either 'tx' or 'rx' to read the control values for the respective LO
    slot: int, LO slot to read (0..n_slots-1). Default 0.

    Returns a dictionary of the lo control values with the following keys:
    - 'tx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'
    - 'rx': dictionary with keys 'formatted_phase_steps', 'formatted_ri_steps', 'formatted_phase_offsets', 'formatted_scaling'
    Each of these keys contains a numpy array with the values for each tone in firmware format

    """
    formatted_lo_control_values={'tx':{},'rx':{}}

    if buf not in [0,1]:
        raise ValueError(f"Buffer index must be 0 or 1. Not {buf}.")
    if not 0 <= slot < r_fast.mixer.n_slots:
        raise ValueError(f"Slot must be in 0..{r_fast.mixer.n_slots-1}. Not {slot}.")

    # Get the dynamically-looked-up base addresses for the control buffers
    addrs = _get_control_buffer_addresses(r_fast)
    # One slot's worth: n_serial_chans * _CONTROL_N_WORDS * 4 bytes (also the slot stride)
    buf_size = r_fast.mixer._n_serial_chans * r_fast.mixer._CONTROL_N_WORDS * 4

    for lo in los:
        if lo not in ['tx','rx']:
            raise ValueError(f"Only LOs 'rx' and 'tx' are understood. Not {lo}.")

        offset = addrs[(lo, buf)] + buf_size * slot
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
    # Flat little-endian u32 view of the whole AXI-lite map. The control buffers are
    # addressed by absolute word index (resolved via _get_control_buffer_addresses),
    # so this view is layout-independent across firmware versions. (v7.11 moved the
    # control buffers; do not hardcode byte ranges here.)
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

def write_control_buffer_data(r,buf,v,slot=0):
    """
    Write a formatted control buffer to the firmware.

    v7.11: the ping-pong buffer is selected by register (``control0``/``control1``)
    and the slot is an offset within the register. ``slot=0`` reproduces the
    pre-v7.11 single-LO behaviour.

    Parameters:
    r: readout object
    buf: int, index of ping-pong buffer to write to, 0 or 1.
    v: dict with 'tx'/'rx' numpy arrays of the formatted control buffer.
    slot: int, LO slot to write (0..n_slots-1). Default 0.

    """
    n_tone = r.mixer.n_chans
    if buf not in [0,1]:
        raise ValueError(f"Buffer index must be 0 or 1. Not {buf}.")
    if not 0 <= slot < r.mixer.n_slots:
        raise ValueError(f"Slot must be in 0..{r.mixer.n_slots-1}. Not {slot}.")
    for lo in ['tx','rx']:
        for i in range(min(r.mixer._n_parallel_chans, n_tone)):
            reg = f'{lo}_lo{i}_control{buf}'
            offset = 4 * r.mixer._CONTROL_N_WORDS * (slot * r.mixer._n_serial_chans + i)
            r.mixer.write(reg, v[lo].tobytes(),offset=offset)
    return

def write_control_buffer_data_fast(r_fast,buf,v,indices,slot=0):
    """
    Write a formatted control buffer to the firmware via the devmem fast path.

    v7.11: the ping-pong buffer is selected by register (``control0``/``control1``)
    and the slot is an offset within the register. ``slot=0`` reproduces the
    pre-v7.11 single-LO behaviour.

    Parameters:
    r: readout object
    buf: int, index of ping-pong buffer to write to, 0 or 1.
    v: dictionary with keys 'tx' and 'rx'
        - 'tx': numpy array of the formatted control buffer values for tx
        - 'rx': numpy array of the formatted control buffer values for rx

    indices: dictionary with keys 'tx' and 'rx'
        - 'tx': numpy array of the indices for the formatted control buffer for tx
        - 'rx': numpy array of the indices for the formatted control buffer for rx
    slot: int, LO slot to write (0..n_slots-1). Default 0.

    """
    n_tone = r_fast.mixer.n_chans
    if buf not in [0,1]:
        raise ValueError(f"Buffer index must be 0 or 1. Not {buf}.")
    if not 0 <= slot < r_fast.mixer.n_slots:
        raise ValueError(f"Slot must be in 0..{r_fast.mixer.n_slots-1}. Not {slot}.")

    # Get the dynamically-looked-up base addresses for the control buffers
    addrs = _get_control_buffer_addresses(r_fast)
    # One slot's worth in bytes: n_serial_chans * _CONTROL_N_WORDS * 4 (also the slot stride)
    buf_size = r_fast.mixer._n_serial_chans * r_fast.mixer._CONTROL_N_WORDS * 4

    if indices is None:
        for lo in ['tx','rx']:
            start = addrs[(lo, buf)] + buf_size * slot
            length = buf_size
            r_fast.mixer.host.transport.axil_mm[int(start):int(start+length)] = v[lo].astype('<u4').tobytes()
    else:
        # Convert byte addresses to word offsets (divide by 4)
        start_tx = (addrs[('tx', buf)] + buf_size * slot) // 4
        start_rx = (addrs[('rx', buf)] + buf_size * slot) // 4
        r_fast.mixer.tx_lo_control_buffer[start_tx+indices['tx']] = v['tx'].astype('<u4')
        r_fast.mixer.rx_lo_control_buffer[start_rx+indices['rx']] = v['rx'].astype('<u4')

    return


def write_phase_offsets_both_buffers_fast(r_fast, phase_offsets, tone_indices=None,
                                          slots=(0,)):
    """
    Write per-tone LO phase offsets into **both** control buffers (0 and 1).

    Phase offsets (the LO start phase) do not change during a sweep or a
    modulation run: they are set once and then left alone while only the
    frequency words (phase_inc + ri_step) are swapped per point. Writing both
    ping-pong buffers keeps their phase reference identical, so flipping the
    active buffer changes only the frequency, not the absolute phase. The same
    offset is applied to the tx and rx LOs (matching the per-tone setters).

    Only the phase-offset words are written; the frequency/amplitude words
    already in each buffer are left untouched (sparse indexed write).

    Parameters:
    r_fast: readout object (fast interface)
    phase_offsets: per-tone LO start phases, in radians.
    tone_indices: LO indices for the tones (may be non-contiguous with VACC).
                  If None, contiguous indices from 0 are assumed.
    """
    phase_offsets = np.atleast_1d(np.asarray(phase_offsets, dtype=float))
    lo_control_values = {
        'tx': {'phase_offsets': phase_offsets},
        'rx': {'phase_offsets': phase_offsets},
    }
    v, i = prepare_control_buffer_data_fast(
        r_fast, 0, lo_control_values, tone_indices=tone_indices)
    for buf in (0, 1):
        write_control_buffer_data_fast(r_fast, buf, v, i)
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
    # When handed a local handle (r_fast), use the memory-mapped read (~200x
    # faster) with its matching little-endian interpreter; otherwise fall back
    # to the katcp transport read + big-endian interpreter. The two read/interpret
    # pairs must stay matched -- mixing them byte-swaps the values.
    if hasattr(r.mixer.host.transport, 'axil_mm'):
        return interpret_raw_control_buffer_data_fast(r, read_raw_control_buffer_data_fast(r, buf, los=los))
    return interpret_raw_control_buffer_data(r, read_raw_control_buffer_data(r, buf, los=los))

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


def get_tone_frequencies(r, r_fast, config_dict, detailed_output=False):
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


    # Use r_fast (memory-mapped) for the slow buffer/chanmap reads when
    # available; the RFDC reads above stay on r (they don't work via r_fast).
    rd = r_fast if r_fast is not None else r

    lo_control_values = read_from_current_control_buffer(rd)

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
    chanmap_psb_inmap = psb_chanselect_get_channel_inmap(rd)
    chanmap_pfb = chanselect_get_channel_outmap(rd)

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
        details.update(get_configured_tone_metadata(
            config_dict, active_count=len(output_freqs)))
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


def _validate_per_tone_values(values, num_tones, name):
    if values is None:
        return None
    values = np.atleast_1d(values).astype(float)
    if len(values) != num_tones:
        raise ValueError(
            f'Number of {name} ({len(values)}) must match number of tones ({num_tones})')
    return values


def _get_current_per_tone_values(getter, r, config_dict, num_tones):
    try:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            values = np.atleast_1d(getter(r, config_dict))
    except Exception:
        return None
    if len(values) != num_tones:
        return None
    return values


def _max_tones_per_bin(bin_indices):
    bin_indices = np.asarray(bin_indices)
    if bin_indices.size == 0:
        return 0
    if bin_indices.ndim == 1:
        _, counts = np.unique(bin_indices, return_counts=True)
        return int(np.max(counts)) if counts.size else 0
    return max(_max_tones_per_bin(row) for row in bin_indices)


def _protect_tone_amplitudes_for_vacc(tone_amplitudes, max_tones_per_bin):
    tone_amplitudes = np.asarray(tone_amplitudes, dtype=float)
    max_tones_per_bin = max(1, int(max_tones_per_bin))
    max_amp = (1 - 2**-12) / max_tones_per_bin
    peak_amp = float(np.max(np.abs(tone_amplitudes))) if tone_amplitudes.size else 0.0
    if peak_amp <= max_amp or peak_amp == 0:
        return tone_amplitudes, 1.0, max_amp
    scale_factor = max_amp / peak_amp
    return tone_amplitudes * scale_factor, scale_factor, max_amp



def prepare_tone_frequency_settings(r, config_dict, tone_frequencies, tone_indices=None,
                                    min_tone_separation=6, tone_amplitudes=None,
                                    tone_phases=None, compensate_rx_ticks=0):
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
    compensate_rx_ticks: if non-zero, add a per-tone RX phase offset to cancel
                         the RX-vs-TX path delay (in 307.2 MHz clock ticks) seen
                         when retuning without a sync. See _rx_phase_compensation.
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

    tone_amplitudes = _validate_per_tone_values(
        tone_amplitudes, num_tones, 'tone_amplitudes')
    tone_phases = _validate_per_tone_values(
        tone_phases, num_tones, 'tone_phases')

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

    lo_control_values = {
        'tx': {
            'phase_steps': phase_incs_tx_full,
            'ri_steps': ri_steps_tx_full,
        },
        'rx': {
            'phase_steps': phase_incs_rx_full,
            'ri_steps': ri_steps_rx_full,
        },
    }
    if tone_amplitudes is not None:
        scaling_tx_full = np.zeros(n_chans, dtype=float)
        scaling_rx_full = np.zeros(n_chans, dtype=float)
        scaling_tx_full[tone_indices] = tone_amplitudes
        scaling_rx_full[tone_indices] = 1.0
        lo_control_values['tx']['scaling'] = scaling_tx_full
        lo_control_values['rx']['scaling'] = scaling_rx_full
    # RX picks up an extra per-tone phase offset to cancel the RX-vs-TX path
    # delay (no-op when compensate_rx_ticks==0). TX keeps the bare tone phases,
    # and is written only when tone_phases is given (tone_phases=None means
    # "leave unchanged"); RX is written whenever phases or compensation apply.
    rx_phase_comp = _rx_phase_compensation(
        phase_incs_rx, fft_rbw_hz, compensate_rx_ticks)
    if tone_phases is not None or compensate_rx_ticks:
        tone_phases_arr = tone_phases if tone_phases is not None else np.zeros(num_tones)
        phase_offsets_rx_full = np.zeros(n_chans, dtype=float)
        phase_offsets_rx_full[tone_indices] = tone_phases_arr + rx_phase_comp
        lo_control_values['rx']['phase_offsets'] = phase_offsets_rx_full
        if tone_phases is not None:
            phase_offsets_tx_full = np.zeros(n_chans, dtype=float)
            phase_offsets_tx_full[tone_indices] = tone_phases_arr
            lo_control_values['tx']['phase_offsets'] = phase_offsets_tx_full

    #prepare the formatted lo control buffer values
    buf = get_next_buffer_idx(r)
    v = prepare_control_buffer_data(r, buf, lo_control_values)

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

def apply_tone_frequency_settings(r, tone_settings_dict, autosync=False, mrst=False):
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

    write_control_buffer_data(r,buf,v)
    set_control_buffer_idx(r,buf)
    _sync_if_requested(r, autosync=autosync, mrst=mrst)
    return

def prepare_tone_frequency_settings_fast(r, config_dict, tone_frequencies,
                                         tone_indices=None, min_tone_separation=6,
                                         detailed_output=False,
                                         tone_amplitudes=None, tone_phases=None,
                                         compensate_rx_ticks=0):
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
    compensate_rx_ticks: if non-zero, add a per-tone RX phase offset to cancel
                         the RX-vs-TX path delay (in 307.2 MHz clock ticks) seen
                         when retuning without a sync. See _rx_phase_compensation.
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
    dac_nyquist_zone = defaults.get('nyquist_zone')
    adc_nyquist_zone = defaults.get('nyquist_zone')
    _fs = 2 * r.adc_clk_hz  # RFDC sampling frequency
    if dac_nyquist_zone is not None:
        duc_frequency = _fs / 4 if int(dac_nyquist_zone) == 1 else -_fs * 3 / 4
    else:
        duc_frequency = defaults.get('dac_duc_mixer_frequency_hz', 0)
    if adc_nyquist_zone is not None:
        ddc_frequency = -_fs / 4 if int(adc_nyquist_zone) == 1 else _fs * 3 / 4
    else:
        ddc_frequency = defaults.get('adc_ddc_mixer_frequency_hz', 0)

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

    tone_amplitudes = _validate_per_tone_values(
        tone_amplitudes, num_tones, 'tone_amplitudes')
    tone_phases = _validate_per_tone_values(
        tone_phases, num_tones, 'tone_phases')

    # #zero pad out to nchans
    # phase_incs_tx = np.pad(phase_incs_tx, (0,nc-len(phase_incs_tx)), 'constant', constant_values=(0,0))
    # phase_incs_rx = np.pad(phase_incs_rx, (0,nc-len(phase_incs_rx)), 'constant', constant_values=(0,0))
    # ri_steps_tx = np.pad(ri_steps_tx, (0,nc-len(ri_steps_tx)), 'constant', constant_values=(0,0))
    # ri_steps_rx = np.pad(ri_steps_rx, (0,nc-len(ri_steps_rx)), 'constant', constant_values=(0,0))

    # Pass tone_indices to prepare_control_buffer_data_fast for correct sparse indexing
    lo_control_values = {
        'tx': {'phase_steps': phase_incs_tx, 'ri_steps': ri_steps_tx},
        'rx': {'phase_steps': phase_incs_rx, 'ri_steps': ri_steps_rx},
    }
    if tone_amplitudes is not None:
        lo_control_values['tx']['scaling'] = tone_amplitudes
        lo_control_values['rx']['scaling'] = np.ones_like(
            tone_amplitudes, dtype=float)
    # RX picks up an extra per-tone phase offset to cancel the RX-vs-TX path
    # delay (no-op when compensate_rx_ticks==0). TX keeps the bare tone phases.
    rx_phase_comp = _rx_phase_compensation(
        phase_incs_rx, fft_rbw_hz, compensate_rx_ticks)
    if tone_phases is not None:
        lo_control_values['tx']['phase_offsets'] = tone_phases
        lo_control_values['rx']['phase_offsets'] = tone_phases + rx_phase_comp
    elif compensate_rx_ticks:
        # No tone phases set, but still need the RX compensation on its own.
        lo_control_values['rx']['phase_offsets'] = np.zeros(num_tones) + rx_phase_comp
    buf = get_next_buffer_idx(r)
    v,i = prepare_control_buffer_data_fast(r, buf, lo_control_values,
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
                            'control_buffer_index':buf,
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


def _rf_to_digital_baseband(r, config_dict, tone_frequencies):
    """
    Convert requested RF tone frequencies to on-chip digital-baseband (DBB)
    frequencies, plus the filterbank bin grid needed to place them.

    This reproduces the analog-up/down-conversion + Nyquist-zone + DUC/DDC
    chain used by :func:`prepare_tone_frequency_settings_fast`, factored out so
    the fast-modulation preparer can reuse exactly the same mapping. Keeping a
    single source of truth avoids the two paths drifting apart.

    Parameters
    ----------
    r : object
        Readout firmware object (provides ``adc_clk_hz`` and ``mixer`` geometry).
    config_dict : dict
        Configuration dict (``rf_frontend`` UDC settings and ``firmware``
        Nyquist-zone / DUC-DDC settings).
    tone_frequencies : numpy.ndarray
        Requested RF tone frequencies in Hz (1D, length = number of tones).

    Returns
    -------
    dict
        ``dbb_freqs_tx`` / ``dbb_freqs_rx`` : DBB frequencies (Hz) for the TX
        (DAC) and RX (ADC) paths; ``fft_rbw_hz`` : filterbank channel spacing
        (Hz); ``all_tx_bin_centers_hz`` / ``all_rx_bin_centers_hz`` : the full
        TX/RX bin-centre grids the tones are snapped onto.
    """
    tone_frequencies = np.atleast_1d(tone_frequencies)

    # --- analog up/down conversion (external mixer, if fitted) ---
    udc_connected = config_dict['rf_frontend']['connected']
    udc_lo_frequency = config_dict['rf_frontend']['tx_mixer_lo_frequency_hz']
    udc_sideband = config_dict['rf_frontend']['tx_mixer_sideband']
    udc_connected = False if not udc_connected else udc_connected
    udc_lo_frequency = 0 if not udc_lo_frequency else float(udc_lo_frequency)
    udc_sideband = 1 if not udc_sideband else int(udc_sideband)

    # --- RFDC Nyquist zone + digital up/down converter (DUC/DDC) mixer ---
    defaults = config_dict['firmware']['defaults']
    dac_nyquist_zone = defaults.get('nyquist_zone')
    adc_nyquist_zone = defaults.get('nyquist_zone')
    _fs = 2 * r.adc_clk_hz  # RFDC sampling frequency
    if dac_nyquist_zone is not None:
        duc_frequency = _fs / 4 if int(dac_nyquist_zone) == 1 else -_fs * 3 / 4
    else:
        duc_frequency = defaults.get('dac_duc_mixer_frequency_hz', 0)
    if adc_nyquist_zone is not None:
        ddc_frequency = -_fs / 4 if int(adc_nyquist_zone) == 1 else _fs * 3 / 4
    else:
        ddc_frequency = defaults.get('adc_ddc_mixer_frequency_hz', 0)

    # filterbank bin grid (channel spacing and centre frequencies)
    fft_period_s = r.mixer._n_upstream_chans / r.mixer._upstream_oversample_factor / r.adc_clk_hz
    fft_rbw_hz = 1. / fft_period_s
    all_tx_bin_centers_hz = np.fft.fftfreq(2 * N_TX_FFT, 1. / r.adc_clk_hz)
    all_rx_bin_centers_hz = np.fft.fftfreq(N_RX_FFT, 1. / r.adc_clk_hz)

    # RF -> analog (DAC out / ADC in) frequencies
    if udc_connected:
        dac_out_freqs = (tone_frequencies - udc_lo_frequency) / udc_sideband
        adc_in_freqs = (tone_frequencies - udc_lo_frequency) / udc_sideband
    else:
        dac_out_freqs = tone_frequencies
        adc_in_freqs = tone_frequencies

    # analog -> digital baseband, accounting for the Nyquist zone fold
    if int(dac_nyquist_zone) == 1:
        dbb_freqs_tx = dac_out_freqs - duc_frequency
    elif int(dac_nyquist_zone) == 2:
        dbb_freqs_tx = dac_out_freqs + duc_frequency
    else:
        raise ValueError(f'Invalid DAC nyquist zone ({dac_nyquist_zone})')
    if int(adc_nyquist_zone) == 1:
        dbb_freqs_rx = adc_in_freqs + ddc_frequency
    elif int(adc_nyquist_zone) == 2:
        dbb_freqs_rx = adc_in_freqs - ddc_frequency
    else:
        raise ValueError(f'Invalid ADC nyquist zone ({adc_nyquist_zone})')

    return {
        'dbb_freqs_tx': dbb_freqs_tx,
        'dbb_freqs_rx': dbb_freqs_rx,
        'fft_rbw_hz': fft_rbw_hz,
        'all_tx_bin_centers_hz': all_tx_bin_centers_hz,
        'all_rx_bin_centers_hz': all_rx_bin_centers_hz,
    }


def prepare_modulation_settings_fast(r_fast, config_dict, center_frequencies, point_offsets,
                                     min_tone_separation=6, tone_amplitudes=None,
                                     tone_phases=None, armed=None, compensate_rx_ticks=0):
    """
    Prepare a fast-frequency-modulation bundle: per-point mixer control words
    computed **relative to a single armed set of filterbank bins**.

    Why this exists (and why we don't reuse the sweep preparer): the normal
    fast preparers snap *every* point to its own nearest FFT bin. Fast tone
    modulation instead holds the channel maps **fixed** at the centre comb's
    bins and dithers each tone a little around them (riding the ~2x filterbank
    overlap). If we computed each point's mixer phase relative to its own
    nearest bin, the phase increment would jump by a whole bin the moment a
    tone drifted across the half-bin boundary — wrong. So here every point's
    mixer phase increment / rotation-step is computed against the **armed**
    bin centre, which is exactly what the (fixed) channel map selects.

    Parameters
    ----------
    r_fast : object
        Fast readout firmware object (mixer / channel-select geometry,
        ``adc_clk_hz``, and devmem-backed ``axil_mm`` transport).
    config_dict : dict
        Configuration dict (RF front-end + firmware settings).
    center_frequencies : numpy.ndarray
        Per-tone centre (operating-point) RF frequencies in Hz, in user-facing
        tone order. Length = number of active tones.
    point_offsets : numpy.ndarray
        Per-point, per-tone probe offsets in Hz, shape ``(n_points, n_tones)``,
        added to ``center_frequencies`` (0 for tones that are not modulated).
        Built by the server from the user's ``offsets`` at ``mod_indices``.
    min_tone_separation : int, optional
        Minimum LO-index separation for tones sharing an FFT bin, used only when
        computing the armed VACC tone indices. Default 6.
    tone_amplitudes : numpy.ndarray or None, optional
        Per-tone amplitude scalings written into every point's control words.
        ``None`` leaves amplitudes at their current values.
    tone_phases : numpy.ndarray or None, optional
        Per-tone phase offsets (rad) written into every point's control words.
        ``None`` leaves phases unchanged.
    compensate_rx_ticks : int, optional
        If non-zero, add a per-tone, per-point RX phase offset to cancel the
        RX-vs-TX path delay (in 307.2 MHz clock ticks) seen when modulating
        without a per-point sync. The RX offset tracks each point's RX baseband
        offset, so it is baked into the per-point control words rather than the
        once-at-arm write; the returned ``phase_offsets`` is then ``None``.
        See _rx_phase_compensation.

    Returns
    -------
    dict
        Bundle consumed by the modulation scheduler:
        ``num_points`` / ``num_tones`` ; ``tone_indices`` (armed VACC/LO indices,
        user order); ``chanmap_psb_inmap`` / ``chanmap_pfb`` (armed channel maps,
        written **once** at arm); ``control_values`` / ``control_indices`` :
        lists (len ``n_points``) of buffer-agnostic control words + sparse
        indices for each point (write to whichever buffer is inactive);
        ``occupancy`` : ``(n_points, n_tones)`` of ``'nearest'|'second'|'beyond'``;
        ``drift_bins`` : ``(n_points, n_tones)`` signed drift from the armed bin
        centre in units of channels; ``needs_recenter`` : True if any
        (tone, point) is beyond overlap coverage.
    """
    center_frequencies = np.atleast_1d(np.asarray(center_frequencies, dtype=float))
    point_offsets = np.atleast_2d(np.asarray(point_offsets, dtype=float))
    num_points, num_tones = point_offsets.shape
    if len(center_frequencies) != num_tones:
        raise ValueError(
            f'center_frequencies ({len(center_frequencies)}) must match '
            f'point_offsets columns ({num_tones})')

    # --- 1) Determine the armed bins/maps ---
    # ``armed=None`` (enable / recenter): snap the centre comb to its nearest
    # bins and build fresh channel maps + VACC indices. ``armed`` provided
    # (live update): reuse the *existing* armed bins so the update rides the same
    # fixed maps (tones may sit on the overlapping neighbour) instead of snapping
    # to new bins — which is the whole point of overlap riding.
    if armed is None:
        dbb_center = _rf_to_digital_baseband(r_fast, config_dict, center_frequencies)
        fft_rbw_hz = dbb_center['fft_rbw_hz']
        tx_bin_width_hz = _bin_spacing_hz(dbb_center['all_tx_bin_centers_hz'])
        rx_bin_width_hz = _bin_spacing_hz(dbb_center['all_rx_bin_centers_hz'])
        tx_bins = get_closest_bin_indices(dbb_center['dbb_freqs_tx'], dbb_center['all_tx_bin_centers_hz'])
        rx_bins = get_closest_bin_indices(dbb_center['dbb_freqs_rx'], dbb_center['all_rx_bin_centers_hz'])
        # The bin-centre frequencies the fixed channel map will select for each tone.
        tx_bin_centers_hz = dbb_center['all_tx_bin_centers_hz'][tx_bins]
        rx_bin_centers_hz = dbb_center['all_rx_bin_centers_hz'][rx_bins]
        # Armed VACC/LO tone indices (handle tones sharing an FFT bin). Fixed for
        # the whole modulation run; changing these is what a recenter is for.
        tone_indices = compute_vacc_tone_indices(tx_bins, r_fast.mixer.n_chans, min_tone_separation)
        # Armed channel maps (written once at arm; never touched in the hot loop).
        chanmap_psb_inmap = np.full(r_fast.psb_chanselect.n_chans_in,
                                    r_fast.psb_chanselect.DISCARD_BIN, dtype=np.uint32)
        chanmap_psb_inmap[tone_indices] = tx_bins
        chanmap_pfb = np.full(r_fast.chanselect.n_chans_out, -1, dtype=int)
        chanmap_pfb[tone_indices] = rx_bins
        armed = {
            'fft_rbw_hz': fft_rbw_hz,
            'tx_bin_width_hz': tx_bin_width_hz,
            'rx_bin_width_hz': rx_bin_width_hz,
            'tx_bins': tx_bins, 'rx_bins': rx_bins,
            'tx_bin_centers_hz': tx_bin_centers_hz, 'rx_bin_centers_hz': rx_bin_centers_hz,
            'tone_indices': tone_indices,
            'chanmap_psb_inmap': chanmap_psb_inmap, 'chanmap_pfb': chanmap_pfb,
        }
    else:
        # Reuse the existing armed bins/maps (live update rides the same maps).
        fft_rbw_hz = armed['fft_rbw_hz']
        tx_bin_width_hz = armed.get('tx_bin_width_hz', fft_rbw_hz)
        rx_bin_width_hz = armed.get('rx_bin_width_hz', fft_rbw_hz)
        tx_bins = armed['tx_bins']
        rx_bins = armed['rx_bins']
        tx_bin_centers_hz = armed['tx_bin_centers_hz']
        rx_bin_centers_hz = armed['rx_bin_centers_hz']
        tone_indices = armed['tone_indices']
        chanmap_psb_inmap = armed['chanmap_psb_inmap']
        chanmap_pfb = armed['chanmap_pfb']

    tone_amplitudes = _validate_per_tone_values(tone_amplitudes, num_tones, 'tone_amplitudes')
    tone_phases = _validate_per_tone_values(tone_phases, num_tones, 'tone_phases')

    # --- 2) Per-point mixer words, all relative to the armed bins ---
    control_values = []
    control_indices = []
    drift_bins = np.zeros((num_points, num_tones), dtype=float)
    occupancy = np.empty((num_points, num_tones), dtype=object)
    for p in range(num_points):
        dbb = _rf_to_digital_baseband(r_fast, config_dict, center_frequencies + point_offsets[p])
        # Residual offset of this point from the *armed* bin centre (NOT the
        # point's own nearest bin). The raw residual is used for diagnostics
        # (coverage/occupancy); the NCO word itself is periodic by one FFT bin,
        # so control words and RX delay compensation use the wrapped residual.
        tx_off = dbb['dbb_freqs_tx'] - tx_bin_centers_hz
        rx_off = dbb['dbb_freqs_rx'] - rx_bin_centers_hz
        tx_drift_bins = tx_off / tx_bin_width_hz
        rx_drift_bins = rx_off / rx_bin_width_hz
        tx_nco_bins = _wrap_bin_offsets_for_nco(tx_off / fft_rbw_hz)
        rx_nco_bins = _wrap_bin_offsets_for_nco(rx_off / fft_rbw_hz)
        phase_incs_tx = tx_nco_bins * 2 * np.pi
        phase_incs_rx = rx_nco_bins * 2 * np.pi
        ri_steps_tx = np.cos(phase_incs_tx) + 1j * np.sin(phase_incs_tx)
        ri_steps_rx = np.cos(phase_incs_rx) + 1j * np.sin(phase_incs_rx)

        lo_control_values = {
            'tx': {'phase_steps': phase_incs_tx, 'ri_steps': ri_steps_tx},
            'rx': {'phase_steps': phase_incs_rx, 'ri_steps': ri_steps_rx},
        }
        # Amplitude is identical for every point; including it here keeps each
        # buffer self-consistent. Phase offsets are deliberately *not* written
        # per point: they never change during a run, so the scheduler writes them
        # once into both buffers at arm (see
        # ``write_phase_offsets_both_buffers_fast``), leaving the per-point write
        # frequency(+amplitude)-only.
        if tone_amplitudes is not None:
            lo_control_values['tx']['scaling'] = tone_amplitudes
            lo_control_values['rx']['scaling'] = np.ones_like(tone_amplitudes, dtype=float)

        # Exception to the above: RX path-delay compensation tracks this point's
        # RX baseband offset, so it varies per point and cannot be written once.
        # The write-once helper writes the same offset to both tx and rx, which
        # would clobber the per-point RX value, so when compensating we bake the
        # offsets per point (and return phase_offsets=None below). The RX offset
        # is always written (base phase + compensation); the TX offset is written
        # per point only when tone_phases is given, otherwise TX offsets are left
        # untouched -- matching the tone_phases=None ("leave unchanged") contract.
        if compensate_rx_ticks:
            rx_phase_comp = _rx_phase_compensation(
                phase_incs_rx, fft_rbw_hz, compensate_rx_ticks)
            base = tone_phases if tone_phases is not None else 0.0
            if tone_phases is not None:
                lo_control_values['tx']['phase_offsets'] = tone_phases
            lo_control_values['rx']['phase_offsets'] = base + rx_phase_comp

        # buf=0 here is irrelevant: the formatted values are buffer-agnostic; the
        # scheduler writes them into whichever buffer is currently inactive.
        v, i = prepare_control_buffer_data_fast(r_fast, 0, lo_control_values, tone_indices=tone_indices)
        control_values.append(v)
        control_indices.append(i)

        # Bin occupancy: how far (in channels) each tone sits from its armed bin
        # centre. With ~2x overlap the neighbouring channel still covers a tone
        # out to ~1 full channel, so <=0.5 = on the nearest bin, 0.5-1.0 = riding
        # the overlapping neighbour (fine, flagged), >1.0 = no longer covered.
        drift = np.maximum(np.abs(tx_drift_bins), np.abs(rx_drift_bins))
        # signed drift (TX path) for reporting; magnitude drives the class
        drift_bins[p] = tx_drift_bins
        occupancy[p] = np.where(drift <= 0.5, 'nearest',
                                np.where(drift <= 1.0, 'second', 'beyond'))

    needs_recenter = bool(np.any(occupancy == 'beyond'))

    return {
        'num_points': num_points,
        'num_tones': num_tones,
        'tone_indices': tone_indices,
        # Per-tone LO start phases (rad), written once into both buffers; None
        # leaves them unchanged. When RX path-delay compensation is active the
        # offsets vary per point and are baked into control_values above, so the
        # once-write is suppressed (None) to avoid clobbering them.
        'phase_offsets': None if compensate_rx_ticks else tone_phases,
        'chanmap_psb_inmap': chanmap_psb_inmap,
        'chanmap_pfb': chanmap_pfb,
        'control_values': control_values,     # list len n_points (buffer-agnostic)
        'control_indices': control_indices,   # list len n_points
        'occupancy': occupancy,               # (n_points, n_tones) str
        'drift_bins': drift_bins,             # (n_points, n_tones) signed, channels
        'armed_tx_bins': tx_bins,
        'armed_rx_bins': rx_bins,
        'needs_recenter': needs_recenter,
        'armed': armed,                       # reuse for in-place live updates (same maps)
    }


def prepare_sweep_settings_fast(r_fast, config_dict, sweep_frequencies,
                                min_tone_separation=6, detailed_output=False,
                                tone_amplitudes=None, tone_phases=None,
                                compensate_rx_ticks=0):
    """
    Prepare sweep step settings with VACC-aware tone index assignment.

    sweep_freqs: 2D array (num_points, num_tones)
    compensate_rx_ticks: if non-zero, add a per-tone, per-point RX phase offset to
        cancel the RX-vs-TX path delay (in 307.2 MHz clock ticks) seen when the
        sweep steps without a per-step sync. Because the RX offset varies per
        point, the offsets ride in each per-point write rather than the
        write-once path (TX still uses write-once). See _rx_phase_compensation.
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
    dac_nyquist_zone = defaults.get('nyquist_zone')
    adc_nyquist_zone = defaults.get('nyquist_zone')
    _fs = 2 * r_fast.adc_clk_hz  # RFDC sampling frequency
    if dac_nyquist_zone is not None:
        duc_frequency = _fs / 4 if int(dac_nyquist_zone) == 1 else -_fs * 3 / 4
    else:
        duc_frequency = defaults.get('dac_duc_mixer_frequency_hz', 0)
    if adc_nyquist_zone is not None:
        ddc_frequency = -_fs / 4 if int(adc_nyquist_zone) == 1 else _fs * 3 / 4
    else:
        ddc_frequency = defaults.get('adc_ddc_mixer_frequency_hz', 0)

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
    max_tones_per_bin = _max_tones_per_bin(tx_nearest_bins)

    tone_amplitudes = _validate_per_tone_values(
        tone_amplitudes, num_tones, 'tone_amplitudes')
    tone_phases = _validate_per_tone_values(
        tone_phases, num_tones, 'tone_phases')
    amplitude_scale_factor = 1.0
    vacc_max_amplitude = None
    if tone_amplitudes is not None:
        tone_amplitudes, amplitude_scale_factor, vacc_max_amplitude = (
            _protect_tone_amplitudes_for_vacc(
                tone_amplitudes, max_tones_per_bin))
        if amplitude_scale_factor < 1.0:
            print(
                f'prepare_sweep_settings_fast: reducing tone amplitudes by '
                f'{amplitude_scale_factor:.4f} for up to '
                f'{max_tones_per_bin} tones per FFT bin during sweep')

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

    # print('phase_incs_tx',phase_incs_tx.shape,'\n',phase_incs_tx)
    # print('phase_incs_rx',phase_incs_rx.shape,'\n',phase_incs_rx)
    # print('ri_steps_tx',ri_steps_tx.shape,'\n',ri_steps_tx)
    # print('ri_steps_rx',ri_steps_rx.shape,'\n',ri_steps_rx)


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

    # Phase offsets don't change during a sweep. When the tone LO slots are
    # stable across the whole sweep (the common narrow-sweep case) the caller
    # writes them once into both buffers (write_phase_offsets_both_buffers_fast)
    # and the per-point write stays frequency-only. When bins move, a tone's LO
    # slot changes per point, so the offset must ride along in each per-point
    # write to land in the right slot.
    write_offsets_once = bool(tx_bins_stable) and (tone_phases is not None)

    # RX path-delay compensation is per-tone *and* per-point (it tracks the RX
    # baseband offset, which moves with frequency across the sweep), so the RX
    # offset can never be written once. The write-once helper
    # (write_phase_offsets_both_buffers_fast) writes the *same* offset to both
    # tx and rx, which would clobber the per-point RX compensation -- so when
    # compensating we drop the write-once path entirely and bake both tx and rx
    # offsets per point. A few extra offset words per point; negligible.
    compensate_rx = bool(compensate_rx_ticks)
    if compensate_rx:
        write_offsets_once = False


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
    first_buf = get_next_buffer_idx(r_fast)
    allbuf = (first_buf + np.arange(num_points, dtype=int)) % 2

    for p in points:
        # #zero pad out to nchans for the fast write
        # phase_incs_tx_formatted_padded[p,:len(phase_incs_tx_formatted[p])] = phase_incs_tx_formatted[p]
        # phase_incs_rx_formatted_padded[p,:len(phase_incs_rx_formatted[p])] = phase_incs_rx_formatted[p]
        # ri_steps_tx_formatted_padded[p,:len(ri_steps_tx_formatted[p])] = ri_steps_tx_formatted[p]
        # ri_steps_rx_formatted_padded[p,:len(ri_steps_rx_formatted[p])] = ri_steps_rx_formatted[p]
        # print('prep_sweep, prep_buf',p)
        lo_control_values = {
            'tx': {
                'phase_steps': phase_incs_tx[p],
                'ri_steps': ri_steps_tx[p],
            },
            'rx': {
                'phase_steps': phase_incs_rx[p],
                'ri_steps': ri_steps_rx[p],
            },
        }
        if tone_amplitudes is not None:
            lo_control_values['tx']['scaling'] = tone_amplitudes
            lo_control_values['rx']['scaling'] = np.ones_like(
                tone_amplitudes, dtype=float)
        # Bake the TX offset per point only when the LO slots move across the
        # sweep; otherwise it is written once into both buffers by the caller.
        if tone_phases is not None and not write_offsets_once:
            lo_control_values['tx']['phase_offsets'] = tone_phases
        # The RX offset rides per point whenever it varies across points: either
        # because the LO slots move (not write_offsets_once) or because the
        # path-delay compensation is active (it tracks this point's RX offset).
        if (tone_phases is not None and not write_offsets_once) or compensate_rx:
            rx_phase_comp = _rx_phase_compensation(
                phase_incs_rx[p], fft_rbw_hz, compensate_rx_ticks)
            base_rx = tone_phases if tone_phases is not None else 0.0
            lo_control_values['rx']['phase_offsets'] = base_rx + rx_phase_comp
        allv[p], alli[p] = prepare_control_buffer_data_fast(
            r_fast, allbuf[p], lo_control_values,
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
                            # Constant phase offsets to write once into both buffers
                            # (None when bins move -> offsets are baked per point instead).
                            'phase_offsets': tone_phases if write_offsets_once else None,
                            'phase_offset_tone_indices': tone_indices_arr[0] if write_offsets_once else None,
                            'num_tones':num_tones,
                            'max_tones_per_bin':max_tones_per_bin,
                            'amplitude_scale_factor':amplitude_scale_factor,
                            'vacc_max_amplitude':vacc_max_amplitude}

    return sweep_settings_dict

def apply_sweep_step_fast(r, r_fast, sweep_settings, step_index,
                          autosync=False, mrst=False,
                          chanmap_settle_accumulations=4):
    # phase_incs_tx_formatted = sweep_settings.get('phase_incs_tx_formatted')
    # phase_incs_rx_formatted = sweep_settings.get('phase_incs_rx_formatted')
    # ri_steps_tx_formatted   = sweep_settings.get('ri_steps_tx_formatted')
    # ri_steps_rx_formatted   = sweep_settings.get('ri_steps_rx_formatted')
    # print('apply_step', step_index)
    allv= sweep_settings.get('control_buffer_data_values')
    alli= sweep_settings.get('control_buffer_data_indices')
    allbuf = sweep_settings.get('control_buffer_index')
    chanmap_psb_inmap   = sweep_settings.get('chanmap_psb_inmap')
    chanmap_pfb   = sweep_settings.get('chanmap_pfb')
    skip_chanmap_psb_inmap = sweep_settings.get('skip_chanmap_psb_inmap')
    skip_chanmap_pfb = sweep_settings.get('skip_chanmap_pfb')
    num_tones     = sweep_settings.get('num_tones')

    chanmap_settle_accumulations = int(chanmap_settle_accumulations)
    if chanmap_settle_accumulations < 0:
        raise ValueError('chanmap_settle_accumulations must be >= 0')

    c1=not skip_chanmap_psb_inmap[step_index]
    c2=not skip_chanmap_pfb[step_index]
    if c1:
        # print('set chanmap 1 (psb inmap)')
        # v7.9: use inmap setter for psb_chanselect
        psb_chanselect_set_channel_inmap(r_fast, chanmap_psb_inmap[step_index])

        # while not (r.psb_chanselect.get_channel_outmap()==chanmap_psb[step_index]).all():
        #     print('waiting for psb chanmap to update')
        #     time.sleep(0.001)
        # print('psb chanmap updated')
    if c2:
        # print('set chanmap 2 (pfb outmap)')
        # r_fast.chanselect.set_channel_outmap(np.copy(chanmap_pfb[step_index]))
        chanselect_set_channel_outmap(r_fast,chanmap_pfb[step_index])
        # while not (r.chanselect.get_channel_outmap()==chanmap_pfb[step_index]).all():
        #     print('waiting for pfb chanmap to update')
        #     time.sleep(0.001)
        # print('pfb chanmap updated')

    if c1 or c2:
        for _ in range(chanmap_settle_accumulations):
            _wait_for_acc(r_fast,0,0.0001)

    # print('apply_step, write_buf',step_index, allbuf[step_index])
    write_control_buffer_data_fast(r_fast,allbuf[step_index],allv[step_index],alli[step_index])

    # print('apply_step, set_buf',step_index,allbuf[step_index])
    set_control_buffer_idx_fast(r_fast,allbuf[step_index])

    if autosync:
        force_sync_fast(r_fast, 0.00001, mrst=mrst)

    # if c1 or c2:
    #     _wait_for_acc(r_fast,0,0.0001)


    # fast_write_mixer(r_fast,
    #                   phase_incs_tx_formatted[step_index],
    #                     phase_incs_rx_formatted[step_index],
    #                       ri_steps_tx_formatted[step_index],
    #                         ri_steps_rx_formatted[step_index])

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



def apply_tone_frequency_settings_fast(r, r_fast, fast_tone_frequency_settings, autosync=False, mrst=False):

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
    if autosync:
        force_sync_fast(r_fast, autosync_time_delay, mrst=mrst)
    if c1 or c2 or autosync:
        _wait_for_acc(r_fast,0,0.0001)

    # fast_write_mixer(r_fast,
    #                   phase_incs_tx_formatted,
    #                     phase_incs_rx_formatted,
    #                       ri_steps_tx_formatted,
    #                         ri_steps_rx_formatted)


def set_tone_frequencies(r, config_dict, tone_frequencies, tone_indices=None,
                         min_tone_separation=6, autosync=False, mrst=False,
                         detailed_output=False, tone_amplitudes=None,
                         tone_phases=None, compensate_rx_ticks=0):
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
    num_tones = len(np.atleast_1d(tone_frequencies))
    if tone_amplitudes is None:
        tone_amplitudes = _get_current_per_tone_values(
            get_tone_amplitudes, r, config_dict, num_tones)
    if tone_phases is None:
        tone_phases = _get_current_per_tone_values(
            get_tone_phases, r, config_dict, num_tones)

    tone_frequency_settings, details = prepare_tone_frequency_settings(
        r, config_dict, tone_frequencies,
        tone_indices=tone_indices,
        min_tone_separation=min_tone_separation,
        tone_amplitudes=tone_amplitudes,
        tone_phases=tone_phases,
        compensate_rx_ticks=compensate_rx_ticks)
    apply_tone_frequency_settings(r, tone_frequency_settings, autosync=autosync, mrst=mrst)

    if detailed_output:
        return details
    else:
        return

def set_tone_frequencies_fast(r, r_fast, config_dict, tone_frequencies,
                              tone_indices=None, min_tone_separation=6,
                              autosync=False, mrst=False, tone_amplitudes=None,
                              tone_phases=None, compensate_rx_ticks=0):
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
    compensate_rx_ticks: if non-zero, add a per-tone RX phase offset to cancel the
                         RX-vs-TX path delay (in 307.2 MHz clock ticks) seen when
                         retuning without a sync (e.g. autosync=False). Pass
                         firmware_lib.TX_RX_PATH_DELAY_TICKS for the measured delay.
    """
    num_tones = len(np.atleast_1d(tone_frequencies))
    if tone_amplitudes is None:
        tone_amplitudes = _get_current_per_tone_values(
            get_tone_amplitudes, r, config_dict, num_tones)
    if tone_phases is None:
        tone_phases = _get_current_per_tone_values(
            get_tone_phases, r, config_dict, num_tones)

    tone_frequency_settings = prepare_tone_frequency_settings_fast(
        r_fast, config_dict, tone_frequencies,
        tone_indices=tone_indices,
        min_tone_separation=min_tone_separation,
        tone_amplitudes=tone_amplitudes,
        tone_phases=tone_phases,
        compensate_rx_ticks=compensate_rx_ticks)
    apply_tone_frequency_settings_fast(r, r_fast, tone_frequency_settings, autosync=autosync, mrst=mrst)

    return tone_frequency_settings

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



def get_tone_amplitudes(r, r_fast, config_dict):
    """
    Query the RFSOC for the current TX tone amplitude scale factors.
    """
    rd = r_fast if r_fast is not None else r
    # moved from outmap to inmap in the v7.9 psb_chanselect
    chanmap_psb_inmap = psb_chanselect_get_channel_inmap(rd)
    chanmap_pfb = chanselect_get_channel_outmap(rd)

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

    control_buffer = read_from_current_control_buffer(rd)
    scaling_tx = control_buffer['tx']['scaling']
    # index by psb_tones_active (not :num_tones) since tone indices may be non-contiguous with VACC
    return scaling_tx[psb_tones_active]

def set_tone_amplitudes(r, config_dict, tone_amplitudes,autosync=False, mrst=False):
    """
    Set the TX tone amplitude scale factors in the RFSOC.

    RX tone amplitude scale factors are kept at unity so readout-side LO
    scaling stays at maximum while TX power is controlled independently.

    Currently updates both halves of the double buffer.
    """
    tone_amplitudes = np.atleast_1d(tone_amplitudes)

    # Get active tone indices - with VACC these may be non-contiguous
    chanmap_psb_inmap = psb_chanselect_get_channel_inmap(r)
    psb_discard_bit = r.psb_chanselect.DISCARD_BIT
    psb_tones_active = np.nonzero((chanmap_psb_inmap & psb_discard_bit) == 0)[0]

    if len(tone_amplitudes) != len(psb_tones_active):
        raise ValueError(f'Number of amplitudes ({len(tone_amplitudes)}) does not match number of active tones ({len(psb_tones_active)})')

    # Create full-sized arrays and place values at correct LO indices.
    scaling_tx_full = np.zeros(r.mixer.n_chans, dtype=float)
    scaling_rx_full = np.zeros(r.mixer.n_chans, dtype=float)
    scaling_tx_full[psb_tones_active] = tone_amplitudes
    scaling_rx_full[psb_tones_active] = 1.0

    # buf = get_control_buffer_idx(r)
    for buf in [0,1]:
        v = prepare_control_buffer_data(r,buf,{'tx':{'scaling':scaling_tx_full},
                                    'rx':{'scaling':scaling_rx_full}})

        write_control_buffer_data(r,buf,v)


    # scaling = _format_amp_scale(tone_amplitudes, r.mixer._n_scale_bits)
    # for i in range(min(r.mixer._n_parallel_chans, num_tones)):
    #     r.mixer.write(f'tx_lo{i}_scale', scaling[i::r.mixer._n_parallel_chans].tobytes())
    #     r.mixer.write(f'rx_lo{i}_scale', scaling[i::r.mixer._n_parallel_chans].tobytes())

    _sync_if_requested(r, autosync=autosync, mrst=mrst)

    return

def get_tone_phases(r, r_fast, config_dict):
    """
    Query the RFSOC for the current tone phase offsets.
    Note that the returned values are in the range [-pi,pi] regardless of how they were set.
    """
    rd = r_fast if r_fast is not None else r
    # moved from outmap to inmap in the v7.9 psb_chanselect
    chanmap_psb_inmap = psb_chanselect_get_channel_inmap(rd)
    chanmap_pfb = chanselect_get_channel_outmap(rd)

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

    control_buffer = read_from_current_control_buffer(rd)
    phase_offsets_tx = control_buffer['tx']['phase_offsets']
    phase_offsets_rx = control_buffer['rx']['phase_offsets']
    # index by psb_tones_active (not :num_tones) since tone indices may be non-contiguous with VACC
    return phase_offsets_tx[psb_tones_active]

def set_tone_phases(r, config_dict, tone_phases, autosync=False, mrst=False):
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
    _sync_if_requested(r, autosync=autosync, mrst=mrst)
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


def psb_chanselect_set_channel_inmap_slow(r, inmap):
    """
    Slow KATCP/builtin version of psb_chanselect_set_channel_inmap.
    Kept for comparing the fast direct-memory implementation at runtime.
    """
    r.psb_chanselect.set_channel_inmap(inmap)


def psb_chanselect_get_channel_inmap_slow(r):
    """
    Slow KATCP/builtin version of psb_chanselect_get_channel_inmap.
    Kept for comparing the fast direct-memory implementation at runtime.
    """
    return r.psb_chanselect.get_channel_inmap()


def psb_chanselect_set_channel_inmap(r, inmap):
    """
    Remap the channels such that input channel `i`
    contributes to output channel `inmap[i]`

    :param inmap: The mapping of input to output data. I.e.,
        if `inmap[16] = 0` then input channel 16 will contribute to
        output channel 0.
    :type inmap: list
    """
    psb = r.psb_chanselect

    if not hasattr(psb, '_fast_inmap_cached'):
        n_exp = psb._expansion_factor
        reorder_depth = psb._reorder_depth
        n_parallel_samples = psb.n_parallel_samples
        n_parallel_chans_out = psb.n_parallel_chans_out
        n_chans_out = psb.n_chans_out
        discard_bin = np.uint32(psb.DISCARD_BIN)
        outchans = np.arange(n_chans_out, dtype=np.int64)
        block_id = ((outchans // n_parallel_samples) % n_exp).astype(np.intp)
        block_offset = (
            (outchans // n_parallel_chans_out) * n_parallel_samples
            + (outchans % n_parallel_samples)
        ).astype(np.intp)

        serial_maps_blank = np.empty((n_exp, reorder_depth), dtype=np.uint32)
        serial_maps_blank[:] = discard_bin
        offset_to_outchan = np.empty((n_exp, reorder_depth), dtype=np.uint32)
        offset_to_outchan[:] = discard_bin
        offset_to_outchan[block_id, block_offset] = outchans.astype(np.uint32)
        map_regs = [f'map{i}_{psb._map_reg}' for i in range(n_exp)]

        psb._fast_inmap_n_exp = n_exp
        psb._fast_inmap_reorder_depth = reorder_depth
        psb._fast_inmap_n_chans_in = psb.n_chans_in
        psb._fast_inmap_n_chans_out = n_chans_out
        psb._fast_inmap_discard_bin = discard_bin
        psb._fast_inmap_discard_bit = np.uint32(psb.DISCARD_BIT)
        psb._fast_inmap_addr_mask = np.uint32(psb.ADDR_MASK)
        psb._fast_inmap_block_id = block_id
        psb._fast_inmap_block_offset = block_offset
        psb._fast_inmap_serial_maps_blank = serial_maps_blank
        psb._fast_inmap_offset_to_outchan = offset_to_outchan
        psb._fast_inmap_input_idx = np.arange(psb.n_chans_in)
        psb._fast_inmap_map_regs = map_regs
        psb._fast_inmap_fast_nbytes = reorder_depth * 4
        psb._fast_inmap_fast_dtype = np.dtype('<u4')
        psb._fast_inmap_slow_dtype = np.dtype('u4').newbyteorder(np.dtype(psb._map_format).byteorder)
        try:
            psb._fast_inmap_mm = psb.host.transport.axil_mm
            psb._fast_inmap_map_addrs = [
                psb.host.transport._get_device_address(f'{psb.prefix}{reg}')
                for reg in map_regs
            ]
            psb._fast_inmap_use_devmem = True
        except AttributeError:
            psb._fast_inmap_mm = None
            psb._fast_inmap_map_addrs = None
            psb._fast_inmap_use_devmem = False
        psb._fast_inmap_cached = True

    n_chans_in = psb._fast_inmap_n_chans_in
    n_chans_out = psb._fast_inmap_n_chans_out
    discard_bin = psb._fast_inmap_discard_bin
    discard_bit = psb._fast_inmap_discard_bit

    inmap_i64 = np.asarray(inmap, dtype=np.int64)
    if inmap_i64.ndim != 1:
        raise ValueError('PSB channel inmap must be a 1D array')
    if len(inmap_i64) > n_chans_in:
        raise ValueError(f'PSB channel inmap has {len(inmap_i64)} entries, '
                         f'but firmware has only {n_chans_in} input channels')

    serial_maps = psb._fast_inmap_serial_maps_blank.copy()

    inmap_u32 = np.empty(len(inmap_i64), dtype=np.uint32)
    inmap_u32[:] = discard_bin
    non_negative = inmap_i64 >= 0
    inmap_u32[non_negative] = inmap_i64[non_negative].astype(np.uint32)
    valid = non_negative & ((inmap_u32 & discard_bit) == 0)

    valid_outputs = inmap_i64[valid]
    if np.any(valid_outputs >= n_chans_out):
        bad = int(valid_outputs[valid_outputs >= n_chans_out][0])
        raise ValueError(f'PSB channel inmap contains invalid output channel {bad}; '
                         f'max valid channel is {n_chans_out - 1}')

    input_indices = np.nonzero(valid)[0].astype(np.intp)
    if len(input_indices):
        valid_outputs = valid_outputs.astype(np.intp)
        serial_maps[psb._fast_inmap_block_id[valid_outputs], input_indices] = (
            psb._fast_inmap_block_offset[valid_outputs].astype(np.uint32)
        )

    if psb._fast_inmap_use_devmem:
        mm = psb._fast_inmap_mm
        fast_dtype = psb._fast_inmap_fast_dtype
        fast_nbytes = psb._fast_inmap_fast_nbytes
        for i, addr in enumerate(psb._fast_inmap_map_addrs):
            data = serial_maps[i].astype(fast_dtype, copy=False).tobytes()
            mm[addr:addr + fast_nbytes] = data
    else:
        slow_dtype = psb._fast_inmap_slow_dtype
        for i, reg in enumerate(psb._fast_inmap_map_regs):
            psb.write(reg, serial_maps[i].astype(slow_dtype, copy=False).tobytes())


def psb_chanselect_get_channel_inmap(r):
    """
    Get the currently loaded reorder map.
    :return: The reorder map currently loaded. Entry `i` in this map
        corresponds to the output channel to which input `i` contributes.
    :rtype: list
    """
    psb = r.psb_chanselect

    if not hasattr(psb, '_fast_inmap_cached'):
        n_exp = psb._expansion_factor
        reorder_depth = psb._reorder_depth
        n_parallel_samples = psb.n_parallel_samples
        n_parallel_chans_out = psb.n_parallel_chans_out
        n_chans_out = psb.n_chans_out
        discard_bin = np.uint32(psb.DISCARD_BIN)
        outchans = np.arange(n_chans_out, dtype=np.int64)
        block_id = ((outchans // n_parallel_samples) % n_exp).astype(np.intp)
        block_offset = (
            (outchans // n_parallel_chans_out) * n_parallel_samples
            + (outchans % n_parallel_samples)
        ).astype(np.intp)
        offset_to_outchan = np.empty((n_exp, reorder_depth), dtype=np.uint32)
        offset_to_outchan[:] = discard_bin
        offset_to_outchan[block_id, block_offset] = outchans.astype(np.uint32)
        serial_maps_blank = np.empty((n_exp, reorder_depth), dtype=np.uint32)
        serial_maps_blank[:] = discard_bin
        map_regs = [f'map{i}_{psb._map_reg}' for i in range(n_exp)]

        psb._fast_inmap_n_exp = n_exp
        psb._fast_inmap_reorder_depth = reorder_depth
        psb._fast_inmap_n_chans_in = psb.n_chans_in
        psb._fast_inmap_n_chans_out = n_chans_out
        psb._fast_inmap_discard_bin = discard_bin
        psb._fast_inmap_discard_bit = np.uint32(psb.DISCARD_BIT)
        psb._fast_inmap_addr_mask = np.uint32(psb.ADDR_MASK)
        psb._fast_inmap_block_id = block_id
        psb._fast_inmap_block_offset = block_offset
        psb._fast_inmap_serial_maps_blank = serial_maps_blank
        psb._fast_inmap_offset_to_outchan = offset_to_outchan
        psb._fast_inmap_input_idx = np.arange(psb.n_chans_in)
        psb._fast_inmap_map_regs = map_regs
        psb._fast_inmap_fast_nbytes = reorder_depth * 4
        psb._fast_inmap_fast_dtype = np.dtype('<u4')
        psb._fast_inmap_slow_dtype = np.dtype('u4').newbyteorder(np.dtype(psb._map_format).byteorder)
        try:
            psb._fast_inmap_mm = psb.host.transport.axil_mm
            psb._fast_inmap_map_addrs = [
                psb.host.transport._get_device_address(f'{psb.prefix}{reg}')
                for reg in map_regs
            ]
            psb._fast_inmap_use_devmem = True
        except AttributeError:
            psb._fast_inmap_mm = None
            psb._fast_inmap_map_addrs = None
            psb._fast_inmap_use_devmem = False
        psb._fast_inmap_cached = True

    n_exp = psb._fast_inmap_n_exp
    reorder_depth = psb._fast_inmap_reorder_depth
    n_chans_in = psb._fast_inmap_n_chans_in
    serial_maps = np.empty((n_exp, reorder_depth), dtype=np.uint32)
    if psb._fast_inmap_use_devmem:
        mm = psb._fast_inmap_mm
        fast_dtype = psb._fast_inmap_fast_dtype
        fast_nbytes = psb._fast_inmap_fast_nbytes
        for i, addr in enumerate(psb._fast_inmap_map_addrs):
            raw = mm[addr:addr + fast_nbytes]
            serial_maps[i] = np.frombuffer(raw, dtype=fast_dtype).copy()
    else:
        slow_dtype = psb._fast_inmap_slow_dtype
        nbytes = reorder_depth * slow_dtype.itemsize
        for i, reg in enumerate(psb._fast_inmap_map_regs):
            raw = psb.read(reg, nbytes)
            serial_maps[i] = np.frombuffer(raw, dtype=slow_dtype).astype(np.uint32)

    serial_maps = serial_maps[:, :n_chans_in]
    is_valid = (serial_maps & psb._fast_inmap_discard_bit) == 0
    has_mapping = np.any(is_valid, axis=0)
    first_exp = np.argmax(is_valid, axis=0)
    stored = (serial_maps[first_exp, psb._fast_inmap_input_idx] & psb._fast_inmap_addr_mask).astype(np.intp)

    inmap = np.empty(n_chans_in, dtype=np.uint32)
    inmap[:] = psb._fast_inmap_discard_bin

    stored_ok = stored < reorder_depth
    valid = has_mapping & stored_ok
    if np.any(valid):
        inmap[valid] = psb._fast_inmap_offset_to_outchan[first_exp[valid], stored[valid]]

    invalid = has_mapping & ~stored_ok
    if np.any(invalid):
        logger = getattr(psb, 'logger', None)
        if logger is not None:
            bad_input = int(np.nonzero(invalid)[0][0])
            logger.warning(
                'psb_chanselect_get_channel_inmap: unexpected mapping value '
                '0x%08x for input %d; treating as discard',
                int(serial_maps[first_exp[bad_input], bad_input]),
                bad_input,
            )

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


















def chanselect_set_channel_outmap_slow(r, outmap, descramble_input=None):
    """
    Slow KATCP/builtin version of chanselect_set_channel_outmap.
    Kept for comparing the fast direct-memory implementation at runtime.
    """
    r.chanselect.set_channel_outmap(outmap, descramble_input=descramble_input)


def chanselect_get_channel_outmap_slow(r, descramble_input=None):
    """
    Slow KATCP/builtin version of chanselect_get_channel_outmap.
    Kept for comparing the fast direct-memory implementation at runtime.
    """
    return r.chanselect.get_channel_outmap(descramble_input=descramble_input)


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

    chanselect = r.chanselect
    if not hasattr(chanselect, '_fast_outmap_cached'):
        serial_map_blank = np.zeros(chanselect._reorder_depth, dtype=np.int32)
        parallel_map_blank = np.empty(chanselect._reorder_depth, dtype=np.int32)
        parallel_map_blank[:] = chanselect._reduction_factor + 1
        map_reg = f'map0_{chanselect._map_reg}'
        pmap_reg = 'pmap'

        chanselect._fast_outmap_reorder_depth = chanselect._reorder_depth
        chanselect._fast_outmap_n_chans_out = chanselect.n_chans_out
        chanselect._fast_outmap_n_parallel_chans_in = chanselect.n_parallel_chans_in
        chanselect._fast_outmap_n_parallel_samples = chanselect.n_parallel_samples
        chanselect._fast_outmap_disabled_parallel_path = chanselect._reduction_factor + 1
        chanselect._fast_outmap_max_input = len(chanselect._descramble_order)
        chanselect._fast_outmap_serial_map_blank = serial_map_blank
        chanselect._fast_outmap_parallel_map_blank = parallel_map_blank
        chanselect._fast_outmap_map_reg = map_reg
        chanselect._fast_outmap_pmap_reg = pmap_reg
        chanselect._fast_outmap_fast_nbytes = chanselect._reorder_depth * 4
        chanselect._fast_outmap_fast_dtype = np.dtype('<i4')
        chanselect._fast_outmap_slow_map_dtype = np.dtype(chanselect._map_format)
        chanselect._fast_outmap_slow_pmap_dtype = np.dtype(chanselect._pmap_format)
        try:
            chanselect._fast_outmap_mm = chanselect.host.transport.axil_mm
            chanselect._fast_outmap_map_addr = chanselect.host.transport._get_device_address(
                f'{chanselect.prefix}{map_reg}')
            chanselect._fast_outmap_pmap_addr = chanselect.host.transport._get_device_address(
                f'{chanselect.prefix}{pmap_reg}')
            chanselect._fast_outmap_use_devmem = True
        except AttributeError:
            chanselect._fast_outmap_mm = None
            chanselect._fast_outmap_map_addr = None
            chanselect._fast_outmap_pmap_addr = None
            chanselect._fast_outmap_use_devmem = False
        chanselect._fast_outmap_cached = True

    n_chans_out = chanselect._fast_outmap_n_chans_out
    max_input = chanselect._fast_outmap_max_input

    outmap = np.asarray(outmap, dtype=int).copy()
    nout = len(outmap)
    if nout > n_chans_out:
        raise ValueError(f'PFB channel outmap has {nout} entries, '
                         f'but firmware has only {n_chans_out} output channels')

    outmap_isnt_n1 = outmap != -1
    if np.any(outmap < -1):
        bad = int(outmap[outmap < -1][0])
        raise ValueError(f'PFB channel outmap contains invalid channel {bad}')

    if np.any(outmap[outmap_isnt_n1] >= max_input):
        bad = int(outmap[outmap_isnt_n1][outmap[outmap_isnt_n1] >= max_input][0])
        raise ValueError(f'PFB channel outmap contains invalid input channel {bad}; '
                         f'max valid channel is {max_input - 1}')

    if descramble_input or (descramble_input is None and chanselect._descramble_default):
        outmap[outmap_isnt_n1] = chanselect._descramble_order[outmap[outmap_isnt_n1]]

    serial_map = chanselect._fast_outmap_serial_map_blank.copy()
    parallel_map = chanselect._fast_outmap_parallel_map_blank.copy()

    output_positions = np.nonzero(outmap_isnt_n1)[0].astype(np.intp)
    selected_inputs = outmap[outmap_isnt_n1].astype(np.intp)
    if len(selected_inputs):
        n_parallel_chans_in = chanselect._fast_outmap_n_parallel_chans_in
        n_parallel_samples = chanselect._fast_outmap_n_parallel_samples
        block_id = selected_inputs // n_parallel_chans_in
        input_parallel_position = selected_inputs % n_parallel_chans_in
        block_s_offset = input_parallel_position % n_parallel_samples
        block_p_offset = input_parallel_position // n_parallel_samples

        serial_map[output_positions] = (
            block_id * n_parallel_samples + block_s_offset
        ).astype(np.int32)
        parallel_map[output_positions] = block_p_offset.astype(np.int32)

    if chanselect._fast_outmap_use_devmem:
        mm = chanselect._fast_outmap_mm
        fast_dtype = chanselect._fast_outmap_fast_dtype
        fast_nbytes = chanselect._fast_outmap_fast_nbytes
        data = serial_map.astype(fast_dtype, copy=False).tobytes()
        mm[chanselect._fast_outmap_map_addr:chanselect._fast_outmap_map_addr + fast_nbytes] = data

        data = parallel_map.astype(fast_dtype, copy=False).tobytes()
        mm[chanselect._fast_outmap_pmap_addr:chanselect._fast_outmap_pmap_addr + fast_nbytes] = data
    else:
        chanselect.write(chanselect._fast_outmap_map_reg,
                         serial_map.astype(chanselect._fast_outmap_slow_map_dtype, copy=False).tobytes())
        chanselect.write(chanselect._fast_outmap_pmap_reg,
                         parallel_map.astype(chanselect._fast_outmap_slow_pmap_dtype, copy=False).tobytes())



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

    chanselect = r.chanselect
    if not hasattr(chanselect, '_fast_outmap_cached'):
        serial_map_blank = np.zeros(chanselect._reorder_depth, dtype=np.int32)
        parallel_map_blank = np.empty(chanselect._reorder_depth, dtype=np.int32)
        parallel_map_blank[:] = chanselect._reduction_factor + 1
        map_reg = f'map0_{chanselect._map_reg}'
        pmap_reg = 'pmap'

        chanselect._fast_outmap_reorder_depth = chanselect._reorder_depth
        chanselect._fast_outmap_n_chans_out = chanselect.n_chans_out
        chanselect._fast_outmap_n_parallel_chans_in = chanselect.n_parallel_chans_in
        chanselect._fast_outmap_n_parallel_samples = chanselect.n_parallel_samples
        chanselect._fast_outmap_disabled_parallel_path = chanselect._reduction_factor + 1
        chanselect._fast_outmap_max_input = len(chanselect._descramble_order)
        chanselect._fast_outmap_serial_map_blank = serial_map_blank
        chanselect._fast_outmap_parallel_map_blank = parallel_map_blank
        chanselect._fast_outmap_map_reg = map_reg
        chanselect._fast_outmap_pmap_reg = pmap_reg
        chanselect._fast_outmap_fast_nbytes = chanselect._reorder_depth * 4
        chanselect._fast_outmap_fast_dtype = np.dtype('<i4')
        chanselect._fast_outmap_slow_map_dtype = np.dtype(chanselect._map_format)
        chanselect._fast_outmap_slow_pmap_dtype = np.dtype(chanselect._pmap_format)
        try:
            chanselect._fast_outmap_mm = chanselect.host.transport.axil_mm
            chanselect._fast_outmap_map_addr = chanselect.host.transport._get_device_address(
                f'{chanselect.prefix}{map_reg}')
            chanselect._fast_outmap_pmap_addr = chanselect.host.transport._get_device_address(
                f'{chanselect.prefix}{pmap_reg}')
            chanselect._fast_outmap_use_devmem = True
        except AttributeError:
            chanselect._fast_outmap_mm = None
            chanselect._fast_outmap_map_addr = None
            chanselect._fast_outmap_pmap_addr = None
            chanselect._fast_outmap_use_devmem = False
        chanselect._fast_outmap_cached = True

    reorder_depth = chanselect._fast_outmap_reorder_depth
    if chanselect._fast_outmap_use_devmem:
        mm = chanselect._fast_outmap_mm
        fast_dtype = chanselect._fast_outmap_fast_dtype
        fast_nbytes = chanselect._fast_outmap_fast_nbytes

        raw = mm[chanselect._fast_outmap_map_addr:chanselect._fast_outmap_map_addr + fast_nbytes]
        serial_map = np.frombuffer(raw, dtype=fast_dtype).copy()

        raw = mm[chanselect._fast_outmap_pmap_addr:chanselect._fast_outmap_pmap_addr + fast_nbytes]
        parallel_map = np.frombuffer(raw, dtype=fast_dtype).copy()
    else:
        slow_map_dtype = chanselect._fast_outmap_slow_map_dtype
        slow_pmap_dtype = chanselect._fast_outmap_slow_pmap_dtype
        nbytes = reorder_depth * slow_map_dtype.itemsize
        raw = chanselect.read(chanselect._fast_outmap_map_reg, nbytes)
        serial_map = np.frombuffer(raw, dtype=slow_map_dtype).astype(np.int32)

        nbytes = reorder_depth * slow_pmap_dtype.itemsize
        raw = chanselect.read(chanselect._fast_outmap_pmap_reg, nbytes)
        parallel_map = np.frombuffer(raw, dtype=slow_pmap_dtype).astype(np.int32)

    n_parallel_samples = chanselect._fast_outmap_n_parallel_samples
    block_id = serial_map // n_parallel_samples
    block_s_offset = serial_map % n_parallel_samples
    block_p_offset = parallel_map

    outmap = (
        chanselect._fast_outmap_n_parallel_chans_in * block_id
        + block_s_offset
        + (n_parallel_samples * block_p_offset)
    )
    outmap[parallel_map == chanselect._fast_outmap_disabled_parallel_path] = -1
    if descramble_input or (descramble_input is None and chanselect._descramble_default):
        # for i in range(len(outmap)):
        #     if outmap[i] == -1:
        #         continue
        #     outmap[i] = chanselect._scramble_order[outmap[i]]

        outmap_isnt_n1 = outmap != -1
        outmap[outmap_isnt_n1] = chanselect._scramble_order[outmap[outmap_isnt_n1]]

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


def _resolve_optional_cal_value(value, freq_axis):
    """Resolve a calibration parameter, preserving None as unavailable."""
    if value is None:
        return None
    return _resolve_cal_value(value, freq_axis)


def _mean_cal_value_db(value):
    """Return a scalar dB value for control decisions."""
    arr = np.asarray(value, dtype=float)
    finite = arr[np.isfinite(arr)]
    if finite.size == 0:
        return float('nan')
    return float(np.mean(finite))


def _gather_bypass_amp_s21_for_state(config_dict, rf_peripherals, path,
                                     freq_axis, bypassed):
    """Return bypass-amp S21 for an explicit bypass state."""
    rf_cfg = config_dict.get('rf_frontend', {}) or {}
    mixerless_cfg = rf_cfg.get('mixerless_module', {}) or {}
    direct_key = (
        f'{path}_amp_bypassed_s21_db'
        if bypassed else f'{path}_amp_enabled_s21_db'
    )
    direct = _resolve_optional_cal_value(mixerless_cfg.get(direct_key), freq_axis)
    if direct is not None:
        return direct

    delta = _resolve_optional_cal_value(
        mixerless_cfg.get(f'{path}_amp_bypass_delta_s21_db'), freq_axis)
    if delta is not None:
        enabled = _resolve_optional_cal_value(
            mixerless_cfg.get(f'{path}_amp_enabled_s21_db'), freq_axis)
        bypass_state = _resolve_optional_cal_value(
            mixerless_cfg.get(f'{path}_amp_bypassed_s21_db'), freq_axis)
        if bypassed and enabled is not None:
            return enabled + delta
        if not bypassed and bypass_state is not None:
            return bypass_state - delta

    if not _rf_supports_bypass_amps(rf_peripherals):
        return 0

    dev_name = 'transmit_atten' if path == 'tx' else 'recv_atten'
    get_bypass = (
        rf_peripherals.get_tx_amp_bypass if path == 'tx'
        else rf_peripherals.get_rx_amp_bypass
    )
    set_bypass = (
        rf_peripherals.set_tx_amp_bypass if path == 'tx'
        else rf_peripherals.set_rx_amp_bypass
    )
    original_state = bool(get_bypass())
    if original_state != bypassed:
        set_bypass(bypassed)
        time.sleep(0.1)
    try:
        return rf_peripherals._get_amp_s21(dev_name)
    finally:
        if bool(get_bypass()) != original_state:
            set_bypass(original_state)
            time.sleep(0.1)


def _gather_bypass_amp_s21(config_dict, rf_peripherals, path, freq_axis):
    """Return the bypass-amp S21 for TX/RX, preferring measured config.

    For mixerless hardware, the submodule can model the amp-enabled and
    bypassed S21 values.  Measured values are better for absolute power
    calibration, so optional per-state config keys take priority:
    ``tx_amp_enabled_s21_db`` / ``tx_amp_bypassed_s21_db`` and RX equivalents.
    """
    if path not in ('tx', 'rx'):
        raise ValueError("path must be 'tx' or 'rx'")

    if _rf_supports_bypass_amps(rf_peripherals):
        get_bypass = (
            rf_peripherals.get_tx_amp_bypass if path == 'tx'
            else rf_peripherals.get_rx_amp_bypass
        )
        bypassed = bool(get_bypass())
        state_value = _gather_bypass_amp_s21_for_state(
            config_dict, rf_peripherals, path, freq_axis, bypassed)
        return state_value

    bypass_cfg = (
        config_dict.get('rf_frontend', {}).get('bypass_amps', {}) or {}
    )
    bypassed = bool(bypass_cfg.get(f'{path}_amp_bypass', True))
    return _gather_bypass_amp_s21_for_state(
        config_dict, rf_peripherals, path, freq_axis, bypassed)


def _calibration_report_estimate_db(value, freq_axis=None):
    """Return a scalar dB estimate for report-only calibration summaries."""
    if value is None:
        return None
    try:
        if freq_axis is not None:
            return _mean_cal_value_db(
                _resolve_optional_cal_value(value, freq_axis))
        if isinstance(value, str):
            try:
                return float(value)
            except ValueError:
                path = os.path.join(USER_DIR, value)
                try:
                    value = np.loadtxt(path, ndmin=2)
                except ValueError:
                    value = np.loadtxt(path, ndmin=2, delimiter=',')
        if not np.isscalar(value):
            arr = np.asarray(value, dtype=float)
            if arr.ndim >= 2 and arr.shape[-1] >= 2:
                value = arr[..., 1]
            else:
                value = arr
        return _mean_cal_value_db(value)
    except (OSError, TypeError, ValueError):
        return None


def estimate_rf_bypass_amp_s21_from_config(config_dict, path, bypassed,
                                           freq_axis=None):
    """Estimate active TX/RX bypass-amp S21 from measured config values."""
    if path not in ('tx', 'rx'):
        raise ValueError("path must be 'tx' or 'rx'")

    rf_cfg = config_dict.get('rf_frontend', {}) or {}
    mixerless_cfg = rf_cfg.get('mixerless_module', {}) or {}
    direct_key = (
        f'{path}_amp_bypassed_s21_db'
        if bypassed else f'{path}_amp_enabled_s21_db'
    )
    direct = _calibration_report_estimate_db(
        mixerless_cfg.get(direct_key), freq_axis=freq_axis)
    if direct is not None:
        return direct

    delta = _calibration_report_estimate_db(
        mixerless_cfg.get(f'{path}_amp_bypass_delta_s21_db'),
        freq_axis=freq_axis)
    if delta is None:
        return None

    enabled = _calibration_report_estimate_db(
        mixerless_cfg.get(f'{path}_amp_enabled_s21_db'), freq_axis=freq_axis)
    bypassed_s21 = _calibration_report_estimate_db(
        mixerless_cfg.get(f'{path}_amp_bypassed_s21_db'), freq_axis=freq_axis)
    if bypassed and enabled is not None:
        return enabled + delta
    if not bypassed and bypassed_s21 is not None:
        return bypassed_s21 - delta
    return None


def estimate_rf_total_gain_from_config(config_dict, rf_status, path,
                                       freq_axis=None):
    """Estimate active TX/RX frontend gain from measured config values."""
    if path not in ('tx', 'rx'):
        raise ValueError("path must be 'tx' or 'rx'")
    bypassed = rf_status.get(f'{path}_amp_bypass')
    attenuation = rf_status.get(f'{path}_attenuation_db')
    if bypassed is None or attenuation is None:
        return None
    amp_s21 = estimate_rf_bypass_amp_s21_from_config(
        config_dict, path, bool(bypassed), freq_axis=freq_axis)
    if amp_s21 is None:
        return None
    return amp_s21 - abs(float(attenuation))


def _gather_tx_chain_params(r, r_fast, config_dict, rf_peripherals=None):
    """Read firmware state and resolve all TX chain calibration parameters.

    Returns a dict with everything needed by calibration.calc_tone_powers /
    calc_tone_amplitudes, plus metadata (freqs, freq_details, connectivity).

    When rf_peripherals is provided, live attenuator/amp values are read
    from hardware, overriding any None values in config_dict.
    """
    dac_tile = int(config_dict['firmware']['dac0_tile'])
    dac_block = int(config_dict['firmware']['dac0_block'])

    freqs, freq_details = get_tone_frequencies(r, r_fast, config_dict, detailed_output=True)
    analog_freq = freq_details['tx']['analog_output_freq']
    rf_freq = freq_details['tx']['rf_output_freq']

    # Live firmware state
    amps = get_tone_amplitudes(r, r_fast, config_dict)
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

    has_rf = _rf_has_readable_attenuator(rf_peripherals)
    has_live_rf = has_rf and getattr(rf_peripherals, 'is_hardware', False)

    # Resolve calibration values (scalar / file / array → per-tone)
    dac_dbfs_to_dbm = _resolve_cal_value(config_dict['firmware']['dac0_dbfs_to_dbm'], analog_freq)
    tx_combiner_loss_db = _resolve_cal_value(config_dict['rf_frontend']['tx_combiner_loss_db'], analog_freq)

    # TX attenuator: read live hardware state when available, else config, else 0.
    tx_attenuator_value_db = None
    if has_live_rf:
        tx_attenuator_value_db = rf_peripherals.get_tx_attenuation()
    if tx_attenuator_value_db is None:
        tx_attenuator_value_db = config_dict['rf_frontend'].get('attenuator', {}).get('tx_value_db')
    if tx_attenuator_value_db is None:
        tx_attenuator_value_db = 0

    tx_if_s21_db = _resolve_cal_value(config_dict['rf_frontend']['tx_if_s21_db'], analog_freq)
    tx_mixer_conversion_loss_db = _resolve_cal_value(config_dict['rf_frontend']['tx_mixer_conversion_loss_db'], analog_freq)
    tx_rf_s21_db = _resolve_cal_value(config_dict['rf_frontend']['tx_rf_s21_db'], rf_freq)

    tx_bypass_amp_s21_db = _gather_bypass_amp_s21(
        config_dict, rf_peripherals, 'tx', rf_freq)

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
    _, freq_details = get_tone_frequencies(r, None, config_dict, detailed_output=True)
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

RX_POLICIES = ('protect', 'compensate', 'maximise', 'raise', 'none')


def _rf_supports_bypass_amps(rf_peripherals):
    """True when the active RF frontend has controllable bypass amps."""
    return bool(
        rf_peripherals is not None
        and rf_peripherals.enabled
        and getattr(rf_peripherals, 'supports_bypass_amps', False)
    )


def _rf_has_controllable_attenuator(rf_peripherals):
    """True when the active RF attenuator can be changed by software."""
    return bool(
        rf_peripherals is not None
        and rf_peripherals.enabled
        and getattr(rf_peripherals, 'is_controllable', False)
    )


def _rf_has_readable_attenuator(rf_peripherals):
    """True when attenuation can be read from hardware or fixed config."""
    return bool(
        rf_peripherals is not None
        and rf_peripherals.enabled
        and (
            getattr(rf_peripherals, 'is_hardware', False)
            or getattr(rf_peripherals, 'attenuator_backend', None) == 'fixed'
        )
    )


def _validate_control_scope(digital_only=False, rf_only=False):
    """Validate mutually-exclusive digital/RF control-scope flags."""
    digital_only = bool(digital_only)
    rf_only = bool(rf_only)
    if digital_only and rf_only:
        raise ValueError('digital_only and rf_only are mutually exclusive')
    return digital_only, rf_only


POWER_FORCE_CONTROL_KEYS = (
    'force_tx_amp_bypass',
    'force_rx_amp_bypass',
    'force_tx_attenuation_db',
    'force_rx_attenuation_db',
    'force_adc_dsa_db',
    'force_tone_amplitudes',
    'force_psb_fftshift',
    'force_psb_scale',
    'force_pfb_fftshift',
)


def _coerce_forced_bool(value, name):
    """Return a bool for a forced-control value, accepting common strings."""
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ('true', '1', 'yes', 'on'):
            return True
        if text in ('false', '0', 'no', 'off'):
            return False
        raise ValueError(f'{name} must be boolean, got {value!r}')
    return bool(value)


def _set_adc_dsa_db(r, config_dict, value):
    """Set ADC DSA to an integer dB value and return the applied setting."""
    if config_dict is None:
        raise ValueError('config_dict is required to set ADC DSA')
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    dsa = int(np.clip(round(float(value)), 0, 27))
    r.rfdc.core.set_dsa(adc_tile, adc_block, dsa)
    time.sleep(0.1)
    return dsa


def _apply_forced_power_controls(
        r, config_dict, rf_peripherals=None,
        force_tx_amp_bypass=None, force_rx_amp_bypass=None,
        force_tx_attenuation_db=None, force_rx_attenuation_db=None,
        force_adc_dsa_db=None, force_tone_amplitudes=None,
        force_psb_fftshift=None, force_psb_scale=None,
        force_pfb_fftshift=None):
    """Apply fixed-value controls requested by a power optimisation caller."""
    applied = {}

    if force_tx_amp_bypass is not None:
        if not _rf_supports_bypass_amps(rf_peripherals):
            raise RuntimeError('Cannot force TX amp bypass: bypass amps are not available')
        value = _coerce_forced_bool(force_tx_amp_bypass, 'force_tx_amp_bypass')
        rf_peripherals.set_tx_amp_bypass(value)
        time.sleep(0.1)
        applied['tx_amp_bypass'] = value
        print(f'  forced TX amp bypass={value}')

    if force_rx_amp_bypass is not None:
        if not _rf_supports_bypass_amps(rf_peripherals):
            raise RuntimeError('Cannot force RX amp bypass: bypass amps are not available')
        value = _coerce_forced_bool(force_rx_amp_bypass, 'force_rx_amp_bypass')
        rf_peripherals.set_rx_amp_bypass(value)
        time.sleep(0.1)
        applied['rx_amp_bypass'] = value
        print(f'  forced RX amp bypass={value}')

    if force_tx_attenuation_db is not None:
        value = float(force_tx_attenuation_db)
        if _rf_has_controllable_attenuator(rf_peripherals):
            rf_peripherals.set_tx_attenuation(value)
            time.sleep(0.1)
        elif _rf_has_readable_attenuator(rf_peripherals):
            current = float(rf_peripherals.get_tx_attenuation())
            if not np.isclose(current, value, atol=1e-6):
                raise RuntimeError(
                    f'Cannot force TX attenuation to {value:.1f} dB: '
                    f'attenuator is read-only at {current:.1f} dB')
        else:
            raise RuntimeError('Cannot force TX attenuation: no RF attenuator is available')
        applied['tx_attenuation_db'] = value
        print(f'  forced TX attenuation={value:.1f} dB')

    if force_rx_attenuation_db is not None:
        value = float(force_rx_attenuation_db)
        if _rf_has_controllable_attenuator(rf_peripherals):
            rf_peripherals.set_rx_attenuation(value)
            time.sleep(0.1)
        elif _rf_has_readable_attenuator(rf_peripherals):
            current = float(rf_peripherals.get_rx_attenuation())
            if not np.isclose(current, value, atol=1e-6):
                raise RuntimeError(
                    f'Cannot force RX attenuation to {value:.1f} dB: '
                    f'attenuator is read-only at {current:.1f} dB')
        else:
            raise RuntimeError('Cannot force RX attenuation: no RF attenuator is available')
        applied['rx_attenuation_db'] = value
        print(f'  forced RX attenuation={value:.1f} dB')

    if force_adc_dsa_db is not None:
        value = _set_adc_dsa_db(r, config_dict, force_adc_dsa_db)
        applied['adc_dsa_db'] = value
        print(f'  forced ADC DSA={value} dB')

    if force_tone_amplitudes is not None:
        if config_dict is None:
            raise ValueError('config_dict is required to set tone amplitudes')
        amplitudes = np.atleast_1d(force_tone_amplitudes).astype(float)
        current_amplitudes = get_tone_amplitudes(r, None, config_dict)
        if amplitudes.size == 1 and current_amplitudes.size > 1:
            amplitudes = np.full(current_amplitudes.size, amplitudes[0], dtype=float)
        set_tone_amplitudes(r, config_dict, amplitudes)
        time.sleep(0.01)
        applied['tone_amplitudes'] = amplitudes
        print(f'  forced tone amplitudes ({len(amplitudes)} tone(s))')

    if force_psb_fftshift is not None:
        value = int(force_psb_fftshift)
        r.psb.set_fftshift(value)
        time.sleep(0.01)
        applied['psb_fftshift'] = value
        print(f'  forced PSB fftshift={format(value, "#016b")}')

    if force_psb_scale is not None:
        value = float(force_psb_scale)
        r.psbscale.set_scale(value)
        time.sleep(0.01)
        applied['psb_scale'] = value
        print(f'  forced psb_scale={value:.6f}')

    if force_pfb_fftshift is not None:
        value = int(force_pfb_fftshift)
        r.pfb.set_fftshift(value)
        time.sleep(0.01)
        applied['pfb_fftshift'] = value
        print(f'  forced PFB fftshift={format(value, "#016b")}')

    return applied


def _sum_tone_powers_dbm(powers_dbm):
    """Sum independent per-tone powers in dBm and return total power in dBm."""
    powers = np.asarray(powers_dbm, dtype=float)
    powers = powers[np.isfinite(powers)]
    if powers.size == 0:
        return float('-inf')
    return float(10 * np.log10(np.sum(np.power(10.0, powers / 10.0))))


def _get_tx_input_1db_comp_dbm(rf_peripherals):
    """Return the TX chain input 1 dB compression point, if available."""
    if rf_peripherals is None or not getattr(rf_peripherals, 'enabled', False):
        return None
    getter = getattr(rf_peripherals, 'get_tx_input_1db_comp', None)
    if getter is None:
        return None
    try:
        value = getter()
    except Exception as exc:
        print(f'  WARNING: could not read TX input 1 dB compression point: {exc}')
        return None
    if value is None:
        return None
    value = float(value)
    if not np.isfinite(value):
        return None
    return value


def _calculate_current_tx_chain(r, config_dict, rf_peripherals=None):
    """Return current TX-chain parameters, detector powers, and stage details."""
    p = _gather_tx_chain_params(r, None, config_dict, rf_peripherals=rf_peripherals)
    tx_powers, tx_details = calibration.calc_tone_powers(
        p['amps'], p['psb_fftshift'], p['psb_scale'],
        p['mixer_scale_is_1p0'], p['mixer_qmc_gain'], p['vop_current'],
        p['vop_current_fs'], p['dac_dbfs_to_dbm'],
        p['tx_combiner_loss_db'], p['tx_attenuator_value_db'],
        p['tx_if_s21_db'], p['tx_mixer_conversion_loss_db'],
        p['tx_rf_s21_db'], p['tx_bypass_amp_s21_db'],
        p['cryostat_input_s21_db'], p['dac_fs_bits'],
        detailed_output=True)
    return p, tx_powers, tx_details


def _get_tx_compression_state(r, config_dict, rf_peripherals,
                              compression_headroom_db=None):
    """Estimate margin between composite TX input power and P1dB.

    The RF peripheral model reports input P1dB referred to the start of the
    controllable TX chain.  The matching power plane in calc_tone_powers is
    ``combiner_dbm``: after DAC/combiner loss and before TX attenuation.
    """
    requested_headroom = (
        None if compression_headroom_db is None
        else float(compression_headroom_db)
    )
    state = {
        'available': False,
        'compression_headroom_db': requested_headroom,
    }
    comp_dbm = _get_tx_input_1db_comp_dbm(rf_peripherals)
    if comp_dbm is None or config_dict is None:
        state['reason'] = 'TX input 1 dB compression point unavailable'
        return state

    _, _, details = _calculate_current_tx_chain(
        r, config_dict, rf_peripherals=rf_peripherals)
    tx_input_per_tone = np.asarray(details['combiner_dbm'], dtype=float)
    tx_input_total_dbm = _sum_tone_powers_dbm(tx_input_per_tone)
    finite = tx_input_per_tone[np.isfinite(tx_input_per_tone)]
    peak_tone_dbm = float(np.max(finite)) if finite.size else float('-inf')
    margin_db = comp_dbm - tx_input_total_dbm

    state.update({
        'available': True,
        'tx_input_1db_comp_dbm': float(comp_dbm),
        'tx_input_total_dbm': float(tx_input_total_dbm),
        'tx_input_peak_tone_dbm': peak_tone_dbm,
        'compression_margin_db': float(margin_db),
    })
    if requested_headroom is not None:
        limit_dbm = comp_dbm - requested_headroom
        excess_db = tx_input_total_dbm - limit_dbm
        state.update({
            'compression_limit_dbm': float(limit_dbm),
            'compression_excess_db': float(excess_db),
            'safe': bool(excess_db <= 0),
        })
    return state


def _enforce_tx_compression_margin(r, config_dict, rf_peripherals,
                                   compression_headroom_db,
                                   scalemin, scalemax):
    """Reduce psb_scale so total TX input power stays below P1dB margin."""
    if compression_headroom_db is None:
        return r.psbscale.get_scale(), None
    compression_headroom_db = float(compression_headroom_db)
    if compression_headroom_db < 0:
        raise ValueError('compression_headroom_db must be non-negative')

    state = _get_tx_compression_state(
        r, config_dict, rf_peripherals, compression_headroom_db)
    if not state.get('available'):
        print('  compression guard: skipped '
              f'({state.get("reason", "not available")})')
        return r.psbscale.get_scale(), state

    print(f'  compression guard: TX input {state["tx_input_total_dbm"]:.1f} dBm, '
          f'P1dB {state["tx_input_1db_comp_dbm"]:.1f} dBm, '
          f'margin {state["compression_margin_db"]:.1f} dB '
          f'(target {compression_headroom_db:.1f} dB)')
    excess_db = state.get('compression_excess_db', 0.0)
    if excess_db > 0.1:
        scale_before = r.psbscale.get_scale()
        scale_after = scale_before * 10**(-excess_db / 20)
        scale_after = float(np.clip(scale_after, scalemin, scalemax))
        r.psbscale.set_scale(scale_after)
        time.sleep(0.01)
        state = _get_tx_compression_state(
            r, config_dict, rf_peripherals, compression_headroom_db)
        state['psb_scale_before_compression_limit'] = float(scale_before)
        state['psb_scale_after_compression_limit'] = float(scale_after)
        if state.get('compression_excess_db', 0.0) > 0.1:
            state['limited_by_min_psb_scale'] = bool(scale_after <= scalemin)
            print('    WARNING: compression margin still exceeded by '
                  f'{state["compression_excess_db"]:.1f} dB')
        else:
            print(f'    psb_scale: {scale_before:.4f} -> {scale_after:.4f} '
                  'to satisfy compression margin')
    return r.psbscale.get_scale(), state


def _apply_rx_policy(r, r_fast, config_dict, rf_peripherals, rx_policy,
                     tx_power_change_db=None, digital_only=False, rf_only=False,
                     force_rx_amp_bypass=None, force_rx_attenuation_db=None,
                     force_adc_dsa_db=None, force_pfb_fftshift=None):
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

        ``'maximise'``
            Run :func:`maximise_rx_power` after the TX change.  This actively
            optimises the RX path for ADC dynamic range by adjusting RX
            attenuation, ADC DSA, RX amp bypass state, and PFB FFT shift.

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
    digital_only, rf_only = _validate_control_scope(digital_only, rf_only)
    allow_digital = not rf_only
    allow_rf = not digital_only
    force_rx_atten = force_rx_attenuation_db is not None
    force_dsa = force_adc_dsa_db is not None

    if rx_policy == 'none':
        return None

    if rx_policy == 'maximise':
        dsa, pfb_fftshift, dsp, adc, rx_atten = maximise_rx_power(
            r, r_fast, config_dict, rf_peripherals=rf_peripherals,
            digital_only=digital_only, rf_only=rf_only,
            force_rx_amp_bypass=force_rx_amp_bypass,
            force_rx_attenuation_db=force_rx_attenuation_db,
            force_adc_dsa_db=force_adc_dsa_db,
            force_pfb_fftshift=force_pfb_fftshift)
        if isinstance(adc, dict):
            threshold = float(adc.get('threshold', 0.45))
            peak = max(
                abs(float(adc.get('imax_fs', 0.0))),
                abs(float(adc.get('imin_fs', 0.0))),
                abs(float(adc.get('qmax_fs', 0.0))),
                abs(float(adc.get('qmin_fs', 0.0))),
            )
            saturated = bool(
                peak >= threshold
                or adc.get('rts_over_range', False)
                or adc.get('rts_over_voltage', False)
            )
        else:
            saturated = None
        action = (
            f'maximise: DSA={dsa} dB, '
            f'PFB fftshift={format(int(pfb_fftshift), "#016b")}'
        )
        if rx_atten is not None:
            action += f', RX atten={float(rx_atten):.1f} dB'
        print(f'    rx_policy: {action}')
        return {
            'policy': 'maximise',
            'saturated': saturated,
            'action': action,
            'rx_attenuation_db': None if rx_atten is None else float(rx_atten),
            'dsa_db': int(dsa),
            'pfb_fftshift': int(pfb_fftshift),
            'dsp_overflow': dsp,
            'adc_levels': adc,
        }

    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    DSA_MAX = 27
    has_rf = allow_rf and _rf_has_controllable_attenuator(rf_peripherals)

    # --- Compensate mode: proactively mirror TX change onto RX path ---
    if rx_policy == 'compensate' and tx_power_change_db is not None:
        # Positive tx_power_change_db means TX got louder → increase RX
        # attenuation by the same amount to keep round-trip constant.
        delta = tx_power_change_db
        rx_atten_change = 0.0
        dsa_change = 0.0

        if delta > 0.1:
            # TX power increased — add attenuation on RX side
            if has_rf and not force_rx_atten:
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

            if allow_digital and not force_dsa and delta > 0.5:
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
            if has_rf and not force_rx_atten:
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

            if allow_digital and not force_dsa and recover > 0.5:
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

    if has_rf and not force_rx_atten:
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

    if allow_digital and not force_dsa and saturated:
        current_dsa = init_dsa
        while saturated and current_dsa < DSA_MAX:
            current_dsa = min(current_dsa + 2, DSA_MAX)
            r.rfdc.core.set_dsa(adc_tile, adc_block, int(current_dsa))
            time.sleep(0.1)
            saturated, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
    elif saturated and force_dsa:
        print('    rx_policy: ADC DSA fixed; skipping DSA protection step')

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
        action = f'WARNING: ADC still saturating after allowed RX controls'
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
                      power_limit_dbm=None, compression_headroom_db=None,
                      rx_policy='protect', digital_only=False, rf_only=False,
                      force_tx_amp_bypass=None, force_rx_amp_bypass=None,
                      force_tx_attenuation_db=None,
                      force_rx_attenuation_db=None,
                      force_adc_dsa_db=None, force_tone_amplitudes=None,
                      force_psb_fftshift=None, force_psb_scale=None,
                      force_pfb_fftshift=None):
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
    compression_headroom_db : float or None
        Optional margin below the RF frontend TX input 1 dB compression
        point.  When set, the composite power into the TX frontend is kept
        at least this many dB below the modelled P1dB by reducing psb_scale.
        If None (default), no compression guard is applied.
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
        - ``'maximise'`` — run ``maximise_rx_power()`` after each TX
          power change to optimise the RX attenuator, DSA, RX amp, and
          PFB FFT shift.
        - ``'raise'`` — raise ``RuntimeError`` if ADC saturates.
        - ``'none'`` — do not check or touch the RX path.

        See :func:`_apply_rx_policy` for full details.
    digital_only : bool
        If True, only firmware/RFDC parameters are adjusted.  RF frontend
        attenuators and bypass amplifiers are left unchanged.
    rf_only : bool
        If True, only RF frontend attenuators and bypass amplifiers are
        adjusted.  Firmware/RFDC parameters are left unchanged.
    """
    if rx_policy not in RX_POLICIES:
        raise ValueError(
            f"rx_policy must be one of {RX_POLICIES}, got {rx_policy!r}")
    digital_only, rf_only = _validate_control_scope(digital_only, rf_only)
    allow_digital = not rf_only
    allow_rf = not digital_only
    if compression_headroom_db is not None:
        compression_headroom_db = float(compression_headroom_db)
        if compression_headroom_db < 0:
            raise ValueError('compression_headroom_db must be non-negative')
    print(f'maximise_tx_power: starting (reference_plane={reference_plane}, '
          f'digital_only={digital_only}, rf_only={rf_only})')
    if reference_plane not in ('dac', 'rf_output', 'detector'):
        raise ValueError(f"reference_plane must be 'dac', 'rf_output', or 'detector', "
                         f"got '{reference_plane}'")

    _apply_forced_power_controls(
        r, config_dict, rf_peripherals=rf_peripherals,
        force_tx_amp_bypass=force_tx_amp_bypass,
        force_rx_amp_bypass=force_rx_amp_bypass,
        force_tx_attenuation_db=force_tx_attenuation_db,
        force_rx_attenuation_db=force_rx_attenuation_db,
        force_adc_dsa_db=force_adc_dsa_db,
        force_tone_amplitudes=force_tone_amplitudes,
        force_psb_fftshift=force_psb_fftshift,
        force_psb_scale=force_psb_scale,
        force_pfb_fftshift=force_pfb_fftshift)

    has_rf = allow_rf and _rf_has_controllable_attenuator(rf_peripherals)
    tx_atten_fixed = force_tx_attenuation_db is not None
    tx_amp_fixed = force_tx_amp_bypass is not None
    tone_amplitudes_fixed = force_tone_amplitudes is not None
    psb_fftshift_fixed = force_psb_fftshift is not None
    psb_scale_fixed = force_psb_scale is not None

    init_dac_saturation, _ = check_output_saturation(r_fast, iterations=250, verbose=False)
    init_amps = get_tone_amplitudes(r, r_fast, config_dict)
    init_psb_scale = r.psbscale.get_scale()
    init_psb_fftshift = r.psb.get_fftshift()
    print(f'  init: psb_scale={init_psb_scale:.4f}, '
          f'fftshift={format(init_psb_fftshift, "#016b")}, '
          f'max_amp={np.max(init_amps):.4f}')

    max_amp = 1 - 2**-12
    scalemin = 1 / 256
    scalemax = 255
    tx_compression_state = None

    if rf_only:
        if init_dac_saturation:
            print('  WARNING: DAC is saturating; rf_only=True leaves digital settings unchanged')
        amps = init_amps
        best_fftshift = init_psb_fftshift
        psb_scale = init_psb_scale
        print('  digital stages skipped (rf_only=True)')
    else:
        if init_dac_saturation and psb_scale_fixed:
            raise ValueError(
                'DAC is saturating and force_psb_scale prevents automatic recovery')
        if init_dac_saturation:
            print('  DAC saturating — fixing first')
            fix_dac_saturation(r, r_fast, config_dict)

        # --- Step 1: Mute, maximise amplitudes ---
        if tone_amplitudes_fixed:
            amps = init_amps
            amps_gain = 1.0
            print('  step 1: amplitudes fixed — skipping amplitude maximisation')
        else:
            amps_max = np.max(init_amps)
            if amps_max == 0:
                raise ValueError('Tone powers are all zero')
            amps_gain = max_amp / amps_max
            if psb_scale_fixed:
                print('  step 1: psb_scale fixed — changing amplitudes without mute')
            else:
                r.psbscale.set_scale(0)
                time.sleep(0.01)
            amps = init_amps * amps_gain
            amps = _apply_per_bin_scaling(r, config_dict, amps)
            set_tone_amplitudes(r, config_dict, amps)
            time.sleep(0.01)
            print(f'  step 1: amplitudes maximised (x{amps_gain:.4f})')

        # --- Step 2: Find best PSB FFT shift (output muted) ---
        if psb_fftshift_fixed:
            best_fftshift = r.psb.get_fftshift()
            print(f'  step 2: PSB fftshift fixed at {format(best_fftshift, "#016b")}')
        elif psb_scale_fixed:
            best_fftshift = r.psb.get_fftshift()
            print('  step 2: PSB fftshift search skipped because psb_scale is fixed')
        else:
            print(f'  step 2: PSB fftshift search')
            best_fftshift, _, _ = _find_best_psb_fftshift(r)

        init_popcount = bin(init_psb_fftshift).count('1')
        best_popcount = bin(best_fftshift).count('1')
        fftshift_gain_ratio = 2.0 ** (init_popcount - best_popcount)

        # --- Step 3: Multi-resolution psb_scale ramp ---
        if psb_scale_fixed:
            psb_scale = r.psbscale.get_scale()
            print(f'  step 3: psb_scale fixed at {psb_scale:.6f}')
        else:
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

        # Optional RF compression guard.  This compares total multitone power
        # at the TX frontend input with the modelled input-referred P1dB.
        if compression_headroom_db is not None and psb_scale_fixed:
            tx_compression_state = _get_tx_compression_state(
                r, config_dict, rf_peripherals, compression_headroom_db)
            print('  compression guard: skipped because psb_scale is fixed')
        else:
            psb_scale, tx_compression_state = _enforce_tx_compression_margin(
                r, config_dict, rf_peripherals, compression_headroom_db,
                scalemin, scalemax)

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
                         tx_power_change_db=tx_change_db,
                         digital_only=digital_only, rf_only=rf_only,
                         force_rx_amp_bypass=force_rx_amp_bypass,
                         force_rx_attenuation_db=force_rx_attenuation_db,
                         force_adc_dsa_db=force_adc_dsa_db,
                         force_pfb_fftshift=force_pfb_fftshift)

    # --- Step 4: Maximise analog chain (if reference plane beyond DAC) ---
    tx_atten_db = None
    tx_amp_bypass = None
    if allow_rf and (reference_plane != 'dac' or rf_only):
        if has_rf:
            print(f'  step 4: maximise analog chain')
            # Enable TX amplifier
            if _rf_supports_bypass_amps(rf_peripherals):
                current_bypass = rf_peripherals.get_tx_amp_bypass()
            else:
                current_bypass = None

            if tx_amp_fixed:
                tx_amp_bypass = rf_peripherals.get_tx_amp_bypass() if current_bypass is not None else None
                print(f'    TX amp fixed at bypass={tx_amp_bypass}')
            elif current_bypass:
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
                                     tx_power_change_db=expected_gain_db,
                                     digital_only=digital_only, rf_only=rf_only,
                                     force_rx_amp_bypass=force_rx_amp_bypass,
                                     force_rx_attenuation_db=force_rx_attenuation_db,
                                     force_adc_dsa_db=force_adc_dsa_db,
                                     force_pfb_fftshift=force_pfb_fftshift)
                    tx_amp_bypass = False
            elif current_bypass is False:
                tx_amp_bypass = False
            else:
                print('    TX amp: not available — skipping')

            # Reduce TX attenuator gradually (3 dB steps), checking the
            # RX path after each step via _apply_rx_policy.
            current_atten = rf_peripherals.get_tx_attenuation()
            min_atten = rf_peripherals.ATTEN_MIN
            atten_step = rf_peripherals.ATTEN_STEP
            if tx_atten_fixed:
                print(f'    TX atten fixed at {current_atten:.1f} dB')
                tx_atten_db = current_atten
            elif current_atten > min_atten:
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
                                     tx_power_change_db=step_change_db,
                                     digital_only=digital_only, rf_only=rf_only,
                                     force_rx_amp_bypass=force_rx_amp_bypass,
                                     force_rx_attenuation_db=force_rx_attenuation_db,
                                     force_adc_dsa_db=force_adc_dsa_db,
                                     force_pfb_fftshift=force_pfb_fftshift)
                print(f'    TX atten: {current_atten:.1f} -> {atten:.1f} dB')
                tx_atten_db = atten
            else:
                print(f'    TX atten: already at minimum ({current_atten:.1f} dB)')
                tx_atten_db = min_atten
        else:
            print(f'  step 4: WARNING — no controllable rf_peripherals')
    elif reference_plane != 'dac':
        print('  step 4: analog chain skipped (digital_only=True)')

    # --- Step 5: Enforce power limit ---
    if power_limit_dbm is not None and config_dict is not None:
        achieved_powers = get_tone_powers(r, r_fast, config_dict, reference_plane=reference_plane,
                                          rf_peripherals=rf_peripherals)
        max_achieved = float(np.max(achieved_powers))
        excess_db = max_achieved - power_limit_dbm
        print(f'  step 5: power limit {power_limit_dbm} dBm — '
              f'achieved {max_achieved:.1f} dBm, excess {excess_db:+.1f} dB')
        if excess_db > 0.1:
            if has_rf and not tx_atten_fixed:
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
                    if rf_only or psb_scale_fixed:
                        print(f'    WARNING: power limit still exceeded by '
                              f'{remaining_db:.1f} dB; psb_scale reduction is disabled')
                    else:
                        psb_scale = r.psbscale.get_scale()
                        psb_scale *= 10**(-remaining_db / 20)
                        psb_scale = float(np.clip(psb_scale, scalemin, scalemax))
                        r.psbscale.set_scale(psb_scale)
                        time.sleep(0.01)
                        print(f'    psb_scale reduced to {psb_scale:.4f} '
                              f'({remaining_db:.1f} dB remaining)')
            else:
                if rf_only or psb_scale_fixed:
                    print(f'    WARNING: power limit still exceeded by '
                          f'{excess_db:.1f} dB; psb_scale reduction is disabled')
                else:
                    psb_scale = r.psbscale.get_scale()
                    psb_scale *= 10**(-excess_db / 20)
                    psb_scale = float(np.clip(psb_scale, scalemin, scalemax))
                    r.psbscale.set_scale(psb_scale)
                    time.sleep(0.01)
                    print(f'    psb_scale reduced to {psb_scale:.4f} (no RF peripherals)')

    if compression_headroom_db is not None:
        tx_compression_state = _get_tx_compression_state(
            r, config_dict, rf_peripherals, compression_headroom_db)

    _, dsp_overflow_details = check_dsp_overflow(r, 0.1, verbose=False)
    _, dac_saturation_details = check_output_saturation(r_fast, iterations=250, verbose=False)
    if tx_compression_state is not None:
        dac_saturation_details['tx_compression'] = tx_compression_state
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
                    rf_peripherals=None, headroom_db=1.0,
                    digital_only=False, rf_only=False,
                    force_tx_amp_bypass=None, force_rx_amp_bypass=None,
                    force_tx_attenuation_db=None,
                    force_rx_attenuation_db=None,
                    force_adc_dsa_db=None, force_tone_amplitudes=None,
                    force_psb_fftshift=None, force_psb_scale=None,
                    force_pfb_fftshift=None):
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
    digital_only : bool
        If True, only firmware/RFDC parameters are adjusted.  RF frontend
        attenuators and bypass amplifiers are left unchanged.
    rf_only : bool
        If True, only RF frontend attenuators and bypass amplifiers are
        eligible for adjustment.  Firmware/RFDC parameters are left unchanged.
    """
    digital_only, rf_only = _validate_control_scope(digital_only, rf_only)
    print(f'optimise_tx_snr (reference_plane={reference_plane}, '
          f'digital_only={digital_only}, rf_only={rf_only})')
    if reference_plane not in ('dac', 'rf_output', 'detector'):
        raise ValueError(f"reference_plane must be 'dac', 'rf_output', or 'detector', "
                         f"got '{reference_plane}'")

    _apply_forced_power_controls(
        r, config_dict, rf_peripherals=rf_peripherals,
        force_tx_amp_bypass=force_tx_amp_bypass,
        force_rx_amp_bypass=force_rx_amp_bypass,
        force_tx_attenuation_db=force_tx_attenuation_db,
        force_rx_attenuation_db=force_rx_attenuation_db,
        force_adc_dsa_db=force_adc_dsa_db,
        force_tone_amplitudes=force_tone_amplitudes,
        force_psb_fftshift=force_psb_fftshift,
        force_psb_scale=force_psb_scale,
        force_pfb_fftshift=force_pfb_fftshift)

    has_rf = (not digital_only) and _rf_has_controllable_attenuator(rf_peripherals)
    tx_atten_fixed = force_tx_attenuation_db is not None
    tone_amplitudes_fixed = force_tone_amplitudes is not None
    psb_fftshift_fixed = force_psb_fftshift is not None
    psb_scale_fixed = force_psb_scale is not None

    init_dac_saturation, _ = check_output_saturation(r_fast, iterations=250, verbose=False)
    if init_dac_saturation and psb_scale_fixed:
        raise ValueError(
            'DAC is saturating and force_psb_scale prevents automatic recovery')
    if init_dac_saturation and not rf_only:
        print('  DAC saturation detected — fixing first')
        fix_dac_saturation(r, r_fast, config_dict)
    elif init_dac_saturation:
        print('  WARNING: DAC is saturating; rf_only=True leaves digital settings unchanged')
    init_amps = get_tone_amplitudes(r, r_fast, config_dict)
    init_psb_scale = r.psbscale.get_scale()
    init_psb_fftshift = r.psb.get_fftshift()
    init_tx_atten = rf_peripherals.get_tx_attenuation() if has_rf else None
    init_powers = get_tone_powers(r, r_fast, config_dict, reference_plane=reference_plane,
                                          rf_peripherals=rf_peripherals)
    scalemin = 1 / 256
    scalemax = 255
    print(f'  init: psb_scale={init_psb_scale:.6f}, '
          f'fftshift={format(init_psb_fftshift, "#016b")}, '
          f'max_amp={np.max(init_amps):.6f}'
          + (f', tx_atten={init_tx_atten:.1f} dB' if has_rf else ''))

    if rf_only:
        print('  digital stages skipped (rf_only=True)')
        dsp_overflow_details = check_dsp_overflow(r, 0.1, verbose=False)[1]
        _, dac_levels = check_output_saturation(r_fast, iterations=250, verbose=False)
        return init_amps, init_psb_fftshift, init_psb_scale, dsp_overflow_details, dac_levels

    # --- Step 1: Mute and maximise amplitudes ---
    max_amp = 1 - 2**-12
    if tone_amplitudes_fixed:
        amps = init_amps
        amps_gain = 1.0
        print('  step 1: amplitudes fixed — skipping amplitude maximisation')
    else:
        print('  step 1: mute and maximise amplitudes')
        amps_max = np.max(init_amps)
        if amps_max == 0:
            raise ValueError('Tone powers are all zero')
        amps_gain = max_amp / amps_max
        if psb_scale_fixed:
            print('    psb_scale fixed — changing amplitudes without mute')
        else:
            r.psbscale.set_scale(0)
            time.sleep(0.01)
        amps = init_amps * amps_gain
        amps = _apply_per_bin_scaling(r, config_dict, amps)
        set_tone_amplitudes(r, config_dict, amps)
        time.sleep(0.01)
        print(f'    amplitudes scaled by {amps_gain:.4f}')

    # --- Step 2: Find best PSB FFT shift (output muted) ---
    if psb_fftshift_fixed:
        best_fftshift = r.psb.get_fftshift()
        print(f'  step 2: PSB fftshift fixed at {format(best_fftshift, "#016b")}')
    elif psb_scale_fixed:
        best_fftshift = r.psb.get_fftshift()
        print('  step 2: PSB fftshift search skipped because psb_scale is fixed')
    else:
        print('  step 2: find best PSB fftshift')
        best_fftshift, _, _ = _find_best_psb_fftshift(r)

    init_popcount = bin(init_psb_fftshift).count('1')
    best_popcount = bin(best_fftshift).count('1')
    fftshift_gain_ratio = 2.0 ** (init_popcount - best_popcount)

    # --- Step 3: Maximise psb_scale ---
    if has_rf:
        print('  step 3: ramp psb_scale to max (analog compensation)')
        if psb_scale_fixed:
            psb_scale = r.psbscale.get_scale()
            print(f'    psb_scale fixed at {psb_scale:.6f}')
        else:
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
        new_powers = get_tone_powers(r, r_fast, config_dict, reference_plane=reference_plane,
                                          rf_peripherals=rf_peripherals)
        power_increase_db = float(np.max(new_powers)) - float(np.max(init_powers))
        print(f'    power change from digital maximisation: {power_increase_db:+.1f} dB')

        if tx_atten_fixed:
            print(f'    TX attenuator fixed at {rf_peripherals.get_tx_attenuation():.1f} dB')
            if abs(power_increase_db) > 0.1 and not psb_scale_fixed:
                psb_scale = r.psbscale.get_scale()
                psb_scale *= 10**(-power_increase_db / 20)
                psb_scale = float(np.clip(psb_scale, scalemin, scalemax))
                r.psbscale.set_scale(psb_scale)
                time.sleep(0.01)
                print(f'    psb_scale adjusted to {psb_scale:.6f} to preserve power')
        elif power_increase_db > 0.1:
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
                if psb_scale_fixed:
                    print(f'    WARNING: psb_scale fixed; residual power change {remaining_db:.1f} dB')
                else:
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
        if psb_scale_fixed:
            psb_scale = r.psbscale.get_scale()
            print(f'  step 3: psb_scale fixed at {psb_scale:.6f} (no RF peripherals)')
        else:
            print('  step 3: compute psb_scale analytically (no RF peripherals)')
            psb_scale = init_psb_scale / (fftshift_gain_ratio * amps_gain)
            psb_scale = float(np.clip(psb_scale, scalemin, scalemax))
            print(f'    fftshift_gain={fftshift_gain_ratio:.4f}, amps_gain={amps_gain:.4f}, '
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
        if has_rf and not tx_atten_fixed:
            rf_peripherals.set_tx_attenuation(init_tx_atten)
        raise ValueError('TX DSP overflow detected after optimisation')

    # Verify output power is preserved
    achieved_powers = get_tone_powers(r, r_fast, config_dict, reference_plane=reference_plane,
                                          rf_peripherals=rf_peripherals)
    power_error = float(np.max(np.abs(achieved_powers - init_powers)))
    if power_error > 0.5:
        print(f'  WARNING: output power changed by {power_error:.1f} dB')
    else:
        print(f'  power preserved (max error {power_error:.2f} dB)')

    return amps, best_fftshift, r.psbscale.get_scale(), dsp_overflow_details, dac_levels


def maximise_rx_power(r, r_fast, config_dict, headroom_db=1.0, rf_peripherals=None,
                      digital_only=False, rf_only=False,
                      force_tx_amp_bypass=None, force_rx_amp_bypass=None,
                      force_tx_attenuation_db=None,
                      force_rx_attenuation_db=None,
                      force_adc_dsa_db=None, force_tone_amplitudes=None,
                      force_psb_fftshift=None, force_psb_scale=None,
                      force_pfb_fftshift=None):
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
    digital_only : bool
        If True, only firmware/RFDC parameters are adjusted.  RF frontend
        attenuators and bypass amplifiers are left unchanged.
    rf_only : bool
        If True, only RF frontend attenuators and bypass amplifiers are
        adjusted.  Firmware/RFDC parameters are left unchanged.
    """
    digital_only, rf_only = _validate_control_scope(digital_only, rf_only)
    allow_digital = not rf_only
    allow_rf = not digital_only
    print(f'maximise_rx_power (digital_only={digital_only}, rf_only={rf_only})')

    _apply_forced_power_controls(
        r, config_dict, rf_peripherals=rf_peripherals,
        force_tx_amp_bypass=force_tx_amp_bypass,
        force_rx_amp_bypass=force_rx_amp_bypass,
        force_tx_attenuation_db=force_tx_attenuation_db,
        force_rx_attenuation_db=force_rx_attenuation_db,
        force_adc_dsa_db=force_adc_dsa_db,
        force_tone_amplitudes=force_tone_amplitudes,
        force_psb_fftshift=force_psb_fftshift,
        force_psb_scale=force_psb_scale,
        force_pfb_fftshift=force_pfb_fftshift)

    DSA_MAX = 27
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    rf_available = _rf_has_controllable_attenuator(rf_peripherals)
    has_rf = allow_rf and rf_available
    has_bypass_amps = allow_rf and _rf_supports_bypass_amps(rf_peripherals)
    dsa_fixed = force_adc_dsa_db is not None
    rx_atten_fixed = force_rx_attenuation_db is not None
    rx_amp_fixed = force_rx_amp_bypass is not None
    pfb_fftshift_fixed = force_pfb_fftshift is not None

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
    init_rx_amp_bypass = rf_peripherals.get_rx_amp_bypass() if has_bypass_amps else None
    init_rf_msg = ''
    if has_rf:
        init_rf_msg = f', RX atten={init_rx_atten:.1f} dB'
        if has_bypass_amps:
            init_rf_msg += f', RX amp bypass={init_rx_amp_bypass}'
    print(f'  init: DSA={init_dsa} dB' + init_rf_msg)

    # --- Handle RTS over-voltage (hidden firmware DSA) ---
    rts_event, rts_details = check_rfdc_rts_events(r, clear=False)
    if allow_digital and rts_details.get('rts_over_voltage', False):
        print('  step 0: RTS over-voltage — clearing hidden firmware DSA')
        _set_dsa(DSA_MAX)
        check_rfdc_rts_events(r, clear=True)
        time.sleep(0.1)
    elif rts_details.get('rts_over_voltage', False):
        print('  step 0: RTS over-voltage detected — rf_only=True leaves DSA unchanged')
        check_rfdc_rts_events(r, clear=True)
    elif rts_details.get('rts_over_range', False):
        print('  step 0: RTS over-range — clearing')
        check_rfdc_rts_events(r, clear=True)

    # --- If saturated, fix first ---
    saturated, levels = check_input_saturation(r, r_fast, iterations=250, verbose=False)
    peak_db = _peak_dbfs(levels)
    print(f'  step 1: initial check — saturated={saturated}, peak={peak_db:.1f} dBFS')
    if saturated:
        print('    ADC saturated — fixing first')
        fix_adc_saturation(r, r_fast, config_dict, rf_peripherals=rf_peripherals,
                           digital_only=digital_only, rf_only=rf_only,
                           force_rx_amp_bypass=force_rx_amp_bypass,
                           force_rx_attenuation_db=force_rx_attenuation_db,
                           force_adc_dsa_db=force_adc_dsa_db)

    # --- Reduce DSA towards zero ---
    if allow_digital and not dsa_fixed:
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
    else:
        reason = 'fixed' if dsa_fixed else 'rf_only=True'
        print(f'  step 2: DSA skipped ({reason})')

    # --- Reduce RX attenuator ---
    if has_rf and not rx_atten_fixed:
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
        if has_bypass_amps and not rx_amp_fixed:
            print('  step 4: try enabling RX amp')
        elif rx_amp_fixed:
            print('  step 4: RX amp fixed — skipping')
        else:
            print('  step 4: RX amp not available — skipping')
        if has_bypass_amps and not rx_amp_fixed and rf_peripherals.get_rx_amp_bypass():
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
    elif has_rf:
        print(f'  step 3: RX attenuator fixed at {rf_peripherals.get_rx_attenuation():.1f} dB')

    # --- Final assessment ---
    print('  step 5: final assessment and PFB fftshift')
    saturated, levels = check_input_saturation(r, r_fast, iterations=250, verbose=False)
    best_dsa = _get_dsa()
    best_rx_atten = rf_peripherals.get_rx_attenuation() if rf_available else None
    peak_db = _peak_dbfs(levels)
    print(f'  done: peak={peak_db:.1f} dBFS, DSA={best_dsa} dB, saturated={saturated}')

    # --- Optimise PFB FFT shift ---
    if allow_digital and not pfb_fftshift_fixed:
        best_fftshift, _ = _find_best_pfb_fftshift(r)
    else:
        best_fftshift = r.pfb.get_fftshift()
        reason = 'fixed' if pfb_fftshift_fixed else 'rf_only=True'
        print(f'  PFB fftshift unchanged ({reason}): {format(best_fftshift, "#016b")}')

    return best_dsa, best_fftshift, check_dsp_overflow(r, 0.1, verbose=False)[1], levels, best_rx_atten


def maximise_rx_dsp_gain(r):
    """Maximise post-ADC RX DSP gain using the PFB overflow flags.

    This changes only the PFB FFT-shift schedule. It deliberately avoids ADC
    snapshots, RFDC DSA, and RF frontend controls, making it suitable when
    internal loopback replaces the physical ADC data path.
    """
    print('maximise_rx_dsp_gain')
    best_fftshift, _ = _find_best_pfb_fftshift(r)
    dsp = check_dsp_overflow(r, 0.1, verbose=False)[1]
    return best_fftshift, dsp


def fix_adc_saturation(r, r_fast, config_dict, rf_peripherals=None,
                       digital_only=False, rf_only=False,
                       force_rx_amp_bypass=None,
                       force_rx_attenuation_db=None,
                       force_adc_dsa_db=None):
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
    digital_only, rf_only = _validate_control_scope(digital_only, rf_only)
    allow_digital = not rf_only
    allow_rf = not digital_only
    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    DSA_MAX = 27
    has_rf = allow_rf and _rf_has_controllable_attenuator(rf_peripherals)
    has_bypass_amps = allow_rf and _rf_supports_bypass_amps(rf_peripherals)
    rx_amp_fixed = force_rx_amp_bypass is not None
    rx_atten_fixed = force_rx_attenuation_db is not None
    dsa_fixed = force_adc_dsa_db is not None

    _apply_forced_power_controls(
        r, config_dict, rf_peripherals=rf_peripherals,
        force_rx_amp_bypass=force_rx_amp_bypass,
        force_rx_attenuation_db=force_rx_attenuation_db,
        force_adc_dsa_db=force_adc_dsa_db)

    def _make_result():
        check_rfdc_rts_events(r, clear=True)
        time.sleep(0.1)
        sat, lvls = check_input_saturation(r, r_fast, iterations=500, verbose=False)
        d = float(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])
        ra = rf_peripherals.get_rx_attenuation() if has_rf else None
        ab = rf_peripherals.get_rx_amp_bypass() if has_bypass_amps else None
        return {'dsa': d, 'adc_levels': lvls, 'rx_attenuation_db': ra,
                'rx_amp_bypass': ab, 'saturation': sat}

    def _check_saturated():
        check_rfdc_rts_events(r, clear=True)
        time.sleep(0.1)
        sat, _ = check_input_saturation(r, r_fast, iterations=500, verbose=False)
        return sat

    # --- Handle RTS over-voltage (hidden firmware DSA) ---
    rts_event, rts_details = check_rfdc_rts_events(r, clear=False)
    if allow_digital and not dsa_fixed and rts_details.get('rts_over_voltage', False):
        print('fix_adc_saturation: RTS over-voltage — clearing hidden firmware DSA')
        r.rfdc.core.set_dsa(adc_tile, adc_block, int(DSA_MAX))
        time.sleep(0.1)
        check_rfdc_rts_events(r, clear=True)
        time.sleep(0.1)
    elif rts_details.get('rts_over_voltage', False):
        reason = 'force_adc_dsa_db set' if dsa_fixed else 'rf_only=True'
        print(f'fix_adc_saturation: RTS over-voltage detected; {reason} leaves DSA unchanged')
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
    if has_bypass_amps and not rx_amp_fixed and not rf_peripherals.get_rx_amp_bypass():
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
    elif has_bypass_amps and rx_amp_fixed:
        print('  RX amplifier bypass fixed — skipping bypass step')

    # --- Step 2: Step RX attenuator up in 3 dB increments ---
    if has_rf and not rx_atten_fixed:
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
    elif has_rf:
        print(f'  RX attenuator fixed at {rf_peripherals.get_rx_attenuation():.1f} dB')

    # --- Step 3: Step ADC DSA up in 2 dB increments ---
    if allow_digital and not dsa_fixed:
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
    else:
        reason = 'fixed' if dsa_fixed else 'rf_only=True'
        print(f'  ADC DSA skipped ({reason})')

    print('  WARNING: ADC saturation persists after allowed saturation controls')
    return _make_result()

def optimise_rx_snr(r, r_fast=None, config_dict=None, headroom_db=1.0,
                    rf_peripherals=None, digital_only=False, rf_only=False,
                    force_tx_amp_bypass=None, force_rx_amp_bypass=None,
                    force_tx_attenuation_db=None,
                    force_rx_attenuation_db=None,
                    force_adc_dsa_db=None, force_tone_amplitudes=None,
                    force_psb_fftshift=None, force_psb_scale=None,
                    force_pfb_fftshift=None):
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
    digital_only : bool
        If True, only firmware/RFDC parameters are adjusted.  RF frontend
        attenuators and bypass amplifiers are left unchanged.
    rf_only : bool
        If True, only RF frontend attenuators and bypass amplifiers are
        adjusted.  Firmware/RFDC parameters are left unchanged.
    """
    digital_only, rf_only = _validate_control_scope(digital_only, rf_only)
    allow_digital = not rf_only
    allow_rf = not digital_only

    _apply_forced_power_controls(
        r, config_dict, rf_peripherals=rf_peripherals,
        force_tx_amp_bypass=force_tx_amp_bypass,
        force_rx_amp_bypass=force_rx_amp_bypass,
        force_tx_attenuation_db=force_tx_attenuation_db,
        force_rx_attenuation_db=force_rx_attenuation_db,
        force_adc_dsa_db=force_adc_dsa_db,
        force_tone_amplitudes=force_tone_amplitudes,
        force_psb_fftshift=force_psb_fftshift,
        force_psb_scale=force_psb_scale,
        force_pfb_fftshift=force_pfb_fftshift)

    adc_tile = int(config_dict['firmware']['adc_tile'])
    adc_block = int(config_dict['firmware']['adc_block'])
    has_rf = allow_rf and _rf_has_controllable_attenuator(rf_peripherals)
    has_bypass_amps = allow_rf and _rf_supports_bypass_amps(rf_peripherals)
    dsa_fixed = force_adc_dsa_db is not None
    rx_atten_fixed = force_rx_attenuation_db is not None
    rx_amp_fixed = force_rx_amp_bypass is not None
    pfb_fftshift_fixed = force_pfb_fftshift is not None

    print(f'optimise_rx_snr (digital_only={digital_only}, rf_only={rf_only})')

    # --- Fix existing saturation ---
    sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
    if sat:
        print('  ADC saturated — fixing first')
        fix_adc_saturation(r, r_fast, config_dict, rf_peripherals=rf_peripherals,
                           digital_only=digital_only, rf_only=rf_only,
                           force_rx_amp_bypass=force_rx_amp_bypass,
                           force_rx_attenuation_db=force_rx_attenuation_db,
                           force_adc_dsa_db=force_adc_dsa_db)

    # --- Step 1: Set DSA to 0 ---
    if allow_digital and not dsa_fixed:
        current_dsa = float(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])
        if current_dsa > 0:
            print(f'  step 1: set DSA to 0 (was {current_dsa:.0f} dB)')
            r.rfdc.core.set_dsa(adc_tile, adc_block, 0)
            time.sleep(0.1)

            sat, _ = check_input_saturation(r, r_fast, iterations=250, verbose=False)
            if sat and has_rf and not rx_atten_fixed:
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
    else:
        reason = 'fixed' if dsa_fixed else 'rf_only=True'
        print(f'  step 1: DSA skipped ({reason})')

    # --- Step 2: Reduce RX attenuation to maximise signal ---
    if has_rf and not rx_atten_fixed:
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

        if (rf_only and has_bypass_amps and not rx_amp_fixed
                and rf_peripherals.get_rx_amp_bypass()):
            print('  step 2b: try enabling RX amp')
            s21_bypassed = rf_peripherals._get_amp_s21('recv_atten')
            rf_peripherals.set_rx_amp_bypass(False)
            time.sleep(0.1)
            s21_enabled = rf_peripherals._get_amp_s21('recv_atten')
            expected_gain_db = s21_enabled - s21_bypassed
            if abs(expected_gain_db) < 0.5:
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
    elif has_rf:
        print(f'  step 2: RX attenuation fixed at {rf_peripherals.get_rx_attenuation():.1f} dB')

    if rx_amp_fixed:
        print('  step 2b: RX amp fixed — skipping')

    # --- Step 3: Find best PFB FFT shift ---
    if allow_digital and not pfb_fftshift_fixed:
        print('  step 3: find best PFB fftshift')
        best_fftshift, _ = _find_best_pfb_fftshift(r)
    else:
        best_fftshift = r.pfb.get_fftshift()
        reason = 'fixed' if pfb_fftshift_fixed else 'rf_only=True'
        print(f'  step 3: PFB fftshift unchanged ({reason}): '
              f'{format(best_fftshift, "#016b")}')

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
    # v7.11: get_new_spectra returns (data, gpio_counts, timestamp, buf_id, slot_id);
    # take element 0 (the spectra). buf/slot IDs need get_buf_id=True (not used here).
    data = np.asarray(r.accumulators[0].get_new_spectra()[0])
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
    details = get_tone_frequencies(r, None, config_dict, detailed_output=True)[1]
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

    # Let the firmware's valid gating select complete I/Q samples for this
    # channel. As of firmware v7.10.2, forcing man_trig can start the capture
    # between I and Q and swap them for the whole snapshot.
    mm[addrs['ctrl']:addrs['ctrl']+4] = struct.pack('<I', 0)
    mm[addrs['ctrl']:addrs['ctrl']+4] = struct.pack('<I', 1)

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
        details = get_tone_frequencies(r, r_fast, config_dict, detailed_output=True)[1]
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


def perform_sweep(r, r_fast, config_dict, centers, spans, points, samples_per_point,
                  direction, autosync=False, setup_sync=True, mrst=False,
                  setup_mrst=False, settle_accumulations=4,
                  chanmap_settle_accumulations=4):
    """
    A blocking call to perform a frequency sweep of the RFSOC.
    An asynchronous version of this function is available in the readout_server code.
    ``autosync`` controls the per-step sync after each sweep buffer flip.
    ``setup_sync`` controls the one-time sync at the start of the sweep.
    ``settle_accumulations`` controls how many accumulations are discarded after
    each sweep point buffer switch before samples are recorded.
    ``chanmap_settle_accumulations`` controls how many accumulations are waited
    immediately after a PSB/PFB channel-map update, before the new mixer control
    buffer is written and made active.

    """
    centers = np.atleast_1d(centers)
    spans = np.atleast_1d(spans)
    if len(spans)==1:
        spans = np.full(len(centers),spans[0])
    assert len(centers) == len(spans)
    num_points=int(points)
    samples_per_point=int(samples_per_point)
    settle_accumulations=int(settle_accumulations)
    if settle_accumulations < 0:
        raise ValueError('settle_accumulations must be >= 0')
    chanmap_settle_accumulations=int(chanmap_settle_accumulations)
    if chanmap_settle_accumulations < 0:
        raise ValueError('chanmap_settle_accumulations must be >= 0')
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

    initial_freqs = get_tone_frequencies(r, r_fast, config_dict)
    if len(initial_freqs)==0:
        initial_freqs = centers
    sweep_tone_amplitudes = _get_current_per_tone_values(
        get_tone_amplitudes, r, config_dict, num_tones)
    sweep_tone_phases = _get_current_per_tone_values(
        get_tone_phases, r, config_dict, num_tones)

    fast_read_params = get_fast_read_params(r_fast)

    # Prepare sweep settings - this computes tone_indices for each point
    # as they may change when tones cross FFT bin boundaries
    fast_sweep_params = prepare_sweep_settings_fast(
        r_fast, config_dict, sweepfreqs.T,
        tone_amplitudes=sweep_tone_amplitudes,
        tone_phases=sweep_tone_phases)  # transpose to (num_points, num_tones)
    tone_indices_arr = fast_sweep_params.get('tone_indices')  # shape: (num_points, num_tones)

    # Phase offsets are constant across the sweep: when LO slots are stable,
    # write them once into both control buffers so per-point writes stay
    # frequency-only and both buffers share the same phase reference.
    if fast_sweep_params.get('phase_offsets') is not None:
        write_phase_offsets_both_buffers_fast(
            r_fast, fast_sweep_params['phase_offsets'],
            tone_indices=fast_sweep_params.get('phase_offset_tone_indices'))

    # One-time sync at sweep start establishes the TX/RX phase reference.
    # Independent of the per-step ``autosync``.
    if setup_sync:
        force_sync_fast(r_fast, mrst=setup_mrst)

    for p in range(num_points):
        apply_sweep_step_fast(
            r, r_fast, fast_sweep_params, p, autosync=autosync, mrst=mrst,
            chanmap_settle_accumulations=chanmap_settle_accumulations)

        for _ in range(settle_accumulations):
            _wait_for_acc(r_fast,0,0.0001)

        # Get tone_indices for this sweep point
        tone_indices_p = tone_indices_arr[p] if tone_indices_arr is not None else np.arange(num_tones)

        for s in range(samples_per_point):
            cnt,data,err,_tt = read_accumulated_data_fast(fast_read_params,
                                                      tone_indices=tone_indices_p)
            acc_counts[p,s] = cnt
            sweep_data[:,p,s] = data[::2]+1j*data[1::2]
            acc_errs[p,s] = err

    for _ in range(settle_accumulations):
            _wait_for_acc(r_fast,0,0.0001)

    set_tone_frequencies_fast(
        r, r_fast, config_dict, initial_freqs, autosync=autosync, mrst=mrst,
        tone_amplitudes=sweep_tone_amplitudes,
        tone_phases=sweep_tone_phases)

    sweep_responses = np.mean(sweep_data.real,axis=1) + 1j*np.mean(sweep_data.imag,axis=1)
    sweep_stds = np.std(sweep_data.real,axis=1) + 1j*np.std(sweep_data.imag,axis=1)
    sweep_sems = np.std(sweep_data.real,axis=1)/np.sqrt(samples_per_point) + 1j*np.std(sweep_data.imag,axis=1)/np.sqrt(samples_per_point)

    results = {
        'sweep_frequencies': sweepfreqs,
        'sweep_responses': sweep_responses,
        'sweep_stds': sweep_stds,
        'sweep_sems': sweep_sems,
        'samples_per_point': samples_per_point,
        'settle_accumulations': settle_accumulations,
        'chanmap_settle_accumulations': chanmap_settle_accumulations,
        'samples_per_second': get_sample_rate(r_fast),
        'accumulation_counts': acc_counts,
        'accumulation_errors': acc_errs
        }
    return results

def perform_retune(r, r_fast,config_dict, centers, spans, points,
                   samples_per_point, direction, method, smooth_len=3,
                   freq_offsets=None, autosync=False, setup_sync=True, mrst=False,
                   setup_mrst=False, settle_accumulations=4,
                   chanmap_settle_accumulations=4):
    """
    A blocking call to perform a frequency retune of the RFSOC.
    SImply performs a sweep and then retunes to the frequencies of maximum gradient or minimum magnitude.
    An asynchronous version of this function is available in the readout_server code.
    if freq_offsets is given, it is added to the retune frequencies before setting them.
    ``autosync`` / ``setup_sync`` are forwarded to :func:`perform_sweep`.
    ``settle_accumulations`` is forwarded to :func:`perform_sweep`.
    ``chanmap_settle_accumulations`` is forwarded to :func:`perform_sweep`.
    """
    if freq_offsets is None:
        freq_offsets = np.zeros_like(centers)
    elif np.isscalar(freq_offsets):
        freq_offsets = np.full_like(centers,freq_offsets)
    elif freq_offsets.shape != np.atleast_1d(centers).shape:
            raise ValueError("freq_offsets must be None, a scalar, or have the same shape as centers")

    #results = r.retune(center, span, points, samples_per_point,direction,method)
    if method not in ('max_gradient','min_mag','max_dphidf'):
        raise ValueError(f'Invalid retune method "{method}", must be "max_gradient", "min_mag", or "max_dphidf"')
    results = perform_sweep(
        r, r_fast, config_dict, centers, spans, points, samples_per_point,
        direction, autosync=autosync, setup_sync=setup_sync, mrst=mrst,
        setup_mrst=setup_mrst, settle_accumulations=settle_accumulations,
        chanmap_settle_accumulations=chanmap_settle_accumulations)

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
    elif method == 'max_dphidf':
        retune_freqs = np.zeros_like(results['sweep_frequencies'])
        for t in range(len(centers)):
            freqs = results['sweep_frequencies'][t]
            phase = np.unwrap(np.angle(results['sweep_responses'][t]))
            dphidf = np.abs(np.gradient(phase, freqs))
            max_slope = np.argmax(dphidf)
            retune_freqs[t] = freqs[max_slope] + freq_offsets[t]

    set_tone_frequencies_fast(
        r, r_fast, config_dict, retune_freqs, autosync=autosync)
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
    return bool(int(freeze['CalFrozen']))

def refresh_adc_cal(r, config_dict, adc_cal_settle_time=2.0):
    """
    Refresh the ADC calibration by unfreezing, waiting for the calibration
    to settle, then freezing again.

    Args:
        r: The FPGA register interface.
        config_dict: Configuration dictionary.
        adc_cal_settle_time (float): Seconds to wait for calibration to settle
            after unfreezing. Default 2.0.
    """
    set_cal_freeze(r, config_dict, False)
    time.sleep(adc_cal_settle_time)
    set_cal_freeze(r, config_dict, True)


# ---------------------------------------------------------------------------
# Clock source control via krc-utils
# ---------------------------------------------------------------------------

KRC_UTILS_BIN = '/home/casper/krc-utils/krc-utils'
KRC_CLOCK_DIR = '/etc/krc-utils.d/clock.d'
KRC_LMK_SYMLINK = os.path.join(KRC_CLOCK_DIR, 'lmk04208.txt')

# Map user-facing names to the LMK config filenames
_CLOCK_SOURCE_FILES = {
    'internal': 'lmk04208_in_12M8_out_122M88.txt',
    'external': 'lmk04208_in_10M_clk0_out_122M88.txt',
}
# Reverse lookup: filename -> source name
_CLOCK_FILE_TO_SOURCE = {v: k for k, v in _CLOCK_SOURCE_FILES.items()}


def get_clock_source():
    """
    Read the current clock source selection from the LMK symlink.

    Returns
    -------
    str
        'internal', 'external', or 'unknown' if the symlink target is
        not recognised.
    """
    try:
        target = os.readlink(KRC_LMK_SYMLINK)
        basename = os.path.basename(target)
        return _CLOCK_FILE_TO_SOURCE.get(basename, 'unknown')
    except OSError as exc:
        print(bcolors.FAIL + f'Failed to read clock source symlink: {exc}' + bcolors.ENDC)
        return 'unknown'


def get_clock_status():
    """
    Query the PLL lock status of all clock chips via ``krc-utils status``.

    Returns
    -------
    dict
        Keys: 'all_locked' (bool), 'chips' (list of dicts with 'name' and
        'status' for each clock chip).
    """
    try:
        result = subprocess.run(
            [KRC_UTILS_BIN, 'status'],
            capture_output=True, text=True, timeout=10,
        )
        chips = []
        for line in result.stdout.strip().splitlines():
            # Lines look like: "[lmk04208.0] status: locked"
            line = line.strip()
            if not line:
                continue
            try:
                name_part, status_part = line.split('] status:')
                name = name_part.strip().lstrip('[').strip()
                status = status_part.strip()
                chips.append({'name': name, 'status': status})
            except ValueError:
                continue
        all_locked = all(c['status'] == 'locked' for c in chips) if chips else False
        return {'all_locked': all_locked, 'chips': chips}
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as exc:
        print(bcolors.FAIL + f'Failed to query clock status: {exc}' + bcolors.ENDC)
        return {'all_locked': False, 'chips': [], 'error': str(exc)}


def select_clock_source(source):
    """
    Point the LMK symlink at the requested source WITHOUT applying it.

    Updates (and chowns) the active-config symlink only; it does NOT run
    ``krc-utils init``. Use :func:`apply_clock_config` / :func:`reload_firmware`
    to apply the selection on a blank PL. Separating selection from init keeps
    the dangerous step (init) out of any path that could run while the PL is
    loaded (issue #14).

    Parameters
    ----------
    source : str
        'internal' for the on-board 12.8 MHz oscillator, or
        'external' for a 10 MHz reference on clk0.

    Returns
    -------
    str
        The resolved (normalised) source name.

    Raises
    ------
    ValueError
        If *source* is not 'internal' or 'external'.
    FileNotFoundError
        If the target clock config file is missing.
    """
    source = source.strip().lower()
    if source not in _CLOCK_SOURCE_FILES:
        raise ValueError(f"clock_source must be 'internal' or 'external', got '{source}'")

    target_file = _CLOCK_SOURCE_FILES[source]
    target_path = os.path.join(KRC_CLOCK_DIR, target_file)

    # Verify the target config file exists
    if not os.path.isfile(target_path):
        raise FileNotFoundError(f'Clock config file not found: {target_path}')

    # Update symlink — remove old, create new
    print(f'Selecting clock source: {source} -> {target_file}')
    if os.path.islink(KRC_LMK_SYMLINK) or os.path.exists(KRC_LMK_SYMLINK):
        os.unlink(KRC_LMK_SYMLINK)
    os.symlink(target_file, KRC_LMK_SYMLINK)

    # Ensure casper owns the symlink (server often runs as root)
    try:
        pw = pwd.getpwnam('casper')
        os.lchown(KRC_LMK_SYMLINK, pw.pw_uid, pw.pw_gid)
    except (KeyError, OSError) as exc:
        print(bcolors.WARNING + f'Could not chown symlink to casper: {exc}' + bcolors.ENDC)

    return source


def set_clock_source(source):
    """
    Select the reference clock source AND apply it via ``krc-utils init``.

    WARNING: this runs ``krc-utils init``, which must only happen on a blank /
    deprogrammed PL (issue #14: running it while the PL is loaded with tones
    collapses the PL power rail and hangs the PS). It is called from
    :func:`apply_clock_config` inside the deprogram-first :func:`reload_firmware`
    sequence. Do not call it directly while the PL is programmed.

    Parameters
    ----------
    source : str
        'internal' for the on-board 12.8 MHz oscillator, or
        'external' for a 10 MHz reference on clk0.

    Returns
    -------
    dict
        Clock status after applying the change (see :func:`get_clock_status`).

    Raises
    ------
    ValueError
        If *source* is not 'internal' or 'external'.
    """
    current = get_clock_source()
    source = select_clock_source(source)
    if current == source:
        print(f'Clock source already set to {source}, re-applying settings.')

    # Apply the new clock configuration
    print('Applying clock configuration via krc-utils init ...')
    try:
        result = subprocess.run(
            [KRC_UTILS_BIN, 'init'],
            capture_output=True, text=True, timeout=30,
        )
        print(result.stdout)
        if result.returncode != 0:
            print(bcolors.FAIL + f'krc-utils init returned non-zero exit code: {result.returncode}' + bcolors.ENDC)
            if result.stderr:
                print(result.stderr)
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as exc:
        print(bcolors.FAIL + f'Failed to run krc-utils init: {exc}' + bcolors.ENDC)
        return {'all_locked': False, 'chips': [], 'error': str(exc)}

    # Return the lock status after applying
    status = get_clock_status()
    if not status.get('all_locked', False):
        print(bcolors.WARNING + 'WARNING: Not all clocks are locked after setting clock source.' + bcolors.ENDC)
    return status


def get_tone_powers(r, r_fast, config_dict, detailed_output=False, reference_plane='detector',
                    rf_peripherals=None):
    """
    Get current tone powers at the specified reference plane.

    Covers the full signal chain from DAC through to the accumulator. TX
    planes are computed from current tone settings. RX planes are modelled
    forward from the configured TX endpoint through the RX chain, which is most
    useful for loopback or known-through paths. Detector/resonator S21 must be
    included in the configured model or applied separately. This function does
    not read accumulator samples. When detailed_output is True, returns power
    at every intermediate stage in both the TX and RX chains.

    Parameters
    ----------
    reference_plane : str
        TX chain (DAC -> detector):
            'dac'              - DAC output (after VOP, before analog frontend)
            'rf_output'        - RF frontend output (after amp, before cryostat)
            'detector'         - cryogenic focal plane (default)
        RX chain, modelled forward from the configured TX endpoint:
            'cryostat_output'  - cryostat output (before RX frontend)
            'adc_input'        - ADC input (after RX frontend)
            'accumulator'      - modelled accumulated IQ magnitude in dB
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

    has_rf = _rf_has_readable_attenuator(rf_peripherals)

    # ---- TX chain ----
    p = _gather_tx_chain_params(r, r_fast, config_dict, rf_peripherals=rf_peripherals)
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

    # ---- RX chain (forward computation from the configured TX endpoint) ----
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
        acc_len = r.mixer.get_acc_len()
        rx_mix_scale = config_dict['firmware'].get('rx_mix_scale', 1.0)

        # ADC DSA (RFDC digital step attenuator, before ADC)
        adc_dsa_db = float(r.rfdc.core.get_dsa(adc_tile, adc_block)['dsa'])

        # RX frontend parameters.  The forward RX model starts from the TX
        # tone powers, so frequency-dependent cryostat/RF terms must be
        # resolved in TX tone order.
        tx_rf_freq = np.asarray(freq_details['tx']['rf_output_freq'], dtype=float)
        tx_if_freq = np.asarray(freq_details['tx']['analog_output_freq'], dtype=float)
        rx_if_freq = np.asarray(
            freq_details['rx'].get('analog_input_freq', tx_if_freq),
            dtype=float)
        if rx_if_freq.size != tx_rf_freq.size:
            rx_if_freq = tx_if_freq

        rx_combiner_loss_db = _resolve_cal_value(
            config_dict['rf_frontend'].get('rx_combiner_loss_db', 0),
            rx_if_freq)
        has_live_rf = has_rf and getattr(rf_peripherals, 'is_hardware', False)
        rx_attenuator_value_db = None
        if has_live_rf:
            rx_attenuator_value_db = rf_peripherals.get_rx_attenuation()
        if rx_attenuator_value_db is None:
            rx_attenuator_value_db = config_dict['rf_frontend'].get('attenuator', {}).get('rx_value_db', None)
        if rx_attenuator_value_db is None:
            rx_attenuator_value_db = 0
        rx_if_s21_db = _resolve_cal_value(
            config_dict['rf_frontend'].get('rx_if_s21_db', 0),
            rx_if_freq)
        rx_mixer_conversion_loss_db = _resolve_cal_value(
            config_dict['rf_frontend'].get('rx_mixer_conversion_loss_db', 0),
            rx_if_freq)
        rx_rf_s21_db = _resolve_cal_value(
            config_dict['rf_frontend'].get('rx_rf_s21_db', 0),
            tx_rf_freq)
        rx_bypass_amp_s21_db = _gather_bypass_amp_s21(
            config_dict, rf_peripherals, 'rx', tx_rf_freq)
        cryostat_output_s21_db = _resolve_cal_value(
            config_dict['cryostat'].get('output_s21_db', 0),
            tx_rf_freq)

        if not p['rf_connected']:
            rx_combiner_loss_db = 0
            rx_attenuator_value_db = 0
            rx_if_s21_db = 0
            rx_mixer_conversion_loss_db = 0
            rx_rf_s21_db = 0
            rx_bypass_amp_s21_db = 0
        if not p['cryo_connected']:
            cryostat_output_s21_db = 0

        # Forward computation: start from the detector-side TX endpoint, then
        # apply the cryostat output S21 to reach the RX frontend input plane.
        tx_endpoint_power_dbm = tx_powers
        cryostat_output_power_dbm = tx_endpoint_power_dbm + cryostat_output_s21_db

        rx_iq, rx_details = calibration.calc_accumulated_iq_level(
            cryostat_output_power_dbm, adc_dbm_to_dbfs, adc_mixer_qmc_gain, adc_mixer_scale_is_1p0,
            adc_bits, pfb_fftshift, rx_mix_scale, acc_len,
            rx_combiner_loss_db=rx_combiner_loss_db,
            rx_attenuator_value_db=rx_attenuator_value_db,
            rx_if_s21_db=rx_if_s21_db,
            rx_mixer_conversion_loss_db=rx_mixer_conversion_loss_db,
            rx_rf_s21_db=rx_rf_s21_db,
            rx_bypass_amp_s21_db=rx_bypass_amp_s21_db,
            cryostat_output_s21_db=0,
            adc_dsa_db=adc_dsa_db,
            detailed_output=True)
        rx_details['detector_dbm'] = np.atleast_1d(tx_endpoint_power_dbm).tolist()
        rx_details['cryostat_output_s21_db'] = np.atleast_1d(cryostat_output_s21_db).tolist()
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
                    rx_policy='protect',
                    autosync=False,
                    mrst=False,
                    force_tx_amp_bypass=None, force_rx_amp_bypass=None,
                    force_tx_attenuation_db=None,
                    force_rx_attenuation_db=None,
                    force_adc_dsa_db=None, force_tone_amplitudes=None,
                    force_psb_fftshift=None, force_psb_scale=None,
                    force_pfb_fftshift=None):
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
        - ``'maximise'`` — run ``maximise_rx_power()`` after the TX
          change to optimise the RX attenuator, DSA, RX amp, and PFB
          FFT shift.
        - ``'raise'`` — raise ``RuntimeError`` if ADC saturates.
        - ``'none'`` — do not check or touch the RX path.

        See :func:`_apply_rx_policy` for full details.
    autosync : bool
        If True, trigger firmware sync after tone-amplitude writes.

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

    _apply_forced_power_controls(
        r, config_dict, rf_peripherals=rf_peripherals,
        force_tx_amp_bypass=force_tx_amp_bypass,
        force_rx_amp_bypass=force_rx_amp_bypass,
        force_tx_attenuation_db=force_tx_attenuation_db,
        force_rx_attenuation_db=force_rx_attenuation_db,
        force_adc_dsa_db=force_adc_dsa_db,
        force_tone_amplitudes=force_tone_amplitudes,
        force_psb_fftshift=force_psb_fftshift,
        force_psb_scale=force_psb_scale,
        force_pfb_fftshift=force_pfb_fftshift)

    tx_atten_fixed = force_tx_attenuation_db is not None
    tx_amp_fixed = force_tx_amp_bypass is not None
    tx_amp_forced_bypassed = (
        tx_amp_fixed
        and _coerce_forced_bool(force_tx_amp_bypass, 'force_tx_amp_bypass')
    )
    tone_amplitudes_fixed = force_tone_amplitudes is not None
    psb_fftshift_fixed = force_psb_fftshift is not None
    psb_scale_fixed = force_psb_scale is not None

    # ---- Phase 1: GATHER ----
    p = _gather_tx_chain_params(r, r_fast, config_dict, rf_peripherals=rf_peripherals)
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
    has_rf = (_rf_has_controllable_attenuator(rf_peripherals)
              and reference_plane != 'dac')
    has_bypass_amps = has_rf and _rf_supports_bypass_amps(rf_peripherals)
    s21_enabled_value = 0.0
    s21_bypassed_value = 0.0
    s21_enabled = 0.0
    s21_bypassed = 0.0
    if has_bypass_amps:
        current_bypass = rf_peripherals.get_tx_amp_bypass()
        if not tx_amp_fixed:
            s21_enabled_value = _gather_bypass_amp_s21_for_state(
                config_dict, rf_peripherals, 'tx',
                p['freq_details']['tx']['rf_output_freq'], False)
            s21_bypassed_value = _gather_bypass_amp_s21_for_state(
                config_dict, rf_peripherals, 'tx',
                p['freq_details']['tx']['rf_output_freq'], True)
            s21_enabled = _mean_cal_value_db(s21_enabled_value)
            s21_bypassed = _mean_cal_value_db(s21_bypassed_value)

    if not optimise_dynamic_range:
        # Simple mode: just compute amplitudes with current settings
        max_amp = (1 - 2**-12) / max_tones_per_bin
        if tone_amplitudes_fixed:
            amps = get_tone_amplitudes(r, r_fast, config_dict)
            print('set_tone_powers: tone amplitudes fixed — skipping amplitude calculation')
        else:
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
        init_amps_max = float(np.max(get_tone_amplitudes(r, r_fast, config_dict)))
        if not tone_amplitudes_fixed:
            set_tone_amplitudes(r, config_dict, amps, autosync=autosync, mrst=mrst)
        new_amps_max = float(np.max(np.abs(amps)))
        if init_amps_max > 0 and new_amps_max > 0:
            amp_change_db = float(20 * np.log10(new_amps_max / init_amps_max))
            _apply_rx_policy(r, r_fast, config_dict, rf_peripherals, rx_policy,
                             tx_power_change_db=amp_change_db,
                             force_rx_amp_bypass=force_rx_amp_bypass,
                             force_rx_attenuation_db=force_rx_attenuation_db,
                             force_adc_dsa_db=force_adc_dsa_db,
                             force_pfb_fftshift=force_pfb_fftshift)
        else:
            _apply_rx_policy(r, r_fast, config_dict, rf_peripherals, rx_policy,
                             tx_power_change_db=None,
                             force_rx_amp_bypass=force_rx_amp_bypass,
                             force_rx_attenuation_db=force_rx_attenuation_db,
                             force_adc_dsa_db=force_adc_dsa_db,
                             force_pfb_fftshift=force_pfb_fftshift)

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

        achieved = get_tone_powers(r, r_fast, config_dict, reference_plane=reference_plane,
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
            'tx_amp_bypass': rf_peripherals.get_tx_amp_bypass() if has_bypass_amps else None,
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
    if tone_amplitudes_fixed:
        amps = get_tone_amplitudes(r, r_fast, config_dict)
        print('  step 1: tone amplitudes fixed')
    else:
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

        if psb_scale_fixed:
            print('  step 1: psb_scale fixed — setting amplitudes without mute')
        else:
            r.psbscale.set_scale(0)
            time.sleep(0.01)
        set_tone_amplitudes(r, config_dict, amps, autosync=autosync, mrst=mrst)
        time.sleep(0.01)
        print(f'  step 1: maximise digital — amplitudes at max')

    if psb_fftshift_fixed:
        best_fftshift = r.psb.get_fftshift()
        print(f'  step 2: PSB fftshift fixed at {format(best_fftshift, "#016b")}')
    elif psb_scale_fixed:
        best_fftshift = r.psb.get_fftshift()
        print('  step 2: PSB fftshift search skipped because psb_scale is fixed')
    else:
        best_fftshift, _, _ = _find_best_psb_fftshift(r)

    # Ramp psb_scale to just below DAC saturation (same as maximise_tx_power)
    if psb_scale_fixed:
        optimal_psb_scale = r.psbscale.get_scale()
        print(f'  step 2: psb_scale fixed at {optimal_psb_scale:.6f}')
    else:
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

        safe_scale = scalemin
        failed_scale = float('inf')
        if estimated_scale > scalemin:
            if _is_ok(estimated_scale):
                safe_scale = estimated_scale
                print(f'     estimate {estimated_scale:.4f} ok')
            else:
                failed_scale = estimated_scale
                print(f'     estimate {estimated_scale:.4f} saturates')
                probe_scale = max(estimated_scale / 2.0, scalemin)
                while probe_scale >= scalemin:
                    if _is_ok(probe_scale):
                        safe_scale = probe_scale
                        print(f'     backoff 6.0 dB: {probe_scale:.4f} ok')
                        break
                    failed_scale = min(failed_scale, probe_scale)
                    print(f'     backoff 6.0 dB: {probe_scale:.4f} saturates')
                    if probe_scale <= scalemin:
                        break
                    probe_scale = max(probe_scale / 2.0, scalemin)

        step_factors = [2.0, 2**0.5, 10**(1/20), 10**(0.5/20), 10**(0.1/20)]
        for step_db, factor in zip([6, 3, 1, 0.5, 0.1], step_factors):
            scale = safe_scale
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
    max_digital_powers = get_tone_powers(r, r_fast, config_dict,
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
    tx_amp_bypass = rf_peripherals.get_tx_amp_bypass() if has_bypass_amps else None
    tx_amp_s21 = cal['tx_bypass_amp_s21_db']
    current_tx_amp_s21_db = _mean_cal_value_db(tx_amp_s21)
    if not np.isfinite(current_tx_amp_s21_db):
        current_tx_amp_s21_db = 0.0

    if abs(delta_db) > 0.5:
        print(f'  step 4: adjust analog ({delta_db:+.1f} dB)')
        remaining = delta_db  # positive = need to reduce, negative = need to increase

        if has_rf:
            atten_step = rf_peripherals.ATTEN_STEP
            atten_max = rf_peripherals.ATTEN_MAX
            min_atten = rf_peripherals.ATTEN_MIN

            def _round_attenuation(value):
                if atten_step <= 0:
                    return float(np.clip(value, min_atten, atten_max))
                rounded = round(value / atten_step) * atten_step
                return float(np.clip(rounded, min_atten, atten_max))

            if has_bypass_amps:
                if tx_amp_fixed:
                    amp_candidates = [(
                        tx_amp_bypass, tx_amp_s21, current_tx_amp_s21_db,
                        'fixed')]
                else:
                    amp_candidates = [
                        (True, s21_bypassed_value, s21_bypassed, 'bypassed'),
                        (False, s21_enabled_value, s21_enabled, 'enabled'),
                    ]
            else:
                amp_candidates = [(
                    tx_amp_bypass, tx_amp_s21, current_tx_amp_s21_db,
                    'current')]

            candidates = []
            for bypass_state, amp_s21_value, amp_s21_db, amp_label in amp_candidates:
                if not np.isfinite(amp_s21_db):
                    amp_s21_db = current_tx_amp_s21_db
                required_atten = (
                    tx_atten_db + delta_db
                    + (amp_s21_db - current_tx_amp_s21_db)
                )
                new_atten = (
                    tx_atten_db if tx_atten_fixed
                    else _round_attenuation(required_atten)
                )
                candidate_remaining = (
                    delta_db
                    + (amp_s21_db - current_tx_amp_s21_db)
                    - (new_atten - tx_atten_db)
                )
                margin = min(new_atten - min_atten, atten_max - new_atten)
                candidates.append({
                    'bypass': bypass_state,
                    'amp_s21': amp_s21_value,
                    'amp_s21_db': amp_s21_db,
                    'amp_label': amp_label,
                    'atten': new_atten,
                    'remaining': float(candidate_remaining),
                    'margin': float(margin),
                })

            exact_candidates = [
                c for c in candidates if abs(c['remaining']) <= 0.5
            ]
            positive_candidates = [
                c for c in candidates if c['remaining'] > 0.1
            ]

            def _bypass_rank(candidate):
                if not has_bypass_amps or tx_amp_fixed:
                    return 0
                return 0 if candidate['bypass'] is True else 1

            if exact_candidates:
                best = min(
                    exact_candidates,
                    key=lambda c: (
                        _bypass_rank(c),
                        abs(c['remaining']),
                        -c['margin'],
                    ))
            elif positive_candidates:
                best = min(
                    positive_candidates,
                    key=lambda c: (
                        c['remaining'],
                        _bypass_rank(c),
                        -c['margin'],
                    ))
            else:
                best = max(candidates, key=lambda c: c['remaining'])

            if (has_bypass_amps and best['bypass'] != tx_amp_bypass
                    and not tx_amp_fixed):
                rf_peripherals.set_tx_amp_bypass(best['bypass'])
                time.sleep(0.1)
                state = 'bypassed' if best['bypass'] else 'enabled'
                print(f'    TX amp: {state}')
            elif has_bypass_amps and tx_amp_fixed:
                print(f'    TX amp fixed at bypass={tx_amp_bypass}')

            if abs(best['atten'] - tx_atten_db) > 1e-6 and not tx_atten_fixed:
                old_atten = tx_atten_db
                rf_peripherals.set_tx_attenuation(best['atten'])
                time.sleep(0.1)
                print(f'    TX atten: {old_atten:.1f} -> {best["atten"]:.1f} dB')
            elif tx_atten_fixed:
                print(f'    TX atten fixed at {tx_atten_db:.1f} dB')

            tx_amp_bypass = best['bypass']
            tx_amp_s21 = best['amp_s21']
            tx_atten_db = best['atten']
            remaining = best['remaining']
            print(f'    RF candidate: amp {best["amp_label"]}, '
                  f'atten {tx_atten_db:.1f} dB, residual {remaining:+.2f} dB')

            if remaining < -0.5:
                hint = (
                    ' Try disabling the TX amp bypass.'
                    if tx_amp_forced_bypassed else ''
                )
                raise ValueError(
                    f'Target power exceeds maximum achievable by '
                    f'{-remaining:.1f} dB at '
                    f'reference_plane={reference_plane!r}. '
                    f'Reduce tone_powers_dbm or num_tones.{hint}')

        elif remaining < -0.5:
            # No RF peripherals and need more power than digital max
            raise ValueError(
                f'Target power exceeds maximum achievable by '
                f'{-remaining:.1f} dB at reference_plane={reference_plane!r} '
                f'with no RF peripherals. Reduce tone_powers_dbm or num_tones.')

        # 4c: Last resort — reduce psb_scale for any remaining excess
        if remaining > 0.1:
            if psb_scale_fixed:
                msg = (f'psb_scale fixed; {remaining:.1f} dB excess power remains')
                print(f'    WARNING: {msg}')
                warnings_list.append(msg)
            else:
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
    if tone_amplitudes_fixed:
        final_amps = get_tone_amplitudes(r, r_fast, config_dict)
        print('  final amplitudes fixed — skipping exact amplitude solve')
    else:
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
        set_tone_amplitudes(r, config_dict, final_amps, autosync=autosync)
        time.sleep(0.01)

    # RX policy check for the overall TX power change
    init_powers = get_tone_powers(r, r_fast, config_dict, reference_plane=reference_plane,
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
                     tx_power_change_db=tx_power_change_db,
                     force_rx_amp_bypass=force_rx_amp_bypass,
                     force_rx_attenuation_db=force_rx_attenuation_db,
                     force_adc_dsa_db=force_adc_dsa_db,
                     force_pfb_fftshift=force_pfb_fftshift)

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
    if has_bypass_amps:
        print(f'    TX atten: {tx_atten_db:.1f} dB, TX amp bypass: {tx_amp_bypass}')
    elif has_rf:
        print(f'    TX atten: {tx_atten_db:.1f} dB')
    print(f'    effective DAC bits (worst): {float(np.nanmin(eff_bits)):.1f}')
    print(f'    DAC headroom (measured): {dac_headroom_db:.1f} dB')

    if max_tones_per_bin > 1:
        warnings_list.append(
            f'Up to {max_tones_per_bin} tones share an FFT bin, '
            f'amplitudes scaled by 1/{max_tones_per_bin}')

    # --- Verify ---
    achieved = get_tone_powers(r, r_fast, config_dict, reference_plane=reference_plane,
                               rf_peripherals=rf_peripherals)
    final_amps_read = get_tone_amplitudes(r, r_fast, config_dict)
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


def force_sync_fast(r_fast, wait_s=0.0001, mrst=False, do_sync=True):
    """
    Fire a firmware sync by poking the mmap directly (bypasses the slow casperfpga
    transport) — the fast-path equivalent of ``r.sync.sw_sync``.

    v7.10 timed-sync register layout: the sync pulse lives in the ``timed_sync_ctrl``
    register (bit ``OFFSET_TIMED_SYNC_SW_SYNC``) and no longer needs arming; master-reset
    is a separate, independent bit in ``ctrl`` (``OFFSET_MRST``).

    :param mrst: if True, pulse the master reset *before* the sync (full reset+start, the
        old welded behaviour). If False (default, per-step), re-reference only — the LO
        rides continuous accumulation across the buffer flip.
    :param do_sync: if True (default), fire the sync pulse. Set do_sync=False with mrst=True
        for a reset-only poke. The two knobs give every combination for bench testing.

    NOTE (deferred capability — see [[fast-mod-sync-investigation]]): a per-step *immediate*
    sync is still software-timed, so it can land mid-accumulation (the occasional glitch).
    The deterministic fix is the v7.10 *timed* sync — ``set_timed_sync(tt)`` firing at an
    accumulation-boundary telescope time — but that needs TT/PPS disciplining
    (``update_internal_time``), which this codebase does NOT do yet (telescope_time is
    free-running). Wiring that up is the follow-on; this port only moves the existing
    immediate sync onto the new registers. Firmware caveats to raise with the HDL owner:
    7.10 ``set_timed_sync`` writes ``timed_sync_msb`` + ``timed_sync_enable`` but never a
    low word (looks incomplete), and raises if the target TT is already in the past.
    """
    pid = r_fast.pipeline_id
    cache = f'_p{pid}_sync_fast'
    if not hasattr(r_fast, f'{cache}_ctrl_addr'):
        sync = r_fast.sync
        if not hasattr(sync, 'OFFSET_TIMED_SYNC_SW_SYNC'):
            raise RuntimeError(
                'souk_mkid_readout predates v7.10 timed-sync (no OFFSET_TIMED_SYNC_SW_SYNC); '
                'upgrade the server venv library to match the 7.10 firmware.')
        tr = sync.host.transport
        setattr(r_fast, f'{cache}_ctrl_addr', tr._get_device_address(f'{sync.prefix}ctrl'))
        setattr(r_fast, f'{cache}_timed_addr', tr._get_device_address(f'{sync.prefix}timed_sync_ctrl'))
        setattr(r_fast, f'{cache}_mrst_bit', 1 << sync.OFFSET_MRST)
        setattr(r_fast, f'{cache}_swsync_bit', 1 << sync.OFFSET_TIMED_SYNC_SW_SYNC)

    ctrl_addr = getattr(r_fast, f'{cache}_ctrl_addr')
    timed_addr = getattr(r_fast, f'{cache}_timed_addr')
    mrst_bit = getattr(r_fast, f'{cache}_mrst_bit')
    swsync_bit = getattr(r_fast, f'{cache}_swsync_bit')
    mm = r_fast.sync.host.transport.axil_mm

    def _pulse(addr, bit):
        # 0 -> 1 -> 0 on a single bit, read-modify-write so other bits in the register
        # (e.g. the timed-sync EN bit) are preserved.
        (value,) = struct.unpack('<I', mm[addr:addr+4])
        for level in (0, bit, 0):
            value = (value & ~bit) | level
            mm[addr:addr+4] = struct.pack('<I', value)

    if mrst:
        _pulse(ctrl_addr, mrst_bit)   # master reset (independent ctrl bit in 7.10)

    time.sleep(wait_s)

    if do_sync:
        _pulse(timed_addr, swsync_bit)  # sync pulse on timed_sync_ctrl[SW_SYNC]

    return


# ---------------------------------------------------------------------------
# v7.10 firmware *timed* sync (telescope-time-aligned, deterministic)
#
# These three functions lift the working logic out of the standalone bring-up
# scripts in scripts/timed_sync/ into the library, so the server can expose them.
# Each is a near 1:1 port of one script -- the comments name the same registers
# and souk_mkid_readout/blocks/sync.py functions the scripts use:
#   timed_sync_alignment <- 03_set_tt.py   (read PPS-latched TT, offset from boundary)
#   timed_sync_drift     <- 07_tt_drift.py (slope of that offset = ppm drift)
#   timed_sync_arm       <- 04_timed_sync.py (load TT, arm set_timed_sync at a future sec)
# The telescope-time (TT) counter runs at the fabric/DSP clock = adc_clk_hz / 8 =
# 307.2 MHz (NOT adc_clk_hz). All three use the fast (local=True, /dev/mem)
# interface -- ~35x lower read jitter than KATCP (see scripts 06/07).
# ---------------------------------------------------------------------------

def timed_sync_alignment(r_fast):
    """
    Read how well the firmware telescope time (TT) is sitting on the PPS second
    boundary right now. Read-only -- does NOT load or move the TT.

    Port of scripts/timed_sync/03_set_tt.py's read-back block. The TT counter (sync
    block ``p{pid}_sync``) is latched into ``ext_sync_tt_*`` on every PPS edge from
    the TSU strobe. A correctly aligned TT (freshly loaded) lands within ~1 us of the
    integer second; that offset grows as the fabric clock drifts vs the PPS (see
    :func:`timed_sync_drift`).

    IMPORTANT: there is NO live readback of the internal TT counter. The sync block
    exposes the TT only as values latched at an event -- ``ext_sync_tt`` (last PPS),
    ``tt_sync`` (last system sync) -- and ``int_tt_load`` is a write/staging register,
    not a counter readback. This function reads ``ext_sync_tt``, latched at the LAST PPS
    edge, so the value always sits ~on an integer second; it is the *last PPS* TT, NOT
    the instantaneous current TT. Hence the keys are named ``last_pps_*`` and
    ``last_pps_unix_s`` reads ~N.9999/N.0000; see ``pps_boundary_offset_s`` for the
    sub-us offset from the second boundary.

    Registers (souk_mkid_readout/blocks/sync.py):
      ext_sync_tt_msb/lsb -- internal TT latched at the last PPS. Read directly (a
        plain register read is non-blocking), NOT via Sync.get_tt_of_ext_sync(),
        which calls wait_for_sync() and blocks for a whole PPS period.

    :param r_fast: fast (local=True, /dev/mem) readout interface.
    :return: dict with
        last_pps_tt          -- raw TT (fabric clocks since the UNIX epoch) at the last PPS
        last_pps_unix_s      -- that last-PPS TT as UNIX seconds (sits ~on an integer second)
        last_pps_utc         -- last_pps_unix_s as an ISO-8601 UTC string
        pps_boundary_offset_s -- last-PPS TT offset from the integer second, folded to
                             +/-0.5 s. THE precise (~us) alignment metric: how far the
                             PPS-latched TT sits off the second boundary; both sides are
                             firmware-internal (TT counter vs PPS edge).
        system_offset_s      -- last_pps_unix_s - time.time(): the last-PPS TT minus the
                             host Linux clock. COARSE, whole-second check only that the
                             firmware TT is on the right second (used by timed_sync_arm to
                             decide a reload). Its sub-second part is dominated by the time
                             elapsed since that PPS + host read/bus latency, NOT a precise
                             clock error -- use pps_boundary_offset_s for precision.
        last_sync_tt         -- raw TT of the last firmware sync event (tt_sync register,
                             Sync.get_tt_of_sync). 0/None if no sync has fired since program.
        last_sync_unix_s     -- last_sync_tt as UNIX seconds (None if no sync yet).
        last_sync_utc        -- last_sync_unix_s as an ISO-8601 UTC string (None if no sync).
        time_since_last_sync_s -- last_pps_unix_s - last_sync_unix_s; how long since the last
                             firmware sync FIRED (None if no sync yet). Display only -- it is
                             NOT the drift reference: a timed sync does not reload the TT, so
                             pps_boundary_offset_s keeps accumulating across syncs. The drift
                             reference is the last TT *load* (update_internal_time), which this
                             firmware-only read does not know -- the server divides
                             pps_boundary_offset_s by the time since the last load it tracked.
        clk_hz               -- fabric clock used (adc_clk_hz / 8).

    The drift *in seconds* since the last TT load is pps_boundary_offset_s itself; converting
    to ppm needs the load time (server-side), so drift_ppm is NOT computed here.
    """
    sync = r_fast.sync
    clk_hz = int(round(r_fast.adc_clk_hz / 8))  # fabric/DSP clock, 307.2 MHz
    # Non-blocking read of the PPS-latched TT (two 32-bit halves). This is the TT at the
    # last PPS edge, NOT the live counter (the sync block exposes no live TT readback).
    last_pps_tt = (sync.read_uint('ext_sync_tt_msb') << 32) + sync.read_uint('ext_sync_tt_lsb')
    last_pps_unix_s = last_pps_tt / clk_hz
    # Precise check: a TT latched on a PPS edge should sit on an integer second. This offset
    # is the accumulated TT-vs-PPS drift since the last TT *load* (not since the last sync).
    pps_boundary_offset_s = (last_pps_tt % clk_hz) / clk_hz
    if pps_boundary_offset_s > 0.5:
        pps_boundary_offset_s -= 1.0  # fold to +/-0.5 s so a tiny negative offset reads as such

    # TT of the last firmware sync (tt_sync register; non-blocking) -- for "time since last
    # sync" only. A sync does NOT reload the TT, so it is not the drift reference.
    last_sync_tt = sync.get_tt_of_sync()
    if last_sync_tt:
        last_sync_unix_s = last_sync_tt / clk_hz
        time_since_last_sync_s = last_pps_unix_s - last_sync_unix_s
    else:  # 0 => no sync has fired since the FPGA was programmed
        last_sync_unix_s = None
        time_since_last_sync_s = None

    return {
        'last_pps_tt': last_pps_tt,
        'last_pps_unix_s': last_pps_unix_s,
        'last_pps_utc': unix_to_iso(last_pps_unix_s),
        'pps_boundary_offset_s': pps_boundary_offset_s,
        'system_offset_s': last_pps_unix_s - time.time(),
        'last_sync_tt': last_sync_tt or None,
        'last_sync_unix_s': last_sync_unix_s,
        'last_sync_utc': unix_to_iso(last_sync_unix_s),
        'time_since_last_sync_s': time_since_last_sync_s,
        'clk_hz': clk_hz,
    }


def set_telescope_time(r_fast):
    """
    Load the firmware telescope time (TT) on this pipeline and confirm it aligns to
    the PPS edge. The operator-facing "set the TT" call: it stages the next integer
    second and the firmware latches it into the TT counter on the next PPS edge, so
    ``telescope_time`` tracks real (PTP-disciplined) UNIX time.

    Wraps ``Sync.update_internal_time`` (see also: scripts/timed_sync/03_set_tt.py).
    BLOCKS ~3-4 s while it waits on PPS edges to measure the period and latch the
    load, so a healthy TSU 1-PPS strobe must be running (it gives the firmware its
    PPS). ``sync_period`` is forced to ``clk_hz``. This sets the TT *value*; to
    arm a deterministic resync at a chosen future second use :func:`timed_sync_arm`.

    Registers (souk_mkid_readout/blocks/sync.py): int_tt_load_msb/lsb + ctrl[ext_load]
    (the staged load, via Sync.load_internal_time) and ext_sync_tt_msb/lsb (the
    PPS-latched read-back).

    :param r_fast: fast (local=True, /dev/mem) readout interface.
    :return: dict -- ``loaded`` (True) and ``aligned`` (|pps_boundary_offset_s| <
        DEFAULT_ALIGN_TOL_S, 0.1 ms) plus the :func:`timed_sync_alignment` fields
        (last_pps_tt, last_pps_unix_s,
        last_pps_utc, pps_boundary_offset_s, system_offset_s, clk_hz).
    """
    sync = r_fast.sync
    clk_hz = int(round(r_fast.adc_clk_hz / 8))  # fabric/DSP clock, 307.2 MHz
    # Stage the next integer second; the firmware latches it on the next PPS edge.
    # BLOCKS ~3-4 s on PPS. sync_period=clk_hz forces a 1 s period (the auto-detect
    # can mis-read it; the load is identical either way).
    sync.update_internal_time(clk_hz=clk_hz, sync_period=clk_hz)
    result = timed_sync_alignment(r_fast)
    result['loaded'] = True
    result['aligned'] = bool(abs(result['pps_boundary_offset_s']) < DEFAULT_ALIGN_TOL_S)
    return result


def timed_sync_drift(r_fast, samples=4, interval_s=1.5):
    """
    Measure how fast the firmware TT is currently drifting against the PPS, by
    watching the PPS-latched boundary offset grow over a few seconds and fitting the
    slope. Read-only -- deliberately does NOT re-load the TT, so it reports the LIVE
    drift of the running counter rather than disturbing it.

    Port of scripts/timed_sync/07_tt_drift.py (minus its up-front update_internal_time).
    The drift is mostly this RFSoC's fabric oscillator vs the PHC-disciplined PPS
    (~1.6 ppm in lab testing), so it is fairly independent of grandmaster quality.
    Because offset is in microseconds and time in seconds, the slope in us/s == ppm.

    NOTE: Sync.get_drift() (drift_msb/lsb) is deliberately NOT used -- on this gateware
    it reads a frozen 0 or some very large number, so it does not track the
    accumulating drift (see 08_arm_drift.py). The ext_sync_tt boundary-offset slope is
    the trustworthy metric.

    :param r_fast: fast (local=True) readout interface.
    :param samples: number of boundary-offset samples (need >=3 for a fit after the
        first is dropped).
    :param interval_s: seconds between samples.
    :return: dict with drift_ppm, drift_us_per_s (== drift_ppm), samples, dwell_s.
    """
    sync = r_fast.sync
    clk_hz = int(round(r_fast.adc_clk_hz / 8))  # fabric/DSP clock, 307.2 MHz

    def boundary_off_us():
        tt = (sync.read_uint('ext_sync_tt_msb') << 32) + sync.read_uint('ext_sync_tt_lsb')
        rem = (tt % clk_hz) / clk_hz
        if rem > 0.5:
            rem -= 1.0
        return rem * 1e6

    t0 = time.time()
    ts, offs = [], []
    for i in range(samples):
        ts.append(time.time() - t0)
        offs.append(boundary_off_us())
        if i < samples - 1:
            time.sleep(interval_s)

    # Drop the first sample: the t=0 read can catch a stale latch (a ~100+ us outlier)
    # before a prior load fully settled -- 07_tt_drift.py's lesson.
    ts_fit, offs_fit = ts[1:], offs[1:]
    n = len(ts_fit)
    drift_ppm = float('nan')
    if n >= 2:
        # Least-squares slope of offset(us) vs t(s). us/s == ppm.
        mt = sum(ts_fit) / n
        mo = sum(offs_fit) / n
        den = sum((t - mt) ** 2 for t in ts_fit)
        if den:
            drift_ppm = sum((t - mt) * (o - mo) for t, o in zip(ts_fit, offs_fit)) / den
    return {
        'drift_ppm': drift_ppm,
        'drift_us_per_s': drift_ppm,
        'samples': samples,
        'dwell_s': ts[-1] if ts else 0.0,
    }


def timed_sync_arm(r_fast, target_unix_s=None, seconds_from_now=None,
                   mrst=False, reload_tt='auto', wait=False):
    """
    Perform a v7.10 firmware *timed* sync: arm a reset+sync (Sync.set_timed_sync) to fire
    when the running telescope-time (TT) counter reaches a chosen FUTURE second. This is
    the low-overhead resync -- the firmware re-aligns the DSP timing network to the PPS
    when it fires, which is what zeroes the timing drift. Returns as soon as the sync is
    armed -- the firmware fires it autonomously at the target -- unless ``wait`` is set.

    The TT *counter value* (Sync.update_internal_time) is reloaded ONLY WHEN NEEDED, not on
    every arm: that call blocks ~3-4 s on PPS edges, and the value only needs reloading
    when it is not yet loaded or has drifted onto the wrong whole second. The value
    free-runs and drifts ~1.6 ppm, but that reaches a whole second -- the only thing that
    changes which second the sync fires on -- after ~7 days; sub-second drift is taken out
    by the PPS re-align at fire. So a reload every arm would just waste ~3-4 s.

    Port of scripts/timed_sync/04_timed_sync.py, following the canonical Sync flow
    (initialize -> update_internal_time -> set_timed_sync(mrst=True); ``initialize()`` and
    the first TT load are setup, then arm repeatedly). The multi-board primitive: point
    every board at the SAME ``target_unix_s`` and they all fire on the same TT edge.

    Sequence:
      [1] (optional) master reset -- assert/deassert ctrl[mrst], which resets the DSP
          *timing network* (the sync/alignment distribution; NOT the sample pipeline or
          accumulator data). Off by default: set_timed_sync already pulses mrst at the arm.
      [2] Sync.update_internal_time() -- ONLY if ``reload_tt`` says so (see below). Stages
          the next integer second and the firmware latches it into the TT counter on the
          next PPS edge. BLOCKS ~3-4 s. sync_period=clk_hz forces a 1 s period.
      [3] Sync.set_timed_sync() -- write timed_sync_time_*, arm timed_sync_ctrl[en]; when
          the running TT reaches the target second the firmware fires the reset+sync, with
          the release **gated on the PPS edge** (so sub-second TT drift does not shift the
          fire instant -- which is why a reload every arm is unnecessary).

    Registers (souk_mkid_readout/blocks/sync.py):
      ctrl[mrst]                     -- reset the DSP timing network (step 1)
      ext_sync_tt_msb/lsb            -- PPS-latched TT, read to decide on a reload
      int_tt_load_*, ctrl[ext_load]  -- TT load (Sync.update_internal_time, step 2)
      timed_sync_time_msb/lsb        -- target TT for the timed sync
      timed_sync_ctrl[en]            -- arm the timed-sync trigger
      timed_sync_countdown           -- fabric clocks remaining until it fires

    NOTE: Sync.set_timed_sync() pulses mrst itself at the arm (and currently ignores its
    own ``mrst`` argument), so the DSP timing network is reset at the arm regardless. The
    ``mrst`` flag on THIS function is the separate, optional EARLY reset in step 1 (04's
    addition, not in the canonical flow). Targets are quantised to a whole second.

    :param r_fast: fast (local=True) readout interface.
    :param target_unix_s: absolute UNIX second to fire on. Mutually exclusive with
        ``seconds_from_now``.
    :param seconds_from_now: fire this many whole seconds after the current TT. Defaults
        to 5 if neither target is given.
    :param mrst: also do the step-1 early master reset (see above). Default False.
    :param reload_tt: when to reload the TT *value*. ``'auto'`` (default) reloads only if
        the TT is not loaded / has drifted ~1 s from the system clock; ``True`` always
        reloads (re-zero the value, ~3-4 s); ``False`` never reloads (fast; caller
        guarantees the TT is already loaded).
    :param wait: if True, block until the sync fires; else return once armed.
    :return: dict with target_tt_unix_s, target_tt_utc, target_tt_value, clk_hz,
        countdown_remaining_s, armed_at_unix_s, armed_at_utc, reloaded_tt,
        tt_loaded_unix_s (the PPS-aligned load second when reloaded, else None), fired.
    """
    if target_unix_s is not None and seconds_from_now is not None:
        raise ValueError("give target_unix_s OR seconds_from_now, not both")

    sync = r_fast.sync
    clk_hz = int(round(r_fast.adc_clk_hz / 8))  # fabric/DSP clock, 307.2 MHz

    # [1] Optional reset of the DSP timing network before anything else (04 step 1).
    if mrst:
        sync.assert_mrst()
        sync.deassert_mrst()
        time.sleep(2.0)  # let the reset settle (matches 04_timed_sync.py)

    # Last-PPS-latched TT (non-blocking direct read of ext_sync_tt; the sync block has no
    # live TT readback): the second we count from, and how we decide on a slow reload.
    last_pps_tt = (sync.read_uint('ext_sync_tt_msb') << 32) + sync.read_uint('ext_sync_tt_lsb')
    system_offset_s = last_pps_tt / clk_hz - time.time()

    # [2] Reload the TT value ONLY if needed. 'auto' reloads when the TT is unset or has
    # drifted ~1 s from the system clock (|offset| > 0.5 s); the per-fire PPS re-align in
    # step 3 takes out sub-second drift, so otherwise we skip the ~3-4 s update_internal_time.
    if reload_tt == 'auto':
        do_reload = abs(system_offset_s) > 0.5
    else:
        do_reload = bool(reload_tt)
    if do_reload:
        sync.update_internal_time(clk_hz=clk_hz, sync_period=clk_hz)  # BLOCKS ~3-4 s on PPS
        last_pps_tt = (sync.read_uint('ext_sync_tt_msb') << 32) + sync.read_uint('ext_sync_tt_lsb')
    now_sec = last_pps_tt / clk_hz

    # [3] Resolve the target second (quantised to a whole second), in fabric clocks.
    if target_unix_s is not None:
        target_sec = int(round(target_unix_s))
    else:
        lead = 5 if seconds_from_now is None else int(seconds_from_now)
        target_sec = int(now_sec) + lead
    target_tt_value = int(round(target_sec * clk_hz))

    # Guard the past-target case ourselves, with a clear operator message, BEFORE
    # set_timed_sync (whose own past-TT branch hits a driver bug -- self.error does not
    # exist). Require a comfortable lead so the live TT cannot slip past during arming.
    if target_sec <= now_sec + 1:
        raise RuntimeError(
            f"timed sync target {time.ctime(target_sec)} ({target_sec}) is not far "
            f"enough ahead of the current telescope time {time.ctime(now_sec)} "
            f"({now_sec:.3f} s); choose a second at least ~2 s in the future.")

    armed_at_unix_s = time.time()
    # Canonical Sync flow: ...update_internal_time() -> set_timed_sync(mrst=True). (The
    # driver currently ignores this mrst arg and always pulses it, but pass True to match
    # the documented intent: the timed sync IS a reset+start trigger at the target.)
    sync.set_timed_sync(target_tt_value, wait=wait, mrst=True)
    # After wait=True this has elapsed to ~0; after wait=False it is roughly the lead.
    countdown_remaining_s = sync.get_time_to_sync() / clk_hz

    return {
        'target_tt_unix_s': float(target_sec),
        'target_tt_utc': unix_to_iso(target_sec),
        'target_tt_value': target_tt_value,
        'clk_hz': clk_hz,
        'countdown_remaining_s': countdown_remaining_s,
        'armed_at_unix_s': armed_at_unix_s,
        'armed_at_utc': unix_to_iso(armed_at_unix_s),
        'reloaded_tt': do_reload,
        # PPS-aligned second the TT was (re)loaded to, when this call reloaded -- the drift
        # reference the server caches. None when no reload happened (TT load unchanged).
        'tt_loaded_unix_s': (now_sec if do_reload else None),
        'fired': bool(wait),
    }


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
