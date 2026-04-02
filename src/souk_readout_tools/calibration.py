

import numpy as np

def calc_tone_powers(amps,
                     psb_fftshift,
                     psb_scale,
                     mixer_scale_is_1p0,
                     mixer_qmc_gain,
                     vop_current,
                     vop_current_fs=20000,
                     dac_dbfs_to_dbm=-6.0,
                     tx_combiner_loss_db=0,
                     tx_attenuator_value_db=0,
                     tx_if_s21_db=0,
                     tx_mixer_conversion_loss_db=0,
                     tx_rf_s21_db=0,
                     tx_bypass_amp_s21_db=0,
                     cryostat_input_s21_db=0,
                     dac_fs_bits=16,
                     detailed_output=False):
    """
    Calculate tone powers from raw amplitude settings.
    Accounts for scaling in the firmware, DAC DUC Mixer, and VOP.
    Requires a calibration of the DAC output power to convert from dBFS to dBm,
    Optionally accounts for analog RF frontend and/or cryostat.
    """
    #convert to numpy array
    amps=np.atleast_1d(amps)
    #convert to shifted PSB units
    psb_shift_units = amps / 2**((bin(psb_fftshift).count('1'))+1)
    #convert to scaled PSB units
    psb_scale_units = psb_shift_units * psb_scale
    #convert to digital units for the DAC 
    dac_units = psb_scale_units * 2**dac_fs_bits
    #convert the digital DAC units to full scale units
    dac_fs = dac_units/2**dac_fs_bits
    #account for any QMC gain correction in the DAC DUC
    duc_fs = dac_fs * mixer_qmc_gain
    #account for mixer scaling in the DAC DUC
    if mixer_scale_is_1p0:
        duc_fs /= 0.7071067811865475
    #convert from full scale voltage untis to full scale power units
    duc_fs_power = duc_fs**2
    #scale by the VOP output current ratio
    vop_fs = duc_fs_power * (vop_current/vop_current_fs)**2
    #convert to dB full scale
    vop_dbfs = 10*np.log10(abs(vop_fs))
    #convert to absolute dBm using the calibration 
    dac_dbm = vop_dbfs + dac_dbfs_to_dbm
    combiner_dbm = dac_dbm - abs(tx_combiner_loss_db)
    tx_attenuator_dbm = combiner_dbm - abs(tx_attenuator_value_db)
    tx_if_dbm = tx_attenuator_dbm + tx_if_s21_db
    tx_mixer_dbm = tx_if_dbm - abs(tx_mixer_conversion_loss_db)
    tx_rf_dbm = tx_mixer_dbm + tx_rf_s21_db
    tx_amp_dbm = tx_rf_dbm + tx_bypass_amp_s21_db
    cryostat_dbm = tx_amp_dbm + cryostat_input_s21_db
    output_power_dbm = cryostat_dbm
    if detailed_output:
        details = {'amps':amps.tolist(),
                   'psb_shift_units':psb_shift_units.tolist(),
                   'psb_scale_units':psb_scale_units.tolist(),
                   'dac_units':dac_units.tolist(),
                   'dac_fs':dac_fs.tolist(),
                   'duc_fs':duc_fs.tolist(),
                   'duc_fs_power':duc_fs_power.tolist(),
                   'vop_fs':vop_fs.tolist(),
                   'vop_dbfs':vop_dbfs.tolist(),
                   'dac_dbm':dac_dbm.tolist(),
                   'combiner_dbm':combiner_dbm.tolist(),
                   'tx_attenuator_dbm':tx_attenuator_dbm.tolist(),
                   'tx_if_dbm':tx_if_dbm.tolist(),
                   'tx_mixer_dbm':tx_mixer_dbm.tolist(),
                   'tx_rf_dbm':tx_rf_dbm.tolist(),
                   'tx_amp_dbm':tx_amp_dbm.tolist(),
                   'cryostat_dbm':cryostat_dbm.tolist(),}
        return output_power_dbm, details
    else:
        return output_power_dbm

def calc_tone_amplitudes(powers_dbm,
                   psb_fftshift,
                   psb_scale,
                   mixer_scale_is_1p0,
                   mixer_qmc_gain,
                   vop_current,
                   vop_current_fs=20000,
                   dac_dbfs_to_dbm=-6.0,
                   tx_combiner_loss_db=0,
                   tx_attenuator_value_db=0,
                   tx_if_s21_db=0,
                   tx_mixer_conversion_loss_db=0,
                   tx_rf_s21_db=0,
                   tx_bypass_amp_s21_db=0,
                   cryostat_input_s21_db=0,
                   dac_fs_bits=16,
                   detailed_output=False):
    """
    Calculate tone amplitudes from desired powers.
    Accounts for scaling in the firmware, DAC DUC Mixer, VOP, rf frontend and cryostat.
    Requires a calibration of the DAC output power to convert from dBFS to dBm, defaulting to 0 dBFS = -6.0 dBm if unavailable.
    """
    #convert to numpy array
    output_powers_dbm = np.atleast_1d(powers_dbm)
    #assume reference power is final stage in the chain
    cryostat_dbm = output_powers_dbm
    #account for cryostat s21
    tx_amp_dbm = cryostat_dbm - cryostat_input_s21_db
    #account for amp s21 (gain when enabled, insertion loss when bypassed)
    tx_rf_dbm = tx_amp_dbm - tx_bypass_amp_s21_db
    #account for rf frontend s21
    tx_mixer_dbm = tx_rf_dbm - tx_rf_s21_db
    tx_if_dbm = tx_mixer_dbm + abs(tx_mixer_conversion_loss_db)
    tx_attenuator_dbm = tx_if_dbm - tx_if_s21_db
    combiner_dbm = tx_attenuator_dbm + abs(tx_attenuator_value_db)
    dac_dbm = combiner_dbm + abs(tx_combiner_loss_db)
    #account for the calibration of the DAC, converting dBm to dBFS
    vop_dbfs = dac_dbm - dac_dbfs_to_dbm
    #convert dBFS to full scale power units
    vop_fs = 10**(vop_dbfs/10)
    #account for the VOP output current ratio
    duc_fs_power = vop_fs / (vop_current/vop_current_fs)**2
    #convert to full scale voltage units
    duc_fs = np.sqrt(duc_fs_power)
    #account for mixer scaling in the DAC DUC
    if mixer_scale_is_1p0:
        duc_fs *= 0.7071067811865475
    #account for any QMC gain correction in the DAC DUC
    dac_fs = duc_fs / mixer_qmc_gain
    #convert to DAC digital units
    dac_units = dac_fs * 2**dac_fs_bits
    #convert to scaled PSB units
    psb_scale_units = dac_units / 2**dac_fs_bits
    #convert to shifted PSB units
    psb_shift_units = psb_scale_units / psb_scale 
    #convert to amplitudes
    # amps = psb_shift_units * (psb_fftshift+1) *2
    amps = psb_shift_units * 2**((bin(psb_fftshift).count('1'))+1)

    if detailed_output:
        details = {'powers_dbm':powers_dbm.tolist(),
                   'cryostat_dbm':cryostat_dbm.tolist(),
                   'tx_amp_dbm':tx_amp_dbm.tolist(),
                   'tx_rf_dbm':tx_rf_dbm.tolist(),
                   'tx_mixer_dbm':tx_mixer_dbm.tolist(),
                   'tx_if_dbm':tx_if_dbm.tolist(),
                   'tx_attenuator_dbm':tx_attenuator_dbm.tolist(),
                   'combiner_dbm':combiner_dbm.tolist(),
                   'dac_dbm':dac_dbm.tolist(),
                   'vop_dbfs':vop_dbfs.tolist(),
                   'vop_fs':vop_fs.tolist(),
                   'duc_fs_power':duc_fs_power.tolist(),
                   'duc_fs':duc_fs.tolist(),
                   'dac_fs':dac_fs.tolist(),
                   'dac_units':dac_units.tolist(),
                   'psb_scale_units':psb_scale_units.tolist(),
                   'psb_shift_units':psb_shift_units.tolist(),
                   'amps':amps.tolist(),}
        return amps, details
    else:
        return amps
    
def calc_accumulated_iq_level(adc_input_power_dbm,adc_dbm_to_dbfs,mixer_qmc_gain,mixer_scale_is_1p0,adc_bits,pfb_fftshift,rx_mix_scale,acclen,
                              rx_combiner_loss_db=0,
                              rx_attenuator_value_db=0,
                              rx_if_s21_db=0,
                              rx_mixer_conversion_loss_db=0,
                              rx_rf_s21_db=0,
                              rx_bypass_amp_s21_db=0,
                              cryostat_output_s21_db=0,
                              windowfactor=1,accumulated_iq_phase=0):
    """
    Estimate the final accumulated IQ value levels for a given power level at the ADC input.
    Optionally accounts for the RX analog frontend between cryostat output and ADC.
    When RX frontend parameters are zero (default), adc_input_power_dbm is used directly.
    """
    #convert to numpy array
    sig_dbm = np.atleast_1d(adc_input_power_dbm)

    # Account for RX analog frontend: cryostat output -> ADC input
    # These default to 0 so the function is backwards-compatible when called
    # with just adc_input_power_dbm representing power already at the ADC.
    sig_dbm = sig_dbm + cryostat_output_s21_db
    sig_dbm = sig_dbm + rx_bypass_amp_s21_db
    sig_dbm = sig_dbm + rx_rf_s21_db
    sig_dbm = sig_dbm - abs(rx_mixer_conversion_loss_db)
    sig_dbm = sig_dbm + rx_if_s21_db
    sig_dbm = sig_dbm - abs(rx_attenuator_value_db)
    sig_dbm = sig_dbm - abs(rx_combiner_loss_db)
    
    sig_dbfs = sig_dbm - adc_dbm_to_dbfs

    sig_rms = 10**(sig_dbfs/20)

    sig_amp = sig_rms * np.sqrt(2)

    ddc_amp = sig_amp * mixer_qmc_gain

    if not mixer_scale_is_1p0:
        ddc_amp /= np.sqrt(2)
    
    adc_amp = ddc_amp * 2**(adc_bits-1)

    adc_i_amp = adc_q_amp = adc_amp #eg phase=45 deg = pi/4 rad

    pfb_i_out = adc_i_amp * 2**(13-bin(pfb_fftshift).count('1')) * np.cos(accumulated_iq_phase) 
    pfb_q_out = adc_q_amp * 2**(13-bin(pfb_fftshift).count('1')) * np.sin(accumulated_iq_phase) 

    rx_mix_i_out = pfb_i_out * rx_mix_scale
    rx_mix_q_out = pfb_q_out * rx_mix_scale

    acc_i_out = rx_mix_i_out * acclen * windowfactor
    acc_q_out = rx_mix_q_out * acclen * windowfactor
    # print('\n'.join([str(i) for i in (acc_i_out+1j*acc_q_out,acc_i_out,acc_q_out,rx_mix_i_out,rx_mix_q_out,pfb_i_out,pfb_q_out,adc_i_amp,adc_q_amp,adc_amp,ddc_amp,sig_rms,sig_dbfs,sig_dbm)]))

    return acc_i_out+1j*acc_q_out

def calc_adc_input_power(accumulated_iq_level,adc_dbm_to_dbfs,mixer_qmc_gain,mixer_scale_is_1p0,adc_bits,pfb_fftshift,rx_mix_scale,acclen,
                         rx_combiner_loss_db=0,
                         rx_attenuator_value_db=0,
                         rx_if_s21_db=0,
                         rx_mixer_conversion_loss_db=0,
                         rx_rf_s21_db=0,
                         rx_bypass_amp_s21_db=0,
                         cryostat_output_s21_db=0,
                         windowfactor=1,
                         detailed_output=False):
    """
    Estimate tone powers from accumulated IQ values at a point in the RX chain.

    Converts accumulated IQ levels back through the digital and analog RX
    stages. When RX frontend parameters are zero (default), the result is
    power at the ADC input. When they are provided, the result is referred
    back to the cryostat output.
    """
    #convert to numpy array
    accumulated_iq_levels = np.atleast_1d(accumulated_iq_level)

    acc_i_out = np.real(accumulated_iq_levels)
    acc_q_out = np.imag(accumulated_iq_levels)
    accumulator_db = 20 * np.log10(np.abs(accumulated_iq_levels) + 1e-30)

    rx_mix_i_out = acc_i_out / (acclen * windowfactor)
    rx_mix_q_out = acc_q_out / (acclen * windowfactor)

    pfb_i_out = rx_mix_i_out / rx_mix_scale
    pfb_q_out = rx_mix_q_out / rx_mix_scale

    pfb_amp = np.abs(pfb_i_out + 1j * pfb_q_out)
    adc_amp = pfb_amp / 2**(13-bin(pfb_fftshift).count('1'))

    ddc_amp = adc_amp / 2**(adc_bits-1)

    if not mixer_scale_is_1p0:
        ddc_amp *= np.sqrt(2)

    ddc_amp /= mixer_qmc_gain

    sig_rms = ddc_amp / np.sqrt(2)

    sig_dbfs = 20*np.log10(sig_rms)

    adc_dbm = sig_dbfs + adc_dbm_to_dbfs

    # Remove RX frontend gains to refer power back to cryostat output
    rx_combiner_dbm = adc_dbm + abs(rx_combiner_loss_db)
    rx_attenuator_dbm = rx_combiner_dbm + abs(rx_attenuator_value_db)
    rx_if_dbm = rx_attenuator_dbm - rx_if_s21_db
    rx_mixer_dbm = rx_if_dbm + abs(rx_mixer_conversion_loss_db)
    rx_rf_dbm = rx_mixer_dbm - rx_rf_s21_db
    rx_amp_dbm = rx_rf_dbm - rx_bypass_amp_s21_db
    cryostat_output_dbm = rx_amp_dbm - cryostat_output_s21_db

    output_power_dbm = cryostat_output_dbm
    if detailed_output:
        details = {'accumulator_db':accumulator_db.tolist(),
                   'adc_dbfs':sig_dbfs.tolist(),
                   'adc_dbm':adc_dbm.tolist(),
                   'rx_combiner_dbm':rx_combiner_dbm.tolist(),
                   'rx_attenuator_dbm':rx_attenuator_dbm.tolist(),
                   'rx_if_dbm':rx_if_dbm.tolist(),
                   'rx_mixer_dbm':rx_mixer_dbm.tolist(),
                   'rx_rf_dbm':rx_rf_dbm.tolist(),
                   'rx_amp_dbm':rx_amp_dbm.tolist(),
                   'cryostat_output_dbm':cryostat_output_dbm.tolist(),}
        return output_power_dbm, details
    else:
        return output_power_dbm



def check_sticky_adc_overvoltage_protection_status():
    """
    Check the status of the sticky ADC overvoltage register.

    Deprecated — use firmware_lib.check_rfdc_rts_events() instead, which
    checks all RFDC RTS sticky flags (DAC and ADC overvoltage/overrange)
    via the souk_mkid_readout interface.
    """
    pass

def clear_sticky_adc_overvoltage_protection_status():
    """
    Clear the sticky ADC overvoltage register.

    Deprecated — use firmware_lib.check_rfdc_rts_events(clear=True) instead.
    """
    pass