

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
    tx_if_dbm = combiner_dbm + tx_if_s21_db - abs(tx_attenuator_value_db)
    tx_mixer_dbm = tx_if_dbm - abs(tx_mixer_conversion_loss_db)
    tx_rf_dbm = tx_mixer_dbm + tx_rf_s21_db
    cryostat_dbm = tx_rf_dbm + cryostat_input_s21_db
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
                   'tx_if_dbm':tx_if_dbm.tolist(),
                   'tx_mixer_dbm':tx_mixer_dbm.tolist(),
                   'tx_rf_dbm':tx_rf_dbm.tolist(),
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
    tx_rf_dbm = cryostat_dbm - cryostat_input_s21_db
    #account for rf frontend s21
    tx_mixer_dbm = tx_rf_dbm - tx_rf_s21_db
    tx_if_dbm = tx_mixer_dbm + abs(tx_mixer_conversion_loss_db)
    combiner_dbm = tx_if_dbm - tx_if_s21_db + abs(tx_attenuator_value_db)
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
                   'tx_rf_dbm':tx_rf_dbm.tolist(),
                   'tx_mixer_dbm':tx_mixer_dbm.tolist(),
                   'tx_if_dbm':tx_if_dbm.tolist(),
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
    
def calc_accumulated_iq_level(adc_input_power_dbm,adc_dbm_to_dbfs,mixer_qmc_gain,mixer_scale_is_1p0,adc_bits,pfb_fftshift,rx_mix_scale,acclen,windowfactor=1,accumulated_iq_phase=0):
    """
    Estimate the final accumulated IQ value levels for a given power level at the ADC input.
    """
    #convert to numpy array
    sig_dbm = np.atleast_1d(adc_input_power_dbm)
    
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

def calc_adc_input_power(accumulated_iq_level,adc_dbm_to_dbfs,mixer_qmc_gain,mixer_scale_is_1p0,adc_bits,pfb_fftshift,rx_mix_scale,acclen,windowfactor=1):
    """
    Estimate the tone powers incident at the ADC from accumulated IQ values.
    """
    #convert to numpy array
    accumulated_iq_levels = np.atleast_1d(accumulated_iq_level)
    
    acc_i_out = np.real(accumulated_iq_levels)
    acc_q_out = np.imag(accumulated_iq_levels)

    rx_mix_i_out = acc_i_out / (acclen * windowfactor)
    rx_mix_q_out = acc_q_out / (acclen * windowfactor)

    pfb_i_out = rx_mix_i_out / rx_mix_scale
    pfb_q_out = rx_mix_q_out / rx_mix_scale

    adc_i_amp = pfb_i_out / 2**(13-bin(pfb_fftshift).count('1')) / np.cos(-np.angle(accumulated_iq_levels))
    adc_q_amp = pfb_q_out / 2**(13-bin(pfb_fftshift).count('1')) / np.sin(-np.angle(accumulated_iq_levels))

    adc_amp = abs(adc_i_amp +1j*adc_q_amp) / np.sqrt(2) # amp_i = amp_q = amp

    ddc_amp = adc_amp / 2**(adc_bits-1) 

    if not mixer_scale_is_1p0:
        ddc_amp *= np.sqrt(2)
    
    ddc_amp /= mixer_qmc_gain

    sig_rms = ddc_amp / np.sqrt(2)

    sig_dbfs = 20*np.log10(sig_rms)
    
    sig_dbm = sig_dbfs + adc_dbm_to_dbfs

    # print('\n'.join([str(i) for i in (accumulated_iq_levels,acc_i_out,acc_q_out,rx_mix_i_out,rx_mix_q_out,pfb_i_out,pfb_q_out,adc_i_amp,adc_q_amp,adc_amp,ddc_amp,sig_rms,sig_dbfs,sig_dbm)]))

    return sig_dbm



def check_sticky_adc_overvoltage_protection_status():
    """
    Check the status of the sticky ADC overvoltage register.
    
    There is no access to the register so we have to see of the measure signal compares with the expected signal

    """
    pass

def clear_sticky_adc_overvoltage_protection_status():
    """
    Clear the sticky ADC overvoltage register.
    
    There is no access to the register so we have to see of the measure signal compares with the expected signal

    """
    pass