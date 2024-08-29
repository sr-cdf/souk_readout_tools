# Calibrations

This directory is for storing S2p files for the RF system and calibration files for converting between dBm and digital units with the DAC/ADC.

## Specifying Calibrations in the Config File

Calibrations are specified in the general config file and can be given in three ways:
1. Enter a single value for the conversion to/from dBm
2. Enter a sequence of (frequency_hz, conversion_dbm) pairs to enable interpolation
3. Enter the path of a calibration file

## Calibration Files:

Calibration files should contain 2 columns:
-  Column 1: frequency in Hz
-  Column 2: power in dBm corresponding to Full Scale digital signal

Calibration files may include comments describing the conditions of the
calibration measurement.



## Definition of Full Scale Output

For RFSoC RFDC calibrations, full scale output is defined as follows:

    1. The DAC is presented with a full scale 16 bit complex signal (I.cos(wt) + j.Q.sin(wt)) where I and Q both range from -32768 to +32767. In the SOUK firmware with PSB output and single DAC mode, this can be achieved with the folling settings:
        Tone amplitude = 1.0 (maximum scaling)
        PSB FFT shift schedule = 0b000000000000
        PFBSCALE scaling = 2.0
        This was confirmed by applying these settings with a single tone and inspecting a DAC snapshot.

    2. The DAC digital upconverter (DUC) fine mixer scale is set to 0P7
    ***Setting to 1P0 gives 3dB higher power with risk of overflow in the datapath

    3. The DAC quadrature modulation correction (QMC) gain/offset correction is disabled or the gain is set to 1.0 and offset is 0.
    ***Setting the gain to the maximum (1.999) gives 6dB higher power at the expense of distortion in the analog output

    4. The DAC variable output power current is set to 20000 uA.
    ***Setting the current to the maximum 40500 uA will give ~6dB higher power with some distortion but anything other than 20000 is explicitly not supported for the RFSoC4x2 and KRM-4ZU47DR due to V_dac_vtt being connected to 2.5V.

    5. The DAC inverse sinc filters are disabled.
    *** maybe 1dB power is gained near the band edges if this enabled so shouldn't affect the calibration too badly

## Example DAC Calibration File

So, the simplest DAC calibration file can be produced as follows:

    1. Set a tone to output at 500MHz with all the settings described above so that we have a 0dBFS signal.

    2. Measure the DAC with a spectrum analyser and subtract cable losses. For example, today I measured -5.9dBm on the KRM board dac0.

    3. Write the calibration file with some comments and a single entry as follows:
        <start of file 'calibrations/dac0.txt'>
        #
        # First calibration of the CARDIFF KRM board DAC0 output
        #
        # DAC clock = 4096MHz
        # Interpolation = 2x
        # Effective sample rate: 2048 MHz
        # DUC fine mixer frequency = 1024MHz
        # Nyquist Zone = 1
        # DAC Mode = C2R
        #
        # Frequency_Hz Power_dBm
        500e6 -5.9
        <end of file>

    4. Update the relevant lines in the config yaml file with the location of the calibration file:
        <start of file 'config/test_config.yaml'>
        ...
        firmware:
            dac_fullscale_bits: 16
            adc_fullscale_bits: 16
            dac_physical_bits: 14
            adc_physical_bits: 14
            dac0_dbfs_to_dbm: 'calibrations/dac0.txt'
            dac1_dbfs_to_dbm: -6.0
            adc_dbm_to_dbfs: +12
            vop_current_fullscale: 20000
        ...
        <end of file>

Copy the calibration file and config file to the RFSoC and restart the readout server service:
    [user@client]$ scp -r calibrations config casper@<rfsoc_host>:src/souk_readout_tools/
    [user@client]$ ssh casper@<rfsoc_host>
    [casper@rfsoc]$ sudo systemctl restart readout_server
    [sudo] password for casper:
    [casper@rfsoc]$ logout
    Connection to 10.11.11.11 closed.

