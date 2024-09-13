# Getting Started with this Project

To get started with this project, follow the steps below:

This has been tested on linux with bash and python-3.10 and on Windows with PowerShell and python-3.12. 

## 1. Set up the network

Ensure the RFSoC (eg. 10.11.11.11/24) is connected and on the same subnet. For example, manually set the IP address of the client machine to 10.11.11.1/24.

It should be possible to see the RFSoC from the client machine with a ping:

`ping 10.11.11.11`

SSH should also be possible, for example:

`ssh casper@10.11.11.11`

Optionally, you can set the DNS hostname in the client machine's `/etc/hosts` or `C:\Windows\System32\drivers\etc\hosts` file with the line:

`10.11.11.11 rfsoc`

Then you can use the hostname `rfsoc` instead of the IP address in the config file.

## 2. Prepare the environment on the client machine:

Ensure python3 and pip are installed.

Clone the project repository from GitHub: 

`git clone https://github.com/sr-cdf/souk_readout_tools`

Navigate to the project directory: 
`cd souk_readout_tools`

Create a virtual python environment: 

Linux bash: `python3 -m venv client_venv`

Windows PowerShell: `python -m venv client_venv`

Activate the python environment:

Linux bash: `source ./client_venv/bin/activate`

Windows PowerShell: `.\client_venv\Scripts\Activate.ps1`

With PowerShell you may need to set the execution policy to allow running scripts: 

`Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`

Install the required packages:

`pip install -r requirements.txt`


## 3. Prepare a configuration file:

A basic configuration file with the minimum required parameters is provided in [config/template_config.yaml](../config/template_config.yaml). This should be used as a template to create a new configuration file with the correct parameters for your setup.

Copy the default configuration file to a new file:

`cp config/default_config.yaml config/my_config.yaml`

Edit the new configuration file with the correct parameters for your setup. 

Important parameters are the RFSoC IP address, the analog frontend configuration, cryostat configuration, and detector information.

The file [config/config.yaml](../config/config.yaml) is a symbolic link and can be used to point to the latest configuration file. In Windows, the symbolic link may simply contain the path to the configuration file.

## 4. Start the client:

Start an interactive python session in the activated virtual environment and create the client interface:

If no config file is provided, the default `config.yaml` symlink will be followed.

```
(client_venv) [user@client souk_readout_tools]$ ipython
Python 3.10.11 (main, Apr  5 2023, 00:00:00) [GCC 12.2.1 20221121 (Red Hat 12.2.1-4)]
Type 'copyright', 'credits' or 'license' for more information
IPython 8.26.0 -- An enhanced Interactive Python. Type '?' for help.

In [1]: import readout_client

In [2]: client = readout_client.ReadoutClient(config_file='config/my_config.yaml')

```

## 4. Copy the new configuration to the RFSoC:

New configuration files should be copied to the RFSoC with the method:

```
In [3]: client.set_config('config/my_config.yaml')

```

The server will save the config file to the RFSoC and restart itself to apply the new configuration, which takes a few seconds.

To set a config file as the default config on the RFSoC that will be preserved after power cycling, use the method:

```
In [5]: client.set_config_default('config/my_config.yaml')

```

TBD: make this implicit.

## 4. Requesting Information

The current status of the readout server on the RFSoC can be inspected with the `get_server_status` method:
```
In [6]: client.get_server_status()
Out[6]:
```

A summary of the current firmware configuration can be obtained with the `get_system_information` method:
```
In [7]: client.get_system_information()
Out[7]:
```


## 5. Setting Readout Tones:

Methods are available to set the readout tone frequencies, powers and phase offsets.

Use `set_tone_frequencies` to set the desired readout tone frequencies. The system will configure the necessary firmware setting to achieve the target frequencies, accounting for the RF-DAC and any analog converter configurations specified in the config file.

For example, to probe two resonators at 2.100 and 3.200 GHz, use:

```
In [8]: client.set_tone_frequencies([2.100e9, 3.200e9])
```

Check the current tone frequencies with the `get_tone_frequencies` method:

```
In [9]: client.get_tone_frequencies()
Out[9]:
```

To get the details of the firmware mixer frequencies, the DAC output frequencies after the digital upconversion, and the analog frequencies after the analog upconversion, use the `get_tone_frequencies` method with `detailed_output=True`:

```
In [10]: client.get_tone_frequencies(detailed_output=True)
Out[10]:
``` 

There are two methods to set the tone powers: `set_tone_amplitudes` and `set_tone_powers`. The former takes the power in linear units referring to the LO-scale parameter in the firmware. The values can be in the range [0, 1.0] but other factors that control the output power such as the PSB scale factor and FFT shift schedule are not accounted for. The latter takes the power in dBm and applies settings that do account for the current PSB scale and FFT shift schedule settings, as well as other parameters and calibration values for the DAC, the RF frontend and the cryostat, if they are specified in the config file.

For example, to set the power of the first tone to -20 dBm and the second tone to -25 dBm, use:

```
In [11]: client.set_tone_powers([-20, -25])
```

And to get the detailed output of the tone powers through the system:

```
In [12]: client.get_tone_powers(detailed_output=True)
```

The phase offsets of the tones can be set with the `set_tone_phases` method. The phases are in radians and are applied at the firmware mixer LO stage. Adjustments for later conversion stages are not currently accounted for, although they ought to be. Crest factor of the output waveform can be minimised by setting the phases to random values but the preferred method is to use Newman's quadratic multisine phase offset method as this avoids random variations in peak power when re-setting tones. Use `generate_random_phases(num_tones)` or `generate_newman_phases(num_tones)` to generate the phase offsets and apply them:

```
In [13]: num_tones = len(client.get_tone_frequencies())
In [14]: phases = client.generate_newman_phases(num_tones)
In [15]: client.set_tone_phases(phases)
In [16]: client.get_tone_phases()
```

## 6. Getting Data:

To collect a fixed number of continuous samples, use the `get_samples` method. This method will return a sequence of raw samples from the RFSoC.

The raw samples should be parsed into IQ and meta data using the `parse_samples` method. This method will return a dictionary that can be used to access the IQ data and the meta data.

For example, to collect one second of data and to plot the magnitude of IQ for for the first tone:

```
In [17]: import numpy as np, matplotlib.pyplot as plt

In [18]: client.get_sample_rate()
Out[18]: 500.0

In [19]: num_tones = len(client.get_tone_frequencies())

In [20]: raw_samples = client.get_samples(500)
Received 500 samples in ~0.9946386814117432 seconds (~502.6951086301245 samples per second)

In [21]: data = client.parse_samples(raw_samples,num_tones)

In [22]: print( np.all(np.diff(data['packet_counter']) == 1) )
True

In [23]: t = np.arange(len(data['packet_counter'])) / client.get_sample_rate()

In [24]: z0 = data['i_data']['0000'] + 1j*data['q_data']['0000']

In [25]: plt.plot(t, np.abs(z0))
Out[25]: [<matplotlib.lines.Line2D at 0x7f5fda517580>]

In [25]: plt.show()

```

The data dictionary can be exported to a file with the `export_samples` method in npy, json or csv format. Placeholders exist for hdf5 and other formats. For example:

```
In [26]: client.export_samples('test_samples.csv',data)
```

Exported data can also be imported with the `import_samples` method for comparing different data sets. For example:

```
In [27]: data2 = client.import_samples('test_samples.csv')
In [28]: np.all(data['i_data']['0000'] == data2['i_data']['0000'])
Out[28]: True
```


## 8. Streaming Data:

If continuous streams of arbitrary length are required, the server can be instructed to start streaming, then a seperate client script can be started to record the stream to disk. The server stream or the client logger can be stopped at any time. For example:

With a client, call the `enable_stream` method:

```
In [29]: client.enable_stream()
```
Then, in a separate terminal, start the logger script, specifying the number of tones to save (saving all 2048 tones can waste disk space when only a handful are being used):

Linux bash: 

```
(client_venv)[user@client souk_readout_tools]$ python3 client_scripts/receive_stream.py -p -n 2
```

Or windows powershell: 
```
(client_venv) PS C:\Users\user\souk_readout_tools> python client_scripts\receive_stream.py -p -n 2
```


The `-p` flag will print the latest sample to the screen in the terminal. The `-n` flag specifies the number of tones to save to disk. 

By default the script will save raw data into `./tmp/tmp_stream`, overwriting any existing data there, but a directory and filename can be specified with `-d` and `-f` flags.

A format file will also be saved in the same directory as the data file, containing the information required to parse the raw dumped stream.

To stop the stream, call the `disable_stream` method in the client:

```
In [30]: client.disable_stream()
```

The receiver script can be stopped with `Ctrl-C`, or it can be left running, ready to continue appending to the file when the stream is re-enabled.

Raw data stream files can be parsed with the `parse_stream` method in the client:

```
In [29]: data = client.parse_stream('tmp/tmp_stream')
In [30]: t = np.arange(len(data['packet_counter'])) / data['sample_rate']
In [31]: z0 = data['i_data']['0000'] + 1j*data['q_data']['0000']
In [32]: plt.plot(t, np.abs(z0))
In [33]: plt.show()

```

## 9. Triggered Streaming:

The server can be configured to send a sample when a trigger pulse is received on a GPIO input. The process is identical to regular streaming except that we have to call `enable_triggered_stream` instead of `enable_stream`:

```
client.enable_triggered_stream()
```

Note that the packet counter increments by one for every new sample, regardless of whether the sample was sent out on a trigger or not. The trigger signal is not recorded in the data stream, but the time of the trigger can be inferred from the packet counter and the sample rate.

## 10. Frequency sweeping:

To perform a frequency sweep of readout tones, use `perform_sweep` and pass the center frequencies, the spans for each tone (can be a single span that applies to all tones), the number of points to sweep, the number of samples to integrate per point, and the sweep direction (`'up'` or `'down'`). 

The sweep data can then be parsed, plotted, exported and/or imported as before.

The server will continue to respond to commands while the sweep is being performed, to check if the sweep is finished and there is data ready to receive, check the 'latest_sweep_data_valid' key in the server status dictionary.

```
In [34]: center_frequencies = [2.100e9, 3.200e9]
In [35]: spans = [0.2e6, 0.2e6]
In [36]: num_points = 101
In [37]: samples_per_point = 100
In [38]: client.perform_sweep(center_frequencies, spans, num_points, samples_per_point, 'up')
In [39]: import time
In [40]: while client.get_server_status()['latest_sweep_data_valid'] == False:
    ...:     time.sleep(1)
    ...: 
In [41]: raw_sweep = client.get_sweep_data()
In [42]: data = client.parse_sweep(raw_sweep)
In [43]: for k in range(data['num_tones']:
    ...:     l, = plt.plot(data['sweep_f'][k]/1e6, 20*np.log10(abs(data['sweep_i'][k]+1j*data['sweep_q'][k])),label='Tone {k} sweep')
    ...:     plt.axvline(centers[k]/1e6, color=l.get_color(), linestyle='--',label=f'Tone {k} center')
    ...: plt.legend()
    ...: plt.show()
    ...:
Out[43]:
```

At the end of the sweep, the tones are returned to the center frequencies.

## 11. Retuning:

Retuning is identical to sweeping except the sweep data is inspected, the resonant frequencies are found and the tones are placed at the resonant frequencies, instead of being return to the center of the sweeps. 

The method for finding the resonance frequencies is is either `'min_mag'` or `'max_gradient'`. 

The former finds the frequency with the minimum magnitude of the IQ data, the latter finds the frequency with the maximum gradient of the IQ data. 

The method can be specified for each tone with a list of strings, or a single string that applies to all tones. 

For example:

```
In [44]: client.perform_retune(center_frequencies, spans, num_points, samples_per_point, 'up', method='max_gradient')
In [45]: while client.get_server_status()['latest_sweep_data_valid'] == False:
    ...:     time.sleep(1)
    ...:
In [46]: new_tones = client.get_tone_frequencies()
In [47]: raw_sweep = client.get_sweep_data()
In [48]: data = client.parse_sweep(raw_sweep)
In [49]: for k in range(data['num_tones']:
    ...:     plt.plot(data['sweep_f'][k]/1e6, 20*np.log10(abs(data['sweep_i'][k]+1j*data['sweep_q'][k])),label=f'Tone {k} sweep')
    ...:     plt.axvline(new_tones[k]/1e6, color=l.get_color(), linestyle='--',label=f'Tone {k} center')
    ...: plt.legend()
    ...: plt.show()
    ...:
Out[43]:
```

TBD: A smoothing factor can be applied per tone. 

TBD: 

## 12. Resonator tracking:



## 13. Calibration:

