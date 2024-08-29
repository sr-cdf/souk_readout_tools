# SOUK Readout Tools

This repo hosts the tools required to operate the SOUK MKID Readout system.

These are early days, so please report any issues or feature requests, no matter how small.

Also, take a look in the [doc](doc) directory for more information. Especially the [getting started](doc/getting_started.md) guide.

## Readout Server

The readout server software runs on the RFSoC boards and uses the lower level [souk_mkid_readout](https://github.com/realtimeradio/souk-firmware) library to talk to the FPGA firmware. It listens for requests from external clients to perform high level actions such as sweeping, tuning, streaming and general housekeeping, returning raw data to the client for further processing.

To start the server process manually, run ther following on the board: `[casper@rfsoc]$ sudo /home/casper/py38venv/bin/python3 readout_server.py`

A systemd service file is available to enable automatic starting of the server process on power up, and automatic restarting if it crashes. See the note below for instructions on setting this up.

## Readout Client

The readout client software is for general users to run on their laptops or desktops. Commands are sent to the server to set up and take measurements. The raw data that gets returned can be parsed, analysed and exported.

See the [getting started](doc/getting_started.md) info in [doc](doc) for more details.

Once the system is set up and an appropriate config file is available, it should be simple to connect to the board and start taking measurements.

Useful scripts for common tasks can be found in [client_scripts](client_scripts) or the client can be started in an interactive python shell, for example:

```
    (client_venv)[user@client-machine]$ ipython

    In [1]: import readout_client, numpy as np, matplotlib.pyplot as plt

    In [2]: client = readout_client.ReadoutClient(config_file='config/config.yaml')

    In [3]: client.get_server_status()
    Out[3]:
    {'process_name': 'readout_daemon',
     'ip_addresses': '10.11.11.11 192.168.2.224',
     'pwd': '/home/casper/src/readout_server',
    ....}

    In [4]: client.get_sample_rate()
    Out[4]: 500.0

    In [5]: num_tones = len(client.get_tone_frequencies())

    In [6]: raw_samples = client.get_samples(500)
    Received 500 samples in ~0.9946386814117432 seconds (~502.6951086301245 samples per second)

    In [7]: data = readout_client.ReadoutClient.parse_samples(raw_samples,num_tones)

    In [8]: print( np.all(np.diff(data['packet_counter']) == 1) )
    True

    In [9]: t = np.arange(len(data['packet_counter'])) / client.get_sample_rate()

    In [10]: z0 = data['i_data']['0000'] + 1j*data['q_data']['0000']

    In [11]: plt.plot(t, np.abs(z0))
    Out[11]: [<matplotlib.lines.Line2D at 0x7f5fda517580>]

    In [12]: plt.show()
```

## Config File

A config file is required to operate the system. It is read by both the client and the server. It contains all the configuration information necessary to operate the system. An example config file is provided in [config](config). By default, the client and server will look for a file called `config.yaml` in the `config` directory. This can be a symlink to the actual config file if desired.

The main section are:

`rfsoc_host:` for the IP address of the RFSoC board, communication ports and other board specific information

`firmware:` for default firmware parameter settings and DAC/ADC paramters and calibrations

`rf_frontend:` for tone frequency and power calibrations relating to analog up/down converters

`cryostat:` for cryo channel RF calibrations, optical setup information and temperature logs

`detector:` for detector specific information

## Client Scripts

The client scripts are a collection of useful scripts for common tasks. They are written in python and can be run from the command line or imported into an interactive python shell.

Examples include: TBD

## Documentation

The [doc](doc) directory contains documentation for the project. This includes getting started guides, API documentation, example ipython notebooks and other useful information.

## Calibrations

The [calibrations](calibrations) directory contains calibration data for the system. This includes calibration files and S2P files for tone frequency and power calibrations for the RF-DAC, RF-ADC, analog front-end, cryo channel and detectors. The choices of calibration files are set in the config file.

## Misc:

### Readout server service

To install the systemd service on the RFSoC ARM processor, follow these steps:

Copy the service file to the systemd service directory with `sudo cp readout_server.py /etc/systemd/system/`

Reload the systemd manager configuration with `sudo systemctl daemon-reload`

Start the service with `sudo systemctl start readout_server`

Enable the service to start on boot with `sudo systemctl enable readout_server`

Verify the service is running with `sudo systemctl status readout_server`

You can also stop the service with `sudo systemctl stop readout_server` and disable it with `sudo systemctl disable readout_server`.

Remember to restart the server after any software updates with `sudo systemctl restart readout_server`

For more information on systemd services, you can refer to the official documentation: https://www.freedesktop.org/software/system

Standard output and error are redirected to the system logand the identifier for the server process is 'readout_server'. Inspect the log with `sudo journalctl -xu readout_server`.

Monitor the system log in real time with `sudo journalctl -f -xu readout_server`.

The server process name is set to `readout_daemon`, unless it is started manually, in which case it is set to `readout_server` or read from the environment variable READOUT_SERVER_NAME.



[def]: URL