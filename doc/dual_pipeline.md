# Note on using dual pipeline firmware

The dual pipeline firmware is designed to run two identical instances ("pipelines") of the SOUK readout system in parallel on a single RFSoC device. 

This allows the readout of two RF networks suimultaineously with one RFSoC board.

At present, to use the dual pipeline firmware with `souk_readout_tools`, you will need to configure and run two separate instances of the SOUK readout server on the RFSoC ARM core, each configured to communicate with one of the pipelines. This involves creating two seperate config files, one for each pipeline, starting two instances of the `readout_server` and creating two clients to connect to each respective server instance.

In the `firmware` section of the souk_readout_tools config file there is a parameter called `pipeline_id`. This should be to set to either `0` or `1` to specify which pipeline the server instance should use.

It is important to ensure that each server instance uses different TCP/IP ports for streaming and control to avoid conflicts. You can specify different port numbers in the config files for each instance.

It is also immportant to check the firmware bitfile or yaml file specified in the config actually supports dual pipelines. The dual pipeline firmware versions are usually indicated by having `dual-pipeline` in the file name. This can be confirmed after programming by reading the `fw_type` register via `souk_mkid_readout`, (`SoukMkidReadout.fpga.get_firmware_type()`) which should return `2` for single pipeline and `3` for dual pipeline firmwares.

Finally, each pipeline uses its own DAC and ADC channels, so the config files should also specify the correct tile and block numbers for each pipeline. Pipeline 0 uses DAC channels 0 and 2, and ADC channel 4, while pipeline 1 uses DAC channels 1 and 3, and ADC channel 4.


The key differences between the two config files would be as follows:

`config_pipeline_0.yaml` :
```
rfsoc_host:
  request_port: 10000
  stream_port: 20000
  ...
firmware: 
  fw_config_file: "/home/casper/src/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml"
  pipeline_id: 0
  dac0_tile: 0
  dac0_block: 0
  dac1_tile: 0
  dac1_block: 2
  adc_tile: 0
  adc_block: 4
  ...
```

and 

`config_pipeline_1.yaml` :
```
rfsoc_host:
  request_port: 10001
  stream_port: 20001
  ...
firmware: 
  fw_config_file: "/home/casper/src/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml"
  pipeline_id: 1
  dac0_tile: 0
  dac0_block: 1
  dac1_tile: 0
  dac1_block: 3
  adc_tile: 0
  adc_block: 5
  ...
```

You should copy these to the RFSoC device, for example:
```bash
scp config_pipeline_0.yaml casper@rfsoc:~/.souk_readout_tools/config/
scp config_pipeline_1.yaml casper@rfsoc:~/.souk_readout_tools/config/
```

Then you would start two instances of the readout server on the RFSoC, each with its own config file:

At the moment this needs to be done in two terminals.

In the first terminal, start the first instance, specifying `config_pipeline_0.yaml`:
```bash
ssh casper@rfsoc
sudo /home/casper/py38venv/bin/python
>>> import souk_readout_tools, asyncio
>>> server = souk_readout_tools.server.readout_server.ReadoutServer(config_file='/home/casper/.souk_readout_tools/config/config_pipeline_0.yaml')
>>> asyncio.run(server.async_main())

```
In a separate terminal, start the second instance, specifying `config_pipeline_1.yaml`:
```bash
ssh casper@rfsoc
sudo /home/casper/py38venv/bin/python
>>> import souk_readout_tools, asyncio
>>> server = souk_readout_tools.server.readout_server.ReadoutServer(config_file='/home/casper/.souk_readout_tools/config/config_pipeline_1.yaml')
>>> asyncio.run(server.async_main())
```

Each server instance will now communicate with its respective pipeline on the RFSoC device.

Then you can start two separate clients on your local machine, each connecting to the appropriate server instance using the corresponding config file.

This can be done in a python session on your local machine, for example:
```
import souk_readout_tools
client0=souk_readout_tools.client.ReadoutClient(config_file='config_pipeline_p0.yaml')
client1=souk_readout_tools.client.ReadoutClient(config_file='config_pipeline_1.yaml')
info0 = client1.get_system_info()
info1 = client1.get_system_info()
print(info0['pipeline_id'])  # should print 0
print(info1['pipeline_id'])  # should print 1
```

Note that any command issued to program the firmware from one client will require re-initialisation of both pipelines, so both clients will need to re-initialise their respective pipelines after a firmware reload.