# Note on using dual-pipeline firmware

The dual-pipeline firmware runs **two identical readout pipelines** (pipeline **0** and pipeline **1**) on a single RFSoC board. In practice this allows you to read out **two independent RF networks** simultaneously from one device.

At present, to use dual-pipeline firmware with `souk_readout_tools`, you run **two separate instances of the readout server** on the RFSoC ARM core—one server instance per pipeline—and connect to each one with a client configured for that server.

---

## 1. Required setup: two server instances, two config files

### 1.1 Pipeline selection (`pipeline_id`)
In the `firmware:` section of your `souk_readout_tools` config files, set:

- `pipeline_id: 0` for the server that controls pipeline 0
- `pipeline_id: 1` for the server that controls pipeline 1

Each server instance must use a config file with the correct `pipeline_id`.

### 1.2 Ports must not clash
Each server instance provides:
- a **request/control** TCP port (`request_port`)
- a **stream** TCP port (`stream_port`)

These **must be different** between the two configs to avoid conflicts.

### 1.3 Confirm the firmware actually supports dual pipelines
Dual-pipeline firmware is typically indicated by `dual-pipeline` in the firmware filename or yaml.

You can confirm at runtime by reading the firmware type reported by the FPGA:
- `SoukMkidReadout.fpga.get_firmware_type()` returns:
  - `2` for single-pipeline firmware
  - `3` for dual-pipeline firmware

You can also check the output of `client.get_system_information()` and confirm it reports the expected `fw_type` in the `fpga_status` metadata.

### 1.4 RFDC tile/block mapping differs per pipeline
Each pipeline uses different DAC/ADC channels. Your two config files must specify the correct RFDC {tile/block} values. The correct mapping for v7.9 firmwares and later is:

- **Pipeline 0:** DACs {0/0} and {0/2}, ADC channel {2/0}  
- **Pipeline 1:** DACs {1/0} and {1/2}, ADC channel {3/0}  


---

## 2. Example config files

Below are representative examples showing only the important differences.

### `config_pipeline_0.yaml`
```yaml
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
  adc_tile: 2
  adc_block: 0
  ...
```

### `config_pipeline_1.yaml`
```yaml
rfsoc_host:
  request_port: 10001
  stream_port: 20001
  ...

firmware:
  fw_config_file: "/home/casper/src/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml"
  pipeline_id: 1
  dac0_tile: 1
  dac0_block: 0
  dac1_tile: 1
  dac1_block: 2
  adc_tile: 3
  adc_block: 0
  ...
```

---

## 3. Deploy configs to the RFSoC

Copy both config files to the RFSoC. Note that each pipeline now uses its own subdirectory:

```bash
# Pipeline 0 configs go in pipeline_0 subdirectory
scp config_pipeline_0.yaml casper@rfsoc:~/.souk_readout_tools/pipeline_0/config/

# Pipeline 1 configs go in pipeline_1 subdirectory  
scp config_pipeline_1.yaml casper@rfsoc:~/.souk_readout_tools/pipeline_1/config/
```

The new directory structure for dual-pipeline operation is:
```
~/.souk_readout_tools/
├── pipeline_0/
│   ├── config/
│   │   ├── default_config.lnk
│   │   └── config_pipeline_0.yaml
│   ├── calibrations/
│   └── tmp/
└── pipeline_1/
    ├── config/
    │   ├── default_config.lnk
    │   └── config_pipeline_1.yaml
    ├── calibrations/
    └── tmp/
```

This ensures each pipeline instance has completely separate runtime data, avoiding race conditions or file clobbering.

---

## 4. Start two servers (one per pipeline)

### Recommended: start via the server CLI
You can start each server instance with a different config file. The `-p` / `--pipeline` flag can explicitly specify the pipeline ID:

#### Terminal 1 (pipeline 0)
```bash
ssh casper@rfsoc
sudo /home/casper/py38venv/bin/souk-readout-server -p 0 ~/.souk_readout_tools/pipeline_0/config/config_pipeline_0.yaml
```

#### Terminal 2 (pipeline 1)
```bash
ssh casper@rfsoc
sudo /home/casper/py38venv/bin/souk-readout-server -p 1 ~/.souk_readout_tools/pipeline_1/config/config_pipeline_1.yaml
```

Note: If you omit `-p`, the server will extract `pipeline_id` from the config file automatically.

### Alternative: start from Python
If you prefer launching manually from a Python session:

```bash
ssh casper@rfsoc
sudo /home/casper/py38venv/bin/python
```

```python
import asyncio
from souk_readout_tools.server.readout_server import ReadoutServer

# Explicitly specify pipeline_id for clarity
server = ReadoutServer(config_file="/home/casper/.souk_readout_tools/pipeline_0/config/config_pipeline_0.yaml", pipeline_id=0)
asyncio.run(server.async_main())
```

Repeat in a second terminal for pipeline 1 with the pipeline 1 config.

---

## 5. Connect with two clients

On your local machine (or wherever you run the clients), each client also uses pipeline-specific directories:

```python
from souk_readout_tools.client.readout_client import ReadoutClient

# Each client uses its own pipeline-specific directories:
#   ~/.souk_readout_tools/pipeline_0/  for client0
#   ~/.souk_readout_tools/pipeline_1/  for client1

# Method 1: Specify pipeline_id explicitly (recommended for clarity)
client0 = ReadoutClient(config_file="config_pipeline_0.yaml", pipeline_id=0)
client1 = ReadoutClient(config_file="config_pipeline_1.yaml", pipeline_id=1)

# Method 2: Let the client extract pipeline_id from the config file
client0 = ReadoutClient(config_file="config_pipeline_0.yaml")
client1 = ReadoutClient(config_file="config_pipeline_1.yaml")

info0 = client0.get_system_information()
info1 = client1.get_system_information()

print(info0["pipeline_id"])  # expected: 0
print(info1["pipeline_id"])  # expected: 1
```

---

# Updated initialisation procedure for dual-pipeline firmware

With **single-pipeline** firmware, it was often acceptable for one client/server instance to “initialise the firmware” whenever needed, because there was only one pipeline and firmware state changes only affected that one system.

With **dual-pipeline** firmware there are two independent pipelines sharing some common FPGA blocks. That means operations that **reprogram the FPGA** or **initialise shared blocks** can affect *both* pipelines (and therefore the other server instance).

To make this explicit, initialisation is now treated as a **3-stage process**:

1. **Programming** (bitfile load)  
   This wipes FPGA state and therefore invalidates *both pipelines*.  
   The RFDC block is also reset, so ADC/DAC settings are lost.

2. **Shared resource initialisation** (common blocks)  
   This initialises blocks that are shared across pipelines (e.g. snapshots, LUT generator, autocorrelator, etc).  
   If shared resources are reset/reinitialised, both pipelines may need pipeline reinitialisation afterwards.

3. **Pipeline resource initialisation** (per-pipeline blocks)  
   This initialises resources that belong to a specific pipeline (e.g. accumulator settings, mixer frequencies, channel maps).  
   This can be done independently for pipeline 0 and pipeline 1.

These stages are implemented in `souk_readout_tools.firmware_lib` using:

- `needs_programming()`
- `needs_shared_resource_initialising()`
- `needs_pipeline_initialising()`

and the corresponding actions:

- `reload_firmware()`
- `initialise_shared_resources()`
- `initialise_pipeline_resources()`

A new client command, `ensure_ready(level=...)`, has been added to manage this process.

---

## Updated client commands

### `client.ensure_ready(level="pipeline")` (recommended)
Use this for normal operation and automation.

The server will move the system forward *only as far as necessary*:

- It will **only reprogram** the FPGA if `needs_programming()` indicates it is required.
- It will **only initialise shared resources** if `needs_shared_resource_initialising()` indicates it is required.
- It will **only initialise pipeline resources** if `needs_pipeline_initialising()` indicates it is required.

This is the safest way to bring a pipeline into a usable state without unnecessarily disturbing the other pipeline.

Common usage:
```python
client0.ensure_ready(level="pipeline")
client1.ensure_ready(level="pipeline")
```

### `client.hard_reset()` (force reprogram + reinit)
Use this when you explicitly want to reset the FPGA state.

- The server will **reprogram** the FPGA (wiping state for **both pipelines**).
- The server will then initialise shared resources and the pipeline configured for that server instance.

After a `hard_reset()` on one pipeline’s server, the other pipeline’s server must run:
```python
client_other.ensure_ready(level="pipeline")
```
before it can reliably stream/read data again.

---

## Practical guidance when running two servers

Assume:
- server A controls pipeline 0
- server B controls pipeline 1

Then:

- Calling `ensure_ready()` on server A should not disrupt server B **unless** a firmware reprogram or shared init is actually required.
- Calling `hard_reset()` on either server **will wipe both pipelines**, so plan on running `ensure_ready(level="pipeline")` on both servers afterwards.

---

## After pushing a new config

When you push/apply a config to one server instance:

- That server will apply the config and bring *its* pipeline back to a ready state.
- If the change causes a reprogram or shared reinitialisation, the other pipeline will also need to call `ensure_ready()` afterwards.

In practice:
- If you only changed a pipeline-local setting (e.g. accumulator length, PSB scale), only that pipeline should need reinitialising.
- If you changed firmware-level settings (bitfile / fw config file / anything that triggers reprogram or shared init), expect to reinitialise both pipelines.

---

## TBD / future improvements

- Provide a supported systemd setup that runs **two server services** (one per pipeline) with clear unit names.
- Provide helper scripts to:
  - start/stop/restart both servers together,
  - check both server statuses,
  - run `ensure_ready()` on both pipelines in a single command ?