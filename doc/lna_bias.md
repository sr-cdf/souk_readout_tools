# LNA Bias Control

The readout tools provide control and monitoring of up to 14 cryogenic LNA
bias channels per cryostat, via the
[souk-peripherals-control](https://github.com/WeiDaZhang/souk-peripherals-control)
library (shared with the RF peripherals module).

## What it does

Each cryostat has 14 LNA channels (reference designators M17-M30), addressed
by index 1-14.  Each readout pipeline maps to one LNA channel via the config
key `cryostat.lna_bias.lna_channel`.  The controller supports:

- **Setting bias voltage** — by remote (iterative feedback, default) or local
  (direct DAC) method.
- **Reading bias status** — remote voltage, local voltage, and bias current
  for any channel.
- **Bulk operations** — set or read all 14 channels at once.

The hardware interface uses a two-level TCA9548 I2C switch tree.  Each channel
has a digitally programmable resistor (AD5110) for voltage setting and two ADCs
(LTC2481) for voltage and current readback.

## How it fits together

```
souk_readout_tools/server/
├── lna_controller.py                  ← LNABiasController wrapper
└── souk-peripherals-control/          ← git submodule
    ├── souk_lna_bias_control_monitor.py  ← hardware driver
    ├── lna_monitor.py                    ← per-channel monitor
    ├── ad511_0_2_4bcpz_5_10_80.py        ← digital pot driver
    └── ltc2481cdd.py                     ← ADC driver
```

`lna_controller.py` wraps the submodule's `SOUKLNABiasControlMonitor` class
and exposes it through the readout server's request/response interface.

## Configuration

LNA bias is configured in the `cryostat` section of the readout config YAML:

```yaml
cryostat:
  connected: true
  lna_bias:
    enabled: true
    i2c_bus: 0                # I2C bus number on the RFSoC
    lna_channel: 1            # LNA index (1-14) for this readout pipeline
```

Setting `enabled: false` (the default) disables all LNA hardware access for
this pipeline.

## Client API

All LNA methods are available on the `ReadoutClient` instance:

### Read status

```python
from souk_readout_tools.client.readout_client import ReadoutClient

rc = ReadoutClient('my_config.yaml')

# Read this pipeline's default LNA channel
status = rc.get_lna_bias_status()
# {'channel': 1, 'remote_voltage_v': 0.65, 'local_voltage_v': 0.64, 'bias_current_a': 0.0012}

# Read a specific channel
status = rc.get_lna_bias_status(channel=7)

# Read all 14 channels
all_status = rc.get_lna_bias_status_all()
# {1: {'channel': 1, ...}, 2: {'channel': 2, ...}, ..., 14: {...}}
```

### Set bias voltage

```python
# Set this pipeline's default LNA channel using remote method (iterative feedback)
result = rc.set_lna_bias_voltage(0.65)
# {'channel': 1, 'voltage_v': 0.6498, 'method': 'remote', 'message': 'OK'}

# Set a specific channel using local method (direct DAC)
result = rc.set_lna_bias_voltage(0.65, channel=3, method='local')

# Set all 14 channels to the same voltage
results = rc.set_lna_bias_voltage_all(0.65)
```

### Controller status

```python
status = rc.get_lna_controller_status()
# {'enabled': True, 'hardware': True, 'lna_channel': 1}
```

### Methods summary

| Method | Description |
|--------|-------------|
| `get_lna_controller_status()` | Controller enabled/hardware/channel info |
| `get_lna_bias_status(channel=None)` | Read voltage and current for one channel |
| `get_lna_bias_status_all()` | Read voltage and current for all 14 channels |
| `set_lna_bias_voltage(voltage_v, channel=None, method='remote', blind=False)` | Set bias voltage for one channel |
| `set_lna_bias_voltage_all(voltage_v, method='remote', blind=False)` | Set bias voltage for all 14 channels |

### Voltage setting methods

- **`remote`** (default): Uses iterative feedback from the remote ADC readback
  to converge on the target voltage.  More accurate but slower.
- **`local`**: Sets the DAC resistance directly.  Faster but less precise
  (open-loop).

The `blind` parameter (remote method only) skips the final validation read.
Use this when speed is critical and the voltage need not be verified.

## Standalone usage

The LNA bias hardware can be controlled directly via the submodule, without
running the full readout server:

```python
import sys, os
sys.path.insert(0, 'src/souk_readout_tools/server/souk-peripherals-control')

from smbus2 import SMBus
from souk_lna_bias_control_monitor import (
    SOUKLNABiasControlMonitor,
    SOUKLNABiasControlMonitorHWConfig,
)
from lna_monitor import LNAMonitorHWConfig
from ad511_0_2_4bcpz_5_10_80 import AD511_0_2_4BCPZ_5_10_80HWConfig
from ltc2481cdd import LTC2481CDDHWConfig

# Build per-channel config (identical for all 14 channels on rev 1 board)
per_channel = LNAMonitorHWConfig(
    r_dac_hw_config=AD511_0_2_4BCPZ_5_10_80HWConfig(
        DEV_ADDR=0x2C, RESOLUTION=128, R_FULL_SCALE_KOHM=10.0,
    ),
    remote_adc_hw_config=LTC2481CDDHWConfig(CA0="low", CA1="float"),
    imonitor_adc_hw_config=LTC2481CDDHWConfig(CA0="float", CA1="float"),
    switch_status=True,
    r_LDO_set_kOhm=150.0,
    r_RTop1_kOhm=9.88,
    r_RBot1_kOhm=12.4,
    r_RAdj1_kOhm=1e7,
    r_RSENSE_OHMS=10.0,
)

# M17-M30 map to LNA channels 1-14
refdes = [f"M{i}" for i in range(17, 31)]
lna_configs = {ref: per_channel for ref in refdes}

hw_config = SOUKLNABiasControlMonitorHWConfig(
    lna_monitor_hw_configs=lna_configs,
    r9_r12="R12", r8_r10="R10", r7_r5="R7",
    r11_r13="R13", r14_r15="R14", r6_r4="R6",
)

bus = SMBus(0)
monitor = SOUKLNABiasControlMonitor(bus, hw_config)

# Set LNA channel 1 bias to 0.65 V (remote method)
result = monitor.set_lna_bias_remote(chn=[1], v_local=0.65)
print(result)  # {1: (achieved_voltage, message)}

# Read back status for channels 1-3
status = monitor.read_lna_status(chn=[1, 2, 3])
for ch, s in status.items():
    print(f"Ch {ch}: {s['remote voltage']:.4f} V, {s['bias current']*1e3:.2f} mA")
```
