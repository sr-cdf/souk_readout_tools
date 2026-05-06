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
- **Soft-off control** — drive a channel to the minimum achievable local
  voltage. This is not a hard shutdown of the LNA rail.
- **Bulk operations** — set or read all 14 channels at once.
- **Fixed-value description** — record a non-controllable bias value when the
  LNA bias is set outside the readout tools.

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
  connected: false            # RF calibration flag; LNA bias is independent
  lna_bias:
    enabled: true
    backend: "i2c"            # 'i2c' or 'fixed'
    lna_channel: 1            # LNA index (1-14) for this readout pipeline
    bias_voltage_v: 1.5       # applied on config push/apply when enabled
    soft_off: false           # true applies minimum local voltage instead
    method: "remote"          # 'remote' feedback or 'local' DAC set
    blind: false              # remote method only
    i2c:
      bus: 0                  # LNA bias board SMBus; always 0 for both pipelines
```

`cryostat.connected` only controls whether cryostat S21 terms are included in
the RF calibration chain. LNA bias control is independent: set
`cryostat.lna_bias.enabled: true` to initialise the LNA controller even when
`cryostat.connected: false`.

Setting `lna_bias.enabled: false` (the default) disables all LNA hardware
access for this pipeline. If `bias_voltage_v` is missing or `null`, config
application defaults to 1.5 V.

The Linux SMBus number is not pipeline-specific: both pipeline daemons use
`SMBus(0)`. Pipeline-specific wiring is selected with `lna_channel`, not
`i2c.bus`. LNA I2C operations are serialized across pipeline daemons so the
shared switch tree is not driven concurrently.

Set `lna_bias.soft_off: true` to make config application drive the configured
channel to the minimum local voltage (about 1.15 V on the present board),
rather than applying `bias_voltage_v`. This is a soft mute only: there is no
software-exposed shutdown pin, so upstream board power is still required for a
hard cutoff.

These config values are desired startup/apply values. Runtime LNA changes made
through client methods are held in the server's LNA controller state and are
reported by status/info calls; they are not written back into the active config
file. Use `sync_config_from_system()` / `sync_config_to_local()` only when you
explicitly want to capture the current runtime state into a local config.

### Backends

- **`i2c`** controls the SOUK LNA bias board. On config push or
  `apply_config()`, the server sets this pipeline's configured
  `lna_channel` to `bias_voltage_v`, or to the minimum local voltage when
  `soft_off: true`.
- **`fixed`** records the configured `bias_voltage_v` as an explicit,
  non-controllable value. Status/info calls report the value, but set methods
  and soft-off requests do not attempt hardware control.

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
# {'status': 'success',
#  'result': {'channel': 1, 'voltage_v': 0.6498, 'method': 'remote',
#             'message': '', 'success': True}}

# Set a specific channel using local method (direct DAC)
result = rc.set_lna_bias_voltage(0.65, channel=3, method='local')

# Set all 14 channels to the same voltage
results = rc.set_lna_bias_voltage_all(0.65)
```

If the upstream feedback algorithm rejects the setpoint (typically because the
LNA is not powered or not drawing current, so the estimated LNA voltage falls
outside `(0, v_remote)`), the response comes back as:

```python
{'status': 'error',
 'message': 'Cannot set remote voltage for channel 1, because estimated LNA '
            'voltage is not between 0 V and remote voltage 2.500 V. ...',
 'result': {'channel': 1, 'voltage_v': 2.5, 'method': 'remote',
            'message': '...', 'success': False}}
```

In that situation pass `blind=True` to skip the LNA-voltage sanity check
(only the remote-voltage target is validated), or use `method='local'` to
set the DAC directly without iterative feedback.

### Soft off

```python
# Soft-off this pipeline's default LNA channel
result = rc.soft_off_lna_bias()
# {'status': 'success',
#  'result': {'channel': 1, 'voltage_v': 1.15, 'method': 'local',
#             'soft_off': True,
#             'message': 'LNA channel 1 driven to minimum local voltage ...',
#             'success': True}}

# Soft-off a specific channel
result = rc.soft_off_lna_bias(channel=7)

# Soft-off every configured channel; unconfigured channels report failures
results = rc.soft_off_lna_bias_all()
```

Soft-off uses the same local-voltage path as `method='local'`, choosing the
minimum local voltage reported by the LNA bias monitor for each channel. It
does not fully power down the LNA.

### Discovery CLI

On the RFSoC, the server install includes a quick hardware probe:

```bash
souk-find-lnas --status
```

This scans the production-board LNA reference designators (M17-M30), reports
the populated channel numbers, and with `--status` includes remote voltage,
local voltage, and bias current readings.

### Controller status

```python
status = rc.get_lna_controller_status()
# {'enabled': True, 'hardware': True, 'controllable': True,
#  'backend': 'i2c', 'lna_channel': 1, 'bias_voltage_v': 1.5,
#  'soft_off': False}
```

### Methods summary

| Method | Description |
|--------|-------------|
| `get_lna_controller_status()` | Controller enabled/hardware/channel info |
| `get_lna_bias_status(channel=None)` | Read voltage and current for one channel |
| `get_lna_bias_status_all()` | Read voltage and current for all 14 channels |
| `set_lna_bias_voltage(voltage_v, channel=None, method='remote', blind=False)` | Set bias voltage for one channel |
| `set_lna_bias_voltage_all(voltage_v, method='remote', blind=False)` | Set bias voltage for all 14 channels |
| `soft_off_lna_bias(channel=None)` | Drive one channel to minimum local voltage |
| `soft_off_lna_bias_all()` | Drive all channels to their minimum local voltage |

### Voltage setting methods

- **`remote`** (default): Uses iterative feedback from the remote ADC readback
  to converge on the target voltage.  More accurate but slower.
- **`local`**: Sets the DAC resistance directly.  Faster but less precise
  (open-loop).

The `blind` parameter (remote method only) skips the LNA-voltage sanity
check (`v_remote > v_lna > 0`).  Use this when the downstream LNA is not
yet drawing current and the estimation would otherwise reject a valid
setpoint, or when speed is critical and the voltage need not be verified.

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
