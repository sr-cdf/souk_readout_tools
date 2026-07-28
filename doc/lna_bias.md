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
├── lna_service.py                     ← TCP service owning the board
├── lna_controller.py                  ← LNABiasController + _BiasMonitor (v1/v2)
├── i2c_lock.py                        ← cross-process lock for the shared bus
└── souk-peripherals-control/          ← git submodule
    ├── souk_lna_bias_control_monitor.py  ← hardware driver
    ├── lna_monitor.py                    ← per-channel monitor
    ├── max732_8_9.py                     ← output-enable GPIO expanders (v2)
    ├── ad511_0_2_4bcpz_5_10_80.py        ← digital pot driver
    └── ltc2481cdd.py                     ← ADC driver
```

`_BiasMonitor` in `lna_controller.py` subclasses the submodule's
`SOUKLNABiasControlMonitor` to cover both board revisions from one class: it
builds the v2 output-enable expanders only when they are present, batches
multi-channel enable operations into one write per expander, and — unlike the
submodule's own constructor — never disables the outputs on construction.

`lna_controller.py` wraps the submodule's `SOUKLNABiasControlMonitor` class
and exposes it through the readout server's request/response interface.

## Topology: one bias board per telescope

Only **one RFSoC per telescope is wired to the LNA bias board** — the board in
**slot 1 of the telescope's rack**. Every RFSoC has its own RF module on I2C,
but the bias board is shared hardware serving all 14 LNA channels from that
single machine.

The slot-1 board runs the **LNA bias service** (`souk-lna-service`), and every
readout server — including the two pipelines on that same machine — reaches the
bias hardware through it:

```
      RFSoC in slot 1                     other RFSoCs
  ┌─────────────────────┐            ┌──────────────────┐
  │  souk-lna-service   │◀───TCP─────│  readout server  │  (backend: remote)
  │        │            │   :10500   └──────────────────┘
  │        │ I2C        │            ┌──────────────────┐
  │        ▼            │◀───TCP─────│  readout server  │  (backend: remote)
  │   LNA bias board    │            └──────────────────┘
  │   (14 channels)     │
  │                     │
  │  readout server ────┼──TCP──┐
  │  readout server ────┼──TCP──┘  (also backend: remote, via localhost)
  └─────────────────────┘
```

Routing every server through the service means there is exactly one process
driving the board, the bias survives readout-server restarts and firmware
reloads, and slow I2C reads are served from the service's status cache instead
of fourteen servers queueing for the bus. Each server still addresses its own
LNA: it sends its configured `lna_channel` explicitly with every request.

## Configuration

LNA bias is configured in the `cryostat` section of the readout config YAML:

```yaml
cryostat:
  connected: false            # RF calibration flag; LNA bias is independent
  lna_bias:
    enabled: true
    backend: "remote"         # 'remote', 'i2c', or 'fixed'
    lna_channel: 1            # LNA index (1-14) for this readout pipeline
    bias_voltage_v: 1.5       # applied on config push/apply when enabled
    soft_off: false           # true applies minimum local voltage instead
    method: "remote"          # 'remote' feedback or 'local' DAC set
    blind: false              # remote method only
    # host:                   # optional override of site.yaml's lna_service.host
    # request_port: 10500     # optional override of site.yaml's lna_service.port
    # hw_version: "auto"      # board revision for the i2c backend: auto, 1, or 2
    i2c:
      bus: 0                  # LNA bias board SMBus; always 0 for both pipelines
```

### Site file

With `backend: remote`, the address of the LNA service comes from the machine's
site file, `~/.souk_readout_tools/site.yaml`, so that a rack change is one edit
per machine rather than one edit per pipeline config:

```yaml
lna_service:
  host: "10.11.11.11"   # the RFSoC wired to the LNA bias board
  port: 10500
  timeout_s: 15.0
```

Prefer a hostname if your site has stable DNS or `/etc/hosts` entries — then a
reshuffle needs no config edits at all. A per-pipeline `cryostat.lna_bias.host`
overrides the site file when you need it (bench setups, testing against a second
board). If neither is set, the controller logs an error naming the site file.

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

- **`remote`** talks to the LNA bias service over TCP. This is the normal
  telescope deployment, used by every readout server including those on the
  machine running the service. On config push or `apply_config()`, the server
  sets its configured `lna_channel` to `bias_voltage_v` through the service,
  exactly as the `i2c` backend does locally. If the service is unreachable, LNA
  calls return a failure result explaining that rather than raising, so the rest
  of the readout server keeps working.
- **`i2c`** drives a bias board attached to this machine's own I2C bus. Bench
  use only: in a telescope this would give the board a second owner alongside
  the service.
- **`fixed`** records the configured `bias_voltage_v` as an explicit,
  non-controllable value. Status/info calls report the value, but set methods
  and soft-off requests do not attempt hardware control.

## Running the service

Enable the service **only on the RFSoC in slot 1 of the telescope's rack** —
the one wired to the bias board. It will refuse to start anywhere else, since
no bias board answers on the bus. See
[installation.md](installation.md#8-enable-the-lna-bias-service-one-board-per-telescope)
for the full setup, including the site file every other board needs.

```bash
souk-lna-service-install     # install + enable + start (needs sudo)
souk-lna-service-status
souk-lna-service-restart
souk-lna-service-stop
souk-lna-service-remove
```

Or run it in the foreground while debugging:

```bash
souk-lna-service --port 10500 --log-level debug
```

The service refuses to start if the bias board does not respond, so a failure
is visible in `systemctl status` rather than surfacing later as confusing
per-request errors. It logs every set with the requesting client's address, so a
mis-configured `lna_channel` somewhere on the telescope is traceable.

Opening the board probes all 14 slots, and an unpopulated slot costs a full I2C
retry cycle, so startup can take tens of seconds on a sparsely populated board.
That cost is paid once by the service rather than on every readout-server start.

## Board revisions

Two revisions of the bias board are supported, detected automatically at
startup (`hw_version: auto`) by probing for the v2 output-enable expanders:

| | v1 | v2 |
|---|---|---|
| Populated channels | partial | all 14 |
| Per-channel output enable | no | yes |
| Turning a channel off | `soft_off` (minimum voltage) | true power cut |

On **v1** there is no shutdown pin, so `soft_off_lna_bias()` drives the channel
to its minimum local voltage (about 1.15 V) — a soft mute, not a power cut.

On **v2** each channel has a hard output switch, so
`set_lna_output_enabled(False)` genuinely removes power from the LNA.
`soft_off` still works and keeps its old meaning. Status calls report
`output_enabled` (`None` on v1, where the concept does not apply).

Neither revision powers the LNAs off when the service starts, so restarting the
service does not disturb a cold cryostat.

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

### Output enable (v2 boards)

On a v2 bias board each channel has a hard output switch, so power can be
removed outright rather than turned down:

```python
# Cut power to this pipeline's LNA
result = rc.set_lna_output_enabled(False)
# {'status': 'success',
#  'result': {'channel': 1, 'output_enabled': False, 'message': '',
#             'success': True, 'hw_version': 2}}

# Power a specific channel back up
result = rc.set_lna_output_enabled(True, channel=7)

# Switch all 14 channels together (the ~1 s rail settle is paid once)
results = rc.set_lna_output_enabled_all(True)
```

On a v1 board these return `status: error` with a message pointing at
`soft_off_lna_bias`, since the hardware has no output switch. Check
`get_lna_controller_status()['supports_output_enable']` if you need to branch.

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
#  'backend': 'remote', 'lna_channel': 1, 'bias_voltage_v': 1.5,
#  'soft_off': False, 'hw_version': 2, 'supports_output_enable': True,
#  'service_endpoint': '10.11.11.11:10500'}
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
| `set_lna_output_enabled(enabled, channel=None)` | Hard power on/off for one channel (v2 boards) |
| `set_lna_output_enabled_all(enabled)` | Hard power on/off for all 14 channels (v2 boards) |

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
