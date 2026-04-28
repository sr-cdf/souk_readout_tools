# RF Peripherals

The readout tools integrate RF peripheral hardware control via the
[souk-peripherals-control](https://github.com/WeiDaZhang/souk-peripherals-control)
library, included as a git submodule at
`src/souk_readout_tools/server/souk-peripherals-control/`.

## What it does

The submodule provides I2C-based drivers for the SOUK RF Mixerless Module
hardware — variable attenuators and bypassable amplifiers on the TX and RX
signal paths.  It also includes RF component models that calculate transfer
functions, total gain, and 1 dB compression points for the full signal chain.

It also provides LNA bias monitoring and control for up to 14 cryogenic LNA
channels (M17-M30).  Each channel can be read back (remote voltage, local
voltage, bias current) and its bias set by targeting a local or remote
voltage.  Channels are addressed via a two-level TCA9548 I2C switch tree.

Key hardware components on the module:

| Component | Part | Notes |
|-----------|------|-------|
| Variable attenuator | ZX76-31R5A-PNS+ | 0-31.5 dB in 0.5 dB steps |
| Amplifier | ZX60-53LNB-S+ | 20.2 dB gain, bypassable |
| Filter | BFCV-2895 | 1.8 dB insertion loss |
| Equalizer | VEQY-5-63+ | 3.5 dB insertion loss |
| Fixed attenuator | BW-S5W2+ | 5.0 dB |

The I2C bus uses MAX7328/7329 GPIO expanders to set attenuator values and
amplifier bypass states.

## How it fits together

```
souk_readout_tools/server/
├── rf_peripherals.py              ← RFPeripheralController wrapper
├── rudat.py                       ← alternative USB attenuator backend
└── souk-peripherals-control/      ← git submodule
    ├── souk_rf_mixerless_module.py           ← hardware I2C driver
    ├── souk_rf_mixerless_atten_amp_level.py  ← RF component models
    ├── souk_lna_bias_control_monitor.py      ← LNA bias monitor/control
    ├── max732_8_9.py                         ← GPIO expander driver
    └── i2c_devices.py                        ← base I2C device class
```

`rf_peripherals.py` is the wrapper that the readout server uses.  It adds the
submodule directory to `sys.path`, imports the submodule classes, and provides a
unified `RFPeripheralController` interface that supports three attenuator
backends:

- **i2c** — real SOUK I2C hardware via `smbus2`
- **rudat** — Mini-Circuits RUDAT-6000-30 USB attenuators via `pyusb`
- **fixed** — explicit, non-controllable attenuation values from config

## Configuration

RF peripherals are configured in the `rf_frontend` section of the readout
config YAML. The `RFPeripheralController` is active when `connected: true`
and `attenuator.backend` is set to `i2c`, `rudat`, or `fixed`. Use `fixed`
for manual attenuators whose values should contribute to calibration but
must not be changed by software.

```yaml
rf_frontend:
  connected: true
  hardware_id: "souk-mixerless-module"
  mixerless_module:
    connected: true                 # true for the SOUK mixerless module
    rf_channel: 0                   # 0 or 1
  attenuator:
    backend: "i2c"                  # 'i2c', 'rudat', or 'fixed'
    tx_value_db: 10.0
    rx_value_db: 15.0
    # rudat_tx_serial: "12345"      # required if backend: rudat
    # rudat_rx_serial: "67890"
  bypass_amps:
    enabled: true                   # true for souk-mixerless-module
    tx_amp_bypass: false
    rx_amp_bypass: false
```

The attenuator keys in the config are desired startup/apply values. Runtime
attenuator and bypass-amp values are kept separately by the server and are
reported by `get_rf_peripheral_status()` / `get_info(['rf_frontend'])`.
`pull_config()` returns the active config file only. Use
`sync_config_from_system()` or `sync_config_to_local()` when you explicitly
want to capture the current hardware state into a local config.

## Standalone usage

The submodule can be used independently of the readout server.  This is useful
for bench testing or debugging I2C connectivity.

### Direct hardware control

```python
import sys, os
sys.path.insert(0, 'src/souk_readout_tools/server/souk-peripherals-control')

from smbus2 import SMBus
from souk_rf_mixerless_module import (
    SOUKRFMixerlessModule,
    SOUKRFMixerlessModuleChnHWConfig,
)

bus = SMBus(0)  # /dev/i2c-0
hw_config = [
    SOUKRFMixerlessModuleChnHWConfig(
        r8_r13="R8", r9_r14="R9", r12_r17="R17",
        r18_r21="R21", r19_r22="R19", r20_r23="R23",
        u4_type="MAX7329", u8_type="MAX7329",
    ),
    SOUKRFMixerlessModuleChnHWConfig(
        r8_r13="R8", r9_r14="R14", r12_r17="R17",
        r18_r21="R21", r19_r22="R22", r20_r23="R23",
        u4_type="MAX7329", u8_type="MAX7329",
    ),
]
module = SOUKRFMixerlessModule(bus, hw_config)

# Set TX attenuator to 12.0 dB
module.set_attenuation(0, 'transmit_atten', 12.0)

# Read back
print(module.get_attenuation_value(0, 'transmit_atten'))

# Bypass the RX amplifier
module.set_amp_bypass_state(0, 'recv_atten', True)
```

### Transfer function modelling (no hardware needed)

```python
import sys
sys.path.insert(0, 'src/souk_readout_tools/server/souk-peripherals-control')

from souk_rf_mixerless_atten_amp_level import (
    SOUKRFMixerlessTransmitAttenAmpLevel,
    SOUKRFMixerlessRecvAttenAmpLevel,
)

tx_model = SOUKRFMixerlessTransmitAttenAmpLevel()
rx_model = SOUKRFMixerlessRecvAttenAmpLevel()

# Get TX transfer function with amp enabled, 10 dB attenuation
tx_model.atten = 10.0
tx_model.bypass_state = False
tx_transfer = tx_model.get_transfer()
print(f"TX total gain: {tx_transfer.total_gain_il:.1f} dB")
print(f"TX input 1dB comp: {tx_model.input_1dB_comp:.1f} dBm")
```

### RUDAT USB attenuator

The `rudat.py` module provides a stateless USB driver for Mini-Circuits
RUDAT-6000-30 programmable attenuators.  Range is 0-30 dB in 0.25 dB steps.

```python
from souk_readout_tools.server.rudat import find_rudats, Attenuator

# Discover all connected RUDATs (prints bus/address/serial for each)
rudats = find_rudats()
# e.g. {12345: {'bus': 1, 'address': 5}, 12346: {'bus': 1, 'address': 6}}

# Open a specific device by bus and address
atten = Attenuator(usb_bus=1, usb_address=5)

# Read current attenuation
print(f"Current: {atten.att} dB")

# Set attenuation (quantised to 0.25 dB, clamped to [0, 30] dB)
atten.att = 12.5

# Device info
print(atten.get_model())             # 'RUDAT-6000-30'
print(atten.get_serial())            # '12345'
print(atten.get_firmware_version())
print(atten.describe())              # one-line summary
```

The driver is stateless — each read/write opens and closes the USB handle, so
multiple processes can share the device safely.  An advisory file lock
(`/tmp/rudat.lock.<serial>`) serialises concurrent access.

A udev rule avoids needing `sudo`:

```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="20ce", ATTRS{idProduct}=="0023", MODE="0666", GROUP="plugdev"
```

You can also probe all connected RUDATs from the command line:

```bash
python -m souk_readout_tools.server.rudat
```

The installed server package also provides discovery/status entry points:

```bash
souk-find-attenuators --status
souk-find-bypass-amps --status
souk-rf-peripherals-status
```

`souk-find-attenuators` reports both RUDAT devices and the two TX/RX
attenuator paths on each SOUK mixerless-module channel. `souk-find-bypass-amps`
reports the corresponding bypass amplifiers. The combined status command runs
the attenuator, bypass-amplifier, and LNA discovery passes in one report.

### LNA bias monitoring and control

`souk_lna_bias_control_monitor.py` provides the `SOUKLNABiasControlMonitor`
class, which monitors and controls bias for up to 14 LNA channels (reference
designators M17-M30).  Channels are multiplexed via a root/leaf pair of
TCA9548 I2C switches.

```python
import sys
sys.path.insert(0, 'src/souk_readout_tools/server/souk-peripherals-control')

from smbus2 import SMBus
from souk_lna_bias_control_monitor import (
    SOUKLNABiasControlMonitor,
    SOUKLNABiasControlMonitorHWConfig,
)
from lna_monitor import LNAMonitorHWConfig
from ad511_0_2_4bcpz_5_10_80 import AD511_0_2_4BCPZ_5_10_80HWConfig
from ltc2481cdd import LTC2481CDDHWConfig

bus = SMBus(0)
hw_config = SOUKLNABiasControlMonitorHWConfig(
    lna_monitor_hw_configs={...},  # see souk_lna_bias_control_monitor.py main() for a full example
    r9_r12="R12", r8_r10="R10", r7_r5="R7",
    r11_r13="R13", r14_r15="R14", r6_r4="R6",
)
monitor = SOUKLNABiasControlMonitor(bus, hw_config)

# Read status for channels 1, 12, 13, 14
status = monitor.read_lna_status(chn=[1, 12, 13, 14])
for chn, s in status.items():
    print(f"Ch{chn}: remote={s['remote voltage']:.3f} V, "
          f"local={s['local voltage']:.3f} V, "
          f"bias={s['bias current']*1e3:.3f} mA")

# Set bias by local voltage
monitor.set_lna_bias_local(chn=1, v_local=1.2)

# Set bias by targeting a remote voltage (iterates DAC to converge)
result = monitor.set_lna_bias_remote(chn=[1, 12], v_local=1.2)
```

### Command-line quick check (mixerless module)

From the submodule directory:

```bash
cd src/souk_readout_tools/server/souk-peripherals-control

# Print current attenuator and amp bypass states
sudo /home/casper/py3.12-venv/bin/python3 souk_rf_mixerless_module.py --get

# LNA bias status
sudo /home/casper/py3.12-venv/bin/python3 souk_lna_bias_control_monitor.py --status --channels 1 12 13 14
```

## Readout tools functions that use RF peripherals

### Server side (`readout_server.py`)

The server creates an `RFPeripheralController` on startup and exposes it
through the request API:

```python
# Initialisation (server startup)
self.rf_peripherals = RFPeripheralController(self.config, self.pipeline_id)

# Applying a config file also updates hardware
self.rf_peripherals.apply_config(config_contents)
```

### Client side (`readout_client.py`)

The `ReadoutClient` provides matching methods that send requests to the server:

```python
from souk_readout_tools.client.readout_client import ReadoutClient

client = ReadoutClient(address='192.168.2.10', request_port=8764)

# Attenuation control
client.set_tx_attenuation(10.0)   # dB
client.set_rx_attenuation(15.0)

# Amplifier bypass
client.set_tx_amp_bypass(True)
client.set_rx_amp_bypass(False)

# Read back full status
status = client.get_rf_peripheral_status()
print(status)
# {'tx_attenuation_db': 10.0, 'rx_attenuation_db': 15.0,
#  'tx_amp_bypass': True, 'rx_amp_bypass': False,
#  'tx_total_gain_db': ..., 'rx_total_gain_db': ..., ...}
```

### Tone power budget (`firmware_lib.get_tone_powers`)

`get_tone_powers()` computes the power of each tone at a specified reference
plane by walking the TX signal chain.  The `reference_plane` parameter selects
where in the chain the power is reported:

| `reference_plane` | Description |
|---|---|
| `'dac'` | Power at DAC output |
| `'rf_output'` | Power at RF frontend output (after attenuator, amp, mixer) |
| `'detector'` | Power at detector (default, full TX chain) |

When `detailed_output=True` it returns a breakdown of every gain/loss stage.
The current peripheral settings are included in this calculation.  With
hardware RF peripherals available, the server reads attenuator and bypass-amp
state directly from the `RFPeripheralController`; otherwise it falls back to
the desired/fixed values in the config.

The `tx_total_gain_db` / `rx_total_gain_db` values reported by
`get_info(['rf_frontend'])` are model summaries from the peripheral
controller.  Tone-power estimates do not use those summary values directly;
they use the staged calibration chain (`attenuator.*_value_db`,
`*_if_s21_db`, `*_mixer_conversion_loss_db`, `*_rf_s21_db`,
`bypass_amps.*_s21_db`, and cryostat S21).  If the S21 fields are left unset,
detector-plane powers are only as good as the remaining defaults.

```python
# Per-tone power at detector plane (default)
powers = client.get_tone_powers()

# Power at DAC output
dac_powers = client.get_tone_powers(reference_plane='dac')

# Detailed breakdown
powers, details = client.get_tone_powers(detailed_output=True)
```

### RX tone power estimation

`get_tone_powers()` also covers the RX chain, estimating received tone powers
from accumulated IQ data using the RX calibration chain to convert back to
physical power.

| `reference_plane` | Description |
|---|---|
| `'cryostat_output'` | Power at cryostat output (before RX frontend) |
| `'adc_input'` | Power at ADC input in dBm |
| `'accumulator'` | Raw accumulated IQ magnitude in dB |

```python
# Estimated power at ADC input
rx_powers = client.get_tone_powers(reference_plane='adc_input')
```

### TX power optimisation (`firmware_lib.set_tone_powers`)

When `optimise_dynamic_range=True`, `set_tone_powers()` maximises DAC bit
utilisation then computes the exact analog attenuation needed from the
calibration chain gains, rather than trial-and-error hardware probing.

The analog adjustment order is:
1. Enable TX amplifier (maximum analog gain)
2. Set minimum required programmable attenuation (0-31.5 dB)
3. If insufficient, bypass TX amplifier
4. As a last resort, use RFDC DAC DSA (up to 12 dB)

The result dict includes `achieved_powers_dbm`, `power_error_db`, and
`warnings`, which are surfaced to the client automatically.

```python
# Set tone powers with dynamic range optimisation
result = client.set_tone_powers(
    [-20, -25],
    reference_plane='detector',
    optimise_dynamic_range=True
)
print(result['warnings'])        # any limitations encountered
print(result['power_error_db'])  # per-tone error vs target
```

`maximise_tx_power()` can optionally include frontend compression headroom:

```python
result = client.maximise_tx_power(
    headroom_db=2.0,
    reference_plane='detector',
    compression_headroom_db=10.0,
)
print(result['result']['tx_compression'])
```
