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

Key hardware components on the module:

| Component | Part | Notes |
|-----------|------|-------|
| Variable attenuator | ZX76-31R5A-PNS+ | 0–31.5 dB in 0.5 dB steps |
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
    ├── souk_rf_mixerless_module.py       ← hardware I2C driver
    ├── souk_rf_mixerless_atten_amp_level.py  ← RF component models
    ├── max732_8_9.py                     ← GPIO expander driver
    └── i2c_devices.py                    ← base I2C device class
```

`rf_peripherals.py` is the wrapper that the readout server uses.  It adds the
submodule directory to `sys.path`, imports the submodule classes, and provides a
unified `RFPeripheralController` interface that supports three backends:

- **mixerless** (default) — real I2C hardware via `smbus2`
- **rudat** — Mini-Circuits RUDAT-6000-30 USB attenuators via `pyusb`
- **software mimic** — automatic fallback when no hardware libraries are
  available; useful for offline transfer-function modelling

## Configuration

RF peripherals are configured in the `rf_frontend.mixerless_module` section of
the readout config YAML:

```yaml
rf_frontend:
  tx_attenuator_value_db: 10.0
  rx_attenuator_value_db: 15.0
  mixerless_module:
    enabled: true
    attenuator_backend: mixerless   # or 'rudat'
    i2c_bus: 0
    channel: 0
    tx_amp_bypass: false
    rx_amp_bypass: false
    # rudat_tx_serial:   # required if backend is 'rudat'
    # rudat_rx_serial:
```

Set `enabled: true` and choose the appropriate `attenuator_backend` for your
hardware.

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
hw_config = [SOUKRFMixerlessModuleChnHWConfig.default_config()]
module = SOUKRFMixerlessModule(bus, hw_config)

# Set TX attenuator to 12.0 dB
module.set_atten(channel=0, tx_rx='tx', atten_dB=12.0)

# Read back
print(module.get_atten(channel=0, tx_rx='tx'))

# Bypass the RX amplifier
module.set_amp_bypass(channel=0, tx_rx='rx', bypass=True)
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
tx_transfer = tx_model.get_transfer(atten_dB=10.0, bypass=False)
print(f"TX total gain: {tx_transfer.total_gain_dB:.1f} dB")
print(f"TX input 1dB comp: {tx_transfer.input_1dB_comp:.1f} dBm")
```

### Command-line quick check

From the submodule directory:

```bash
cd src/souk_readout_tools/server/souk-peripherals-control

# Print current attenuator and amp bypass states
sudo python3 souk_rf_mixerless_module.py --get

# LNA bias status
sudo python3 souk_lna_bias_control_monitor.py --status --channels 1 12 13 14
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

client = ReadoutClient(host='192.168.2.10', port=8764)

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

`get_tone_powers()` computes the power of each tone at the cryostat input by
walking the full TX signal chain.  When `detailed_output=True` it returns a
breakdown of every gain/loss stage.  The current peripheral settings are
included in this calculation — it reads `tx_attenuator_value_db` and
`tx_bypass_amp_s21_db` from the live config, which the `RFPeripheralController`
keeps in sync with the hardware whenever attenuation or amp bypass is changed.

```python
from souk_readout_tools.firmware_lib import get_tone_powers

# Per-tone power at detector plane (accounts for current attenuator + amp state)
powers = get_tone_powers(r, config)

# Detailed breakdown including each stage contribution
powers, details = get_tone_powers(r, config, detailed_output=True)
```

### TX power optimisation (`firmware_lib.set_tone_powers / set_tx_power`)

`set_tone_powers()` and `set_tx_power()` accept an optional `rf_peripherals`
controller and automatically adjust the TX attenuator (and amplifier bypass if
needed) during power-level optimisation:

```python
from souk_readout_tools.firmware_lib import set_tx_power

# Called internally by the server — adjusts attenuator to hit target power
set_tx_power(r, config, target_dbfs=-12.0, rf_peripherals=rf_ctrl)
```
