# Test Plan — v1.1.0

All tests below target the changes on `dev/v1.1.0`.
Hardware-dependent tests are marked **(HW)** — skip these without an RFSoC.

---

## 1  Import & Syntax Validation

These catch typos, missing imports, and circular dependencies.

```bash
# Syntax-check every new/modified Python file
python -c "
import ast, pathlib
files = [
    'src/souk_readout_tools/__init__.py',
    'src/souk_readout_tools/calibration.py',
    'src/souk_readout_tools/firmware_lib.py',
    'src/souk_readout_tools/resonator.py',
    'src/souk_readout_tools/fitting.py',
    'src/souk_readout_tools/measurement.py',
    'src/souk_readout_tools/plotting/__init__.py',
    'src/souk_readout_tools/plotting/_common.py',
    'src/souk_readout_tools/plotting/_psd.py',
    'src/souk_readout_tools/plotting/sweep.py',
    'src/souk_readout_tools/plotting/timestream.py',
    'src/souk_readout_tools/plotting/snapshot.py',
    'src/souk_readout_tools/client/readout_client.py',
    'src/souk_readout_tools/client/client_scripts/wideband_sweep.py',
    'src/souk_readout_tools/client/client_scripts/batch_snapshots.py',
    'src/souk_readout_tools/client/client_scripts/find_resonances.py',
    'src/souk_readout_tools/server/readout_server.py',
    'src/souk_readout_tools/server/rf_peripherals.py',
    'src/souk_readout_tools/server/rudat.py',
]
for f in files:
    ast.parse(pathlib.Path(f).read_text())
    print(f'  OK  {f}')
"

# Top-level import (client side — no souk_mkid_readout needed)
python -c "from souk_readout_tools import calibration, resonator, fitting, measurement, plotting; print('Top-level imports OK')"

# Sub-module imports
python -c "from souk_readout_tools.plotting import plot_sweep, plot_sweep_iq, plot_sweep_magphase, plot_sweep_iq_vs_f; print('sweep imports OK')"
python -c "from souk_readout_tools.plotting import plot_timestream, plot_timestream_psd, plot_timestream_on_resonance; print('timestream imports OK')"
python -c "from souk_readout_tools.plotting import plot_snapshots, plot_snapshots_psd, plot_batch_snapshots; print('snapshot imports OK')"
python -c "from souk_readout_tools.plotting import compute_psd, compute_psd_averaged, compute_psd_concatenated; print('psd imports OK')"
python -c "from souk_readout_tools.fitting import fit_resonance, batch_fit, extract_parameters, FitResult; print('fitting imports OK')"
python -c "from souk_readout_tools.measurement import ParameterSweep, TimedMeasurement, ConditionalMeasurement, save_measurement; print('measurement imports OK')"
python -c "from souk_readout_tools.resonator import remove_cable_delay, center_circle, rotate_to_real_axis, deembed, apply_deembed_params; print('resonator imports OK')"
```

---

## 2  Resonator Module (`resonator.py`)

### 2.1  Cable delay removal
```python
import numpy as np
from souk_readout_tools.resonator import remove_cable_delay

f = np.linspace(4e9, 6e9, 1000)
tau_true = 50e-9
s21 = np.exp(-2j * np.pi * f * tau_true)  # pure delay

s21_corr, tau_est = remove_cable_delay(f, s21)

assert abs(tau_est - tau_true) < 1e-12, f"tau estimate wrong: {tau_est}"
assert np.allclose(np.angle(s21_corr), 0, atol=1e-6), "residual phase slope"
print("2.1 PASS — cable delay removal")
```

### 2.2  Cable delay with explicit tau
```python
s21_corr2, tau2 = remove_cable_delay(f, s21, tau=tau_true)
assert tau2 == tau_true
assert np.allclose(np.angle(s21_corr2), 0, atol=1e-12)
print("2.2 PASS — explicit tau")
```

### 2.3  Circle centering (Kasa fit)
```python
from souk_readout_tools.resonator import center_circle

theta = np.linspace(0, 2*np.pi, 200)
cx, cy, r = 0.5, -0.3, 0.4
circle = (cx + r*np.cos(theta)) + 1j*(cy + r*np.sin(theta))

centered, center, radius = center_circle(circle)
assert abs(radius - r) < 0.01, f"radius wrong: {radius}"
assert abs(center - (cx + 1j*cy)) < 0.01, f"center wrong: {center}"
assert np.max(np.abs(np.abs(centered) - radius)) < 0.01
print("2.3 PASS — Kasa circle fit")
```

### 2.4  Rotate to real axis
```python
from souk_readout_tools.resonator import rotate_to_real_axis

# Place resonance point at known angle
res_point = 0.4 * np.exp(1j * np.pi/3)
s21_test = np.array([1+0j, 0.8+0.2j, res_point, 0.9-0.1j])
rotated, angle = rotate_to_real_axis(s21_test, s21_at_resonance=res_point)

# After rotation the resonance point should be on negative real axis
rot_res = res_point * np.exp(1j * angle)
assert rot_res.real < 0 and abs(rot_res.imag) < 1e-10, f"not on neg real axis: {rot_res}"
print("2.4 PASS — rotation to real axis")
```

### 2.5  Full deembed pipeline
```python
from souk_readout_tools.resonator import deembed, apply_deembed_params

# Synthetic resonance with delay
f = np.linspace(4.99e9, 5.01e9, 500)
fr, Ql = 5e9, 1e4
s21_ideal = 1 - 0.5 / (1 + 2j*Ql*(f - fr)/fr)
tau = 30e-9
s21 = s21_ideal * np.exp(-2j*np.pi*f*tau) + 0.3 + 0.2j

s21_de, params = deembed(f, s21)
assert 'tau' in params and 'center' in params and 'rotation_angle' in params
print(f"2.5 PASS — deembed pipeline (tau={params['tau']:.2e})")

# Apply same params to "timestream" data
ts = np.array([0.5+0.1j, 0.4-0.2j])
ts_de = apply_deembed_params(ts, params)
assert ts_de.shape == ts.shape
print("2.5b PASS — apply_deembed_params")
```

---

## 3  Fitting Module (`fitting.py`)

### 3.1  Fit synthetic resonance
```python
import numpy as np
from souk_readout_tools.fitting import fit_resonance, FitResult

fr_true, Ql_true, Qc_true = 5e9, 20000, 40000
f = np.linspace(fr_true - 5*fr_true/Ql_true, fr_true + 5*fr_true/Ql_true, 500)
s21 = 1 - (Ql_true/Qc_true) / (1 + 2j*Ql_true*(f - fr_true)/fr_true)
s21 += np.random.normal(0, 0.005, len(f)) + 1j*np.random.normal(0, 0.005, len(f))

result = fit_resonance(f, s21, fr_guess=fr_true)
assert isinstance(result, FitResult)
assert result.success, "fit did not converge"
assert abs(result.fr - fr_true)/fr_true < 0.001, f"fr off: {result.fr}"
assert abs(result.Ql - Ql_true)/Ql_true < 0.2, f"Ql off: {result.Ql}"
print(f"3.1 PASS — fit_resonance (fr={result.fr:.6e}, Ql={result.Ql:.0f})")
```

### 3.2  Batch fit with synthetic sweep
```python
from souk_readout_tools.fitting import batch_fit, extract_parameters

# Two resonances
f1, f2 = 4.5e9, 5.5e9
f_all = np.linspace(4e9, 6e9, 10000)
s21 = np.ones_like(f_all, dtype=complex)
for fr in [f1, f2]:
    s21 -= 0.5 / (1 + 2j*15000*(f_all - fr)/fr)

sweep_data = {
    'sweep_f': f_all.reshape(1, -1),
    'sweep_i': s21.real.reshape(1, -1),
    'sweep_q': s21.imag.reshape(1, -1),
}

results = batch_fit(sweep_data, verbose=True)
assert len(results) >= 2, f"expected >=2 fits, got {len(results)}"

params = extract_parameters(results)
assert 'fr' in params and 'Ql' in params
print(f"3.2 PASS — batch_fit found {len(results)} resonances, extract_parameters OK")
```

---

## 4  PSD Computation (`plotting/_psd.py`)

### 4.1  Welch PSD — real data
```python
import numpy as np
from souk_readout_tools.plotting import compute_psd

fs = 1e6
t = np.arange(0, 0.1, 1/fs)
x = np.sin(2*np.pi*1000*t) + 0.1*np.random.randn(len(t))

f_psd, psd = compute_psd(x, fs, nperseg=4096)
peak_idx = np.argmax(psd)
assert abs(f_psd[peak_idx] - 1000) < 500, f"peak at {f_psd[peak_idx]} Hz"
print("4.1 PASS — Welch PSD real data")
```

### 4.2  PSD — complex data (I + Q summed)
```python
z = x + 1j * 0.5*np.sin(2*np.pi*2000*t)
f_psd, psd_c = compute_psd(z, fs)
assert psd_c.ndim == 1, "complex PSD should be 1D (I+Q summed)"
assert psd_c.shape == f_psd.shape
print("4.2 PASS — complex PSD")
```

### 4.3  Averaged PSD
```python
from souk_readout_tools.plotting import compute_psd_averaged

data_2d = np.random.randn(10, 1000)
f_avg, psd_mean, psd_std = compute_psd_averaged(data_2d, fs)
assert psd_mean.shape == psd_std.shape == f_avg.shape
assert np.all(psd_std >= 0)
print("4.3 PASS — averaged PSD")
```

### 4.4  Concatenated PSD
```python
from souk_readout_tools.plotting import compute_psd_concatenated

# Use larger nperseg so concatenated has more bins than per-row
f_cat, psd_cat = compute_psd_concatenated(data_2d, fs, nperseg=2048)
assert len(f_cat) > len(f_avg), "concatenated with large nperseg should have more bins"
print("4.4 PASS — concatenated PSD")
```

---

## 5  Plotting — Sweep (`plotting/sweep.py`)

For plotting tests, verify figure creation and axis count without calling `plt.show()`.

### 5.1  Wideband sweep — magphase
```python
import numpy as np
from souk_readout_tools.plotting import plot_sweep_magphase

f = np.linspace(4e9, 6e9, 2000)
s21 = np.exp(-1j*2*np.pi*f*30e-9) * (1 - 0.3/(1+2j*1e4*(f-5e9)/5e9))
sweep = {
    'sweep_f': f.reshape(1, -1),
    'sweep_i': s21.real.reshape(1, -1),
    'sweep_q': s21.imag.reshape(1, -1),
}

fig = plot_sweep_magphase(sweep, show_errors=False)
assert fig is not None
assert len(fig.axes) >= 2, "magphase should have mag + phase axes"
import matplotlib.pyplot as plt; plt.close(fig)
print("5.1 PASS — plot_sweep_magphase")
```

### 5.2  Per-tone sweep — IQ overlay
```python
from souk_readout_tools.plotting import plot_sweep_iq

n_pts, n_tones = 100, 3
sweep_pt = {
    'sweep_f': np.random.uniform(4e9, 6e9, (n_pts, n_tones)),
    'sweep_i': np.random.randn(n_pts, n_tones),
    'sweep_q': np.random.randn(n_pts, n_tones),
}
fig = plot_sweep_iq(sweep_pt, multi_tone='overlay')
assert fig is not None; plt.close(fig)
print("5.2 PASS — per-tone IQ overlay")
```

### 5.3  Per-tone sweep — grid
```python
from souk_readout_tools.plotting import plot_sweep

fig = plot_sweep(sweep_pt, format='magphase', multi_tone='grid')
assert fig is not None
assert len(fig.axes) >= n_tones * 2  # 2 axes per tone
plt.close(fig)
print("5.3 PASS — per-tone grid")
```

### 5.4  Sweep with deembedding
```python
fig = plot_sweep(sweep, format='iq', deembed=True)
assert fig is not None; plt.close(fig)
print("5.4 PASS — sweep with deembed")
```

---

## 6  Plotting — Timestream (`plotting/timestream.py`)

### 6.1  Timestream IQ vs t
```python
from souk_readout_tools.plotting import plot_timestream

ts = {
    'i_data': {'0000': np.random.randn(5000)},
    'q_data': {'0000': np.random.randn(5000)},
    'sample_rate': 488.0,
}
fig = plot_timestream(ts, format='iq_vs_t')
assert fig is not None; plt.close(fig)
print("6.1 PASS — timestream iq_vs_t")
```

### 6.2  Timestream magphase
```python
fig = plot_timestream(ts, format='magphase')
assert fig is not None; plt.close(fig)
print("6.2 PASS — timestream magphase")
```

### 6.3  Timestream PSD
```python
from souk_readout_tools.plotting import plot_timestream_psd

fig = plot_timestream_psd(ts, format='iq')
assert fig is not None; plt.close(fig)
print("6.3 PASS — timestream PSD")
```

### 6.4  Timestream on resonance
```python
from souk_readout_tools.plotting import plot_timestream_on_resonance

# Needs sweep_data with matching tone — use per-tone shape
sweep_1t = {
    'sweep_f': np.linspace(5e9-1e6, 5e9+1e6, 100).reshape(-1, 1),
    'sweep_i': np.random.randn(100, 1),
    'sweep_q': np.random.randn(100, 1),
}
fig = plot_timestream_on_resonance(ts, sweep_1t, tone_index=0)
assert fig is not None; plt.close(fig)
print("6.4 PASS — timestream on resonance")
```

---

## 7  Plotting — Snapshots (`plotting/snapshot.py`)

### 7.1  Single-tone snapshots — concatenate
```python
from souk_readout_tools.plotting import plot_snapshots

fs_snap = 2.4e9 / 4096
snap = {
    'snapshots': np.random.randn(5, 1024) + 1j * np.random.randn(5, 1024),
    'sample_rate': fs_snap,
    'tone_index': 0,
}
fig = plot_snapshots(snap, format='iq_vs_t', repetitions='concatenate')
assert fig is not None; plt.close(fig)
print("7.1 PASS — snapshots concatenate")
```

### 7.2  Snapshot PSD — averaged with error bars
```python
from souk_readout_tools.plotting import plot_snapshots_psd

fig = plot_snapshots_psd(snap, method='averaged', show_errors=True)
assert fig is not None; plt.close(fig)
print("7.2 PASS — snapshot PSD averaged")
```

### 7.3  Batch snapshots
```python
from souk_readout_tools.plotting import plot_batch_snapshots

batch = {
    'results': {
        0: {'snapshots': np.random.randn(3, 1024) + 1j * np.random.randn(3, 1024),
            'sample_rate': fs_snap},
        1: {'snapshots': np.random.randn(3, 1024) + 1j * np.random.randn(3, 1024),
            'sample_rate': fs_snap},
    },
    'tone_frequencies': {0: 5.0e9, 1: 5.1e9},
}
fig = plot_batch_snapshots(batch, format='iq_vs_t')
assert fig is not None; plt.close(fig)
print("7.3 PASS — batch snapshots")
```

### 7.4  Batch snapshots with PSD column
```python
fig = plot_batch_snapshots(batch, format='iq_vs_t', psd=True)
assert fig is not None; plt.close(fig)
print("7.4 PASS — batch snapshots with PSD")
```

---

## 8  Measurement Framework (`measurement.py`)

### 8.1  ParameterSweep — mock client
```python
from souk_readout_tools.measurement import ParameterSweep, MeasurementPoint, save_measurement
import tempfile, os

class MockClient:
    pass

values_set = []
client = MockClient()
ps = ParameterSweep(
    client, 'attenuation_dB',
    set_parameter=lambda v: values_set.append(v),
    get_parameter=lambda: values_set[-1],
)

results = ps.sweep(
    values=[0, 5, 10, 15],
    measure_func=lambda c: {'power': np.random.randn()},
)
assert len(results) == 4
assert values_set == [0, 5, 10, 15]
assert all(isinstance(r, MeasurementPoint) for r in results)
print("8.1 PASS — ParameterSweep")
```

### 8.2  TimedMeasurement
```python
from souk_readout_tools.measurement import TimedMeasurement

tm = TimedMeasurement(client, interval_s=0.01)
results = tm.run(
    measure_func=lambda c: {'val': 42},
    n_points=3,
)
assert len(results) == 3
print("8.2 PASS — TimedMeasurement")
```

### 8.3  ConditionalMeasurement
```python
from souk_readout_tools.measurement import ConditionalMeasurement

call_count = [0]
def get_temp():
    call_count[0] += 1
    return 100 + call_count[0] * 10  # 110, 120, 130, ...

cm = ConditionalMeasurement(
    client, 'temperature_mK', get_temp,
    condition=lambda v: True,  # trigger every poll
    poll_interval_s=0.001,
)
results = cm.run(
    measure_func=lambda c: {'temp': 42},
    n_points=3,
)
assert len(results) == 3
print("8.3 PASS — ConditionalMeasurement")
```

### 8.4  save_measurement
```python
with tempfile.NamedTemporaryFile(suffix='.npz', delete=False) as tmp:
    save_measurement(results, tmp.name)
    assert os.path.exists(tmp.name)
    loaded = np.load(tmp.name, allow_pickle=True)
    os.unlink(tmp.name)
print("8.4 PASS — save_measurement")
```

---

## 9  Calibration Renames (`calibration.py`)

### 9.1  Renamed parameters exist
```python
import inspect
from souk_readout_tools import calibration

src = inspect.getsource(calibration)
assert 'tx_bypass_amp_s21_db' in src, "rename tx_amp -> tx_bypass_amp missing"
assert 'rx_bypass_amp_s21_db' in src or 'rx' not in src.lower().split('amp')[0], "rx rename check"
print("9.1 PASS — calibration renames present")
```

---

## 10  Client Changes (`readout_client.py`)

### 10.1  batch_snapshots method exists
```python
from souk_readout_tools.client.readout_client import ReadoutClient
assert hasattr(ReadoutClient, 'batch_snapshots'), "batch_snapshots method missing"
print("10.1 PASS — batch_snapshots exists")
```

### 10.2  find_resonances with mode='targeted' signature
```python
import inspect
sig = inspect.signature(ReadoutClient.find_resonances)
assert 'mode' in sig.parameters, "mode parameter missing from find_resonances"
print("10.2 PASS — find_resonances has mode parameter")
```

### 10.3  sync_config_from_system exists
```python
assert hasattr(ReadoutClient, 'sync_config_from_system'), "sync_config_from_system missing"
print("10.3 PASS — sync_config_from_system exists")
```

### 10.4  get_rf_peripheral_status exists
```python
assert hasattr(ReadoutClient, 'get_rf_peripheral_status'), "get_rf_peripheral_status missing"
print("10.4 PASS — get_rf_peripheral_status exists")
```

---

## 11  Firmware Library (`firmware_lib.py`)

### 11.1  New functions exist
```python
from souk_readout_tools import firmware_lib
for func in ['get_system_information', 'check_rfdc_rts_events']:
    assert hasattr(firmware_lib, func), f"{func} missing"
print("11.1 PASS — firmware_lib new functions present")
```

---

## 12  CLI Entry Points

### 12.1  Help output
```bash
python -m souk_readout_tools.client.client_scripts.batch_snapshots --help
python -m souk_readout_tools.client.client_scripts.find_resonances --help
python -m souk_readout_tools.client.client_scripts.wideband_sweep --help
```
Each should print usage and exit 0.

### 12.2  setup.py entry points parse
```python
# Verify entry point strings are importable paths
import importlib
for mod in [
    'souk_readout_tools.client.client_scripts.batch_snapshots',
    'souk_readout_tools.client.client_scripts.find_resonances',
]:
    m = importlib.import_module(mod)
    assert hasattr(m, 'main'), f"{mod} has no main()"
print("12.2 PASS — CLI entry points importable")
```

---

## 13  RF Peripherals (`server/rf_peripherals.py`)

### 13.1  Import and class existence
```python
from souk_readout_tools.server.rf_peripherals import RFPeripheralController, RudatAdapter
assert hasattr(RFPeripheralController, 'get_status')
assert hasattr(RFPeripheralController, 'apply_config')
assert hasattr(RFPeripheralController, 'attenuator_backend')
assert hasattr(RFPeripheralController, 'set_tx_attenuation')
assert hasattr(RFPeripheralController, 'set_rx_attenuation')
assert hasattr(RFPeripheralController, 'set_tx_amp_bypass')
assert hasattr(RFPeripheralController, 'set_rx_amp_bypass')
assert hasattr(RFPeripheralController, 'get_transfer')
print("13.1 PASS — RFPeripheralController importable with full API")
```

### 13.2  Disabled controller (no hardware needed)
```python
cfg = {
    'rf_frontend': {
        'mixerless_module': {'enabled': False},
    }
}
ctrl = RFPeripheralController(cfg, pipeline_id=0)
assert ctrl.enabled == False
status = ctrl.get_status()
assert status == {'enabled': False}
print("13.2 PASS — disabled controller returns correct status")
```

### 13.3  Mimic fallback (no smbus2 hardware)
```python
cfg_mimic = {
    'rf_frontend': {
        'mixerless_module': {
            'enabled': True,
            'attenuator_backend': 'mixerless',
            'i2c_bus': 0,
            'channel': 0,
        },
    }
}
ctrl_mimic = RFPeripheralController(cfg_mimic, pipeline_id=0)
# On a machine without smbus2 hardware, falls back to mimic
if not ctrl_mimic.is_hardware:
    assert ctrl_mimic.attenuator_backend == 'mimic'
    ctrl_mimic.set_tx_attenuation(10.0)
    assert ctrl_mimic.get_tx_attenuation() == 10.0
    ctrl_mimic.set_rx_attenuation(5.5)
    assert ctrl_mimic.get_rx_attenuation() == 5.5
    status = ctrl_mimic.get_status()
    assert status['enabled'] == True
    assert status['tx_attenuation_db'] == 10.0
    assert status['rx_attenuation_db'] == 5.5
    print("13.3 PASS — mimic backend set/get attenuation")
else:
    print("13.3 SKIP — real hardware detected, mimic test not applicable")
```

### 13.4  apply_config updates hardware state
```python
cfg_apply = {
    'rf_frontend': {
        'tx_attenuator_value_db': 15.0,
        'rx_attenuator_value_db': 8.0,
        'mixerless_module': {
            'enabled': True,
            'attenuator_backend': 'mixerless',
            'i2c_bus': 0,
            'channel': 0,
            'tx_amp_bypass': True,
            'rx_amp_bypass': False,
        },
    }
}
ctrl_ap = RFPeripheralController(cfg_apply, pipeline_id=0)
if not ctrl_ap.is_hardware:
    ctrl_ap.apply_config(cfg_apply)
    assert ctrl_ap.get_tx_attenuation() == 15.0
    assert ctrl_ap.get_rx_attenuation() == 8.0
    print("13.4 PASS — apply_config sets attenuation from config dict")
else:
    print("13.4 SKIP — real hardware detected")
```

### 13.5  Invalid backend raises ValueError
```python
cfg_bad = {
    'rf_frontend': {
        'mixerless_module': {
            'enabled': True,
            'attenuator_backend': 'nonexistent',
        },
    }
}
try:
    RFPeripheralController(cfg_bad, pipeline_id=0)
    assert False, "should have raised ValueError"
except ValueError as e:
    assert 'nonexistent' in str(e)
    print("13.5 PASS — invalid backend raises ValueError")
```

### 13.6  RudatAdapter interface
```python
# Verify the adapter class has the expected interface methods
assert hasattr(RudatAdapter, 'set_attenuation')
assert hasattr(RudatAdapter, 'get_attenuation_value')
assert hasattr(RudatAdapter, 'set_amp_bypass_state')
assert hasattr(RudatAdapter, 'get_amp_bypass_state')
assert hasattr(RudatAdapter, 'get_transfer')
print("13.6 PASS — RudatAdapter has required interface")
```

**(HW)** Full RF peripheral testing with real I2C or RUDAT hardware is covered
in the hardware test plan (`hardware_test_plan_v1.1.0.md`, Stage 2).

---

## 14  RUDAT Driver (`server/rudat.py`)

### 14.1  Import and class existence
```python
from souk_readout_tools.server.rudat import Attenuator, find_rudats
assert hasattr(Attenuator, 'att')
assert hasattr(Attenuator, 'get_model')
assert hasattr(Attenuator, 'get_serial')
assert hasattr(Attenuator, 'get_firmware_version')
assert hasattr(Attenuator, 'get_status')
assert hasattr(Attenuator, 'set_params')
assert hasattr(Attenuator, 'params')
assert hasattr(Attenuator, 'describe')
assert callable(find_rudats)
print("14.1 PASS — rudat imports and API present")
```

### 14.2  Attenuation quantisation
```python
from souk_readout_tools.server.rudat import _quantize_att

# Exact value
assert _quantize_att(10.0, 0.25, 0.0, 30.0) == 10.0
# Round to nearest step
assert _quantize_att(10.1, 0.25, 0.0, 30.0) == 10.0
assert _quantize_att(10.13, 0.25, 0.0, 30.0) == 10.25
# Clamp to range
assert _quantize_att(-5.0, 0.25, 0.0, 30.0) == 0.0
assert _quantize_att(50.0, 0.25, 0.0, 30.0) == 30.0
# Combined clamp + round
assert _quantize_att(30.3, 0.25, 0.0, 30.0) == 30.0
print("14.2 PASS — _quantize_att")
```

### 14.3  TX packet construction
```python
from souk_readout_tools.server.rudat import _tx_packet, TX_BUF_SIZE, SET_ATTENUATION

pkt = _tx_packet(SET_ATTENUATION, [10, 2])
assert len(pkt) == TX_BUF_SIZE
assert pkt[0] == SET_ATTENUATION
assert pkt[1] == 10
assert pkt[2] == 2
assert pkt[3] == 0  # padded with nulls

# Empty payload
pkt2 = _tx_packet(0x28)
assert len(pkt2) == TX_BUF_SIZE
assert pkt2[0] == 0x28
assert pkt2[1] == 0

# Verify oversized payload raises
try:
    _tx_packet(0x00, list(range(TX_BUF_SIZE + 1)))
    assert False, "should have raised ValueError"
except ValueError:
    pass
print("14.3 PASS — _tx_packet construction")
```

### 14.4  Attenuator.set_params
```python
# Instantiation will fail without USB hardware, so test set_params on
# a partially constructed object
att = Attenuator.__new__(Attenuator)
att.att_min = 0.0
att.att_max = 30.0
att.resolution = 0.25
att.set_params(att_min=1.0, att_max=20.0, resolution=0.5)
assert att.att_min == 1.0
assert att.att_max == 20.0
assert att.resolution == 0.5
print("14.4 PASS — Attenuator.set_params")
```

### 14.5  Attenuator.params returns expected keys
```python
att2 = Attenuator.__new__(Attenuator)
att2._model = "RUDAT-6000-30"
att2._serial = "12345"
att2._firmware = "1.0"
att2._bus = 1
att2._addr = 5
att2.vid = 0x20CE
att2.pid = 0x0023
att2.att_min = 0.0
att2.att_max = 30.0
att2.resolution = 0.25
p = att2.params()
expected_keys = {'model', 'serial', 'firmware', 'bus', 'address',
                 'vid', 'pid', 'att_min', 'att_max', 'resolution'}
assert set(p.keys()) == expected_keys, f"unexpected keys: {set(p.keys())}"
assert p['model'] == "RUDAT-6000-30"
assert p['serial'] == "12345"
print("14.5 PASS — Attenuator.params keys")
```

**(HW)** Full RUDAT hardware testing (USB enumeration, read/write attenuation)
is covered in the hardware test plan (`hardware_test_plan_v1.1.0.md`,
Stage 0 test 0.9 and Stage 2 test 2.7).

---

## 15  Config Template (`template_config.yaml`)

### 15.1  Template loads and has expected sections
```python
import yaml
from importlib_resources import files

cfg_text = (files('souk_readout_tools.data.config') / 'template_config.yaml').read_text()
cfg = yaml.safe_load(cfg_text)
assert 'rf_frontend' in cfg, "rf_frontend section missing"
assert 'mixerless_module' in cfg['rf_frontend'], "mixerless_module section missing"
print("15.1 PASS — config template loads with rf_frontend and mixerless_module")
```

### 15.2  Attenuator backend and RUDAT fields present
```python
mm = cfg['rf_frontend']['mixerless_module']
assert 'attenuator_backend' in mm, "attenuator_backend key missing"
assert mm['attenuator_backend'] == 'mixerless', f"default should be 'mixerless', got '{mm['attenuator_backend']}'"
# RUDAT serial fields should be present in the raw text (commented out)
assert 'rudat_tx_serial' in cfg_text, "rudat_tx_serial not in template (even as comment)"
assert 'rudat_rx_serial' in cfg_text, "rudat_rx_serial not in template (even as comment)"
print("15.2 PASS — attenuator_backend and RUDAT fields present")
```

### 15.3  Firmware paths updated for new OS image
```python
fw_path = cfg['firmware']['fw_config_file']
assert '/home/casper/src/' not in fw_path, f"old path found: {fw_path}"
assert '/home/casper/souk-firmware/' in fw_path, f"expected new path, got: {fw_path}"
print("15.3 PASS — firmware path updated")
```

---

## 16  Documentation Consistency

### 16.1  getting_started.md references valid modules
```bash
# Check that all module references in getting_started.md correspond to real files
grep -oP 'souk_readout_tools\.\w+' doc/getting_started.md | sort -u | while read mod; do
    modpath="src/${mod//.//}.py"
    moddir="src/${mod//.//}/"
    if [ ! -f "$modpath" ] && [ ! -d "$moddir" ]; then
        echo "WARN: $mod referenced in docs but not found"
    fi
done
```

### 16.2  CLI tool names match setup.py
```bash
# Manually verify the entry point names in setup.py match doc references
grep 'souk-' setup.py
grep 'souk-' doc/getting_started.md
```

### 16.3  No stale paths in source or docs
```bash
# Check that old paths have been updated
grep -rn '/home/casper/src/' src/ setup.py README.md doc/*.md --include='*.py' --include='*.yaml' --include='*.md' --include='*.service' | grep -v 'doc/archive/' && echo "FAIL — old paths found" || echo "16.3 PASS — no stale /home/casper/src/ paths"
grep -rn 'py38venv' src/ setup.py README.md doc/*.md --include='*.py' --include='*.yaml' --include='*.md' --include='*.service' | grep -v 'doc/archive/' && echo "FAIL — old venv paths found" || echo "16.3b PASS — no stale py38venv paths"
```

---

## 17  Integration Tests **(HW)**

These require a connected RFSoC with firmware loaded.

- [ ] `souk-connection-test -C config.yaml` — connects and reports system info
- [ ] `souk-wideband_sweep -C config.yaml -P` — sweep completes, plot renders
- [ ] `souk-batch-snapshots -C config.yaml --tones 0,1 -n 5 -P` — snapshots acquired with correct firmware index mapping
- [ ] `souk-find-resonances -C config.yaml --fit -P` — sweep + find + fit + plot
- [ ] Client `sync_config_from_system()` — pulls config and reports changes
- [ ] Client `get_rf_peripheral_status()` — returns attenuator/switch state
- [ ] `find_resonances(mode='targeted')` — returns per_tone, all_resonances, flagged_tones dicts
- [ ] Plotting functions with real sweep/timestream data produce sensible figures
- [ ] Server handles `get_system_information` and `check_rfdc_rts_events` requests
- [ ] `push_config()` / `pull_config()` round-trip on a fresh server

---

## Running the full offline suite

Paste all code blocks from sections 1–15 into a single script, or run them individually. Expected output: all lines print `PASS`. Any assertion error indicates a bug to investigate before deployment.
