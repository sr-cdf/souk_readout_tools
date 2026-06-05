#!/usr/bin/env python3
"""Synthetic regression checks for canonical resonator IQ conversion."""

import os
import sys
from types import SimpleNamespace

import numpy as np

sys.path.append(os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.pardir, os.pardir, os.pardir)))

from souk_readout_tools import fitting
from souk_readout_tools.client.readout_client import ReadoutClient
from souk_readout_tools.noise import (
    fractional_frequency_and_dissipation_timestreams,
)
from souk_readout_tools.resonator import (
    DeembeddingCalibration,
    ResonatorCalibration,
    linearized_frequency_and_dissipation,
    model_phase_center_geometry,
)


_failures = []


def check(name, condition):
    """Print and record one assertion."""
    print(f'  [{"PASS" if condition else "FAIL"}] {name}')
    if not condition:
        _failures.append(name)


def _calibration(fr, Qi, Qc, phi, gain, gain_phase, tau, anl=0.0):
    """Build an exact calibration for the synthetic notch model."""
    Qe = fitting.complex_coupling_q(Qc, phi)
    Ql = fitting.loaded_q(Qi, Qc, phi)
    phase_center = model_phase_center_geometry(Ql, Qe)
    origin = np.imag(1.0 / Qe) / 2.0 if anl == 0.0 else 0.0
    return ResonatorCalibration(
        fr,
        Ql,
        tau,
        phase_center.center,
        phase_center.radius,
        phase_center.rotation_angle,
        gain_amplitude=gain,
        gain_phase=gain_phase,
        frequency_origin_fraction=origin,
        anl=anl,
    )


print('1) Exact asymmetric linear inversion')
fr = 2.000123e9
Qi, Qc, phi = 8e4, 4e4, 0.21
gain, gain_phase, tau = 1.2, 0.7, 31.7e-9
Ql = fitting.loaded_q(Qi, Qc, phi)
offsets = np.array([-1.0, -0.3, 0.0, 0.3, 1.0]) * fr / Ql
frequencies = fr + offsets
z = fitting.s21_model(
    frequencies, fr, Qi, Qc, phi, gain, gain_phase, tau)
cal = _calibration(fr, Qi, Qc, phi, gain, gain_phase, tau)
df, dd = cal.convert_raw_iq(frequencies, z)
check('physical detuning recovered', np.allclose(df, offsets, atol=1e-6))
check('model-circle dissipation is zero', np.allclose(dd, 0.0, atol=1e-12))
matched_loss = np.array([-0.08, -0.03, 0.0, 0.03, 0.08]) / Ql
Qi_loss = 1.0 / (1.0 / Qi + 2.0 * matched_loss)
z_loss = np.array([
    fitting.s21_model(frequency, fr, qi, Qc, phi, gain, gain_phase, tau)
    for frequency, qi in zip(frequencies, Qi_loss)
])
df_loss, dd_loss = cal.convert_raw_iq(frequencies, z_loss)
_, radial_loss = cal.convert_raw_iq(frequencies, z_loss, method='circle')
check('asymmetric matched dissipation recovered',
      np.allclose(dd_loss, matched_loss, atol=1e-12))
check('asymmetric frequency recovered with simultaneous loss',
      np.allclose(df_loss, offsets, atol=1e-6))
check('diagnostic radial proxy remains separately available',
      not np.allclose(radial_loss, matched_loss))


print('2) Fixed-tone and explicit timestream transforms')
converter = cal.tone_converter(frequencies[1])
df_tone, dd_tone = converter(z[1])
check('fixed-tone conversion matches array conversion',
      np.allclose([df_tone, dd_tone], [df[1], dd[1]], atol=1e-9))
try:
    cal.deembed_timestream(z[1:])
except ValueError:
    check('timestream transform requires probe frequency', True)
else:
    check('timestream transform requires probe frequency', False)
check('explicit timestream and raw-IQ transforms match',
      np.allclose(
          cal.deembed_timestream(z[1:], frequency=frequencies[1:]),
          cal.transform_raw_iq(frequencies[1:], z[1:]),
          atol=1e-12,
      ))


print('3) Exact Duffing inversion')
anl = 0.37
z_nonlinear = fitting.s21_model(
    frequencies, fr, Qi, Qc, phi, gain, gain_phase, tau, anl=anl)
cal_nonlinear = _calibration(
    fr, Qi, Qc, phi, gain, gain_phase, tau, anl=anl)
df_nonlinear, dd_nonlinear = cal_nonlinear.convert_raw_iq(
    frequencies, z_nonlinear)
df_circle, _ = cal_nonlinear.convert_raw_iq(
    frequencies, z_nonlinear, method='circle')
check('Duffing physical detuning recovered',
      np.allclose(df_nonlinear, offsets, atol=1e-6))
check('Duffing circle coordinate remains available for diagnostics',
      np.max(np.abs(df_circle - offsets)) > 1e3)
check('Duffing model-circle dissipation is zero',
      np.allclose(dd_nonlinear, 0.0, atol=1e-12))
z_nonlinear_loss = np.array([
    fitting.s21_model(
        frequency, fr, qi, Qc, phi, gain, gain_phase, tau, anl=anl)
    for frequency, qi in zip(frequencies, Qi_loss)
])
df_nonlinear_loss, dd_nonlinear_loss = cal_nonlinear.convert_raw_iq(
    frequencies, z_nonlinear_loss)
check('Duffing matched dissipation recovered',
      np.allclose(dd_nonlinear_loss, matched_loss, atol=1e-12))
check('Duffing frequency recovered with simultaneous loss',
      np.allclose(df_nonlinear_loss, offsets, atol=1e-6))
check('Duffing fixed-tone conversion matches array conversion',
      np.isclose(
          cal_nonlinear.tone_converter(frequencies[3])(z_nonlinear[3])[0],
          df_nonlinear[3],
          atol=1e-6,
      ))
fit_like = SimpleNamespace(
    fr=fr,
    Ql=cal_nonlinear.Ql,
    Qi=Qi,
    Qc=Qc,
    phi=phi,
    Qe=fitting.complex_coupling_q(Qc, phi),
    a=gain,
    alpha=gain_phase,
    tau=tau,
    anl=anl,
    sweep_direction='up',
    iq_center_deembed=cal_nonlinear.center,
    iq_radius_deembed=cal_nonlinear.radius,
    phase_center_rotation_angle=cal_nonlinear.rotation_angle,
)
df_from_fit, _ = ResonatorCalibration.from_fit(fit_like).convert_raw_iq(
    frequencies, z_nonlinear)
check('Duffing from_fit calibration recovers physical detuning',
      np.allclose(df_from_fit, offsets, atol=1e-6))


print('4) Reused group-delay deembedding')
group_delay_cal = {
    'frequencies': np.array([1.9e9, 2.0e9, 2.1e9]),
    'tau_ns': np.array([10.0, 15.0, 25.0]),
}
deembedding = DeembeddingCalibration(
    scale=0.7 * np.exp(0.2j),
    group_delay_cal=group_delay_cal,
    group_delay_reference_frequency=1.9e9,
)
group_frequencies = np.array([1.95e9, 2.0e9, 2.05e9])
expected = np.array([1.0 + 2.0j, 3.0 - 1.0j, -0.2 + 0.5j])
raw = expected / deembedding.multiplier(group_frequencies)
check('sweep group-delay correction round-trips',
      np.allclose(deembedding.apply(group_frequencies, raw), expected))
check('fixed-tone group-delay correction shares sweep reference',
      np.allclose(deembedding.apply(group_frequencies[1], raw[1]), expected[1]))


print('5) Legacy linearized wrapper and public noise utility')
linearized = linearized_frequency_and_dissipation(
    frequencies, z, fr, z, smooth_window_hz=None, return_calibration=True)
legacy = ReadoutClient.calculate_frequency_and_dissipation_noise(
    frequencies, z, fr, z, smooth_window_hz=None)
check('legacy client wrapper matches canonical linearized conversion',
      all(np.allclose(old, new) for old, new in zip(
          legacy,
          (
              linearized[0],
              linearized[1],
              linearized[2].reference_iq.real,
              linearized[2].reference_iq.imag,
              linearized[2].gradient.real,
              linearized[2].gradient.imag,
          ),
      )))

# Simulate resonator motion around a deliberately detuned fixed readout tone.
# The public noise utility should remove that static probe-detuning offset and
# report detector resonance motion with the historical timestream sign.
noise_sweep_f = np.linspace(fr - fr / Ql, fr + fr / Ql, 401)
noise_sweep_z = fitting.s21_model(
    noise_sweep_f, fr, Qi, Qc, phi, gain, gain_phase, tau)
probe = fr + 0.3 * fr / Ql
detector_motion = np.array([-0.02, 0.0, 0.02]) * fr / Ql
z_motion_raw = np.array([
    fitting.s21_model(
        probe, fr + shift, Qi, Qc, phi, gain, gain_phase, tau)
    for shift in detector_motion
])
sweep_data = {
    'sweep_f': noise_sweep_f[:, np.newaxis],
    'sweep_i': noise_sweep_z.real[:, np.newaxis],
    'sweep_q': noise_sweep_z.imag[:, np.newaxis],
}
ts_data = {
    'i_data': {'0000': z_motion_raw.real},
    'q_data': {'0000': z_motion_raw.imag},
    'info': {'tones': {'frequencies_hz': [probe]}},
}
converted = fractional_frequency_and_dissipation_timestreams(
    ts_data, sweep_data, method='mobius', calibrations={0: cal})
circle_converted = fractional_frequency_and_dissipation_timestreams(
    ts_data, sweep_data, method='circle', calibrations={0: cal})
linear_converted = fractional_frequency_and_dissipation_timestreams(
    ts_data, sweep_data, method='linearized', smooth_window_hz=None)
check('public noise utility removes static fitted probe detuning',
      np.isclose(converted['frequency'][0, 1], 0.0, atol=1e-15))
check('public noise utility reports detector-motion sign',
      np.allclose(
          converted['frequency'][0] * probe, detector_motion, atol=0.1))
check('diagnostic circle frequency uses detector-motion sign',
      np.allclose(
          circle_converted['frequency'][0] * probe, detector_motion, atol=0.1))
check('fitted and local noise-frequency quadratures agree',
      np.allclose(
          converted['frequency'][0],
          linear_converted['frequency'][0],
          rtol=0.03,
          atol=1e-15,
      ))


if _failures:
    print(f'\nFAILED: {len(_failures)} check(s):')
    for failure in _failures:
        print(f'  - {failure}')
    raise SystemExit(1)

print('\nAll resonator conversion checks passed.')
