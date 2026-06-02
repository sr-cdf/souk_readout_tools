#!/usr/bin/env python3
"""
Mock-mode verification for fast frequency modulation (no hardware).

Run with the src tree on the path so it exercises the working copy::

    PYTHONPATH=src python src/souk_readout_tools/client/client_scripts/test_frequency_modulation.py

Covers (plan verification steps 1 & 2): arm != stream, the 1..N point tag with
dwell + settling, gap-free packet counter, uint32 flag5 decode, live update +
revision, per-(tone,point) bin occupancy + recenter, blind/index handling,
pause/resume, the demod tool (dphi/df, d2phi/df2 NaN for 2 points, detuning +
needs_update, fast vs accurate, dissipation needs circle_cal), and
modulation_params_from_sweep.
"""
import os
import sys
import warnings
import numpy as np

warnings.filterwarnings('ignore')   # silence the all-zero-phase crest-factor notice
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, os.pardir)))

from souk_readout_tools.client.readout_client import ReadoutClient
from souk_readout_tools import demod

_failures = []


def check(name, cond):
    print(f'  [{"PASS" if cond else "FAIL"}] {name}')
    if not cond:
        _failures.append(name)


def _resonator_z(f, f0, w, scale=1000.0):
    """Same model the mock uses (for the synthetic sweep)."""
    x = 2.0 * (np.asarray(f, float) - np.asarray(f0, float)) / w
    return scale * (1.0 - 0.9 / (1.0 + x ** 2)) * np.exp(1j * (-2.0 * np.arctan(x)))


def make_client(f0, sample_rate=1000.0, linewidth=1.0e5):
    c = ReadoutClient(mock=True)
    ms = c._mock_server
    ms.tone_frequencies = list(np.asarray(f0, float))
    ms._resize_tone_state(len(f0))
    ms.sample_rate = float(sample_rate)
    ms._mod_linewidth = float(linewidth)
    return c, ms


F0 = np.array([2.0e9, 2.1e9])
W = 1.0e5


print('1) arm != stream, tag structure, gap-free counter')
c, ms = make_client(F0, linewidth=W)
ack = c.enable_modulation(center=F0, offsets=[-1e3, 0.0, 1e3], samples_per_point=4, n_settle=1)
check('enable returns success', ack.get('status') == 'success')
check('enable arms only (mock not streaming)', ms.is_streaming is False)
st = c.get_modulation_state()
check('state enabled after arm', st['enabled'] is True)
check('num_points = 3', st['num_points'] == 3)
check('mod_indices defaulted to all tones', st['mod_indices'] == [0, 1])
d = c.parse_samples(c.get_samples(60))
pts = np.asarray(d['modulation_point'])
check('points are 1..N (never 0 while modulating)', set(np.unique(pts)) == {1, 2, 3})
check('dwell pattern 4 samples/point', list(pts[:12]) == [1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3])
check('settling marks first sample of each point', list(np.asarray(d['modulation_settling'])[:8]) == [1, 0, 0, 0, 1, 0, 0, 0])
check('packet counter monotonic & gap-free', bool(np.all(np.diff(d['packet_counter']) == 1)))
i0 = d['i_data']['0000']; q0 = d['q_data']['0000']
z0 = i0 + 1j * q0
check('IQ differs across points (tone 0 modulated)', np.std(np.abs(z0)) > 0 or np.std(np.angle(z0)) > 1e-6)


print('2) uint32 flag5 decode (revision can set the sign bit)')
point, settling, rev = 3, 1, 0x7ABC
tag = (point & 0xFFFF) | (settling << 16) | ((rev & 0x7FFF) << 17)
as_i4 = np.int32(np.uint32(tag))                 # how it travels in the int32 frame
f5 = np.uint32(as_i4)                             # decode side casts back to unsigned
check('point decodes', int(f5 & 0xFFFF) == point)
check('settling decodes', int((f5 >> 16) & 1) == settling)
check('revision decodes (no sign corruption)', int((f5 >> 17) & 0x7FFF) == rev)
check('raw int32 was negative (proves unsigned decode needed)', int(as_i4) < 0)


print('3) live update bumps revision; new centre takes effect')
rev_before = c.get_modulation_state()['desired_revision']
ack = c.update_modulation(center=F0 + 5e3)
check('update success', ack.get('status') == 'success')
rev_after = c.get_modulation_state()['desired_revision']
check('revision incremented on update', rev_after == rev_before + 1)
d2 = c.parse_samples(c.get_samples(30))
check('frames carry the new revision', int(np.unique(d2['modulation_revision'])[-1]) == rev_after)


print('4) per-(tone,point) bin occupancy + recenter')
c, ms = make_client(np.array([2.0e9]), linewidth=W)   # single tone on a bin centre
ms._mod_bin_hz = 1.0e6
c.enable_modulation(center=[2.0e9], offsets=[-1e3, 0.0, 1e3], samples_per_point=2, n_settle=1)
# Nudge the centre ~0.6 of a bin: still covered by the overlapping neighbour.
c.update_modulation(center=[2.0e9 + 6e5])
occ = c.get_modulation_state()['tones'][0]['occupancy']
check("0.6-bin drift reported as 'second' (rides overlap)", 'second' in occ and 'beyond' not in occ)
# Push ~1.2 bins: beyond coverage -> update rejected by default.
ack = c.update_modulation(center=[2.0e9 + 1.2e6])
check('beyond-coverage update is rejected', ack.get('status') == 'error')
check('rejection reports tones_beyond_coverage', 0 in ack.get('result', {}).get('tones_beyond_coverage', []))
ackr = c.recenter_modulation()
check('recenter succeeds', ackr.get('status') == 'success')
check('recenter clears needs_recenter', c.get_modulation_state()['needs_recenter'] is False)


print('5) index consistency: modulate one tone, only that column moves')
c, ms = make_client(F0, linewidth=W)
c.enable_modulation(center=F0, offsets=[-2e3, 0.0, 2e3], mod_indices=[0], samples_per_point=3, n_settle=1)
d = c.parse_samples(c.get_samples(60))
z0 = d['i_data']['0000'] + 1j * d['q_data']['0000']
z1 = d['i_data']['0001'] + 1j * d['q_data']['0001']
check('modulated tone 0 phase varies', np.ptp(np.angle(z0)) > 1e-3)
check('unmodulated tone 1 phase is ~constant', np.ptp(np.angle(z1)) < 1e-6)


print('6) pause / resume')
c.disable_modulation()
check('disable clears enabled flag', c.get_modulation_state()['enabled'] is False)
d = c.parse_samples(c.get_samples(12))
check('paused stream is untagged (point 0)', int(np.max(d['modulation_point'])) == 0)
c.enable_modulation()   # resume resident config, no args
check('resume re-enables', c.get_modulation_state()['enabled'] is True)
d = c.parse_samples(c.get_samples(12))
check('resumed stream is tagged again', int(np.max(d['modulation_point'])) >= 1)


print('7) demod tool: slope / curvature / detuning')
c, ms = make_client(F0, linewidth=W)
# Centred 3-point capture.
c.enable_modulation(center=F0, offsets=[-1e3, 0.0, 1e3], samples_per_point=8, n_settle=2)
state = c.get_modulation_state()
d = c.parse_samples(c.get_samples(3 * 8 * 12))
grouped = demod.group_modulation_cycles(d, state, reduce='mean')
res_fast = demod.demodulate(grouped, linewidth_hz=W, method='fast')
res_acc = demod.demodulate(grouped, linewidth_hz=W, method='accurate')
slope = np.nanmean(res_fast['dphi_df'][:, 0])
check('dphi/df matches model slope -4/w', np.isclose(slope, -4.0 / W, rtol=0.05))
check('d2phi/df2 finite for 3 points', np.isfinite(res_fast['d2phi_df2'][:, 0]).all())
check('detuning ~0 when centred', np.nanmean(np.abs(res_fast['detuning_linewidths'][:, 0])) < 0.05)
check('needs_update False when centred', not res_fast['needs_update'][:, 0].any())
check('fast vs accurate dphi/df agree', np.isclose(np.nanmean(res_acc['dphi_df'][:, 0]), slope, rtol=0.05))
check('dissipation NaN without circle_cal', np.isnan(res_fast['dissipation'][:, 0]).all())
check('grouped z shape (n_cycles, N, n_tones)', grouped['z'].ndim == 3 and grouped['z'].shape[1] == 3)
g_axis = demod.group_modulation_cycles(d, state, reduce=None)
check('reduce=None retains sample axis', g_axis['z'].ndim == 4)

# Two-point pattern -> curvature/detuning are NaN.
c.enable_modulation(center=F0, offsets=[-1e3, 1e3], samples_per_point=8, n_settle=2)
state2 = c.get_modulation_state()
d = c.parse_samples(c.get_samples(2 * 8 * 12))
g2 = demod.group_modulation_cycles(d, state2)
r2 = demod.demodulate(g2, linewidth_hz=W, method='fast')
check('d2phi/df2 NaN for 2 points', np.isnan(r2['d2phi_df2'][:, 0]).all())
check('detuning NaN for 2 points', np.isnan(r2['detuning_linewidths'][:, 0]).all())
r2nl = demod.demodulate(g2, linewidth_hz=None, method='fast')
check('detuning NaN without linewidth', np.isnan(r2nl['detuning_linewidths'][:, 0]).all())

# Detune the centre -> needs_update trips.
c.enable_modulation(center=F0, offsets=[-1e3, 0.0, 1e3], samples_per_point=8, n_settle=2)
c.update_modulation(center=F0 + 0.2 * W)        # ~0.2 linewidths off resonance
state3 = c.get_modulation_state()
d = c.parse_samples(c.get_samples(3 * 8 * 12))
g3 = demod.group_modulation_cycles(d, state3)
r3 = demod.demodulate(g3, linewidth_hz=W, method='fast')
det = np.nanmean(r3['detuning_linewidths'][:, 0])
check('detuning detected when off-resonance (>0.1 lw)', abs(det) > 0.1)
check('needs_update trips when detuned', r3['needs_update'][:, 0].any())


print('8) modulation_params_from_sweep -> ready-to-use config')
fw = 8.0e4
sweep_f = np.stack([np.linspace(f - 5 * fw, f + 5 * fw, 201) for f in F0], axis=1)
sweep_z = np.stack([_resonator_z(sweep_f[:, t], F0[t], fw) for t in range(len(F0))], axis=1)
cfg = demod.modulation_params_from_sweep({'f': sweep_f, 'z': sweep_z},
                                         n_points=3, samples_per_point=4, n_settle=1,
                                         delta_linewidths=0.25)
check('center ~ true f0', np.allclose(cfg['center'], F0, atol=2 * (sweep_f[1, 0] - sweep_f[0, 0])))
check('linewidth estimate within 25% of truth', np.all(np.abs(cfg['linewidth_hz'] - fw) / fw < 0.25))
check('offsets shape (n_points, n_mod)', cfg['offsets'].shape == (3, len(F0)))
check('delta scaled ~0.25*linewidth', np.allclose(np.max(cfg['offsets'], axis=0), 0.25 * cfg['linewidth_hz'], rtol=0.05))
c, ms = make_client(F0, linewidth=fw)
ack = c.enable_modulation(center=cfg['center'], offsets=cfg['offsets'],
                          mod_indices=cfg['mod_indices'],
                          samples_per_point=cfg['samples_per_point'], n_settle=cfg['n_settle'])
check('sweep-derived config arms successfully', ack.get('status') == 'success')
print(cfg['summary'])


print()
if _failures:
    print(f'RESULT: {len(_failures)} FAILED -> {_failures}')
    sys.exit(1)
print('RESULT: ALL CHECKS PASSED')
