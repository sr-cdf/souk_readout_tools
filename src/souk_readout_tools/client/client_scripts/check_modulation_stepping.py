#!/usr/bin/env python3
"""
Check that modulation actually steps the LO: does the IQ change point-to-point?

Motivated by a loopback filterbank-response measurement that came back flat to
~0.001 dB / 0.1 mrad across a whole channel: the modulation *tags* cycled
(scheduler ran, buffers flipped) but the tone gave no evidence of ever moving.
This script arms each engine/offset-shape path in turn with one large probe
offset and reports the per-point gain and phase relative to point 1 -- an
unambiguous moved/static verdict per path:

  A   sw engine, 1-D offsets (broadcast)      -- the long-exercised path
  A2  sw engine, 2-D per-tone offsets         -- the path the response
                                                 measurement script uses
  B   fw engine, 4 LO slots (auto round-robin) -- hardware-validated on v7.11

In loopback a 250 kHz offset (~0.83 bin spacings) should show a few dB of
combined filterbank rolloff and/or a clear phase shift; identical IQ at every
point means that path's frequency words are not reaching the live LO.

Example usage:
    souk-check-modulation-stepping -C config.yaml
    PYTHONPATH=src python src/souk_readout_tools/client/client_scripts/check_modulation_stepping.py --mock
"""

import argparse
import os
import sys

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, os.pardir)))

from souk_readout_tools.client.readout_client import ReadoutClient

# A path is called MOVED if the point-to-point spread exceeds either of these;
# expected real responses are orders of magnitude above them.
GAIN_THRESHOLD_DB = 0.1
PHASE_THRESHOLD_RAD = 0.05


def _per_point_iq(client, num_points, samples_per_point, n_cycles):
    """Capture and reduce: mean IQ per (point, tone), settling dropped."""
    num = int(n_cycles) * int(num_points) * int(samples_per_point)
    data = client.parse_samples(client.get_samples(num))
    points = np.asarray(data['modulation_point'], dtype=int)
    settling = np.asarray(data['modulation_settling'], dtype=int)
    n_tones = int(data['num_tones'])
    z = np.stack([np.asarray(data['i_data'][f'{t:04d}'], dtype=float)
                  + 1j * np.asarray(data['q_data'][f'{t:04d}'], dtype=float)
                  for t in range(n_tones)], axis=1)
    out = np.full((num_points, n_tones), np.nan + 0j, dtype=complex)
    for p in range(1, num_points + 1):
        keep = (points == p) & (settling == 0)
        if np.any(keep):
            out[p - 1] = z[keep].mean(axis=0)
    return out


def _report(name, z_points, offsets_hz):
    """Print per-point gain/phase relative to point 1 and return the verdict."""
    ref = z_points[0]
    rel = z_points / ref[None, :]
    gain_db = 20 * np.log10(np.abs(rel))
    phase = np.angle(rel)
    med_gain = np.nanmedian(gain_db, axis=1)
    med_phase = np.nanmedian(phase, axis=1)
    span_gain = np.nanmax(np.abs(med_gain))
    span_phase = np.nanmax(np.abs(med_phase))
    moved = span_gain > GAIN_THRESHOLD_DB or span_phase > PHASE_THRESHOLD_RAD
    print(f'{name}:')
    for p, off in enumerate(offsets_hz):
        print(f'  point {p+1} ({off/1e3:+8.1f} kHz): {med_gain[p]:+8.3f} dB  '
              f'{med_phase[p]:+8.4f} rad  (median over tones, rel. point 1)')
    print(f'  -> {"MOVED" if moved else "STATIC"}  '
          f'(gain span {span_gain:.4f} dB, phase span {span_phase:.4f} rad)\n')
    return moved


def check_modulation_stepping(config_file=None, address=None, request_port=None,
                              client=None, mock=False, delta_hz=250e3,
                              samples_per_point=8, n_cycles=6):
    """Run the three stepping checks; returns {'A': bool, 'A2': bool, 'B': bool}."""
    if client is None:
        if mock:
            client = ReadoutClient(mock=True)
        elif config_file is not None:
            client = ReadoutClient(config_file=config_file)
        elif address is not None:
            if request_port is None:
                raise ValueError('request_port is required with address')
            client = ReadoutClient(address=address, request_port=request_port)
        else:
            raise ValueError('give config_file, address+request_port, client or mock=True')

    client.disable_stream()
    verdicts = {}

    # A: software engine, 1-D broadcast offsets.
    offs = [0.0, float(delta_hz)]
    ack = client.enable_modulation(offsets=offs, samples_per_point=samples_per_point,
                                   n_settle=1, engine='sw')
    if ack.get('status') != 'success':
        raise RuntimeError(f'A: sw enable failed: {ack}')
    z = _per_point_iq(client, len(offs), samples_per_point, n_cycles)
    n_mod = len(client.get_modulation_state().get('mod_indices', []))
    client.disable_modulation(engine='sw')
    verdicts['A'] = _report(f'A  sw engine, 1-D offsets [0, {delta_hz/1e3:.0f} kHz]',
                            z, offs)

    # A2: software engine, per-tone (n_points, n_mod) offsets -- same values in
    # every column, so any shape-handling bug shows up against A.
    offs2d = np.tile(np.asarray(offs)[:, None], (1, n_mod))
    ack = client.enable_modulation(offsets=offs2d, samples_per_point=samples_per_point,
                                   n_settle=1, engine='sw')
    if ack.get('status') != 'success':
        raise RuntimeError(f'A2: sw enable failed: {ack}')
    z = _per_point_iq(client, len(offs), samples_per_point, n_cycles)
    client.disable_modulation(engine='sw')
    verdicts['A2'] = _report(f'A2 sw engine, per-tone 2-D offsets (same values, {n_mod} tones)',
                             z, offs)

    # B: firmware-slot engine, 4 slots (needs exactly 4 offset rows in auto mode).
    offs4 = [0.0, delta_hz / 3.0, 2.0 * delta_hz / 3.0, float(delta_hz)]
    ack = client.enable_modulation(offsets=offs4, samples_per_point=samples_per_point,
                                   n_settle=1, engine='fw', mode='auto')
    if ack.get('status') != 'success':
        raise RuntimeError(f'B: fw enable failed: {ack}')
    z = _per_point_iq(client, len(offs4), samples_per_point, n_cycles)
    client.disable_modulation(engine='fw')
    verdicts['B'] = _report('B  fw engine, 4 LO slots (auto)', z, offs4)

    print('verdicts:', ', '.join(f'{k}={"MOVED" if v else "STATIC"}'
                                 for k, v in verdicts.items()))
    return verdicts


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-C', '--config_file', help='config YAML file')
    parser.add_argument('-a', '--address', help='RFSoC IP address (alternative to -C)')
    parser.add_argument('--port', type=int, dest='request_port',
                        help='request port (pipeline 0: 10000, pipeline 1: 10001)')
    parser.add_argument('--mock', action='store_true', help='run against the mock server')
    parser.add_argument('--delta-hz', type=float, default=250e3,
                        help='probe offset in Hz (default 250e3 ~ 0.83 bins)')
    parser.add_argument('--samples-per-point', type=int, default=8,
                        help='accumulations per point (default 8)')
    parser.add_argument('--n-cycles', type=int, default=6,
                        help='cycles to average (default 6)')
    args = parser.parse_args()
    check_modulation_stepping(
        config_file=args.config_file, address=args.address,
        request_port=args.request_port, mock=args.mock, delta_hz=args.delta_hz,
        samples_per_point=args.samples_per_point, n_cycles=args.n_cycles)


if __name__ == '__main__':
    main()
