#!/usr/bin/env python3
"""
Measure the combined TX (PSB) + RX (PFB) filterbank channel response in loopback.

Steps probe tones across a full channel with the **software modulation engine**
-- which holds the channel maps fixed at the armed bins, exactly the geometry
the response describes (a sweep re-snaps every point to its nearest bin and
cannot measure this) -- and reduces the tagged capture to a gain/phase versus
bin-offset table (``filterbank_response.py``). Run it in **digital loopback**
for the pure filterbank cascade (should be zero-phase), and in **RF loopback**
to include the analog group-delay phase slope and near-edge behaviour.

Each tone's probe ladder is centred on **its own armed bin centre** (the
tone's sub-bin residual is measured first and subtracted), so every tone spans
the same +/-``span_bins`` regardless of where its centre frequency landed.

Example usage:
    souk-measure-filterbank-response -C config.yaml -P
    souk-measure-filterbank-response -a 10.11.11.11 --port 10000 --label rf-loopback
    PYTHONPATH=src python src/souk_readout_tools/client/client_scripts/measure_filterbank_response.py --mock -P

Programmatic usage:
    from souk_readout_tools.client.client_scripts.measure_filterbank_response import (
        measure_filterbank_response)
    table = measure_filterbank_response(config_file='config.yaml')
"""

import argparse
import datetime
import os
import sys

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, os.pardir)))

from souk_readout_tools.client.readout_client import ReadoutClient
from souk_readout_tools import modulation as mod
from souk_readout_tools import filterbank_response as fbr


def probe_bin_geometry(client, mod_indices=None, probe_hz=1000.0):
    """
    Measure the bin spacing and each tone's sub-bin centre residual.

    Arms software modulation with a two-point ladder ``[0, probe_hz]`` (arm
    only -- no data is taken) and reads the per-tone ``drift_bins`` back from
    the modulation state: the zero-offset drift is the tone's centre residual
    from its armed bin centre, and the drift step measures the bin spacing.
    Disables modulation again before returning.

    Returns (bin_spacing_hz, {tone_index: residual_bins}).
    """
    ack = client.enable_modulation(offsets=[0.0, float(probe_hz)], mod_indices=mod_indices,
                                   samples_per_point=2, n_settle=0, engine='sw')
    if ack.get('status') != 'success':
        raise RuntimeError(f'probe arm failed: {ack}')
    state = client.get_modulation_state()
    client.disable_modulation(engine='sw')
    residuals = {}
    steps = []
    for tone in state.get('tones', []):
        d = np.asarray(tone.get('drift_bins', []), dtype=float)
        if len(d) != 2:
            raise RuntimeError(f"tone {tone.get('index')}: no drift_bins in the "
                               'modulation state; server too old?')
        residuals[int(tone['index'])] = float(d[0])
        steps.append(d[1] - d[0])
    if not steps:
        raise RuntimeError('no modulated tones in the probe state')
    bin_spacing_hz = float(probe_hz) / float(np.mean(steps))
    return bin_spacing_hz, residuals


def measure_filterbank_response(config_file=None, address=None, request_port=None,
                                client=None, mock=False,
                                span_bins=0.98, n_points=65, samples_per_point=4,
                                n_settle=1, n_cycles=8, mod_indices=None,
                                label='', filename=None, plot_data=False):
    """
    Measure the combined filterbank channel response and save it as a table.

    Connect using a config file, an address + request port, an existing
    ``client``, or ``mock=True`` (self-validating: the measured table is
    checked against the mock's known resonator model).

    Args:
        span_bins (float): Probe span, +/- bin spacings about each tone's armed
            bin centre. Must stay below 1.0 (the coverage limit, where a tone
            wraps). Default 0.98 -- essentially the full channel.
        n_points (int): Probe points across the span (default 65).
        samples_per_point (int): Accumulations per point (default 4).
        n_settle (int): Leading settling accumulations per point (default 1).
        n_cycles (int): Modulation cycles to average (default 8).
        mod_indices: Tones to probe; None = all regular tones.
        label (str): Free-text tag written into the table header (e.g.
            'digital-loopback', 'rf-loopback').
        filename (str): Output table path. None derives a timestamped name.
        plot_data (bool): Plot the per-tone and combined response (saved as a
            PNG next to the table).

    Returns the response table dict (see
    ``filterbank_response.response_from_grouped``), with the saved path in
    ``table['path']``.
    """
    if not 0 < span_bins < 1.0:
        raise ValueError('span_bins must be in (0, 1): beyond 1.0 a tone is no '
                         f'longer covered by its armed bin (got {span_bins})')
    if client is None:
        if mock:
            client = _make_mock_client()
        elif config_file is not None:
            client = ReadoutClient(config_file=config_file)
        elif address is not None:
            if request_port is None:
                raise ValueError('request_port is required with address')
            client = ReadoutClient(address=address, request_port=request_port)
        else:
            raise ValueError('give config_file, address+request_port, client or mock=True')

    client.disable_stream()                     # arming rewrites channel maps

    # Bin spacing + per-tone centre residuals, so each tone's ladder can be
    # centred on its own armed bin centre.
    bin_spacing_hz, residuals = probe_bin_geometry(client, mod_indices=mod_indices)
    print(f'bin spacing: {bin_spacing_hz/1e3:.3f} kHz; probing {len(residuals)} tone(s), '
          f'+/-{span_bins} bins, {n_points} points, residuals '
          f'{min(residuals.values()):+.3f}..{max(residuals.values()):+.3f} bins')

    grid = np.linspace(-span_bins, span_bins, int(n_points))
    tone_order = sorted(residuals)              # server order for (n_points, n_mod) offsets
    offsets = np.stack([(grid - residuals[i]) * bin_spacing_hz for i in tone_order], axis=1)

    ack = client.enable_modulation(offsets=offsets, mod_indices=tone_order,
                                   samples_per_point=samples_per_point,
                                   n_settle=n_settle, engine='sw')
    if ack.get('status') != 'success':
        raise RuntimeError(f'enable_modulation failed: {ack}')
    try:
        num = int(n_cycles) * int(n_points) * int(samples_per_point)
        data = client.parse_samples(client.get_samples(num))
        state = client.get_modulation_state()
        grouped = mod.group_cycles(data, state)
        table = fbr.response_from_grouped(grouped, state)
    finally:
        client.disable_modulation(engine='sw')

    print(f"measured {table['n_tones_used']} tone(s) x {table['n_cycles']} cycle(s); "
          f"gain at span edges: {table['gain_db'][0]:+.3f} / {table['gain_db'][-1]:+.3f} dB; "
          f"tone-to-tone spread <= {np.max(table['gain_db_std']):.4f} dB")

    if filename is None:
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'filterbank_response_{stamp}.txt'
    meta = {'bin_spacing_hz': f'{bin_spacing_hz:.6f}',
            'span_bins': span_bins, 'samples_per_point': samples_per_point,
            'n_settle': n_settle}
    if label:
        meta['label'] = label
    if mock:
        meta['label'] = (meta.get('label', '') + ' mock').strip()
    fbr.save_response(filename, table, meta=meta)
    table['path'] = filename
    print(f'saved {filename}')

    if mock:
        _validate_against_mock(client, table, bin_spacing_hz)
    if plot_data:
        _plot_table(table, filename)
    return table


def _make_mock_client(f0=(2.0e9, 2.1e9), sample_rate=1000.0):
    """Mock client with tones parked on the mock resonators (bin centres)."""
    client = ReadoutClient(mock=True)
    ms = client._mock_server
    ms.tone_frequencies = list(np.asarray(f0, dtype=float))
    ms._resize_tone_state(len(f0))
    ms.acc_len = ms._acc_len_for_rate(float(sample_rate))
    return client


def _validate_against_mock(client, table, bin_spacing_hz):
    """
    Check the measured table against the mock's known response.

    The mock synthesizes each tone's IQ from its resonator model at the probe
    frequency, so the expected "channel response" here is the resonator shape
    normalised at the tone centre -- a smooth, non-trivial gain/phase curve
    that exercises the whole pipeline (tagging, grouping, drift axes,
    normalisation, resampling). Interpolation is the only error source, so the
    tolerances are tight.
    """
    ms = client._mock_server
    ok = True
    for k, idx in enumerate(table['tone_indices']):
        f_center = float(ms.tone_frequencies[idx])
        # Tone offsets are relative to the armed bin centre; convert to Hz
        # about the tone centre using its residual (~0 here: tones on centres).
        f = f_center + table['tone_offset_bins'][k] * bin_spacing_hz
        expected = (ms._resonator_z(f, f_center, ms._mod_linewidth)
                    / ms._resonator_z(f_center, f_center, ms._mod_linewidth))
        gain_err = np.max(np.abs(table['tone_gain_db'][k] - 20 * np.log10(np.abs(expected))))
        phase_err = np.max(np.abs(table['tone_phase_rad'][k] - np.unwrap(np.angle(expected))))
        good = gain_err < 0.05 and phase_err < 0.01
        ok &= good
        print(f"  [{'PASS' if good else 'FAIL'}] tone {idx} matches mock model "
              f'(gain err {gain_err:.4f} dB, phase err {phase_err*1e3:.2f} mrad)')
    if not ok:
        raise RuntimeError('mock validation failed')


def _plot_table(table, filename):
    import matplotlib.pyplot as plt
    fig, (ax_g, ax_p) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    for k, idx in enumerate(table['tone_indices']):
        ax_g.plot(table['tone_offset_bins'][k], table['tone_gain_db'][k],
                  lw=0.8, alpha=0.5, label=f'tone {idx}')
        ax_p.plot(table['tone_offset_bins'][k], table['tone_phase_rad'][k],
                  lw=0.8, alpha=0.5)
    ax_g.plot(table['offset_bins'], table['gain_db'], 'k', lw=2, label='combined (median)')
    ax_p.plot(table['offset_bins'], table['phase_rad'], 'k', lw=2)
    for ax in (ax_g, ax_p):
        for x in (-0.5, 0.5):
            ax.axvline(x, color='grey', ls=':', lw=0.8)   # half-bin edges
    ax_g.set_ylabel('gain (dB)')
    ax_p.set_ylabel('phase (rad)')
    ax_p.set_xlabel('offset from armed bin centre (bin spacings)')
    ax_g.legend(fontsize='small')
    ax_g.set_title('combined TX+RX filterbank channel response')
    fig.tight_layout()
    png = os.path.splitext(filename)[0] + '.png'
    fig.savefig(png, dpi=100)
    print(f'saved {png}')
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-C', '--config_file', help='config YAML file')
    parser.add_argument('-a', '--address', help='RFSoC IP address (alternative to -C)')
    parser.add_argument('--port', type=int, dest='request_port',
                        help='request port (pipeline 0: 10000, pipeline 1: 10001)')
    parser.add_argument('--mock', action='store_true',
                        help='run against the mock server (self-validating)')
    parser.add_argument('--span-bins', type=float, default=0.98,
                        help='probe span, +/- bin spacings (default 0.98)')
    parser.add_argument('--n-points', type=int, default=65,
                        help='probe points across the span (default 65)')
    parser.add_argument('--samples-per-point', type=int, default=4,
                        help='accumulations per point (default 4)')
    parser.add_argument('--n-settle', type=int, default=1,
                        help='settling accumulations per point (default 1)')
    parser.add_argument('--n-cycles', type=int, default=8,
                        help='modulation cycles to average (default 8)')
    parser.add_argument('--mod-indices', type=int, nargs='+',
                        help='tone indices to probe (default: all regular tones)')
    parser.add_argument('--label', default='',
                        help="table header tag, e.g. 'digital-loopback'")
    parser.add_argument('-f', '--filename', help='output table path')
    parser.add_argument('-P', '--plot', action='store_true', dest='plot_data',
                        help='plot the measured response (PNG saved next to the table)')
    args = parser.parse_args()
    measure_filterbank_response(
        config_file=args.config_file, address=args.address,
        request_port=args.request_port, mock=args.mock,
        span_bins=args.span_bins, n_points=args.n_points,
        samples_per_point=args.samples_per_point, n_settle=args.n_settle,
        n_cycles=args.n_cycles, mod_indices=args.mod_indices,
        label=args.label, filename=args.filename, plot_data=args.plot_data)


if __name__ == '__main__':
    main()
