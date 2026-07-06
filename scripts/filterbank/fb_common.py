"""
Shared helpers for the standalone filterbank test scripts in this directory.

These scripts configure the system from scratch (like scripts/timed_sync/):
they snapshot the state they are about to change, switch it, and restore it.
State touched: internal loopback, the digital gain parameters that
``maximise_tx_power(digital_only=True)`` may adjust, and the tone comb.

The digital-loopback guard matters: enabling internal loopback changes the
signal level into the DSP pipeline, risking FFT over/underflow, so
``enter_digital_loopback`` immediately re-optimises the digital gains
(``digital_only=True`` -- no RF/attenuator changes) and ``restore_system``
always puts the previous gains and loopback state back.

Measurements use a **single tone** at a time (repeated at several
frequencies): multiple simultaneous bin-centred tones generate
intermodulation products that can alias onto the measurement (and pairs of
tones an even number of bins apart alias onto each other's DC exactly).
"""

import time
import warnings

import numpy as np

from souk_readout_tools.client.readout_client import ReadoutClient
from souk_readout_tools import firmware_lib

# Digital gain parameters maximise_tx_power(digital_only=True) may adjust.
# Snapshot/restore is defensive: parameters a server build doesn't expose are
# skipped with a notice.
GAIN_PARAMS = ('psb_scale', 'psb_fftshift', 'pfb_fftshift')

# 1 dB (power) below full scale: the drive-restoring TX headroom convention.
TEST_TONE_AMPLITUDE = 10.0 ** (-1.0 / 20.0)


def add_connection_args(parser):
    parser.add_argument('-C', '--config_file', help='config YAML file')
    parser.add_argument('-a', '--address', help='RFSoC IP address (alternative to -C)')
    parser.add_argument('--port', type=int, dest='request_port',
                        help='request port (pipeline 0: 10000, pipeline 1: 10001)')


def connect(args):
    if getattr(args, 'config_file', None):
        return ReadoutClient(config_file=args.config_file)
    if getattr(args, 'address', None):
        if args.request_port is None:
            raise SystemExit('--port is required with --address')
        return ReadoutClient(address=args.address, request_port=args.request_port)
    raise SystemExit('give -C config.yaml or -a address --port N')


def snapshot_system(client):
    """Record everything these scripts may change, for restore_system()."""
    snap = {'gain': {}}
    snap['internal_loopback'] = client.get_parameter('internal_loopback')
    for name in GAIN_PARAMS:
        value = client.get_parameter(name)
        if isinstance(value, dict):        # error response: parameter not exposed
            print(f'  (cannot snapshot {name}; will not restore it)')
        else:
            snap['gain'][name] = value
    snap['tone_frequencies'] = np.asarray(client.get_tone_frequencies(), dtype=float)
    snap['tone_amplitudes'] = np.asarray(client.get_tone_amplitudes(), dtype=float)
    snap['tone_phases'] = np.asarray(client.get_tone_phases(), dtype=float)
    print(f"snapshot: loopback={snap['internal_loopback']}, "
          f"gains={ {k: v for k, v in snap['gain'].items()} }, "
          f"{len(snap['tone_frequencies'])} tone(s)")
    return snap


def enter_digital_loopback(client):
    """
    Enable internal (digital) loopback and re-optimise the digital gains.

    Loopback changes the DSP input level, so the pipeline can over/underflow
    on the previous gain settings; digital_only=True keeps the optimisation
    away from RF attenuators/amps.
    """
    client.set_parameter('internal_loopback', True)
    print('internal loopback ENABLED; re-optimising digital gains '
          '(maximise_tx_power(digital_only=True, rx_policy="maximise"))')
    client.maximise_tx_power(digital_only=True, rx_policy='maximise')


def restore_system(client, snap, restore_tones=False):
    """Put back loopback + digital gains (always) and the comb (optional)."""
    client.disable_modulation()
    for name, value in snap['gain'].items():
        client.set_parameter(name, value)
    client.set_parameter('internal_loopback', bool(snap['internal_loopback']))
    print(f"restored digital gains + internal_loopback={bool(snap['internal_loopback'])}")
    if restore_tones and len(snap['tone_frequencies']):
        client.set_tone_frequencies(snap['tone_frequencies'])
        client.set_tone_amplitudes(snap['tone_amplitudes'])
        client.set_tone_phases(snap['tone_phases'])
        print(f"restored the previous {len(snap['tone_frequencies'])}-tone comb")
    elif not restore_tones:
        print('NOTE: the test tone comb is left loaded (re-run with --restore, '
              'or reload your comb, before science use)')


def set_test_tone(client, freq_hz, amplitude=TEST_TONE_AMPLITUDE):
    """Load a single test tone (one tone: no intermod, no alias collisions)."""
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')     # single tone: crest factor moot
        client.set_tone_frequencies([float(freq_hz)])
        client.set_tone_amplitudes([float(amplitude)])
        client.set_tone_phases([0.0])


def measure_ladder(client, span_bins=0.98, n_points=65, samples_per_point=4,
                   n_settle=1, n_cycles=8, compensate_filterbank=False):
    """
    Measure the single loaded tone's complex response across its channel.

    Steps the software modulation engine (fixed channel maps -- the geometry
    the channel response describes) across ``+/-span_bins`` about the tone's
    **armed bin centre** (its sub-bin residual is probed first and
    subtracted). ``compensate_filterbank=False`` (default) measures the raw
    response; ``True`` measures with the wired-in compensation applied.

    Returns a dict: ``offset_bins`` (true drift from the armed bin centre),
    ``gain_db`` / ``phase_rad`` (relative to the bin centre), ``z_rel``,
    ``bin_spacing_hz``.
    """
    # Probe the bin geometry: residual + spacing from a 2-point arm (no data).
    ack = client.enable_modulation(offsets=[0.0, 1000.0], samples_per_point=2,
                                   n_settle=0, engine='sw',
                                   compensate_filterbank=compensate_filterbank)
    if ack.get('status') != 'success':
        raise RuntimeError(f'probe arm failed: {ack}')
    tone = client.get_modulation_state()['tones'][0]
    drift = np.asarray(tone['drift_bins'], dtype=float)
    residual = float(drift[0])
    bin_spacing_hz = 1000.0 / float(drift[1] - drift[0])
    client.disable_modulation(engine='sw')

    grid = np.linspace(-span_bins, span_bins, int(n_points))
    ack = client.enable_modulation(offsets=(grid - residual) * bin_spacing_hz,
                                   samples_per_point=samples_per_point,
                                   n_settle=n_settle, engine='sw',
                                   compensate_filterbank=compensate_filterbank)
    if ack.get('status') != 'success':
        raise RuntimeError(f'ladder arm failed: {ack}')
    # Per-point software readout-flattening factors the server reports when
    # the compensation's RX part cannot be applied in hardware; None when
    # compensation is off (or a future firmware applies it in fabric). NOTE:
    # the raw per-point z returned below does NOT have them applied (the
    # normal data path applies them in modulation.group_cycles).
    readout_correction = client.get_modulation_state()['tones'][0].get(
        'readout_correction')
    try:
        num = int(n_cycles) * int(n_points) * int(samples_per_point)
        data = client.parse_samples(client.get_samples(num))
    finally:
        client.disable_modulation(engine='sw')

    points = np.asarray(data['modulation_point'], dtype=int)
    settling = np.asarray(data['modulation_settling'], dtype=int)
    z = (np.asarray(data['i_data']['0000'], dtype=float)
         + 1j * np.asarray(data['q_data']['0000'], dtype=float))
    z_points = np.full(int(n_points), np.nan + 0j, dtype=complex)
    for p in range(1, int(n_points) + 1):
        keep = (points == p) & (settling == 0)
        if np.any(keep):
            z_points[p - 1] = z[keep].mean()

    # Normalise at the bin centre (complex interpolation at drift 0).
    ref = (np.interp(0.0, grid, z_points.real)
           + 1j * np.interp(0.0, grid, z_points.imag))
    z_rel = z_points / ref
    phase = np.unwrap(np.angle(z_rel))
    phase -= np.interp(0.0, grid, phase)
    return {
        'offset_bins': grid,
        'z_rel': z_rel,
        'gain_db': 20.0 * np.log10(np.abs(z_rel)),
        'phase_rad': phase,
        'bin_spacing_hz': bin_spacing_hz,
        'readout_correction': (np.asarray(readout_correction, dtype=float)
                               if readout_correction is not None else None),
    }


def combine_tables(tables):
    """Median-combine repeats measured on the same offset grid."""
    grid = tables[0]['offset_bins']
    g = np.stack([t['gain_db'] for t in tables], axis=0)
    p = np.stack([t['phase_rad'] for t in tables], axis=0)
    return {
        'offset_bins': grid,
        'gain_db': np.median(g, axis=0),
        'phase_rad': np.median(p, axis=0),
        'gain_db_std': np.std(g, axis=0),
        'phase_rad_std': np.std(p, axis=0),
        'bin_spacing_hz': tables[0]['bin_spacing_hz'],
        'n_repeats': len(tables),
    }


def save_table(path, table, meta=None):
    header = ['souk filterbank channel response (combined TX+RX, relative to bin centre)',
              'columns: offset_bins gain_db phase_rad gain_db_std phase_rad_std',
              f'date: {time.strftime("%Y-%m-%dT%H:%M:%S")}',
              f"bin_spacing_hz: {table['bin_spacing_hz']:.6f}"]
    for key, value in (meta or {}).items():
        header.append(f'{key}: {value}')
    zeros = np.zeros_like(table['gain_db'])
    np.savetxt(path, np.column_stack([
        table['offset_bins'], table['gain_db'], table['phase_rad'],
        table.get('gain_db_std', zeros), table.get('phase_rad_std', zeros)]),
        fmt='%+.6e', header='\n'.join(header))
    print(f'saved {path}')


def fit_passband_delay(table, passband_bins=0.5):
    """Group delay (s) from the passband phase slope (tau = -slope/(2 pi B))."""
    x, p = table['offset_bins'], table['phase_rad']
    sel = np.abs(x) <= passband_bins
    slope = np.polyfit(x[sel], p[sel], 1)[0]
    return -slope / (2.0 * np.pi * table['bin_spacing_hz'])


def plot_table(table, png_path, model_delay_s=0.0, title=''):
    import matplotlib.pyplot as plt
    x = table['offset_bins']
    fig, (ax_g, ax_p) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    ax_g.plot(x, table['gain_db'], 'k', lw=1.5, label='measured')
    model = firmware_lib.filterbank_cascade_response(
        x, group_delay_s=model_delay_s, bin_spacing_hz=table['bin_spacing_hz'])
    ax_g.plot(x, 20 * np.log10(np.abs(model)), 'r--', lw=1.2,
              label='DPSS+sinc model incl. images')
    ax_p.plot(x, table['phase_rad'], 'k', lw=1.5)
    slope = -2.0 * np.pi * table['bin_spacing_hz'] * model_delay_s
    ax_p.plot(x, np.angle(model) + slope * x, 'r--', lw=1.2)
    for ax in (ax_g, ax_p):
        for edge in (-0.5, 0.5):
            ax.axvline(edge, color='grey', ls=':', lw=0.8)
    ax_g.set_ylabel('gain (dB)')
    ax_p.set_ylabel('phase (rad)')
    ax_p.set_xlabel('offset from armed bin centre (bin spacings)')
    ax_g.legend(fontsize='small')
    ax_g.set_title(title or 'combined TX+RX filterbank channel response')
    fig.tight_layout()
    fig.savefig(png_path, dpi=100)
    print(f'saved {png_path}')
    plt.show()


def default_test_frequencies(snap, n=3):
    """Spread test frequencies across the previously loaded comb's band."""
    freqs = snap['tone_frequencies']
    if len(freqs) == 0:
        raise SystemExit('no tones loaded to infer a band from; '
                         'pass --frequencies explicitly')
    lo, hi = float(np.min(freqs)), float(np.max(freqs))
    if n == 1 or hi == lo:
        return [0.5 * (lo + hi)]
    return list(np.linspace(lo, hi, n))
