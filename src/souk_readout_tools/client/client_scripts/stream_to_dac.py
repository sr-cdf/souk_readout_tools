#!/usr/bin/env python3

"""
Stream a chosen KID tone's response to an analog DAC output in real time.

Receives the readout server's continuous TCP stream, converts one or more
tones' IQ into a scalar (magnitude / phase / frequency shift) and drives a
USB DAC (LabJack T-series, or a no-hardware dummy backend) so external
instruments can record a live analog voltage proportional to the KID
response. See doc/stream_to_dac.md and
src/souk_readout_tools/data/config/template_stream_to_dac_config.yaml.
"""

import sys
import os
import argparse
import signal

import yaml

from souk_readout_tools import stream_dac
from souk_readout_tools.client import readout_client


def handle_signal(signum, frame):
    """Handles incoming signals and exits gracefully."""
    print(f"\nReceived signal {signum}. Exiting gracefully...")
    sys.exit(0)


DEFAULT_CONFIG = {
    'connection': {'config_file': None, 'address': None, 'request_port': None},
    'conversion': {'mode': 'magnitude', 'reference_iq': None,
                   'reference_samples': 100, 'dphi_df': None,
                   'calibration_file': None, 'output': 'frequency'},
    'channels': [{'dac_channel': 'DAC0', 'tone_index': 0,
                  'tone_frequency_hz': None, 'gain': 1.0, 'x_offset': 0.0,
                  'v_offset': 0.0, 'v_min': 0.0, 'v_max': 5.0}],
    'dac': {'backend': 'dummy', 'update_rate_hz': 200.0, 'idle_voltage': 0.0,
            'labjack': {}, 'ain_channels': []},
    'stream': {'queue_depth': None, 'watchdog_s': 5.0,
               'apply_readout_correction': True},
    'recording': {'directory': './tmp', 'filename': 'stream_to_dac',
                  'ain_log': None},
    'status_interval_s': 1.0,
}


def load_config(path):
    """Load the stream-to-dac YAML config, filling in defaults section-wise."""
    config = {key: (dict(value) if isinstance(value, dict) else value)
              for key, value in DEFAULT_CONFIG.items()}
    if path is not None:
        with open(path) as f:
            loaded = yaml.safe_load(f) or {}
        for key, value in loaded.items():
            if isinstance(value, dict) and isinstance(config.get(key), dict):
                config[key].update(value)
            else:
                config[key] = value
    return config


def make_client(config, args, parser):
    """Build a ReadoutClient from CLI args or the config's connection section."""
    connection = config['connection']
    config_file = args.config_file or connection.get('config_file')
    address = args.address or connection.get('address')
    request_port = args.request_port or connection.get('request_port')
    if config_file is not None:
        if args.address is not None or args.request_port is not None:
            parser.error('--config_file cannot be used with --address or '
                         '--request_port')
        return readout_client.ReadoutClient(config_file=config_file)
    if address is not None and request_port is not None:
        client = readout_client.ReadoutClient(address=address,
                                              request_port=int(request_port))
        client.pull_config()
        client.stream_server_port = client.config['rfsoc_host']['stream_port']
        return client
    parser.error('Either --config_file or both --address and --request_port '
                 'are required (CLI or config connection section), '
                 'unless --mock or --replay is used')


def selftest(config):
    """DAC0 -> AIN0 loopback latency check (needs a real LabJack + a wire)."""
    import time
    import numpy as np
    dac = stream_dac.make_dac_backend('labjack',
                                      config['dac'].get('labjack'))
    if isinstance(dac, stream_dac.DummyDac):
        print('selftest: no LabJack available, nothing to measure')
        return 1
    dac_channel = config['channels'][0].get('dac_channel', 'DAC0')
    ain_channel = (config['dac'].get('ain_channels') or ['AIN0'])[0]
    print(f'selftest: wire {dac_channel} to {ain_channel}, measuring '
          'write->read latency over 100 toggles')
    delays, errors = [], []
    try:
        for i in range(100):
            target = 1.0 if i % 2 else 2.5
            t0 = time.monotonic()
            dac.write({dac_channel: target})
            reading = dac.read_ain([ain_channel])[0]
            delays.append(time.monotonic() - t0)
            errors.append(reading - target)
        delays = np.asarray(delays)
        errors = np.asarray(errors)
        print(f'write+read transaction: mean {1e3*delays.mean():.2f} ms, '
              f'median {1e3*np.median(delays):.2f} ms, '
              f'max {1e3*delays.max():.2f} ms')
        print(f'loopback error: mean {1e3*errors.mean():+.1f} mV, '
              f'rms {1e3*errors.std():.1f} mV '
              '(large errors mean the wire is missing)')
    finally:
        dac.write({dac_channel: 0.0})
        dac.close()
    return 0


def main():
    signal.signal(signal.SIGINT, handle_signal)   # Handle Ctrl+C
    signal.signal(signal.SIGTERM, handle_signal)  # Handle termination

    parser = argparse.ArgumentParser(
        description=('Convert a live readout stream into an analog DAC '
                     'output. Connect using either a local readout config '
                     'file, or an address/request-port pair (as '
                     'souk-receive-stream does); script settings come from '
                     'a separate --config YAML file with CLI overrides for '
                     'the common knobs.'))
    parser.add_argument('-C', '--config_file', type=str, default=None,
                        help='Path to a local readout configuration file.')
    parser.add_argument('-a', '--address', type=str, default=None,
                        help='Readout server address. Must be used with '
                             '--request_port if --config_file is not given; '
                             'the remote config will be pulled from this '
                             'server.')
    parser.add_argument('-r', '--request_port', type=int, default=None,
                        help='Readout server request port. Must be used with '
                             '--address if --config_file is not given.')
    parser.add_argument('--config', type=str, default=None,
                        help='Path to the stream-to-dac YAML config file '
                             '(see template_stream_to_dac_config.yaml).')
    parser.add_argument('--tone', type=int, default=None,
                        help='Override: user-order tone index for the first '
                             'output channel.')
    parser.add_argument('--mode', type=str, default=None,
                        choices=['magnitude', 'phase', 'df_modelfree',
                                 'df_calibrated', 'ffm'],
                        help='Override: conversion mode.')
    parser.add_argument('--rate', type=float, default=None,
                        help='Override: DAC update rate in Hz.')
    parser.add_argument('--backend', type=str, default=None,
                        choices=['dummy', 'labjack'],
                        help='Override: DAC backend.')
    parser.add_argument('--replay', type=str, default=None,
                        help='Replay a recorded stream file (with .json '
                             'sidecar) instead of connecting to a server.')
    parser.add_argument('--mock', action='store_true',
                        help='Use the emulated readout server '
                             '(ReadoutClient(mock=True)); no hardware.')
    parser.add_argument('--max-updates', type=int, default=None,
                        help='Stop after this many DAC updates (testing).')
    parser.add_argument('--selftest', action='store_true',
                        help='LabJack DAC->AIN loopback latency check, then '
                             'exit (see doc/stream_to_dac.md).')
    parser.add_argument('-q', '--quiet', action='store_true',
                        help='Suppress the ~1 Hz status line.')

    args = parser.parse_args()
    config = load_config(args.config)

    # CLI overrides for the common knobs.
    if args.mode is not None:
        config['conversion']['mode'] = args.mode
    if args.rate is not None:
        config['dac']['update_rate_hz'] = args.rate
    if args.backend is not None:
        config['dac']['backend'] = args.backend
    if args.tone is not None:
        config['channels'][0]['tone_index'] = args.tone
        config['channels'][0]['tone_frequency_hz'] = None

    if args.selftest:
        sys.exit(selftest(config))

    # --- frame source + system info ------------------------------------
    if args.replay is not None:
        source = stream_dac.ReplayFrameSource(args.replay)
        info = source.info
        sample_rate = source.sample_rate
        num_tones = source.num_tones
    elif args.mock:
        client = readout_client.ReadoutClient(mock=True)
        source = stream_dac.MockFrameSource(client)
        info = client.get_info('all')
        sample_rate = client.get_sample_rate()
        num_tones = len(info['tones']['firmware_indices'])
    else:
        client = make_client(config, args, parser)
        info = client.get_info('all')
        sample_rate = client.get_sample_rate()
        tone_indices = info.get('tones', {}).get('firmware_indices')
        num_tones = len(tone_indices) if tone_indices is not None else 2048
        source = stream_dac.SocketFrameSource(client.stream_server_address,
                                              client.stream_server_port)

    # --- output channels ------------------------------------------------
    mode = config['conversion']['mode']
    channels = []
    for channel_config in config['channels']:
        position = stream_dac.resolve_tone_position(
            info,
            tone_index=channel_config.get('tone_index'),
            tone_frequency_hz=channel_config.get('tone_frequency_hz'))
        channels.append(stream_dac.OutputChannel(
            channel_config.get('dac_channel', 'DAC0'), position,
            converter=None,   # filled in below (may need a reference)
            gain=channel_config.get('gain', 1.0),
            x_offset=channel_config.get('x_offset', 0.0),
            v_offset=channel_config.get('v_offset', 0.0),
            v_min=channel_config.get('v_min', 0.0),
            v_max=channel_config.get('v_max', 5.0)))

    dac = stream_dac.make_dac_backend(config['dac']['backend'],
                                      config['dac'].get('labjack'))

    recording = config['recording']
    recording_path = None
    if recording.get('filename'):
        recording_path = os.path.join(recording.get('directory', '.'),
                                      recording['filename'])

    app = stream_dac.StreamToDac(
        source, info, sample_rate, num_tones, channels, dac,
        dac_update_rate=config['dac']['update_rate_hz'],
        idle_voltage=config['dac']['idle_voltage'],
        queue_depth=config['stream']['queue_depth'],
        watchdog_s=config['stream']['watchdog_s'],
        recording_path=recording_path,
        ain_channels=config['dac'].get('ain_channels') or (),
        ain_log_path=recording.get('ain_log'),
        status_interval=config.get('status_interval_s', 1.0),
        quiet=args.quiet,
        apply_readout_correction=config['stream']['apply_readout_correction'],
        max_updates=args.max_updates)

    print(f'stream-to-dac: mode={mode} backend={config["dac"]["backend"]} '
          f'update_rate={config["dac"]["update_rate_hz"]} Hz '
          f'boxcar N={app.navg} '
          f'tones={[ch.tone_position for ch in channels]}')

    # Start receiving, then build converters (phase modes may need a
    # startup-captured reference from the live stream).
    app.start_reader()
    try:
        references = [None] * len(channels)
        if mode in ('phase', 'df_modelfree') and \
                config['conversion'].get('reference_iq') is None:
            n_ref = int(config['conversion'].get('reference_samples', 100))
            print(f'Capturing reference IQ ({n_ref} samples)...')
            references = app.capture_reference_iq(n_ref)
            for ch, ref in zip(channels, references):
                print(f'  {ch.dac_channel}: reference IQ = {ref:.1f}')
        for ch, ref in zip(channels, references):
            ch.converter = stream_dac.build_converter(
                mode, config['conversion'], info, ch.tone_position,
                reference_iq=ref)
    except Exception:
        app.safe_state('startup failed')
        source.close()
        dac.close()
        raise

    app.run_output()


if __name__ == "__main__":
    main()
