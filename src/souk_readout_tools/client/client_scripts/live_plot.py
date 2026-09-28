"""Matplotlib viewer for existing stream files; --follow watches appended data."""

import argparse
import functools


def _connect(config_file=None, address=None, request_port=None, mock=False):
    """Build a ready ReadoutClient (module level so a receiver process can too)."""
    from souk_readout_tools.client.readout_client import ReadoutClient
    if mock:
        return ReadoutClient(mock=True)
    if config_file:
        return ReadoutClient(config_file=config_file)
    client = ReadoutClient(address=address, request_port=request_port)
    client.pull_config()
    client.stream_server_port = client.config['rfsoc_host']['stream_port']
    return client


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('-C', '--config-file', '--config_file', help='Readout configuration YAML')
    source.add_argument('-a', '--address', help='Readout server address (requires --request-port)')
    source.add_argument('--mock', action='store_true', help='Paced simulated live stream')
    source.add_argument('--file', help='Saved samples (.npy/.json/.csv)')
    source.add_argument('--stream-file', help='Existing binary stream capture basename; no hardware connection')
    parser.add_argument('--follow', action='store_true',
                        help='Follow new records in --stream-file, retaining the latest --history seconds')
    parser.add_argument('-r', '--request-port', '--request_port', type=int)
    parser.add_argument('--enable-stream', action='store_true',
                        help='Enable server streaming before connecting; leave it enabled on exit')
    parser.add_argument('--record', help='Also save the live stream at this basename')
    parser.add_argument('--duration', type=float, help='Stop receiving after this many seconds')
    parser.add_argument('--tones', default='0', help='User-tone indices: all, 0,2 or 0-3,8 (default 0)')
    parser.add_argument('--history', type=float, default=60, help='Retained live history in seconds')
    parser.add_argument('--span', type=float, default=10, help='Visible time-axis span in seconds')
    parser.add_argument('--psd-window', type=float, help='Seconds of history used for spectra (default all)')
    parser.add_argument('--nperseg', type=int, default=1024, help='Welch segment length')
    parser.add_argument('--interval', type=float, default=0.2, help='GUI refresh interval in seconds')
    parser.add_argument('--max-plot-points', type=int,
                        help='Opt-in extrema reduction per time trace (>=6); default draws every sample. '
                             'Retained points can be unevenly spaced; raw data and PSDs are unchanged')
    parser.add_argument('--max-memory-mb', type=float, default=512,
                        help='Raw rolling buffer limit in MiB; parsed arrays need additional memory')
    parser.add_argument('--x-axis', choices=('time', 'sample', 'telescope_time'), default='time')
    parser.add_argument('--format', choices=('iq', 'magphase', 'freq_diss', 'linewidths'),
                        default='magphase',
                        help="linewidths: frequency/dissipation times each tone's Qr "
                             '(needs --calibrations)')
    parser.add_argument('--signal', help='One signal instead of the pair selected by --format')
    parser.add_argument('--layout', choices=('time', 'psd', 'asd', 'time-psd', 'time-asd'), default='time-psd')
    parser.add_argument('--panels', nargs='+', help='Explicit row-major panels, e.g. amp:time phase:time')
    parser.add_argument('--columns', type=int, help='Panel columns (default 2 for four panels, otherwise 1)')
    parser.add_argument('--sweep', help='Saved calibration sweep (.npy/.json/.csv)')
    parser.add_argument('--calibrations',
                        help='Calibrations .npz saved by resonator.save_calibrations (older '
                             'stream_dac archives load too); its sweep is used when --sweep is '
                             'not given. Default: the calibration the stream carries')
    parser.add_argument('--data-mode', choices=('raw', 'demodulated'), default='raw',
                        help='Raw probe IQ or sample-aligned demodulation (default raw)')
    parser.add_argument('--demod-method', choices=('fast', 'accurate'), default='fast')
    parser.add_argument('--settling', choices=('none', 'raw'), default='none',
                        help='Demodulation: leave settling samples missing or convert their raw IQ')
    parser.add_argument('--linewidth-hz', type=float,
                        help='Resonator linewidth for model-free detuning (Python also accepts per-tone arrays)')
    parser.add_argument('--conversion',
                        choices=('linearized', 'phase', 'mobius', 'lookup', 'circle'),
                        default='linearized',
                        help='Fixed-tone conversion (resonator.prepare_conversion); for '
                             'demodulated data mobius selects the exact inversion')
    parser.add_argument('--backend', help='Matplotlib backend, e.g. QtAgg or TkAgg')
    return parser


def panel_options(args):
    if args.panels:
        panels = args.panels
    else:
        signals = [args.signal] if args.signal else {
            'iq': ['i', 'q'], 'magphase': ['amplitude', 'unwrapped_phase'],
            'freq_diss': ['fractional_resonance_shift', 'matched_dissipation_shift'],
            'linewidths': ['resonance_shift_linewidths', 'matched_dissipation_linewidths'],
        }[args.format]
        panels = [f'{signal}:{domain}' for signal in signals for domain in args.layout.split('-')]
    columns = args.columns if args.columns is not None else (2 if len(panels) == 4 else 1)
    if columns < 1 or len(panels) % columns:
        raise ValueError('--columns must divide the number of panels')
    return panels, (len(panels) // columns, columns)


def main(argv=None):
    parser = make_parser()
    args = parser.parse_args(argv)
    if args.follow and not args.stream_file:
        parser.error('--follow requires --stream-file')
    if args.address is not None and args.request_port is None:
        parser.error('--address requires --request-port')
    if args.request_port is not None and args.address is None:
        parser.error('--request-port is only used with --address')
    if (args.file or args.stream_file) and (args.record or args.duration or args.enable_stream):
        parser.error('--record, --duration and --enable-stream require a live source')

    if args.backend:
        import matplotlib
        matplotlib.use(args.backend)
    from souk_readout_tools.client.readout_client import ReadoutClient
    from souk_readout_tools.plotting.live import LivePlot

    viewer = None
    try:
        panels, layout = panel_options(args)
        sweep = ReadoutClient.import_sweep(args.sweep) if args.sweep else None
        calibrations = None
        if args.calibrations:
            from souk_readout_tools.resonator import load_calibration_record
            calibrations, saved_sweep = load_calibration_record(args.calibrations)
            sweep = sweep if sweep is not None else saved_sweep
        viewer = LivePlot(
            panels=panels, layout=layout, tones=args.tones,
            history_seconds=args.history, view_seconds=args.span,
            psd_seconds=args.psd_window, x_axis=args.x_axis,
            nperseg=args.nperseg, interval_ms=args.interval * 1000,
            max_plot_points=args.max_plot_points,
            data_mode=args.data_mode, demod_method=args.demod_method,
            settling=args.settling, linewidth_hz=args.linewidth_hz,
            max_bytes=int(args.max_memory_mb * 1024**2),
            sweep_data=sweep, calibrations=calibrations, conversion_method=args.conversion)
        if args.stream_file and args.follow:
            viewer.start_file(args.stream_file)
        elif args.file or args.stream_file:
            samples = (ReadoutClient.import_samples(args.file) if args.file
                       else ReadoutClient.parse_stream(args.stream_file))
            # Standard raw captures have counters but no explicit time axis.
            # Preserve losses in those files just as the live ring does.
            if ('packet_counter' in samples and 'packet_missing' not in samples
                    and not any(key in samples for key in ('time_s', 'resonance_shift_hz'))):
                from souk_readout_tools.timeseries import fill_gaps
                samples = fill_gaps(samples)
            viewer.update(samples)
            viewer.full_span()
        else:
            connect = functools.partial(
                _connect, config_file=args.config_file, address=args.address,
                request_port=args.request_port, mock=args.mock)
            client = connect()
            # Fetch metadata before enabling streaming to avoid a request-time
            # stall in the initial capture. The worker gets this snapshot.
            info = client.get_info('all')
            if args.enable_stream or args.mock:
                client.enable_stream()
            # Receive in a separate process so slow refreshes never drop frames.
            viewer.start_process(connect, filename=args.record,
                                 duration=args.duration, info=info)
        viewer.show()
        if viewer.error is not None:
            parser.exit(1, f'Stream failed: {viewer.error}\n')
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer.close()


if __name__ == '__main__':
    main()
