import argparse
import os
import sys


# Need to import to make sure user data (including the service file) is set up.
if 'souk_readout_server' in sys.modules:
    pass
else:
    import souk_readout_tools


def main():
    parser = argparse.ArgumentParser(description='Restart the SOUK readout server systemd daemon.')
    parser.add_argument('-p', '--pipeline', type=int, nargs='+', default=[0],
                        choices=[0, 1],
                        help='Pipeline ID(s) to restart (default: 0). '
                             'Use -p 0 1 for both pipelines.')
    args = parser.parse_args()

    pipeline_args = ' '.join(str(p) for p in args.pipeline)
    cmd = f'sudo /home/casper/.souk_readout_tools/daemon/restart_systemd_service.sh {pipeline_args}'
    os.system(cmd)


if __name__ == "__main__":
    main()
