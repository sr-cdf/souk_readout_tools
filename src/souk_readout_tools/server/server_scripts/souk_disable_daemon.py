import argparse
import os
import sys


# Need to import to make sure user data (including the service file) is set up.
if 'souk_readout_server' in sys.modules:
    pass
else:
    import souk_readout_tools


def main():
    parser = argparse.ArgumentParser(description='Disable the SOUK readout server systemd daemon.')
    parser.add_argument('-p', '--pipeline', type=int, nargs='+', default=[0, 1],
                        choices=[0, 1],
                        help='Pipeline ID(s) to disable (default: both). '
                             'Use -p 0 for pipeline 0 only.')
    args = parser.parse_args()

    pipeline_args = ' '.join(str(p) for p in args.pipeline)
    cmd = f'sudo /home/casper/.souk_readout_tools/daemon/remove_systemd_service.sh {pipeline_args}'
    os.system(cmd)


if __name__ == "__main__":
    main()
