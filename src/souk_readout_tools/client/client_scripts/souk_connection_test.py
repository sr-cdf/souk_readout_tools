from .. import readout_client
from .. import bcolors

import argparse

def main():
    try:
        parser = argparse.ArgumentParser(description='Check if the souk_readout_tools server is running on RFSoC')
        parser.add_argument('-C', '--config_file', type=str, default=None,
                            help='Path to config YAML file.')
        parser.add_argument('-a', '--address', type=str, default=None,
                            help='RFSoC IP address (alternative to config file).')
        parser.add_argument('-p', '--port', type=int, default=None,
                            help='Request port (required with --address). Pipeline 0: 10000, pipeline 1: 10001.')

        args = parser.parse_args()

        if args.config_file is not None:
            client = readout_client.ReadoutClient(config_file=args.config_file)
        elif args.address is not None:
            if args.port is None:
                print(bcolors.FAIL + '--port is required with --address (e.g. --port 10000).' + bcolors.ENDC)
                return
            client = readout_client.ReadoutClient(address=args.address, request_port=args.port)
        else:
            print(bcolors.FAIL + 'Provide --config_file or --address (with --port).' + bcolors.ENDC)
            return
        result = client.get_info('server')

        if isinstance(result, dict) and 'ip_addresses' in result:
            print(bcolors.OKGREEN+'Success'+bcolors.ENDC)
            print(f"SOUK Readout Server running on {result['ip_addresses']}")
        else:
            msg = result.get('message', 'Unknown error') if isinstance(result, dict) else result
            print(bcolors.FAIL + f'Failed: {msg}' + bcolors.ENDC)
            if 'Connection refused' in str(msg):
                print(bcolors.WARNING + 'Hint: ensure souk-readout-server is running on the target host '
                      '(e.g. sudo ~/py3.12-venv/bin/souk-readout-server).' + bcolors.ENDC)
    except Exception as e:
        print(bcolors.FAIL+'Failed to get server status with exception:'+bcolors.ENDC,e)
    return


if __name__ == "__main__":
    main()

