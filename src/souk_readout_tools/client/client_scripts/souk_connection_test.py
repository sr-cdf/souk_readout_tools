from .. import readout_client
from .. import bcolors

import argparse
import os

def main():
    try:
        DEFAULT_CONFIG = os.path.expanduser('~/.souk_readout_tools/config/default_config.lnk')

        parser = argparse.ArgumentParser(description='Check if the souk_readout_tools server is running on RFSoC')
        parser.add_argument('-C', '--config_file', type=str,
                            default=DEFAULT_CONFIG,
                            help='Path to the configuration file, Reverts to ~/souk_readout_tools/config/default.lnk if not given.')

        args = parser.parse_args()

        client = readout_client.ReadoutClient(args.config_file)
        result = client.get_server_status()
        
        if result['status']=='success':
            print(bcolors.OKGREEN+'Success'+bcolors.ENDC)
            print(f"SOUK Readout Server running on {result['message']['ip_addresses']}")
        else:
            print(bcolors.FAIL+'Failed to get server status:\n'+bcolors.ENDC,info)
    except Exception as e:
        print(bcolors.FAIL+'Failed to get server status with exception:'+bcolors.ENDC,e)
    return


if __name__ == "__main__":
    main()


