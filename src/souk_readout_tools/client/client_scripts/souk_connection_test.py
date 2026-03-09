from .. import readout_client
from .. import bcolors

import argparse

def main():
    try:
        parser = argparse.ArgumentParser(description='Check if the souk_readout_tools server is running on RFSoC')
        parser.add_argument('-C', '--config_file', type=str,
                            default=None,
                            help='Path to config YAML or .lnk file. If omitted, uses the default_config.lnk for the selected pipeline.')
        parser.add_argument('--pipeline', type=int, default=None, choices=[0, 1],
                            help='Pipeline ID hint used only when --config_file is omitted. If omitted, defaults to pipeline 0.')

        args = parser.parse_args()

        client = readout_client.ReadoutClient(config_file=args.config_file, pipeline_id=args.pipeline)
        result = client.get_server_status()
        
        if result['status']=='success':
            print(bcolors.OKGREEN+'Success'+bcolors.ENDC)
            print(f"SOUK Readout Server running on {result['message']['ip_addresses']}")
        else:
            print(bcolors.FAIL+'Failed to get server status:\n'+bcolors.ENDC, result)
    except Exception as e:
        print(bcolors.FAIL+'Failed to get server status with exception:'+bcolors.ENDC,e)
    return


if __name__ == "__main__":
    main()


