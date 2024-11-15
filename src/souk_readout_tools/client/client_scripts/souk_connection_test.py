from .. import readout_client
from .. import bcolors

def main():
    try:
        client = readout_client.ReadoutClient()
        info = client.get_server_status()
        if info['status']=='success':
            print(bcolors.OKGREEN+'Success'+bcolors.ENDC)
        else:
            print(bcolors.FAIL+'Failed to get server status:\n'+bcolors.ENDC,info)
    except Exception as e:
        print(bcolors.FAIL+'Failed to get server status with exception:'+bcolors.ENDC,e)
    return


if __name__ == "__main__":
    main()


