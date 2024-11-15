import os
import sys

#need to import to make sure user data (including the service file) is setup in case of first use.
if 'souk_readout_server' in sys.modules:
    print('souk_readout_tools already imported')
else:
    import souk_readout_tools

def main():
    cmd='sudo /home/casper/.souk_readout_tools/daemon/remove_systemd_service.sh'
    os.system(cmd)
    return

if __name__=="__main__":
    main()


