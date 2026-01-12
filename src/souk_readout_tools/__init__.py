
def _initialize_user_data():
    import os
    import shutil
    import sys

    try:
        from importlib.resources import files  # Python 3.9+
    except ImportError:
        from importlib_resources import files  # Python < 3.9


    user_data_dir = os.path.expanduser('~/.souk_readout_tools')
    if not os.path.exists(user_data_dir):
        os.makedirs(user_data_dir, exist_ok=True)
        print(f"Created user data directory at {user_data_dir}")
        # Use importlib_resources.files() to get the path to the 'data' directory in the package
        data_package = files('souk_readout_tools').joinpath('data')
        shutil.copytree(str(data_package), user_data_dir, dirs_exist_ok=True)
        print(f"Copied default data to {user_data_dir}")
        print(f"{data_package}")
    


_initialize_user_data()


# Expose common modules
from . import calibration 
from . import firmware_lib 
from . import tone_list_tools
from . import peak_finder

# Conditionally expose 'client' sub-package
import importlib.util as _importlib_util
if _importlib_util.find_spec('.client', package=__name__):
    from . import client

# Conditionally expose 'server' sub-package
if _importlib_util.find_spec('.server', package=__name__):
    from . import server

