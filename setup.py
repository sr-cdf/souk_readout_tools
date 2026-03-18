import os
import sys
from setuptools import setup, find_packages

def is_xilinx_platform():
    try:
        uname = os.uname()
        return 'xilinx' in uname.release.lower()
    except AttributeError:
        return False

# Read environment variables
install_server_env = os.environ.get('INSTALL_SERVER', '').lower() == 'true'
install_client_env = os.environ.get('INSTALL_CLIENT', '').lower() == 'true'

# Determine what to install
if install_server_env and install_client_env:
    print("Error: Cannot force both server and client installation via environment variables.")
    sys.exit(1)
elif install_server_env:
    install_server = True
    install_client = False
elif install_client_env:
    install_client = True
    install_server = False
else:
    # Automatic detection
    if is_xilinx_platform():
        install_server = True
        install_client = False
    else:
        install_client = True
        install_server = False

# Define common dependencies
install_requires = [
    'importlib_resources',
    'numpy',
    'matplotlib',
    'pyyaml',
    'ipython',
    'scipy'
    # Add other common dependencies here
]

# Define client-specific dependencies
client_dependencies = [
    'pyqt5',
    'scipy',
    # Add client dependencies here
]

# Define server-specific dependencies
server_dependencies = [
    "souk_mkid_readout  @ file://localhost//home/casper/src/souk-firmware/software/control_sw",
    'smbus2',
]

# Conditionally add client or server dependencies
if install_client:
    install_requires.extend(client_dependencies)

if install_server:
    install_requires.extend(server_dependencies)

# Define packages
packages = find_packages(where='src')

# Exclude sub-packages not being installed
if not install_client:
    packages = [pkg for pkg in packages if not pkg.startswith('souk_readout_tools.client')]
if not install_server:
    packages = [pkg for pkg in packages if not pkg.startswith('souk_readout_tools.server')]

print("Packages being installed:", packages)


entry_points = {'console_scripts': [],
                'gui_scripts': []}

if install_client:
    entry_points['console_scripts'].extend([
        'souk-connection-test=souk_readout_tools.client.client_scripts.souk_connection_test:main',
        'souk-wideband_sweep=souk_readout_tools.client.client_scripts.wideband_sweep:main',
        'souk-batch-snapshots=souk_readout_tools.client.client_scripts.batch_snapshots:main',
        'souk-find-resonances=souk_readout_tools.client.client_scripts.find_resonances:main',
        'souk-mkid-finder-app=souk_readout_tools.mkid_finder_app:main'
    ])
    entry_points['gui_scripts'].extend([
        'souk-mkid-finder=souk_readout_tools.mkid_finder_app:main'
    ])

if install_server:
    entry_points['console_scripts'].extend([
        'souk-readout-server=souk_readout_tools.server.readout_server:main',
        'souk-enable-daemon=souk_readout_tools.server.server_scripts.souk_enable_daemon:main',
        'souk-disable-daemon=souk_readout_tools.server.server_scripts.souk_disable_daemon:main'
    ])

setup(
    name='souk_readout_tools',
    version='1.1.0',
    description='Tools for the SOUK readout',
    author='Sam Rowe',
    author_email='sam.rowe@astro.cf.ac.uk',
    url='https://github.com/sr-cdf/souk_readout_tools',
    packages=packages,
    package_dir={'':'src'},
    include_package_data=True,
    package_data={'souk_readout_tools': ['mkid_finder_app.png','mkid_finder_app.ico']},
    install_requires=install_requires,
    entry_points=entry_points,
    classifiers=[
        'Programming Language :: Python :: 3',
        ],
    python_requires='>=3.8',
)
