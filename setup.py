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
if install_server_env or install_client_env:
    # Use explicit env vars — both can be enabled simultaneously
    install_server = install_server_env
    install_client = install_client_env
else:
    # Automatic detection
    if is_xilinx_platform():
        install_server = True
        install_client = False
    else:
        install_client = True
        install_server = False

# Define common dependencies (headless-safe)
install_requires = [
    'numpy',
    'matplotlib',
    'pyyaml',
    'ipython',
    'scipy',
]

# Define GUI dependencies (optional, not available on headless systems)
gui_dependencies = [
    'pyqt5',
]

# Define client-specific dependencies
client_dependencies = [
    'so3g',
]

# Define server-specific dependencies
server_dependencies = [
    "souk_mkid_readout",
    'smbus2',
]

# Conditionally add client or server dependencies
if install_client:
    install_requires.extend(client_dependencies)
    if not is_xilinx_platform():
        install_requires.extend(gui_dependencies)

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
        'souk-timing-monitor=souk_readout_tools.server.timing_monitor:main',
        'souk-test-timing-monitor=souk_readout_tools.server.server_scripts.souk_test_timing_monitor:main',
        'souk-enable-daemon=souk_readout_tools.server.server_scripts.souk_enable_daemon:main',
        'souk-enable-daemons=souk_readout_tools.server.server_scripts.souk_enable_daemons:main',
        'souk-disable-daemon=souk_readout_tools.server.server_scripts.souk_disable_daemon:main',
        'souk-disable-daemons=souk_readout_tools.server.server_scripts.souk_disable_daemons:main',
        'souk-restart-daemon=souk_readout_tools.server.server_scripts.souk_restart_daemon:main',
        'souk-restart-daemons=souk_readout_tools.server.server_scripts.souk_restart_daemons:main',
        'souk-enable-timing=souk_readout_tools.server.server_scripts.souk_enable_timing:main',
        'souk-find-attenuators=souk_readout_tools.server.rf_peripherals:_cli_main',
        'souk-find-bypass-amps=souk_readout_tools.server.rf_peripherals:_cli_main_bypass_amps',
        'souk-find-lnas=souk_readout_tools.server.lna_controller:_cli_main',
        'souk-rf-peripherals-status=souk_readout_tools.server.server_scripts.souk_rf_peripherals_status:main',
    ])

setup(
    name='souk_readout_tools',
    version='1.2.0',
    description='Tools for the SOUK readout',
    author='Sam Rowe',
    author_email='sam.rowe@astro.cf.ac.uk',
    url='https://github.com/sr-cdf/souk_readout_tools',
    packages=packages,
    package_dir={'':'src'},
    include_package_data=True,
    package_data={'souk_readout_tools': [
        'mkid_finder_app.png',
        'mkid_finder_app.ico',
        'data/readme',
        'data/config/*',
        'data/calibrations/*',
        'data/daemon/*',
        'data/timing/*',
        'server/souk-peripherals-control/*.py',
    ]},
    install_requires=install_requires,
    entry_points=entry_points,
    classifiers=[
        'Programming Language :: Python :: 3',
        ],
    python_requires='>=3.10',
)
