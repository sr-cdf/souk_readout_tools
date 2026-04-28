"""Top-level RF peripherals status report.

Discovers and reports the live state of every RF peripheral the server
knows about: programmable attenuators, bypass amplifiers, and LNA bias
monitors. Wraps the three discovery helpers in `rf_peripherals` and
`lna_controller` with their `--status` flag set.

CLI entry point: ``souk-rf-peripherals-status``.
"""

from souk_readout_tools.server.rf_peripherals import (
    find_attenuators,
    find_bypass_amps,
    _print_results,
)
from souk_readout_tools.server.lna_controller import find_lnas


def main():
    print('\n=== Programmable attenuators ===\n')
    _print_results(find_attenuators(include_state=True), 'attenuator')

    print('\n=== Bypass amplifiers ===\n')
    _print_results(find_bypass_amps(include_state=True), 'bypass amp')

    print('\n=== LNA bias monitors ===\n')
    _print_results(find_lnas(include_state=True), 'LNA')


if __name__ == '__main__':
    main()
