"""
PSD display units for the plotting layer.

The PSD computation itself lives in ``souk_readout_tools.timeseries`` so that
non-plotting code can reach it without importing ``plotting``; this module
keeps only what turns a linear PSD into plotted values.
"""

import numpy as np

PSD_UNITS = ('linear', 'db', 'dbc')


def check_psd_units(psd_units, format):
    """Validate a PSD display-units request against the plotted quantity."""
    if psd_units not in PSD_UNITS:
        raise ValueError(
            f"psd_units must be one of {PSD_UNITS}, got {psd_units!r}")
    if psd_units == 'dbc' and format == 'freq_diss':
        raise ValueError(
            "psd_units='dbc' references a carrier; fractional "
            "frequency/dissipation rows are already relative, so use "
            "psd_units='db'")


def psd_display_values(psd, psd_units, carrier=1.0):
    """
    Convert a linear PSD into the requested display units.

    ``carrier`` divides out the raw units of quantities measured in them; pass
    ``1.0`` for quantities that are already carrier-relative (phase,
    fractional frequency). ``'dbc'`` then halves the one-sided density to give
    the IEEE single-sideband ``L(f) = S(f) / 2``.
    """
    if psd_units == 'linear':
        return np.asarray(psd)
    values = np.asarray(psd, dtype=float)
    if psd_units == 'dbc':
        values = values / (2.0 * carrier)
    with np.errstate(divide='ignore'):
        return 10.0 * np.log10(values)
