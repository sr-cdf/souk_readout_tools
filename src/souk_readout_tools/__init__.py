
import importlib as _importlib
import importlib.util as _importlib_util
import importlib.metadata as _importlib_metadata

# Version is declared in setup.py only; read it back from the install metadata.
try:
    __version__ = _importlib_metadata.version(__name__)
except _importlib_metadata.PackageNotFoundError:
    # Imported straight from a source checkout (PYTHONPATH=src), not installed.
    __version__ = 'unknown'

# Submodules available for lazy import.
_SUBMODULES = [
    "board_management",
    "calibration",
    "firmware_lib",
    "tone_list_tools",
    "peak_finder",
    "resonance_matching",
    "measurement",
    "power_sweep",
    "resonator",
    "fitting",
    "noise",
    "timeseries",
    "timing",
    "plotting",
]

# Conditionally available sub-packages (only present in certain installs).
if _importlib_util.find_spec('.client', package=__name__):
    _SUBMODULES.append("client")
if _importlib_util.find_spec('.server', package=__name__):
    _SUBMODULES.append("server")


def __getattr__(name):
    if name in _SUBMODULES:
        return _importlib.import_module(f".{name}", __name__)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return __all__ + list(globals().keys())


__all__ = list(_SUBMODULES)
