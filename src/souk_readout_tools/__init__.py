
import importlib as _importlib
import importlib.util as _importlib_util

# Submodules available for lazy import.
_SUBMODULES = [
    "calibration",
    "firmware_lib",
    "tone_list_tools",
    "peak_finder",
    "resonator",
    "fitting",
    "measurement",
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
