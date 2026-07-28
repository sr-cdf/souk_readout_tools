"""
Cross-process lock for the RFSoC's shared I2C bus.

Both the cryostat LNA bias board and the RF module hang off SMBus(0) on the
RFSoC that is wired to them, and several processes may want it at once: the
LNA bias service, the two readout servers, and the ``souk-find-*`` discovery
CLIs. Individual SMBus transactions are atomic in the kernel, but the
sequences here are not -- selecting a mux path, reading a device, then
releasing the mux is three transactions that must not interleave with another
process doing the same thing.

The lock is an flock on a well-known file, so it works across processes
without a daemon and is released automatically if a holder dies.
"""

import fcntl
import logging
import os
import tempfile
from contextlib import contextmanager

logger = logging.getLogger(__name__)

# Both the LNA bias board and the RF module are on bus 0.
I2C_BUS_NUM = 0


@contextmanager
def i2c_bus_lock(bus_num=I2C_BUS_NUM, purpose='I2C'):
    """Serialize access to a shared Linux I2C bus across server processes."""
    lock_path = os.path.join(
        tempfile.gettempdir(),
        f'souk_readout_tools_i2c_bus_{int(bus_num)}.lock',
    )
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o666)
    try:
        os.chmod(lock_path, 0o666)
    except OSError:
        pass
    with os.fdopen(fd, 'r+') as lock_fd:
        logger.debug('Waiting for %s I2C bus %d lock', purpose, bus_num)
        fcntl.flock(lock_fd, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
