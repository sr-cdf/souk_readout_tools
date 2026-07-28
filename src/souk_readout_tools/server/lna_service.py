"""
LNA bias service for the SOUK cryostat.

Only one RFSoC per telescope is wired to the cryostat LNA bias board, so that
machine runs this service and every readout server -- including the ones on
the same board -- reaches the bias hardware through it.  Centralising has
three purposes:

* the board keeps its settings across readout-server restarts and firmware
  reloads, because this service owns it and outlives them;
* exactly one process drives the I2C bus, so a multi-step operation (select
  mux path, read, release mux) cannot interleave with another server's;
* reading one channel costs a few hundred milliseconds of I2C, so a slow
  background poll here answers every pipeline's status queries from cache
  instead of making fourteen servers queue for the bus.

Protocol: a 4-byte big-endian length prefix followed by a JSON object, the
same framing the readout server uses.  Responses are
``{"status": "success", "result": ...}`` or ``{"status": "error", "message": ...}``.

Run it via systemd (``souk-lna-service-start``), or directly for debugging::

    souk-lna-service --port 10500 --log-level debug
"""

import argparse
import json
import logging
import socketserver
import struct
import threading
import time

from souk_readout_tools.server.lna_controller import (
    LNABiasController,
    NUM_LNA_CHANNELS,
)

logger = logging.getLogger('souk-lna-service')

DEFAULT_PORT = 10500
DEFAULT_POLL_INTERVAL_S = 30.0
# Requests older than this fall through to a live read. Two poll intervals
# gives the poller a chance to miss one cycle (a long set-all holds the
# hardware lock) without every reader suddenly hitting the bus.
DEFAULT_MAX_CACHE_AGE_S = 2 * DEFAULT_POLL_INTERVAL_S


class LNAService:
    """Owns the LNA bias board and answers requests about it.

    All hardware access is serialised on ``_hw_lock``; the controller
    additionally takes the cross-process I2C flock, so the CLI discovery
    tools still arbitrate correctly against this service.
    """

    def __init__(self, hw_version='auto', poll_interval_s=DEFAULT_POLL_INTERVAL_S,
                 max_cache_age_s=DEFAULT_MAX_CACHE_AGE_S):
        self.poll_interval_s = float(poll_interval_s)
        self.max_cache_age_s = float(max_cache_age_s)
        self._hw_lock = threading.Lock()
        self._cache_lock = threading.Lock()
        self._cache = {}          # channel -> status dict
        self._cache_time = {}     # channel -> monotonic timestamp
        self._stop = threading.Event()
        self.started_at = time.time()

        # The service is the one process that talks to the board, so it uses
        # the direct-I2C backend. Channel 1 is only the controller's default
        # for calls that omit a channel; every request here passes one
        # explicitly.
        self._controller = LNABiasController({
            'cryostat': {
                'lna_bias': {
                    'enabled': True,
                    'backend': 'i2c',
                    'lna_channel': 1,
                    'hw_version': hw_version,
                },
            },
        })
        if not self._controller.is_hardware:
            raise RuntimeError(
                'LNA bias board did not initialise; refusing to start the '
                'service. Check the board is powered and on the I2C bus.'
            )
        logger.info(
            'LNA bias board ready: hw v%s, channels %s',
            self._controller.hw_version,
            ', '.join(self._controller.detected_refdes) or 'none',
        )

    # -- cache --

    def _store(self, statuses):
        now = time.monotonic()
        with self._cache_lock:
            for chn, status in statuses.items():
                self._cache[chn] = status
                self._cache_time[chn] = now

    def _cached(self, channels, max_age_s):
        """Return cached statuses for ``channels`` if all are fresh enough."""
        now = time.monotonic()
        with self._cache_lock:
            out = {}
            for chn in channels:
                stamp = self._cache_time.get(chn)
                if stamp is None or (now - stamp) > max_age_s:
                    return None
                out[chn] = dict(self._cache[chn])
        return out

    def _invalidate(self, channels):
        with self._cache_lock:
            for chn in channels:
                self._cache_time.pop(chn, None)

    def poll_loop(self):
        """Refresh every channel's status on a slow cadence."""
        while not self._stop.is_set():
            try:
                with self._hw_lock:
                    statuses = self._controller.get_lna_bias_status_all()
                self._store(statuses)
            except Exception:
                logger.exception('LNA status poll failed')
            self._stop.wait(self.poll_interval_s)

    def stop(self):
        """Ask the background poll loop to finish."""
        self._stop.set()

    # -- requests --

    def handle(self, message):
        """Dispatch one request dict and return its result payload."""
        request = message.get('request')
        handler = getattr(self, f'_do_{request}', None) if request else None
        if handler is None:
            raise ValueError(f'Unknown request: {request!r}')
        return handler(message)

    def _channel(self, message):
        chn = message.get('channel')
        if chn is None:
            raise ValueError('Request is missing the required channel argument')
        chn = int(chn)
        if not (1 <= chn <= NUM_LNA_CHANNELS):
            raise ValueError(
                f'LNA channel must be 1-{NUM_LNA_CHANNELS}, got {chn}'
            )
        return chn

    def _with_hw_version(self, result):
        if isinstance(result, dict):
            result.setdefault('hw_version', self._controller.hw_version)
        return result

    def _do_ping(self, message):
        return {'ok': True, 'hw_version': self._controller.hw_version}

    def _do_get_service_status(self, message):
        with self._cache_lock:
            cached_channels = sorted(self._cache_time)
        status = self._controller.get_status()
        status.update({
            'service_uptime_s': time.time() - self.started_at,
            'detected_refdes': self._controller.detected_refdes,
            'poll_interval_s': self.poll_interval_s,
            'cached_channels': cached_channels,
        })
        return status

    def _do_get_lna_bias_status(self, message):
        chn = self._channel(message)
        max_age = 0.0 if message.get('refresh') else self.max_cache_age_s
        cached = self._cached([chn], max_age)
        if cached is not None:
            return self._with_hw_version(cached[chn])
        with self._hw_lock:
            status = self._controller.get_lna_bias_status(channel=chn)
        self._store({chn: status})
        return self._with_hw_version(dict(status))

    def _do_get_lna_bias_status_all(self, message):
        channels = list(range(1, NUM_LNA_CHANNELS + 1))
        max_age = 0.0 if message.get('refresh') else self.max_cache_age_s
        cached = self._cached(channels, max_age)
        if cached is not None:
            return cached
        with self._hw_lock:
            statuses = self._controller.get_lna_bias_status_all()
        self._store(statuses)
        return statuses

    def _do_set_lna_bias_voltage(self, message):
        chn = self._channel(message)
        with self._hw_lock:
            result = self._controller.set_lna_bias_voltage(
                float(message['voltage_v']),
                channel=chn,
                method=message.get('method', 'remote'),
                blind=bool(message.get('blind', False)),
            )
        self._invalidate([chn])
        logger.info('set channel %d: %s', chn, result)
        return self._with_hw_version(result)

    def _do_set_lna_bias_voltage_all(self, message):
        with self._hw_lock:
            result = self._controller.set_lna_bias_voltage_all(
                float(message['voltage_v']),
                method=message.get('method', 'remote'),
                blind=bool(message.get('blind', False)),
            )
        self._invalidate(range(1, NUM_LNA_CHANNELS + 1))
        logger.info('set all channels to %s V', message.get('voltage_v'))
        return result

    def _do_soft_off_lna_bias(self, message):
        chn = self._channel(message)
        with self._hw_lock:
            result = self._controller.soft_off_lna_bias(channel=chn)
        self._invalidate([chn])
        logger.info('soft-off channel %d: %s', chn, result)
        return self._with_hw_version(result)

    def _do_soft_off_lna_bias_all(self, message):
        with self._hw_lock:
            result = self._controller.soft_off_lna_bias_all()
        self._invalidate(range(1, NUM_LNA_CHANNELS + 1))
        logger.info('soft-off all channels')
        return result

    def _do_set_lna_output_enabled(self, message):
        chn = self._channel(message)
        enabled = bool(message['enabled'])
        with self._hw_lock:
            result = self._controller.set_lna_output_enabled(
                enabled, channel=chn,
            )
        self._invalidate([chn])
        logger.info(
            'output %s channel %d: %s',
            'enable' if enabled else 'disable', chn, result,
        )
        return self._with_hw_version(result)

    def _do_set_lna_output_enabled_all(self, message):
        enabled = bool(message['enabled'])
        with self._hw_lock:
            result = self._controller.set_lna_output_enabled_all(enabled)
        self._invalidate(range(1, NUM_LNA_CHANNELS + 1))
        logger.info('output %s all channels', 'enable' if enabled else 'disable')
        return result


class _RequestHandler(socketserver.BaseRequestHandler):
    """One request per connection, length-prefixed JSON in and out."""

    def handle(self):
        try:
            message = self._read_message()
        except (ConnectionError, OSError, ValueError) as e:
            logger.warning('Malformed request from %s: %s', self.client_address[0], e)
            return
        if message is None:
            return

        try:
            result = self.server.service.handle(message)
            response = {'status': 'success', 'result': result}
        except Exception as e:
            logger.warning(
                'Request %r from %s failed: %s',
                message.get('request'), self.client_address[0], e,
            )
            response = {'status': 'error', 'message': str(e)}

        try:
            self._send_message(response)
        except OSError as e:
            logger.warning('Could not reply to %s: %s', self.client_address[0], e)

    def _read_message(self):
        raw_len = self._recv_exactly(4)
        if raw_len is None:
            return None
        (length,) = struct.unpack('>I', raw_len)
        body = self._recv_exactly(length)
        if body is None:
            raise ConnectionError('client closed mid-message')
        return json.loads(body.decode())

    def _recv_exactly(self, count):
        """Read exactly ``count`` bytes, or None if the client disconnects."""
        buf = bytearray(count)
        view = memoryview(buf)
        got = 0
        while got < count:
            n = self.request.recv_into(view[got:], count - got)
            if n == 0:
                return None
            got += n
        return bytes(buf)

    def _send_message(self, response):
        payload = json.dumps(response).encode()
        self.request.sendall(struct.pack('>I', len(payload)) + payload)


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    """CLI entry point for souk-lna-service."""
    parser = argparse.ArgumentParser(
        description='Serve the SOUK cryostat LNA bias board over TCP.',
    )
    parser.add_argument('--host', default='0.0.0.0',
                        help='Address to bind (default: all interfaces).')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT,
                        help=f'TCP port to listen on (default: {DEFAULT_PORT}).')
    parser.add_argument('--hw-version', default='auto', choices=['auto', '1', '2'],
                        help='LNA bias board revision (default: auto-detect).')
    parser.add_argument('--poll-interval', type=float,
                        default=DEFAULT_POLL_INTERVAL_S,
                        help='Seconds between background status polls '
                             f'(default: {DEFAULT_POLL_INTERVAL_S:g}).')
    parser.add_argument('--log-level', default='info',
                        choices=['debug', 'info', 'warning', 'error'])
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )

    try:
        service = LNAService(
            hw_version=args.hw_version,
            poll_interval_s=args.poll_interval,
            max_cache_age_s=2 * args.poll_interval,
        )
    except Exception as e:
        logger.error('Cannot start LNA bias service: %s', e)
        raise SystemExit(1)
    poller = threading.Thread(target=service.poll_loop, name='lna-poll',
                              daemon=True)
    poller.start()

    server = _Server((args.host, args.port), _RequestHandler)
    server.service = service
    logger.info('LNA bias service listening on %s:%d', args.host, args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info('Shutting down')
    finally:
        service.stop()
        server.server_close()


if __name__ == '__main__':
    main()
