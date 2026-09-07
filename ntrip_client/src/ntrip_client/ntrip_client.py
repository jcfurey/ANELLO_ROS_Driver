#!/usr/bin/env python

import ssl
import time
import base64
import socket
import select
import logging
import queue
import re
import threading

from .nmea_parser import NMEAParser
from .rtcm_parser import RTCMParser

_CHUNK_SIZE = 1024
# Cap on the HTTP response header block read during connect(); a real
# caster's headers fit well inside this.
_MAX_RESPONSE_HEADER_BYTES = 16 * 1024


class NTRIPClient:

    # Public constants
    DEFAULT_RECONNECT_ATTEMPT_MAX = 10
    DEFAULT_RECONNECT_ATTEMPT_WAIT_SECONDS = 5
    DEFAULT_RTCM_TIMEOUT_SECONDS = 4

    def __init__(
        self, host, port, mountpoint, ntrip_version,
        username, password, logerr=logging.error,
        logwarn=logging.warning, loginfo=logging.info,
        logdebug=logging.debug
    ):
        # Bit of a strange pattern here, but save the log functions
        # so we can be agnostic of ROS
        self._logerr = logerr
        self._logwarn = logwarn
        self._loginfo = loginfo
        self._logdebug = logdebug

        # Save the server info
        self._host = host
        self._port = port
        self._mountpoint = mountpoint
        self._ntrip_version = ntrip_version
        if username is not None and password is not None:
            self._basic_credentials = base64.b64encode(
                '{}:{}'.format(
                    username, password
                ).encode('utf-8')
            ).decode('utf-8')
        else:
            self._basic_credentials = None

        # Initialize this so we don't throw an exception when closing
        self._raw_socket = None
        self._server_socket = None

        # Setup some parsers to parse incoming messages
        self._rtcm_parser = RTCMParser(
            logerr=logerr,
            logwarn=logwarn,
            loginfo=loginfo,
            logdebug=logdebug
        )
        self._nmea_parser = NMEAParser(
            logerr=logerr,
            logwarn=logwarn,
            loginfo=loginfo,
            logdebug=logdebug
        )

        # Public SSL configuration
        self.ssl = False
        self.cert = None
        self.key = None
        self.ca_cert = None

        # Setup some state
        self._resolver_result = None
        self._shutdown = False
        self._connected = False

        # Private reconnect info
        self._reconnect_attempt_count = 0
        self._nmea_send_failed_count = 0
        self._nmea_send_failed_max = 5
        self._read_zero_bytes_count = 0
        self._read_zero_bytes_max = 5
        self._first_rtcm_received = False
        self._recv_rtcm_last_packet_timestamp = 0

        # Response stream state set up by connect()
        self._pending_stream_data = b''
        self._response_chunked = False
        self._chunk_buffer = b''
        self._chunk_eof = False

        # Public reconnect info
        self.reconnect_attempt_max = self.DEFAULT_RECONNECT_ATTEMPT_MAX
        self.reconnect_attempt_wait_seconds = \
            self.DEFAULT_RECONNECT_ATTEMPT_WAIT_SECONDS
        self.rtcm_timeout_seconds = self.DEFAULT_RTCM_TIMEOUT_SECONDS

    @property
    def connected(self):
        return self._connected

    def connect(self):
        # Drop any partial RTCM frame left over from a previous
        # connection; it belongs to a dead TCP session and must not be
        # spliced onto this one's stream.
        self.disconnect()
        self._rtcm_parser.reset()
        self._pending_stream_data = b''
        self._chunk_buffer = b''
        self._chunk_eof = False
        if self._shutdown:
            return False

        # Every failure path must go through disconnect(): it closes the
        # socket (no fd left to leak toward GC) and clears _connected —
        # a half-open session reported as connected would defeat the
        # ROS node's retry gate.
        if not self._open_socket():
            return False

        # Send the HTTP Request (sendall: a partial send() would
        # truncate the request and corrupt the caster session)
        try:
            self._server_socket.sendall(self._form_request())
        except OSError as e:
            self._logerr(
                'Unable to send request to server at '
                'http://{}:{}'.format(self._host, self._port))
            self._logerr('Exception: {}'.format(str(e)))
            self.disconnect()
            return False

        response = self._read_response_headers()
        if response is None or not self._classify_response(response):
            self.disconnect()
            return False

        # Fresh connection: the RTCM-timeout gate must stay disarmed
        # until this connection delivers its first packet (a stale
        # pre-reconnect timestamp would otherwise tear the new
        # connection down immediately if the reconnect took longer
        # than rtcm_timeout_seconds), and the failure counters
        # belong to the dead connection.
        self._first_rtcm_received = False
        self._recv_rtcm_last_packet_timestamp = time.monotonic()
        self._read_zero_bytes_count = 0
        self._nmea_send_failed_count = 0
        self._loginfo(
            'Connected to http://{}:{}/{}'.format(
                self._host, self._port, self._mountpoint))
        return True

    def _resolve(self, deadline):
        # getaddrinfo has no portable cancellation API. Keep at most one
        # daemon resolver per client; shutdown never waits for system DNS.
        if self._resolver_result is None:
            result = queue.Queue(maxsize=1)
            self._resolver_result = result

            def resolve():
                try:
                    result.put(socket.getaddrinfo(
                        self._host, self._port, type=socket.SOCK_STREAM))
                except OSError as exc:
                    result.put(exc)

            threading.Thread(target=resolve, daemon=True).start()
        while not self._shutdown and time.monotonic() < deadline:
            try:
                result = self._resolver_result.get(timeout=0.05)
            except queue.Empty:
                continue
            self._resolver_result = None
            if isinstance(result, Exception):
                raise result
            return result
        raise TimeoutError('DNS resolution cancelled or timed out')

    def _open_socket(self):
        deadline = time.monotonic() + 5.0
        try:
            addresses = self._resolve(deadline)
            for family, kind, protocol, _, address in addresses:
                if self._shutdown or time.monotonic() >= deadline:
                    return False
                sock = socket.socket(family, kind, protocol)
                self._server_socket = sock
                sock.settimeout(max(0.01, deadline - time.monotonic()))
                try:
                    sock.connect(address)
                    if self.ssl:
                        context = ssl.create_default_context()
                        if self.cert:
                            context.load_cert_chain(self.cert, self.key)
                        if self.ca_cert:
                            context.load_verify_locations(self.ca_cert)
                        self._raw_socket = sock
                        sock = context.wrap_socket(
                            sock, server_hostname=self._host,
                            do_handshake_on_connect=False)
                        self._server_socket = sock
                        sock.settimeout(max(0.01, deadline - time.monotonic()))
                        sock.do_handshake()
                    sock.settimeout(0.2)
                    if self._shutdown:
                        self.disconnect()
                        return False
                    return True
                except OSError:
                    self.disconnect()
            return False
        except (OSError, ValueError) as exc:
            self._logwarn('NTRIP connection failed: {}'.format(exc))
            self.disconnect()
            return False

    def _read_response_headers(self):
        raw = b''
        deadline = time.monotonic() + 5.0
        sock = self._server_socket
        try:
            # Accumulate a complete status line, even a split "HT" prefix.
            while not self._shutdown and time.monotonic() < deadline:
                line_end = raw.find(b'\r\n')
                if line_end >= 0:
                    status = raw[:line_end].decode('ascii')
                    if not self._classify_response(status):
                        return None
                    if status.startswith('ICY '):
                        end = line_end + 2
                    else:
                        end = raw.find(b'\r\n\r\n')
                        end = end + 4 if end >= 0 else -1
                    if end >= 0:
                        if end > _MAX_RESPONSE_HEADER_BYTES:
                            return None
                        headers = raw[:end].decode('ascii')
                        self._pending_stream_data = raw[end:]
                        self._response_chunked = False
                        for line in headers.split('\r\n')[1:]:
                            name, separator, value = line.partition(':')
                            if separator and name.lower() == 'transfer-encoding':
                                if value.strip().lower() != 'chunked':
                                    return None
                                self._response_chunked = True
                        return headers
                if len(raw) >= _MAX_RESPONSE_HEADER_BYTES:
                    return None
                try:
                    chunk = sock.recv(_CHUNK_SIZE)
                except socket.timeout:
                    continue
                if not chunk:
                    return None
                raw += chunk
        except (OSError, UnicodeError):
            return None
        return None

    def _classify_response(self, response):
        lines = response.split('\r\n')
        match = re.fullmatch(r'(?:HTTP/1\.[01]|ICY) ([0-9]{3})(?: .*)?', lines[0])
        valid = match is not None and match.group(1) == '200'
        # Some casters explicitly supply an error Status header. Match its
        # field and code, never arbitrary request IDs or other header text.
        for line in lines[1:]:
            name, separator, value = line.partition(':')
            if separator and name.lower() in ('status', 'x-status'):
                code = value.strip().split(' ', 1)[0]
                if code in ('401', '404'):
                    valid = False
        self._connected = valid
        if not valid:
            self._logwarn('NTRIP response was not a successful stream status')
        return valid

    def disconnect(self):
        # Disconnect the socket. Each socket gets its own shutdown and
        # close attempt: a routine shutdown failure on one (e.g.
        # ENOTCONN after the peer closed) must not skip the other.
        self._connected = False
        for sock in (self._server_socket, self._raw_socket):
            if not sock:
                continue
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception as e:
                self._logdebug(
                    'Encountered exception when shutting down the '
                    'socket. This can likely be ignored')
                self._logdebug('Exception: {}'.format(e))
            try:
                sock.close()
            except Exception as e:
                self._logdebug(
                    'Encountered exception when closing the socket. '
                    'This can likely be ignored')
                self._logdebug('Exception: {}'.format(e))
        self._server_socket = None
        self._raw_socket = None

    def reconnect(self):
        # One attempt only. Persistent retry and its wait belong to the worker.
        self.disconnect()
        return self.connect() if not self._shutdown else False

    def normalize_nmea(self, sentence):
        if sentence.endswith('\\r\\n'):
            sentence = sentence[:-4]
        sentence = sentence.rstrip('\r\n') + '\r\n'
        return sentence if self._nmea_parser.is_valid_sentence(sentence) else None

    def send_nmea(self, sentence):
        sentence = self.normalize_nmea(sentence)
        if not self._connected or sentence is None:
            return False
        try:
            self._server_socket.sendall(sentence.encode('ascii'))
            self._nmea_send_failed_count = 0
            return True
        except (OSError, UnicodeError, AttributeError):
            self._nmea_send_failed_count += 1
            self.disconnect()
            return False

    def recv_rtcm(self):
        if not self._connected:
            return []
        pending = self._pending_stream_data
        self._pending_stream_data = b''
        packets = self._parse_stream(pending) if pending else []
        # Bounded work, including a continuously transmitting caster.
        try:
            for _ in range(16):
                if not self._connected or not self._data_available():
                    break
                try:
                    data = self._server_socket.recv(_CHUNK_SIZE)
                except (socket.timeout, ssl.SSLWantReadError):
                    break
                if not data:
                    self.disconnect()
                    break
                packets.extend(self._parse_stream(data))
        except (OSError, ValueError, AttributeError):
            self.disconnect()
        if packets:
            self._first_rtcm_received = True
            self._recv_rtcm_last_packet_timestamp = time.monotonic()
        elif (time.monotonic() - self._recv_rtcm_last_packet_timestamp
              >= self.rtcm_timeout_seconds):
            self._logwarn('NTRIP valid-correction deadline expired')
            self.disconnect()
        return packets

    def _parse_stream(self, data):
        # Parse the byte stream into complete, checksum-verified RTCM
        # frames so each published rtcm_msgs/Message holds exactly one
        # RTCM message (partial frames are cached until the rest arrives)
        if self._response_chunked:
            data = self._dechunk(data)
        return self._rtcm_parser.parse(data) if data else []

    def _dechunk(self, data):
        if self._chunk_eof:
            return b''
        self._chunk_buffer += data
        payload = bytearray()
        try:
            while True:
                end = self._chunk_buffer.find(b'\r\n')
                if end < 0:
                    if len(self._chunk_buffer) > 128:
                        raise ValueError('Oversized chunk header')
                    break
                if end > 128:
                    raise ValueError('Oversized chunk header')
                token = self._chunk_buffer[:end].split(b';', 1)[0]
                if not re.fullmatch(b'[0-9a-fA-F]+', token):
                    raise ValueError('Invalid chunk size')
                size = int(token, 16)
                if size > 1024 * 1024:
                    raise ValueError('Chunk exceeds 1 MiB limit')
                if size == 0:
                    self._chunk_buffer = b''
                    self._chunk_eof = True
                    self.disconnect()
                    break
                boundary = end + 2 + size
                if len(self._chunk_buffer) < boundary + 2:
                    break
                if self._chunk_buffer[boundary:boundary + 2] != b'\r\n':
                    raise ValueError('Missing chunk terminator')
                payload.extend(self._chunk_buffer[end + 2:boundary])
                self._chunk_buffer = self._chunk_buffer[boundary + 2:]
        except ValueError as exc:
            self._logwarn(str(exc))
            self._chunk_buffer = b''
            self._chunk_eof = True
            self.disconnect()
        return bytes(payload)

    def _data_available(self):
        # select() only sees the raw fd. With TLS a single record can
        # decrypt to more than one recv() worth of bytes; the remainder
        # sits in the SSLSocket's internal buffer, invisible to select,
        # and would otherwise be stranded until the next TCP segment.
        if self.ssl and self._server_socket.pending() > 0:
            return True
        read_sockets, _, _ = select.select(
            [self._server_socket], [], [], 0
        )
        return bool(read_sockets)

    def shutdown(self):
        # Set some state, and then disconnect
        self._shutdown = True
        self.disconnect()

    def _is_ntrip_v2(self):
        return (
            self._ntrip_version is not None
            and str(self._ntrip_version) in ('Ntrip/2.0', '2.0')
        )

    def _form_request(self):
        for value in (self._host, self._mountpoint, self._ntrip_version or ''):
            if any(char in value for char in ('\r', '\n')):
                raise ValueError('NTRIP request fields cannot contain line endings')
        # NTRIP rev2 is proper HTTP/1.1 and requires the Host and
        # Ntrip-Version headers; rev1 casters expect an HTTP/1.0 request.
        # The Host header is legal in HTTP/1.0 too, so always send it.
        http_version = 'HTTP/1.1' if self._is_ntrip_v2() else 'HTTP/1.0'
        request_str = (
            'GET /{} {}\r\n'
            'Host: {}:{}\r\n'
        ).format(self._mountpoint, http_version, self._host, self._port)
        if (
            self._ntrip_version is not None
            and self._ntrip_version != ''
        ):
            request_str += 'Ntrip-Version: {}\r\n'.format(
                self._ntrip_version)
        request_str += 'User-Agent: NTRIP ntrip_client_ros\r\n'
        if self._basic_credentials is not None:
            request_str += 'Authorization: Basic {}\r\n'.format(
                self._basic_credentials)
        request_str += '\r\n'
        return request_str.encode('utf-8')
