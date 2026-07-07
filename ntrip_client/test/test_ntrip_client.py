"""Protocol-level tests for the NTRIP client (loopback sockets only)."""

import socket
import threading
import time

import pytest

from ntrip_client.ntrip_client import NTRIPClient
from ntrip_client.rtcm_parser import RTCMParser


def make_client(version=None, username=None, password=None,
                host='caster.example.com', port=2101):
    return NTRIPClient(
        host, port, 'MOUNT', version, username, password)


def make_rtcm_frame(payload):
    """Build a valid RTCM3 frame (preamble, 10-bit length, payload, CRC)."""
    header = bytes([0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF])
    body = header + payload
    crc = RTCMParser()._checksum(body)
    return body + bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])


class FakeCaster:
    """Loopback listener that answers one connection with canned bytes."""

    def __init__(self, response_segments):
        self._segments = response_segments
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.bind(('127.0.0.1', 0))
        self._listener.listen(1)
        self.port = self._listener.getsockname()[1]
        self._conn = None
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self):
        # A client that aborts mid-exchange (e.g. the SSL-failure test)
        # makes accept/recv/sendall raise here; swallow it so the daemon
        # thread doesn't dump a stack trace over the test output.
        try:
            self._conn, _ = self._listener.accept()
            self._conn.recv(4096)  # the client's HTTP request
            for i, segment in enumerate(self._segments):
                if i > 0:
                    # Let the client drain the previous segment first so
                    # the segments arrive as separate recv() results.
                    time.sleep(0.05)
                self._conn.sendall(segment)
        except OSError:
            pass

    def close(self):
        # Close the listener first: a client that never connected then
        # unblocks accept() immediately instead of waiting out the join.
        self._listener.close()
        self._thread.join(timeout=5)
        if self._conn:
            self._conn.close()


def connect_to_fake_caster(response_segments):
    caster = FakeCaster(response_segments)
    client = make_client(host='127.0.0.1', port=caster.port)
    try:
        return client, client.connect()
    finally:
        caster.close()


def test_v1_request_format():
    request = make_client()._form_request().decode()
    assert request.startswith('GET /MOUNT HTTP/1.0\r\n')
    assert 'Host: caster.example.com:2101\r\n' in request
    assert 'User-Agent: NTRIP ' in request
    assert 'Ntrip-Version' not in request
    assert 'Authorization' not in request
    assert request.endswith('\r\n\r\n')


def test_v2_request_format():
    request = make_client('Ntrip/2.0', 'user', 'pass')._form_request().decode()
    assert request.startswith('GET /MOUNT HTTP/1.1\r\n')
    assert 'Host: caster.example.com:2101\r\n' in request
    assert 'Ntrip-Version: Ntrip/2.0\r\n' in request
    assert 'Authorization: Basic dXNlcjpwYXNz\r\n' in request


def test_version_detection():
    assert make_client('Ntrip/2.0')._is_ntrip_v2()
    assert not make_client()._is_ntrip_v2()
    assert not make_client('Ntrip/1.0')._is_ntrip_v2()


def test_dechunk_whole_and_split_chunks():
    client = make_client()
    payload1 = b'\xd3\x00\x04AAAA'   # 7 bytes
    payload2 = b'BBBB'
    stream = b'7\r\n' + payload1 + b'\r\n4\r\n' + payload2 + b'\r\n'
    out = client._dechunk(stream[:6])
    out += client._dechunk(stream[6:])
    assert out == payload1 + payload2


def test_dechunk_terminating_chunk():
    client = make_client()
    assert client._dechunk(b'0\r\n\r\n') == b''


def test_dechunk_terminating_chunk_drops_trailing_data():
    # The zero-size chunk ends the stream; anything after it is not
    # payload and must not leak into the parser or linger in the buffer.
    client = make_client()
    assert client._dechunk(b'0\r\n\r\njunk-after-terminator') == b''
    assert client._chunk_buffer == b''


def test_dechunk_chunk_extension_tokens():
    client = make_client()
    payload = b'\xd3\x00\x04AAAA'
    assert client._dechunk(b'7;ext=1\r\n' + payload + b'\r\n') == payload


def test_dechunk_passes_through_unframed_data():
    client = make_client()
    raw = b'\xd3\x00\x06 not chunked\r\n'
    out = client._dechunk(raw)
    assert out.startswith(b'\xd3')


def test_dechunk_negative_chunk_size_passes_through():
    # int(token, 16) accepts a signed token; a negative size must be
    # treated as malformed framing (like a ValueError), not used to
    # slice the buffer and silently emit corrupted bytes.
    client = make_client()
    raw = b'-5\r\nhello\r\n'
    assert client._dechunk(raw) == raw


def test_connect_resets_stale_rtcm_buffer():
    # A partial RTCM frame left in the parser from a dead connection
    # must not survive into the next connection's stream.
    client = make_client(host='127.0.0.1', port=1)  # nothing listening
    client._rtcm_parser._buffer = b'stale-partial-frame'
    assert client.connect() is False
    assert client._rtcm_parser._buffer == b''


def test_connect_resets_stream_health_state():
    # A successful (re)connect must disarm the RTCM-timeout gate and
    # zero the dead connection's failure counters, or a reconnect that
    # took longer than rtcm_timeout_seconds is immediately torn down
    # again by the stale timestamp.
    caster = FakeCaster([b'ICY 200 OK\r\n\r\n'])
    client = make_client(host='127.0.0.1', port=caster.port)
    client._first_rtcm_received = True
    client._read_zero_bytes_count = 3
    client._nmea_send_failed_count = 2
    try:
        assert client.connect() is True
    finally:
        caster.close()
    assert client._first_rtcm_received is False
    assert client._read_zero_bytes_count == 0
    assert client._nmea_send_failed_count == 0
    assert client.connected is True


def test_connect_reads_fragmented_http_headers():
    # An HTTP/1.1 header block split across TCP segments must still be
    # read to its \r\n\r\n terminator; stopping at the first segment
    # misses Transfer-Encoding and feeds chunk framing to the parser.
    payload = b'\xd3\x00\x04AAAA'
    client, ok = connect_to_fake_caster([
        b'HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\nTransfer-',
        b'Encoding: chunked\r\n\r\n7\r\n' + payload + b'\r\n',
    ])
    assert ok is True
    assert client._response_chunked is True
    # Stream bytes that arrived with the headers are preserved
    assert client._dechunk(client._pending_stream_data) == payload


def test_connect_mixed_success_and_error_fails_cleanly():
    # Some casters return both a success line and an error in one
    # response; that must count as a failure AND leave the client
    # reporting disconnected — a half-open session flagged as connected
    # would defeat the ROS node's retry gate.
    client, ok = connect_to_fake_caster([
        b'HTTP/1.1 200 OK\r\nX-Status: 401 Unauthorized\r\n\r\n',
    ])
    assert ok is False
    assert client.connected is False


def test_connect_ssl_failure_returns_false():
    # A TLS handshake failure (here: a plain-TCP caster) must fail the
    # connect so reconnect logic can retry, not raise out of connect()
    # and kill the node.
    caster = FakeCaster([b'ICY 200 OK\r\n\r\n'])
    client = make_client(host='127.0.0.1', port=caster.port)
    client.ssl = True
    try:
        assert client.connect() is False
    finally:
        caster.close()
    assert client.connected is False


def test_recv_rtcm_consumes_pending_stream_data():
    # Stream bytes that arrive in the same packet as the response
    # headers must come out of the first recv_rtcm() call even when the
    # socket has nothing new to offer.
    frame = make_rtcm_frame(b'\x43\x50' + b'\x01' * 8)
    chunk = ('%x' % len(frame)).encode() + b'\r\n' + frame + b'\r\n'
    caster = FakeCaster([
        b'HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n' + chunk,
    ])
    client = make_client(host='127.0.0.1', port=caster.port)
    try:
        assert client.connect() is True
        packets = client.recv_rtcm()
    finally:
        caster.close()
    assert packets == [frame]


def test_reconnect_success_on_final_attempt_does_not_raise():
    # Guards the reconnect off-by-one fix: succeeding on the last
    # allowed attempt used to still raise "never succeeded".
    client = make_client()
    client._connected = True
    client.reconnect_attempt_max = 3
    client.reconnect_attempt_wait_seconds = 0
    results = iter([False, False, True])
    client.connect = lambda: next(results)
    client.disconnect = lambda: None
    client.reconnect()
    assert client._reconnect_attempt_count == 0


def test_reconnect_exhaustion_raises_after_max_attempts():
    client = make_client()
    client._connected = True
    client.reconnect_attempt_max = 3
    client.reconnect_attempt_wait_seconds = 0
    calls = []
    client.connect = lambda: calls.append(1) or False
    client.disconnect = lambda: None
    with pytest.raises(ConnectionError):
        client.reconnect()
    assert len(calls) == 3
    assert client._reconnect_attempt_count == 0


def test_reconnect_ignored_when_not_connected():
    client = make_client()
    client.connect = lambda: pytest.fail('connect must not be called')
    client.reconnect()
