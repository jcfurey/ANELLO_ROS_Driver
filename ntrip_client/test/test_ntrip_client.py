"""Protocol-level tests for the NTRIP client (no network required)."""

from ntrip_client.ntrip_client import NTRIPClient


def make_client(version=None, username=None, password=None):
    return NTRIPClient(
        'caster.example.com', 2101, 'MOUNT', version, username, password)


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
    client = make_client()
    client._rtcm_parser._buffer = b'stale-partial-frame'
    client._host, client._port = '127.0.0.1', 1  # nothing listening: fails fast
    assert client.connect() is False
    assert client._rtcm_parser._buffer == b''
