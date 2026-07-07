"""Synthetic-stream tests for the RTCM3 frame parser."""

from ntrip_client.rtcm_parser import RTCMParser


def make_frame(payload):
    """Build a valid RTCM3 frame (preamble, 10-bit length, payload, CRC)."""
    parser = RTCMParser()
    header = bytes([0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF])
    body = header + payload
    crc = parser._checksum(body)
    return body + bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])


def test_single_frame():
    parser = RTCMParser()
    frame = make_frame(b'\x43\x50' + b'\x01' * 20)
    packets = parser.parse(frame)
    assert packets == [frame]


def test_two_concatenated_frames():
    parser = RTCMParser()
    f1 = make_frame(b'\x43\x50' + b'\x01' * 10)
    f2 = make_frame(b'\x43\x60' + b'\x02' * 30)
    packets = parser.parse(f1 + f2)
    assert packets == [f1, f2]


def test_frame_split_across_calls():
    parser = RTCMParser()
    frame = make_frame(b'\x43\x50' + b'\x03' * 25)
    assert parser.parse(frame[:10]) == []
    assert parser.parse(frame[10:]) == [frame]


def test_resync_after_garbage_prefix():
    parser = RTCMParser()
    frame = make_frame(b'\x43\x50' + b'\x04' * 12)
    packets = parser.parse(b'\x00\x11\x22\x33' + frame)
    assert packets == [frame]


def test_false_preamble_skipped():
    parser = RTCMParser()
    frame = make_frame(b'\x43\x50' + b'\x05' * 12)
    # 0xD3 followed by nonzero reserved bits is not a real frame start
    packets = parser.parse(b'\xd3\xff' + frame)
    assert packets == [frame]


def test_corrupt_crc_dropped_and_resyncs():
    parser = RTCMParser()
    good = make_frame(b'\x43\x50' + b'\x06' * 12)
    bad = bytearray(make_frame(b'\x43\x50' + b'\x07' * 12))
    bad[-1] ^= 0xFF
    packets = parser.parse(bytes(bad) + good)
    assert packets == [good]


def test_interleaved_partial_delivery():
    parser = RTCMParser()
    frames = [make_frame(bytes([0x40 + i]) * (8 + i)) for i in range(5)]
    stream = b''.join(frames)
    collected = []
    # Deliver in awkward 7-byte slices
    for i in range(0, len(stream), 7):
        collected.extend(parser.parse(stream[i:i + 7]))
    assert collected == frames


def test_reset_drops_stale_partial_frame():
    # A partial frame left over from a dropped connection must not be
    # spliced onto the next connection's stream after reset().
    parser = RTCMParser()
    frame = make_frame(b'\x43\x50' + b'\x01' * 20)
    assert parser.parse(frame[:10]) == []
    assert parser._buffer == frame[:10]
    parser.reset()
    assert parser._buffer == b''
    assert parser.parse(frame[10:]) == []


def test_buffer_stays_bounded_under_garbage_flood():
    # Adversarial preamble-rich garbage and endless partial frames must
    # not grow the carry-over buffer without bound, and a valid frame
    # afterward must still decode.
    parser = RTCMParser()
    # False preambles with plausible lengths, never completing a frame
    garbage = (b'\xd3\x03\xff' + b'\xd3' * 5 + b'\x00' * 40) * 20
    for _ in range(200):
        parser.parse(garbage)
        assert len(parser._buffer) <= 10 * 1024
    # Flush any pending false frame (a stale bogus length can hold real
    # data hostage until enough bytes arrive), then confirm a valid
    # frame still decodes.
    parser.parse(b'\x00' * 2048)
    frame = make_frame(b'\x43\x50' + b'\x02' * 15)
    assert parser.parse(frame) == [frame]
