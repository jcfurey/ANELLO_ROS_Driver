"""Validation tests for the NMEA sentence checker."""

from ntrip_client.nmea_parser import NMEAParser


def make_sentence(body):
    """Append a correct checksum and terminator to an NMEA body."""
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return '${}*{:02X}\r\n'.format(body, checksum)


GGA_BODY = ('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,'
            '545.4,M,46.9,M,,')


def test_valid_gga_accepted():
    parser = NMEAParser()
    assert parser.is_valid_sentence(make_sentence(GGA_BODY))


def test_bad_checksum_rejected():
    parser = NMEAParser()
    sentence = make_sentence(GGA_BODY)
    flipped = format(int(sentence[-4:-2], 16) ^ 0xFF, '02X')
    tampered = sentence[:-4] + flipped + '\r\n'
    assert not parser.is_valid_sentence(tampered)


def test_missing_dollar_rejected():
    parser = NMEAParser()
    assert not parser.is_valid_sentence(make_sentence(GGA_BODY)[1:])


def test_missing_terminator_rejected():
    parser = NMEAParser()
    assert not parser.is_valid_sentence(make_sentence(GGA_BODY).rstrip())


def test_over_length_rejected():
    parser = NMEAParser()
    long_body = 'GPGGA,' + '9' * 90
    assert not parser.is_valid_sentence(make_sentence(long_body))


def test_non_hex_checksum_rejected_not_raised():
    # A corrupted checksum field must be rejected, not raise ValueError
    # out of the subscription callback (which would kill the node).
    parser = NMEAParser()
    assert not parser.is_valid_sentence('$GPGGA,fake*XX\r\n')
    assert not parser.is_valid_sentence('$GPGGA,fake*\r\n')
    assert not parser.is_valid_sentence('$GPGGA,fake*-1\r\n')


def test_length_boundary():
    # NMEA 0183 caps a sentence at 82 characters: exactly 82 must pass,
    # 83 must be rejected. The C++ GGA builder promises to stay within
    # this limit, so the boundary is an interop contract.
    parser = NMEAParser()
    body = 'GPGGA,' + '9' * 70  # -> 82 chars once framed
    sentence = make_sentence(body)
    assert len(sentence) == 82
    assert parser.is_valid_sentence(sentence)
    assert not parser.is_valid_sentence(make_sentence(body + '9'))


def test_extra_star_uses_last_separator():
    # A '*' inside the body must not confuse the checksum split: the
    # LAST separator delimits the checksum field.
    parser = NMEAParser()
    assert parser.is_valid_sentence(make_sentence('GPTXT,note*worthy'))


def test_lowercase_hex_checksum_accepted():
    parser = NMEAParser()
    sentence = make_sentence(GGA_BODY)
    lowered = sentence[:-4] + sentence[-4:-2].lower() + '\r\n'
    assert parser.is_valid_sentence(lowered)


def test_checksum_valid_embedded_line_ending_is_rejected():
    assert not NMEAParser().is_valid_sentence(make_sentence('GPGGA,ok\r\nINJECTED'))


def test_checksum_must_have_exactly_two_hex_digits():
    sentence = make_sentence(GGA_BODY)
    assert not NMEAParser().is_valid_sentence(sentence[:-4] + '0' + sentence[-4:])
