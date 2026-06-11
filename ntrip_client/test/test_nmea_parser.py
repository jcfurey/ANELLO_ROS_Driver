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
