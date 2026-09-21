# Copyright (c) 2026 ANELLO Photonics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Check snapshot evidence and failures without a ROS graph or physical device."""

import importlib.util
import json
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location(
    'anello_config', Path(__file__).parents[1] / 'scripts' / 'anello_config.py')
config = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(config)


def frame(body):
    """Create a checksum-valid device response."""
    checksum = 0
    for byte in body.encode('ascii'):
        checksum ^= byte
    return '#{}*{:02X}\r\n'.format(body, checksum)


class Device:
    """Supply deterministic responses and record every transmitted body."""

    def __init__(self, overrides=None, serial_after='0000123'):
        self.commands = []
        self.overrides = overrides or {}
        self.serial_after = serial_after

    def verify_target(self):
        pass

    def parameters(self):
        return {'baud_rate': {'type': 'integer', 'value': 230400}}

    def command(self, body):
        self.commands.append(body)
        if body in self.overrides:
            result = self.overrides[body]
            if isinstance(result, Exception):
                raise result
            return result
        replies = {
            'APPID': 'APPID,IMU-A1', 'APSER': 'APSER,0000123', 'APVER': 'APVER,1.3.53',
            'APIHW': 'APIHW,1', 'APFHW': 'APFHW,2', 'APFSN': 'APFSN,0000004',
            'APCFG,r': 'APCFG,odr,100,mfm,1', 'APCFG,R': 'APCFG,odr,200,mfm,4',
            'APVEH,R': 'APVEH,bsl,1.000000,bcal,0',
            'APCFG,r,azupt': 'APCFG,azupt,0', 'APCFG,r,ahdg': 'APCFG,ahdg,-90000',
        }
        if body == 'APSER' and self.commands.count('APSER') > 1:
            return frame('APSER,' + self.serial_after)
        return frame(replies[body])


def test_export_preserves_storage_raw_replies_strings_and_identity_check():
    device = Device()
    result = config.collect_snapshot(device, '/driver', '/anello/send_cmd')
    assert result['complete'] and result['identity_consistent']
    assert result['identity']['serial']['value'] == '0000123'
    assert result['configuration']['unit_ram']['values']['odr'] == '100'
    assert result['configuration']['unit_flash']['values']['odr'] == '200'
    assert result['configuration']['vehicle_flash']['values']['bsl'] == '1.000000'
    assert result['ram_controls']['ahdg']['values']['ahdg'] == '-90000'
    assert result['configuration']['unit_flash']['raw_response'] == frame('APCFG,odr,200,mfm,4')
    assert result['host_parameters']['values']['baud_rate']['value'] == 230400
    serial = result['identity']['serial']
    assert serial['started_utc'] <= serial['finished_utc']
    assert device.commands == [*config.IDENTITY.values(), *config.SECTIONS.values(),
                               *config.RAM_CONTROLS.values(), 'APPID', 'APSER', 'APVER']


@pytest.mark.parametrize('reply,status', [
    (frame('APERR,7'), 'unsupported'), (frame('APERR,11'), 'unsupported'),
    (frame('APERR,9'), 'device_error'), ('ERROR: command rate limit', 'driver_error'),
    (RuntimeError('timeout'), 'transport_error'), (frame('APCFG,wrong,0'), 'invalid_reply'),
])
def test_partial_results_are_retained_and_never_reported_complete(reply, status):
    result = config.collect_snapshot(Device({'APCFG,r,ahdg': reply}), '/driver', '/service')
    assert not result['complete']
    assert result['ram_controls']['ahdg']['status'] == status
    assert result['configuration']['vehicle_flash']['status'] == 'ok'
    assert result['identity_consistent']
    if isinstance(reply, str):
        assert result['ram_controls']['ahdg']['raw_response'] == reply


def test_identity_change_invalidates_an_otherwise_successful_capture():
    result = config.collect_snapshot(Device(serial_after='different'), '/driver', '/service')
    assert not result['complete'] and result['identity_consistent'] is False
    assert result['identity']['serial']['value'] == '0000123'
    assert result['identity_after']['serial']['value'] == 'different'


@pytest.mark.parametrize('reply', [
    frame('APCFG,key,1,key,2'), frame('APCFG,key,1,orphan'), frame('APCFG,,1'),
    frame('APCFG,R,odr,100'), frame('APVEH,key,1'), frame('APCFG,bad-key,1'),
    frame('APCFG,key,1')[:-2], frame('APCFG,key,1') * 2, frame('APERR,7,extra'),
    '#APCFG,key,1*00\r\n', '#APCFG,key,é*00\r\n', frame('APCFG,key,' + 'x' * 4096),
])
def test_malformed_or_unmatched_reply_cannot_become_configuration(reply):
    assert config.decode_reply(reply, 'APCFG,R', settings=True)['status'] == 'invalid_reply'


def test_long_configuration_and_unknown_fields_are_preserved():
    body = 'APCFG,' + ','.join('field{},{}'.format(i, i) for i in range(100))
    result = config.decode_reply(frame(body), 'APCFG,R', settings=True)
    assert result['status'] == 'ok' and len(result['values']) == 100
    assert result['values']['field99'] == '99'
    assert len(frame(body)) > 510


def test_export_cannot_send_a_write_reset_or_arbitrary_read():
    device = Device()
    for command in ('APCFG,W,odr,200', 'APRST,0', 'APVEH,r', 'APECH,hello'):
        with pytest.raises(ValueError):
            config.capture_query(device, command)
    assert not device.commands


def test_snapshot_write_does_not_replace_existing_evidence(tmp_path):
    path = tmp_path / 'snapshot.json'
    snapshot = config.collect_snapshot(Device(), '/driver', '/service')
    config.write_snapshot(path, snapshot)
    assert json.loads(path.read_text()) == snapshot
    original = path.read_bytes()
    with pytest.raises(FileExistsError):
        config.write_snapshot(path, {'different': 'data'})
    assert path.read_bytes() == original
    assert not list(tmp_path.glob('.anello-snapshot-*'))


def test_target_and_host_parameter_failures_are_visible():
    class MissingHost(Device):

        def parameters(self):
            raise RuntimeError('host parameters unavailable')

    class WrongTarget(Device):

        def verify_target(self):
            raise RuntimeError('command service is not owned by selected node')

    result = config.collect_snapshot(MissingHost(), '/driver', '/service')
    assert not result['complete'] and result['host_parameters']['status'] == 'error'
    device = WrongTarget()
    result = config.collect_snapshot(device, '/driver', '/service')
    assert not result['complete'] and 'owned' in result['error']
    assert not device.commands
