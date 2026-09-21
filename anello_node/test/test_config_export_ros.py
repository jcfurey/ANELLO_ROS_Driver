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

"""Exercise the installed exporter and real driver with local emulated devices."""

import json
import os
from pathlib import Path
import pty
import select
import socket
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from anello_interfaces.srv import CmdAndRsp
import pytest

from test_config_export import Device, frame
from test_ros_regressions import Driver


class EmulatedDevice:
    """Reply on a pseudo-terminal, or inject UDP replies from a loopback unit."""

    def __init__(self, transport, overrides=None):
        self.transport = transport
        self.model = Device(overrides)
        self.stop = threading.Event()
        self.failure = None
        self.handles = []
        self.socket = None
        self.thread = None
        if transport == 'UART':
            data, data_slave = pty.openpty()
            config, config_slave = pty.openpty()
            self.handles = [data, data_slave, config, config_slave]
            self.fd = config
            self.params = {'com_type': 'UART', 'uart_data_port': os.ttyname(data_slave),
                           'uart_config_port': os.ttyname(config_slave)}
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()
        else:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Unit destination port 2 is privileged on some CI hosts. Service
            # tests inject replies from the expected unit IP without binding it.
            self.socket.bind(('127.0.0.2', 0))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(('127.0.0.1', 0))
                self.reply_port = probe.getsockname()[1]
            self.fd = self.socket.fileno()
            self.params = {'com_type': 'ETH', 'remote_ip': '127.0.0.2',
                           'local_config_port': self.reply_port}

    def run(self):
        pending = b''
        try:
            while not self.stop.is_set():
                if not select.select([self.fd], [], [], 0.05)[0]:
                    continue
                data = os.read(self.fd, 8192)
                pending += data
                while b'\r\n' in pending:
                    request, pending = pending.split(b'\r\n', 1)
                    body = request[1:-3].decode('ascii')
                    assert request + b'\r\n' == frame(body).encode()
                    reply = self.model.command(body).encode()
                    # Exercise fragmented serial replies, including long read-all frames.
                    os.write(self.fd, reply[:37])
                    time.sleep(0.01)
                    os.write(self.fd, reply[37:])
        except Exception as exc:
            self.failure = exc

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2)
            assert not self.thread.is_alive()
        if self.socket:
            self.socket.close()
        for handle in self.handles:
            os.close(handle)
        assert self.failure is None, repr(self.failure)


@pytest.fixture
def device_driver(tmp_path):
    resources = []

    def start(transport, overrides=None):
        device = EmulatedDevice(transport, overrides)
        resources.append(device)
        # Build identity must ignore user-supplied startup overrides.
        driver = Driver(tmp_path, overrides={**device.params, 'driver_build_revision': 'spoofed'})
        resources.append(driver)
        return device, driver

    yield start
    for resource in reversed(resources):
        resource.close()


def run_export(driver, output):
    prefix = Path(get_package_prefix('anello_ros_driver'))
    executable = prefix / 'lib/anello_ros_driver/anello_config'
    return subprocess.run(
        [str(executable), 'export', '--node', driver.namespace + '/anello_ros_driver',
         '--output', str(output)], capture_output=True, text=True, timeout=30)


def test_installed_exporter_captures_long_replies_and_host_identity(
        device_driver, tmp_path):
    body = 'APCFG,odr,200,' + ','.join('field{},{}'.format(i, i) for i in range(100))
    device, driver = device_driver('UART', {'APCFG,R': frame(body)})
    output = tmp_path / 'snapshot.json'
    result = run_export(driver, output)
    assert result.returncode == 0, result.stdout + result.stderr
    snapshot = json.loads(output.read_text())
    assert snapshot['complete'] and snapshot['identity_consistent']
    flash = snapshot['configuration']['unit_flash']
    assert flash['raw_response'] == frame(body)
    assert len(flash['values']) == 101 and flash['values']['field99'] == '99'
    assert snapshot['configuration']['unit_ram']['values']['odr'] == '100'
    parameters = snapshot['host_parameters']['values']
    assert parameters['command_mode']['value'] == 'read_only'
    assert parameters['driver_build_revision']['value'] != 'spoofed'
    assert len(parameters['driver_build_source_sha256']['value']) == 64
    assert device.model.commands.count('APCFG,R') == 1
    assert len(device.model.commands) == 14
    driver.assert_alive()


def test_installed_exporter_retains_partial_evidence_and_returns_failure(device_driver, tmp_path):
    device, driver = device_driver('UART', {
        'APVEH,R': frame('APERR,7'), 'APCFG,r,ahdg': frame('APCFG,wrong,0'),
        'APIHW': '#APIHW,1*00\r\n'})
    output = tmp_path / 'partial.json'
    result = run_export(driver, output)
    assert result.returncode == 1, result.stdout + result.stderr
    snapshot = json.loads(output.read_text())
    assert not snapshot['complete']
    assert snapshot['configuration']['vehicle_flash']['status'] == 'unsupported'
    assert snapshot['configuration']['vehicle_flash']['raw_response'] == frame('APERR,7')
    assert snapshot['ram_controls']['ahdg']['status'] == 'invalid_reply'
    assert snapshot['identity']['imu_hardware']['status'] == 'driver_error'
    assert snapshot['configuration']['unit_flash']['status'] == 'ok'
    assert len(device.model.commands) == 14  # neither writes nor automatic retries
    driver.assert_alive()


@pytest.mark.parametrize('transport', ['UART', 'UDP'])
def test_reply_size_boundary_and_recovery(device_driver, transport):
    reply = frame('APCFG,k,' + 'x' * 4082)
    assert len(reply) == 4096
    device, driver = device_driver(transport, {'APCFG,R': reply})
    client = driver.node.create_client(CmdAndRsp, 'anello/send_cmd')
    assert client.wait_for_service(timeout_sec=3)

    def query():
        future = client.call_async(CmdAndRsp.Request(command='APCFG,R'))
        if device.socket:
            raw = device.model.command('APCFG,R').encode()
            deadline = time.monotonic() + 1
            while not future.done() and time.monotonic() < deadline:
                driver.spin(0.02)
                device.socket.sendto(raw, ('127.0.0.1', device.reply_port))
        driver.executor.spin_until_future_complete(future, timeout_sec=2)
        assert future.done()
        return future.result().response

    assert query() == reply
    driver.spin(0.56)
    device.model.overrides['APCFG,R'] = frame('APCFG,k,' + 'x' * 4083)
    assert 'exceeds 4096 bytes' in query()
    driver.spin(0.56)
    device.model.overrides['APCFG,R'] = frame('APCFG,odr,100')
    assert query() == frame('APCFG,odr,100')
    assert device.model.commands == ['APCFG,R'] * 3
    driver.assert_alive()


def test_udp_replies_filter_foreign_packets_and_bad_checksums(device_driver):
    device, driver = device_driver('UDP')
    client = driver.node.create_client(CmdAndRsp, 'anello/send_cmd')
    assert client.wait_for_service(timeout_sec=3)
    future = client.call_async(CmdAndRsp.Request(command='APCFG,R'))
    driver.spin(0.05)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as foreign:
        foreign.bind(('127.0.0.3', 0))
        foreign.sendto(b'x' * 5000, ('127.0.0.1', device.reply_port))
    device.socket.sendto(b'#APCFG,odr,999*00\r\n', ('127.0.0.1', device.reply_port))
    driver.spin(0.05)
    assert not future.done(), 'untrusted source or invalid frame completed the command'
    expected = frame('APCFG,odr,100')
    device.socket.sendto(expected.encode(), ('127.0.0.1', device.reply_port))
    driver.executor.spin_until_future_complete(future, timeout_sec=2)
    assert future.done() and future.result().response == expected
