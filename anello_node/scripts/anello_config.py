#!/usr/bin/env python3
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

"""Export a read-only ANELLO device snapshot through the running ROS driver."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import time


IDENTITY = {
    'product': 'APPID', 'serial': 'APSER', 'version': 'APVER',
    'imu_hardware': 'APIHW', 'fog_hardware': 'APFHW', 'fog_serial': 'APFSN',
}
SECTIONS = {'unit_ram': 'APCFG,r', 'unit_flash': 'APCFG,R', 'vehicle_flash': 'APVEH,R'}
RAM_CONTROLS = {'azupt': 'APCFG,r,azupt', 'ahdg': 'APCFG,r,ahdg'}
READ_COMMANDS = frozenset((*IDENTITY.values(), *SECTIONS.values(), *RAM_CONTROLS.values()))
KEY = re.compile(r'[A-Za-z0-9_]{1,32}')
FRAME = re.compile(r'#([\x20-\x22\x24-\x29\x2b-\x7e]+)\*([0-9a-fA-F]{2})\r\n')
ERROR_NAMES = {
    1: 'missing start', 2: 'missing read/write mode', 3: 'incomplete message',
    4: 'invalid checksum', 5: 'invalid talker', 6: 'invalid message type',
    7: 'invalid field', 8: 'invalid value', 9: 'flash locked',
    10: 'unexpected character', 11: 'feature disabled',
}


def utc_now():
    """Return an unambiguous UTC capture time."""
    return datetime.now(timezone.utc).isoformat()


def decode_reply(raw, command, settings=False):
    """Validate the entire reply and retain wire values without guessing units."""
    if raw.startswith('ERROR:'):
        return {'status': 'driver_error', 'error': raw}
    if len(raw) > 4096:
        return {'status': 'invalid_reply', 'error': 'reply exceeds 4096 bytes'}
    match = FRAME.fullmatch(raw)
    if match is None:
        return {'status': 'invalid_reply', 'error': 'expected one ASCII frame with CRLF'}
    body = match.group(1)
    checksum = 0
    for byte in body.encode('ascii'):
        checksum ^= byte
    if checksum != int(match.group(2), 16):
        return {'status': 'invalid_reply', 'error': 'checksum mismatch'}
    fields = body.split(',')
    result = {'checksum_valid': True}
    if fields[0] == 'APERR':
        if len(fields) != 2 or not fields[1].isdigit():
            return {**result, 'status': 'invalid_reply', 'error': 'malformed APERR'}
        code = int(fields[1])
        return {**result, 'status': 'unsupported' if code in (6, 7, 11) else 'device_error',
                'error_code': code, 'error': ERROR_NAMES.get(code, 'unknown device error')}
    if fields[0] != command.split(',')[0]:
        return {**result, 'status': 'invalid_reply', 'error': 'unexpected response identifier'}
    if not settings:
        if len(fields) != 2 or not fields[1]:
            return {**result, 'status': 'invalid_reply', 'error': 'expected one identity value'}
        return {**result, 'status': 'ok', 'value': fields[1]}
    payload = fields[1:]
    if not payload or len(payload) % 2:
        return {**result, 'status': 'invalid_reply', 'error': 'expected nonempty key/value pairs'}
    values = {}
    for key, value in zip(payload[::2], payload[1::2]):
        if KEY.fullmatch(key) is None or key in values:
            return {**result, 'status': 'invalid_reply', 'error': 'invalid or duplicate key'}
        values[key] = value
    requested = command.split(',')[2:]
    if any(key not in values for key in requested):
        return {**result, 'status': 'invalid_reply', 'error': 'reply omits a requested key'}
    return {**result, 'status': 'ok', 'values': values}


def capture_query(transport, command, settings=False):
    """Keep the exact service response and the outcome of one read-only query."""
    if command not in READ_COMMANDS:
        raise ValueError('export only permits its fixed read-only queries')
    result = {'command': command, 'started_utc': utc_now(), 'raw_response': None}
    start = time.monotonic()
    try:
        result['raw_response'] = transport.command(command)
        result.update(decode_reply(result['raw_response'], command, settings))
    except (RuntimeError, ValueError) as exc:
        result.update(status='transport_error', error=str(exc))
    result['finished_utc'] = utc_now()
    result['elapsed_seconds'] = round(time.monotonic() - start, 6)
    return result


def collect_snapshot(transport, node_name, service_name):
    """Collect evidence, retaining failures and marking any partial export."""
    snapshot = {
        'format': 'anello_device_snapshot_v1', 'started_utc': utc_now(),
        'complete': False, 'identity_consistent': None,
        'driver_node': node_name, 'command_service': service_name,
        'exporter_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'identity': {}, 'configuration': {}, 'ram_controls': {}, 'identity_after': {},
        'host_parameters': {'status': 'not_captured'},
    }
    try:
        transport.verify_target()
        try:
            snapshot['host_parameters'] = {
                'status': 'ok', 'captured_utc': utc_now(), 'values': transport.parameters()}
        except (RuntimeError, ValueError) as exc:
            snapshot['host_parameters'] = {'status': 'error', 'error': str(exc)}
        for name, command in IDENTITY.items():
            snapshot['identity'][name] = capture_query(transport, command)
        for name, command in SECTIONS.items():
            snapshot['configuration'][name] = capture_query(transport, command, settings=True)
        # The vendor UI probes these separately; older read-all replies may omit them.
        for name, command in RAM_CONTROLS.items():
            snapshot['ram_controls'][name] = capture_query(transport, command, settings=True)
        for name in ('product', 'serial', 'version'):
            snapshot['identity_after'][name] = capture_query(transport, IDENTITY[name])
        pairs = [(snapshot['identity'][name], result)
                 for name, result in snapshot['identity_after'].items()]
        if all(before['status'] == after['status'] == 'ok' for before, after in pairs):
            snapshot['identity_consistent'] = all(
                before['value'] == after['value'] for before, after in pairs)
        groups = ('identity', 'configuration', 'ram_controls', 'identity_after')
        records = [record for group in groups
                   for record in snapshot[group].values()]
        snapshot['complete'] = (
            snapshot['host_parameters']['status'] == 'ok'
            and snapshot['identity_consistent'] is True
            and all(record['status'] == 'ok' for record in records))
    except (RuntimeError, ValueError) as exc:
        snapshot['error'] = str(exc)
    except KeyboardInterrupt:
        snapshot['error'] = 'capture interrupted'
    snapshot['finished_utc'] = utc_now()
    return snapshot


class RosTransport:
    """Own ROS clients without opening another device serial port or UDP socket."""

    def __init__(self, node_name, service_name, timeout):
        """Initialize bounded service calls in an isolated ROS context."""
        import rclpy
        from anello_interfaces.srv import CmdAndRsp
        from rcl_interfaces.srv import GetParameters, ListParameters
        from rclpy.executors import SingleThreadedExecutor

        self.ros = rclpy
        self.context = rclpy.context.Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node(
            'anello_config_export_' + str(os.getpid()), context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.target = node_name
        self.service_name = service_name
        self.timeout = timeout
        self.next_command = 0.0
        self.command_type = CmdAndRsp
        self.list_type = ListParameters
        self.get_type = GetParameters
        self.command_client = self.node.create_client(CmdAndRsp, service_name)
        self.list_client = self.node.create_client(ListParameters, node_name + '/list_parameters')
        self.get_client = self.node.create_client(GetParameters, node_name + '/get_parameters')

    def verify_target(self):
        """Require the selected driver node to advertise the selected command service."""
        namespace, name = self.target.rsplit('/', 1)
        namespace = namespace or '/'
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            nodes = self.node.get_node_names_and_namespaces()
            if nodes.count((name, namespace)) > 1:
                raise RuntimeError('driver node name is duplicated in the ROS graph')
            if (name, namespace) in nodes:
                services = self.node.get_service_names_and_types_by_node(name, namespace)
                if any(path == self.service_name and 'anello_interfaces/srv/CmdAndRsp' in types
                       for path, types in services):
                    return
            self.executor.spin_once(timeout_sec=0.05)
        raise RuntimeError('selected driver node does not advertise ' + self.service_name)

    def call(self, client, request):
        """Wait within a single wall-clock budget; do not retry timed-out requests."""
        deadline = time.monotonic() + self.timeout
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError('service unavailable: ' + client.srv_name)
        future = client.call_async(request)
        self.executor.spin_until_future_complete(
            future, timeout_sec=max(0.0, deadline - time.monotonic()))
        if not future.done():
            client.remove_pending_request(future)
            raise RuntimeError('service timeout: ' + client.srv_name)
        result = future.result()
        if result is None:
            raise RuntimeError('service returned no result: ' + client.srv_name)
        return result

    def command(self, command):
        """Pace this client's queries below the driver's two-per-second budget."""
        if command not in READ_COMMANDS:
            raise ValueError('not an exporter query')
        time.sleep(max(0.0, self.next_command - time.monotonic()))
        try:
            request = self.command_type.Request(command=command)
            return self.call(self.command_client, request).response
        finally:
            # Space from completion, so late dispatch cannot bunch requests.
            self.next_command = time.monotonic() + 0.55

    def parameters(self):
        """Capture typed host parameters, including the running driver's build identity."""
        names = sorted(self.call(self.list_client, self.list_type.Request()).result.names)
        if not names:
            raise RuntimeError('driver returned no host parameters')
        response = self.call(self.get_client, self.get_type.Request(names=names))
        if len(response.values) != len(names):
            raise RuntimeError('incomplete host parameter reply')
        fields = ('', 'bool_value', 'integer_value', 'double_value', 'string_value',
                  'byte_array_value', 'bool_array_value', 'integer_array_value',
                  'double_array_value', 'string_array_value')
        result = {}
        for name, parameter in zip(names, response.values):
            kind = parameter.type
            if not 1 <= kind < len(fields):
                raise RuntimeError('unset or unknown parameter type: ' + name)
            value = getattr(parameter, fields[kind])
            if kind >= 5:
                value = list(value)
                if kind == 5:
                    value = [int.from_bytes(v, 'little') if isinstance(v, bytes) else v
                             for v in value]
            result[name] = {'type': fields[kind].removesuffix('_value'), 'value': value}
        # Refuse non-JSON numeric values before any output file is committed.
        json.dumps(result, allow_nan=False)
        return result

    def close(self):
        """Release ROS resources after capture."""
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.try_shutdown()


def write_snapshot(path, snapshot):
    """Publish a new snapshot atomically without replacing existing evidence."""
    text = json.dumps(snapshot, indent=2, sort_keys=True, allow_nan=False) + '\n'
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.anello-snapshot-', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink()


def main(argv=None):
    """Export JSON evidence; return nonzero for a partial or failed snapshot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['export'])
    parser.add_argument('--output', required=True, type=Path, help='new JSON snapshot file')
    parser.add_argument('--node', default='/anello_ros_driver', help='fully qualified driver node')
    parser.add_argument('--service',
                        help='command service; default: NODE_NAMESPACE/anello/send_cmd')
    parser.add_argument('--timeout', type=float, default=3.0,
                        help='ROS service timeout in seconds')
    args = parser.parse_args(argv)
    if not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', args.node):
        parser.error('--node must be a fully qualified ROS node name')
    service = args.service or args.node.rsplit('/', 1)[0] + '/anello/send_cmd'
    if not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', service):
        parser.error('--service must be a fully qualified ROS service name')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    output = args.output.expanduser().absolute()
    if os.path.lexists(output) or not output.parent.is_dir():
        parser.error('--output must be a new file in an existing directory')
    try:
        transport = RosTransport(args.node, service, args.timeout)
        try:
            snapshot = collect_snapshot(transport, args.node, service)
        finally:
            transport.close()
        write_snapshot(output, snapshot)
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print('Export failed: ' + str(exc), file=sys.stderr)
        return 2
    status = 'Complete' if snapshot['complete'] else 'Partial'
    print('{} snapshot saved to {}'.format(status, output))
    if not snapshot['complete']:
        print('Inspect per-query status and host_parameters in the snapshot.', file=sys.stderr)
    return 0 if snapshot['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
