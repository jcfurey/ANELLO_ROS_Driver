#!/usr/bin/env python3

import os
import json
import math
import queue
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rtcm_msgs.msg import Message as RTCM
from nmea_msgs.msg import Sentence

from ntrip_client.ntrip_client import NTRIPClient
from ntrip_client.worker import NTRIPWorker
from rcl_interfaces.msg import ParameterDescriptor


class NTRIPRos(Node):

    def __init__(self):
        super().__init__('ntrip_client')

        # Debug from environment (optional)
        try:
            self._debug = json.loads(os.environ.get("NTRIP_CLIENT_DEBUG", "false").lower())
        except (json.JSONDecodeError, ValueError):
            self._debug = False

        self.declare_parameters(
            namespace='',
            parameters=[(name, value, ParameterDescriptor(read_only=True))
                        for name, value in [
                ('host', '127.0.0.1'),
                ('port', 2101),
                ('mountpoint', 'mount'),
                ('ntrip_version', 'None'),
                ('authenticate', False),
                ('username', ''),
                ('password', ''),
                ('ssl', False),
                ('cert', 'None'),
                ('key', 'None'),
                ('ca_cert', 'None'),
                ('rtcm_frame_id', 'gnss_link'),
                ('nmea_max_age_seconds', 30.0),
                ('reconnect_attempt_max', NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_MAX),
                ('reconnect_attempt_wait_seconds',
                 NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_WAIT_SECONDS),
                ('rtcm_timeout_seconds', NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS),
                # NTRIP practice is a fresh GGA every 5-60 s; the ANELLO
                # driver publishes GGA at the 4 Hz APGPS rate, so the
                # forwarded stream is rate-limited here. 0 disables the
                # throttle.
                ('nmea_min_interval_seconds', 10.0),
            ]]
        )

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        mountpoint = self.get_parameter('mountpoint').value

        ntrip_version = self.get_parameter('ntrip_version').value
        if ntrip_version == 'None':
            ntrip_version = None

        if self._debug:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        username = None
        password = None
        if self.get_parameter('authenticate').value:
            username = self.get_parameter('username').value
            password = self.get_parameter('password').value
            if not username:
                self.get_logger().error(
                    'Requested to authenticate, but param "username" was not set')
                raise RuntimeError('NTRIP username not configured')
            if not password:
                self.get_logger().error(
                    'Requested to authenticate, but param "password" was not set')
                raise RuntimeError('NTRIP password not configured')

        self._rtcm_frame_id = self.get_parameter('rtcm_frame_id').value

        # RTCM corrections are low-rate and delivery-critical: publish
        # RELIABLE (KEEP_LAST depth 10) so a dropped frame cannot silently
        # delay RTK reconvergence. The driver subscribes RELIABLE to match.
        rtcm_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._rtcm_pub = self.create_publisher(RTCM, 'ntrip_client/rtcm', rtcm_qos)

        self._client = NTRIPClient(
            host=host,
            port=port,
            mountpoint=mountpoint,
            ntrip_version=ntrip_version,
            username=username,
            password=password,
            logerr=self.get_logger().error,
            logwarn=self.get_logger().warning,
            loginfo=self.get_logger().info,
            logdebug=self.get_logger().debug
        )

        self._client.ssl = self.get_parameter('ssl').value
        self._client.cert = self.get_parameter('cert').value
        self._client.key = self.get_parameter('key').value
        self._client.ca_cert = self.get_parameter('ca_cert').value
        if self._client.cert == 'None':
            self._client.cert = None
        if self._client.key == 'None':
            self._client.key = None
        if self._client.ca_cert == 'None':
            self._client.ca_cert = None

        self._client.reconnect_attempt_max = \
            self.get_parameter('reconnect_attempt_max').value
        self._client.reconnect_attempt_wait_seconds = \
            self.get_parameter('reconnect_attempt_wait_seconds').value
        self._client.rtcm_timeout_seconds = \
            self.get_parameter('rtcm_timeout_seconds').value

        self._nmea_min_interval = \
            self.get_parameter('nmea_min_interval_seconds').value
        for name in ('rtcm_timeout_seconds', 'nmea_max_age_seconds'):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError('{} must be finite and positive'.format(name))
        for name in ('reconnect_attempt_wait_seconds', 'nmea_min_interval_seconds'):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value < 0:
                raise ValueError('{} must be finite and nonnegative'.format(name))
        if not 1 <= port <= 65535:
            raise ValueError('NTRIP port out of range')
        self._worker = NTRIPWorker(
            self._client, self._nmea_min_interval,
            self.get_parameter('nmea_max_age_seconds').value)
        self._rtcm_timer = None
        self._diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._diagnostic_timer = self.create_timer(1.0, self.publish_diagnostics)

    def run(self):
        self._nmea_sub = self.create_subscription(
            Sentence, 'ntrip_client/nmea', self.subscribe_nmea, 10)
        self._rtcm_timer = self.create_timer(0.1, self.publish_rtcm)
        self._worker.start()
        return True

    def stop(self):
        if self._rtcm_timer:
            self._rtcm_timer.cancel()
        self._diagnostic_timer.cancel()
        self._worker.stop()

    def subscribe_nmea(self, nmea):
        self._worker.submit_nmea(nmea.sentence)

    def publish_rtcm(self):
        for _ in range(32):
            try:
                packet = self._worker.next_packet()
            except queue.Empty:
                break
            self._rtcm_pub.publish(RTCM(
                header=Header(stamp=self.get_clock().now().to_msg(),
                              frame_id=self._rtcm_frame_id),
                message=packet))

    def publish_diagnostics(self):
        connected = self._client.connected
        age = time.monotonic() - self._client._recv_rtcm_last_packet_timestamp
        fresh = (connected and self._client._first_rtcm_received
                 and age < self._client.rtcm_timeout_seconds)
        status = DiagnosticStatus(
            name=self.get_fully_qualified_name() + ': caster',
            hardware_id=self._client._host,
            level=DiagnosticStatus.OK if fresh else DiagnosticStatus.WARN,
            message='Receiving corrections' if fresh else 'Waiting for valid corrections',
            values=[KeyValue(key=key, value=str(value)) for key, value in (
                ('connected', connected), ('last_valid_correction_age_s', age),
                ('queue_depth', self._worker.packets.qsize()),
                ('queue_drops', self._worker.dropped_packets),
                ('expired_corrections', self._worker.expired_packets))])
        self._diagnostic_pub.publish(DiagnosticArray(
            header=Header(stamp=self.get_clock().now().to_msg()), status=[status]))


def main():
    rclpy.init()
    node = NTRIPRos()
    if not node.run():
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
