#!/usr/bin/env python3

import os
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from rtcm_msgs.msg import Message as RTCM
from nmea_msgs.msg import Sentence

from ntrip_client.ntrip_client import NTRIPClient


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
            parameters=[
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
                ('rtcm_frame_id', 'odom'),
                ('reconnect_attempt_max', NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_MAX),
                ('reconnect_attempt_wait_seconds',
                 NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_WAIT_SECONDS),
                ('rtcm_timeout_seconds', NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS),
                # NTRIP practice is a fresh GGA every 5-60 s; the ANELLO
                # driver publishes GGA at the 4 Hz APGPS rate, so the
                # forwarded stream is rate-limited here. 0 disables the
                # throttle.
                ('nmea_min_interval_seconds', 10.0),
            ]
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
        self._last_nmea_send = None
        self._next_connect_attempt = None
        self._rtcm_timer = None

    def run(self):
        if not self._client.connect():
            self.get_logger().error('Unable to connect to NTRIP server')
            return False

        self._nmea_sub = self.create_subscription(
            Sentence, 'ntrip_client/nmea', self.subscribe_nmea, 10)
        self._rtcm_timer = self.create_timer(0.1, self.publish_rtcm)
        return True

    def stop(self):
        self.get_logger().info('Shutting down NTRIP client')
        if self._rtcm_timer:
            self._rtcm_timer.cancel()
            self._rtcm_timer.destroy()
        self._client.disconnect()

    def subscribe_nmea(self, nmea):
        now = time.monotonic()
        if (
            self._nmea_min_interval > 0
            and self._last_nmea_send is not None
            and now - self._last_nmea_send < self._nmea_min_interval
        ):
            return
        self._last_nmea_send = now
        try:
            self._client.send_nmea(nmea.sentence)
        except Exception as e:
            # send_nmea can raise through reconnect() exhaustion; a
            # callback exception would kill the node. publish_rtcm's
            # recovery path re-establishes the connection.
            self.get_logger().error(
                'Failed to send NMEA to the NTRIP server: {}'.format(e))

    def publish_rtcm(self):
        # A lost caster must not be fatal: when the client is
        # disconnected (e.g. reconnect() exhausted its attempts and
        # raised), keep retrying at the reconnect cadence so
        # corrections resume when the caster comes back.
        if not self._client.connected:
            now = time.monotonic()
            if (
                self._next_connect_attempt is not None
                and now < self._next_connect_attempt
            ):
                return
            self._next_connect_attempt = \
                now + self._client.reconnect_attempt_wait_seconds
            self.get_logger().info('Attempting to reconnect to the NTRIP server')
            if not self._client.connect():
                return
        try:
            packets = self._client.recv_rtcm()
        except Exception as e:
            self.get_logger().error(
                'Lost connection to the NTRIP server, will keep '
                'retrying: {}'.format(e))
            return
        for packet in packets:
            rtcm_msg = RTCM(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self._rtcm_frame_id
                ),
                message=packet
            )
            self._rtcm_pub.publish(rtcm_msg)


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
