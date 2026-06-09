#!/usr/bin/env python3

import os
import json

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

        rtcm_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
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
        if hasattr(self, '_rtcm_timer') and self._rtcm_timer:
            self._rtcm_timer.cancel()
            self._rtcm_timer.destroy()
        self._client.disconnect()

    def subscribe_nmea(self, nmea):
        self._client.send_nmea(nmea.sentence)

    def publish_rtcm(self):
        for packet in self._client.recv_rtcm():
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
