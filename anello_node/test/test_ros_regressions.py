"""Exercise the installed driver and launch files using local UDP and ROS topics."""

from collections import defaultdict
import math
import os
from pathlib import Path
import pty
import select
import signal
import socket
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from anello_interfaces.srv import CmdAndRsp
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Sentence
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix
from tf2_msgs.msg import TFMessage
import yaml


def frame(body):
    """Wrap a body in the device ASCII checksum and delimiters."""
    checksum = 0
    for byte in body.encode():
        checksum ^= byte
    return ('#{}*{:02X}\r\n'.format(body, checksum)).encode()


def ins(ms, status=4, lat=37, lon=-122):
    return 'APINS,{},1400000000000000000,{},{},{},10,0,0,0,0,0,90,1'.format(
        ms, status, lat, lon)


def imu(ms, kind='APIMU'):
    if kind == 'APIM1':
        return 'APIM1,{},0,0,0,-1,1,2,3,4,25'.format(ms)
    return 'APIMU,{},0,0,0,-1,1,2,3,4,0,{},25'.format(ms, ms)


def cov(ms):
    return 'APCOV,{},1,2,3,0,0,0,4,5,6,0,0,0,1,2,3,0,0,0'.format(ms)


class Driver:

    def __init__(self, tmp_path, overrides=None, launch=None, arguments=()):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.namespace = '/anello_test_' + uuid.uuid4().hex[:8]
        self.node = rclpy.create_node('observer', namespace=self.namespace, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.messages = defaultdict(list)
        for key, msg_type, topic in (
                ('imu', Imu, 'imu/data'), ('raw', Imu, 'imu/data_raw'),
                ('odom', Odometry, 'ins/odometry'), ('fix', NavSatFix, 'ins/fix'),
                ('gps', NavSatFix, 'gps/fix'), ('gga', Sentence, 'ntrip_client/nmea'),
                ('diag', DiagnosticArray, '/diagnostics'),
                ('tf', TFMessage, '/tf')):
            self.node.create_subscription(
                msg_type, topic, lambda msg, k=key: self.messages[k].append(msg),
                qos_profile_sensor_data)
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(3)]
        for sock in sockets:
            sock.bind(('127.0.0.1', 0))
        ports = [sock.getsockname()[1] for sock in sockets]
        for sock in sockets:
            sock.close()
        self.port = ports[0]
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        params = {'com_type': 'ETH', 'remote_ip': '127.0.0.1',
                  'local_data_port': ports[0], 'local_config_port': ports[1],
                  'local_odometer_port': ports[2], 'timestamp_source': 'arrival'}
        params.update(overrides or {})
        params_path = tmp_path / 'driver.yaml'
        params_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        self.log = (tmp_path / 'driver.log').open('w+')
        if launch:
            command = ['ros2', 'launch', 'anello_ros_driver', launch,
                       'params_file:=' + str(params_path), *arguments]
        else:
            executable = Path(get_package_prefix('anello_ros_driver')) / 'lib' / \
                'anello_ros_driver' / 'anello_ros_driver_node'
            command = [str(executable), '--ros-args', '-r', '__ns:=' + self.namespace,
                       '--params-file', str(params_path)]
        self.process = subprocess.Popen(command, stdout=self.log, stderr=self.log,
                                        start_new_session=True)
        self.launch = launch
        try:
            self.spin(1.0)
            self.assert_alive()
        except Exception:
            self.close()
            raise

    def assert_alive(self):
        self.log.flush()
        assert self.process.poll() is None, Path(self.log.name).read_text()

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def send(self, *bodies, delay=0.08):
        self.socket.sendto(b''.join(frame(body) for body in bodies), ('127.0.0.1', self.port))
        self.spin(delay)
        self.assert_alive()

    def clock(self, seconds):
        message = Clock()
        message.clock.sec = seconds
        for _ in range(3):
            self.clock_pub.publish(message)
            self.spin(0.05)

    def parameters(self, target, names):
        client = self.node.create_client(GetParameters, target + '/get_parameters')
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(GetParameters.Request(names=names))
        self.executor.spin_until_future_complete(future, timeout_sec=5)
        assert future.done()
        return future.result().values

    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)
            self.process.wait(timeout=2)
            pytest.fail('Driver/launch failed to shut down')
        finally:
            self.socket.close()
            self.executor.shutdown()
            self.node.destroy_node()
            self.context.shutdown()
            self.log.close()


@pytest.fixture
def driver_factory(tmp_path):
    drivers = []

    def start(**kwargs):
        driver = Driver(tmp_path, **kwargs)
        drivers.append(driver)
        return driver

    yield start
    for driver in reversed(drivers):
        driver.close()


def test_invalid_navigation_never_publishes_position_or_tf(driver_factory):
    driver = driver_factory(overrides={'publish_tf': True})
    driver.send(ins(1000, status=0, lat=0, lon=0))
    assert driver.messages['imu']
    assert not driver.messages['odom'] and not driver.messages['tf']
    assert driver.messages['imu'][-1].angular_velocity_covariance[0] == -1
    driver.send(ins(1010))
    assert len(driver.messages['odom']) == 1
    driver.send(ins(1020, status=8, lat=0, lon=0))
    assert len(driver.messages['odom']) == 1
    assert len(driver.messages['tf']) == 1
    assert driver.messages['fix'][-1].status.status == -1
    assert math.isnan(driver.messages['fix'][-1].latitude)
    odom = driver.messages['odom'][-1]
    assert odom.header.frame_id == 'anello_local'
    assert odom.child_frame_id == 'ins_link'
    assert odom.pose.covariance[0] == 1e6


def test_acquisition_and_wall_age_expire_imu_and_covariance(driver_factory):
    driver = driver_factory(overrides={'covariance.device_convention': 'verified_m2_deg2_euler'})
    driver.send(imu(1000), cov(1000), ins(1010))
    assert driver.messages['imu'][-1].angular_velocity_covariance[0] > 0
    assert driver.messages['fix'][-1].position_covariance_type == 3
    driver.send(ins(1400))
    assert driver.messages['imu'][-1].angular_velocity_covariance[0] == -1
    assert driver.messages['fix'][-1].position_covariance_type == 0
    driver.send(imu(1500, kind='APIM1'), cov(1500), ins(1510))
    assert driver.messages['imu'][-1].angular_velocity_covariance[0] > 0
    driver.spin(0.3)
    driver.send(ins(1520))
    assert driver.messages['imu'][-1].linear_acceleration_covariance[0] == -1
    assert driver.messages['fix'][-1].position_covariance_type == 0


def test_unverified_covariance_and_mems_defaults(driver_factory):
    driver = driver_factory(overrides={'use_fog_wz': False})
    driver.send(imu(1000), cov(1000), ins(1010))
    assert driver.messages['fix'][-1].position_covariance_type == 0
    raw = driver.messages['raw'][-1]
    assert raw.angular_velocity_covariance[8] == pytest.approx(7.6e-7)
    assert raw.angular_velocity.z == pytest.approx(-3 * math.pi / 180)
    assert raw.linear_acceleration.z == pytest.approx(9.80665)
    assert not driver.messages['tf']


def test_bad_frames_cannot_crash_or_publish(driver_factory):
    driver = driver_factory(overrides={'timestamp_source': 'mcu'})
    for i in range(110):
        driver.send(imu(1000 + 10 * i), delay=0.003)
    count = len(driver.messages['raw'])
    driver.send(imu('nan'), imu('1e309'), imu(3000).replace(',-1,', ',nan,'),
                'APIMU,3000,0,0,-1,1,2,3,4,0')
    assert len(driver.messages['raw']) == count
    driver.send('APGPS,3000,1e308,37,-122,10,0,0,0,1,1,1,3,20,1,1,0')
    assert not driver.messages['gps']
    driver.send(ins(3010, status=255))
    assert not driver.messages['odom']
    driver.send(imu(3020))
    assert len(driver.messages['raw']) == count + 1


def test_reboot_invalidates_cached_measurements(driver_factory):
    driver = driver_factory(overrides={'covariance.device_convention': 'verified_m2_deg2_euler'})
    driver.send(imu(10000), cov(10000), ins(10010))
    driver.send(ins(10))
    assert driver.messages['imu'][-1].angular_velocity_covariance[0] == -1
    assert driver.messages['fix'][-1].position_covariance_type == 0


def test_2d_fix_height_is_unavailable(driver_factory):
    driver = driver_factory()
    driver.send('APGPS,1000,1400000000000000000,37,-122,123.4,0,0,0,1,1,1,2,20,1,1,2')
    nav = driver.messages['gps'][-1]
    assert math.isnan(nav.altitude)
    assert nav.status.service == 0
    assert nav.position_covariance_type == 0


def test_stream_loss_cannot_be_hidden_by_ins_traffic(driver_factory):
    driver = driver_factory(overrides={'stream_timeout': 0.2})
    driver.send(imu(1000), ins(1010))
    for i in range(25):
        driver.send(ins(1020 + 100 * i), delay=0.1)
    statuses = [status for msg in driver.messages['diag'] for status in msg.status
                if driver.namespace in status.name and status.name.endswith('Device Status')]
    assert statuses
    assert statuses[-1].level != DiagnosticStatus.OK
    assert 'imu' in statuses[-1].message and 'gps' in statuses[-1].message


def test_sim_clock_zero_pause_and_backward_jump(driver_factory):
    driver = driver_factory(overrides={'use_sim_time': True, 'publish_tf': True})
    driver.send(ins(1000))
    assert not driver.messages['imu']
    driver.clock(100)
    driver.send(ins(1010))
    assert driver.messages['odom'][-1].header.stamp.sec == 100
    count = len(driver.messages['odom'])
    driver.send(ins(1020))
    assert len(driver.messages['odom']) == count
    driver.spin(1.0)
    driver.send(ins(1025))
    assert len(driver.messages['odom']) == count
    driver.clock(50)
    driver.send(ins(1030))
    assert driver.messages['odom'][-1].header.stamp.sec == 50


@pytest.mark.parametrize('launch', ['anello_driver.launch.py', 'anello_ros_driver_launch.xml'])
def test_installed_launch_preserves_types_and_parameter_file(driver_factory, launch):
    assert Path(get_package_share_directory('anello_ros_driver'), 'launch', launch).is_file()
    driver = driver_factory(
        launch=launch, overrides={'flip_accel_sign': True, 'use_fog_wz': False},
        arguments=['uart_config_port:=OFF', 'ntrip_host:=127.0.0.1', 'ntrip_port:=1',
                   'ntrip_username:=123456', 'ntrip_password:=000012',
                   'timestamp_source:=arrival', 'covariance_angular_velocity:=[0.1,0.2,0.3]'])
    values = driver.parameters('/anello_ros_driver', [
        'uart_config_port', 'flip_accel_sign', 'use_fog_wz', 'timestamp_source',
        'covariance.angular_velocity'])
    assert values[0].string_value == 'OFF'
    assert values[1].bool_value and not values[2].bool_value
    assert values[3].string_value == 'arrival'
    assert list(values[4].double_array_value) == [0.1, 0.2, 0.3]
    values = driver.parameters('/ntrip_client', ['username', 'password'])
    assert values[0].string_value == '123456'
    assert values[1].string_value == '000012'
    driver.spin(1.2)
    statuses = [status for msg in driver.messages['diag'] for status in msg.status
                if status.name == '/ntrip_client: caster']
    assert statuses and statuses[-1].level == DiagnosticStatus.WARN


def test_multiple_ins_packets_in_one_read_are_preserved(driver_factory):
    driver = driver_factory()
    driver.send(ins(1000), ins(1010), ins(1020))
    assert len(driver.messages['odom']) == 3
    stamps = [msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
              for msg in driver.messages['odom']]
    assert stamps[0] < stamps[1] < stamps[2]


def test_zero_covariance_does_not_claim_perfect_accuracy(driver_factory):
    driver = driver_factory(overrides={'covariance.device_convention': 'verified_m2_deg2_euler'})
    driver.send('APCOV,1000,' + ','.join(['0'] * 18), ins(1010))
    assert driver.messages['fix'][-1].position_covariance_type == 0
    assert driver.messages['odom'][-1].pose.covariance[0] == 1e6


def test_saturated_fog_is_not_published_as_precise_rate(driver_factory):
    driver = driver_factory()
    driver.send(imu(1000).replace(',1,2,3,4,', ',1,2,250,200,'), ins(1010))
    assert driver.messages['raw'][-1].angular_velocity_covariance[0] == -1
    assert driver.messages['imu'][-1].angular_velocity_covariance[0] == -1
    assert driver.messages['imu'][-1].linear_acceleration_covariance[0] > 0
    assert driver.messages['odom'][-1].twist.covariance[35] == 1e6


def test_oversized_udp_datagram_is_dropped_as_a_whole(driver_factory):
    driver = driver_factory()
    driver.socket.sendto(frame(imu(1000)) + b'x' * 2000, ('127.0.0.1', driver.port))
    driver.spin(0.2)
    assert not driver.messages['raw']
    driver.send(imu(1010))
    assert len(driver.messages['raw']) == 1


def test_gga_does_not_mislabel_pdop_as_hdop(driver_factory):
    driver = driver_factory(overrides={'gps_utc_leap_seconds': 19})
    driver.send('APGPS,1000,100000000000,37,-122,30,10,0,0,1,1,9.9,3,20,1,1,0')
    sentence = driver.messages['gga'][-1].sentence
    fields = sentence.split(',')
    assert fields[1] == '000121.000'
    assert fields[8] == ''  # HDOP is unavailable; APGPS only reports PDOP.
    assert fields[9] == '10.0' and fields[11] == '20.0'
    assert len(sentence) <= 82


def test_command_reply_validation_preserves_receive_progress(driver_factory):
    data_master, data_slave = pty.openpty()
    config_master, config_slave = pty.openpty()
    try:
        driver = driver_factory(overrides={
            'com_type': 'UART', 'uart_data_port': os.ttyname(data_slave),
            'uart_config_port': os.ttyname(config_slave)})
        client = driver.node.create_client(CmdAndRsp, 'anello/send_cmd')
        assert client.wait_for_service(timeout_sec=5)
        rejected = client.call_async(CmdAndRsp.Request(command='APPNG\nAPRST,0'))
        driver.executor.spin_until_future_complete(rejected, timeout_sec=1)
        assert rejected.done() and rejected.result().response.startswith('ERROR:')
        assert not select.select([config_master], [], [], 0)[0]

        future = client.call_async(CmdAndRsp.Request(command='APVEH,R,bsl'))
        assert select.select([config_master], [], [], 2)[0]
        assert os.read(config_master, 512) == frame('APVEH,R,bsl')
        bad_checksum = bytearray(frame('APVEH,bsl,1.0'))
        bad_checksum[-3] = ord('0') if bad_checksum[-3] != ord('0') else ord('1')
        reply = frame('APVEH,bsl,2.5')
        os.write(config_master, frame('APODO,0') + bad_checksum + reply[:9])
        os.write(data_master, frame(imu(1000)))
        driver.spin(0.12)
        assert driver.messages['raw']  # command wait must not stall data polling
        assert not future.done()
        os.write(config_master, reply[9:])
        driver.executor.spin_until_future_complete(future, timeout_sec=1)
        assert future.done() and future.result().response == reply.decode()
    finally:
        for fd in (data_master, data_slave, config_master, config_slave):
            os.close(fd)


@pytest.mark.parametrize('name,value', [
    ('baud_rate', -1), ('local_data_port', 2**32 + 1111),
    ('heading_baseline', -1.0), ('covariance.angular_velocity', [0.1, -1.0, 0.1]),
    ('tf_child_frame', 'base_link'), ('tf_parent_frame', 'odom')])
def test_invalid_configuration_fails_cleanly(driver_factory, name, value):
    with pytest.raises(AssertionError, match=name):
        driver_factory(overrides={name: value})
