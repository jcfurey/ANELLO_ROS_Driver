#ifndef VERSION_H
#define VERSION_H

#ifndef MAJOR_VERSION
#define MAJOR_VERSION 3
#endif

#ifndef MINOR_VERSION
#define MINOR_VERSION 1
#endif

#ifndef PATCH_VERSION
#define PATCH_VERSION 0
#endif

/*
v3.1.0 : Audit fixes, hardening, and EVK alignment
            1. Fix RTCM QoS mismatch (corrections now reach the device)
            2. Memory-safety fixes in the ASCII/RTCM parser and sockets
            3. Convert standard interfaces to REP-103 ENU/FLU; anello/...
               topics stay device-native (NED/FRD)
            4. NavSatFix status from the full APINS status enumeration
            5. Drain data port per tick; non-blocking ethernet reads
            6. Route APODO to the dedicated odometer channel over ethernet
            7. heading_baseline parameter enables the HDG health check
            8. Rewrite the NTRIP RTCM frame parser (no duplicate frames)
            9. Source-filter inbound UDP; reject invalid remote IPs
            10. EVK reference doc aligned to the ANELLO manual
            11. colcon test green; warning-free build

v3.0.0 : Major ROS2 standards upgrade
            1. Composable node (rclcpp_components)
            2. Standard sensor_msgs/Imu and NavSatFix publishing
            3. TF2 transform broadcasting
            4. diagnostic_updater integration
            5. SensorDataQoS for all sensor publishers
            6. Header timestamps on all custom messages
            7. Configurable frame IDs via parameters
            8. APCOV covariance message and decoder
            9. CmdAndRsp service for device commands
            10. Baud rate as runtime parameter
            11. Python launch file with full parameter support
            12. Communication layer: exceptions instead of exit()
            13. Fix critical data_port init bug
            14. flake8-clean Python code (PEP 8)
            15. ament_lint_auto test framework

v2.0.1 : Bring in v1.3.4 changes to ros2

v2.0.0 : ROS2 port of the anello_ros_driver

v1.3.4 : Add ethernet support along with NTRIP bug fixes

v1.3.3 : Add GP2 topic

v1.3.2 : Better version control of ntrip client

v1.3.1 : Gyro discrepancy fix

v1.3.0 : Update Version Number

v1.2.3 : Added health message topic

v1.2.2 : Added option for config port to be disabled if the parameter is set to "OFF"

v1.2.1 : Added support for dynamic gga message generation

v1.2.0 : Feature List:
            1. Added IMU+ support
            2. Update .msg files to include all available data
            3. Update default values to match IMU+ and GNSS INS defaults

v1.1.3 : Fixed serial port connection on boot

v1.1.0 : Feature List:
            1. Added odometer forwarding
            2. Added data-port and config port auto-connect
            3. Add launch file and parameter support

v1.0.0 : Initial basic feature set

*/

#endif
