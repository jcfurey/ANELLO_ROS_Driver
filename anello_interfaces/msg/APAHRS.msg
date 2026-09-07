# Device-native Ground IMU AHRS angles. Yaw is relative unless initialized externally.
# See the integration guide before interpreting this as an absolute orientation.
std_msgs/Header header
float64 time       # MCU milliseconds
float64 sync_time  # Last synchronization pulse in nanoseconds
float64 roll       # degrees
float64 pitch      # degrees
float64 yaw        # degrees
uint8 zupt
