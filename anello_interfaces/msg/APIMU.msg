std_msgs/Header header

# Device MCU time in ms
float64 mcu_time
# External-sync timestamp in ms (firmware >= v1.0.39; 0.0 when sync is
# disabled or the firmware does not output the field)
float64 t_sync
float64 ax
float64 ay
float64 az
float64 wx
float64 wy
float64 wz
float64 wz_fog
float64 odometer_speed
# Odometer timestamp in SECONDS (unlike mcu_time/t_sync, which are ms) —
# both the ASCII and RTCM decoders scale it to s before publishing
float64 odometer_time
float64 temp
