std_msgs/Header header

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
float64 odometer_time
float64 temp
