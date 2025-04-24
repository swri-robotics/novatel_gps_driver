# Parsed corrected IMU data from Novatel OEM6 receiver
std_msgs/Header header

NovatelMessageHeader novatel_msg_header

# Count of the number of IMU Samples used in each log output accumulation.
# 0 if unknown.
uint32 imu_data_count
uint32 gps_week_num
float64 gps_seconds

# All measurements in this message are instantaneous values;
# attitude rate is in radians
float64 pitch_rate
float64 roll_rate
float64 yaw_rate

# accelerations are in m/s
float64 lateral_acceleration
float64 longitudinal_acceleration
float64 vertical_acceleration
