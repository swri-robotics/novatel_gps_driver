# Raw IMU data, from the RAWIMUX or RAWIMUSX log, e.g. for post-processing.
# Based on https://docs.novatel.com/OEM7/Content/SPAN_Logs/RAWIMUX.htm
std_msgs/Header header

NovatelMessageHeader novatel_msg_header

# Bit 0 is set if the IMU reported an error; see imu_status for details.
uint8 imu_info
uint8 IMU_INFO_ERROR = 1

# The IMU type identifier; see the IMU Type table in the OEM7 reference.
uint8 imu_type

uint32 gps_week_num
float64 gps_seconds

# The IMU's status, as the bytes the receiver sent them in.  What they mean
# depends on the IMU type; see its IMU status table in the OEM7 reference.
uint8[4] imu_status

# Changes in velocity and angle since the previous log, in raw counts.  Multiply
# by the IMU type's scale factor from the Raw IMU Scale Factors table in the OEM7
# reference to get velocity and angle increments, and by the IMU's sample rate to
# get accelerations and rotation rates.  The receiver reports the y axis values
# negated, and they're left that way here.
int32 z_acceleration
int32 negated_y_acceleration
int32 x_acceleration
int32 z_rotation
int32 negated_y_rotation
int32 x_rotation
