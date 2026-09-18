# Supersedes Insstdev, which decoded the extended solution status with the GNSS
# bit table instead of the INS one. Insstdev is kept so recordings made with it
# can still be played back; the driver publishes Insstdev2 on the same topic.
# INS PVA standard deviations

std_msgs/Header header

NovatelMessageHeader novatel_msg_header

float32 latitude_dev
float32 longitude_dev
float32 height_dev
float32 north_velocity_dev
float32 east_velocity_dev
float32 up_velocity_dev
float32 roll_dev
float32 pitch_dev
float32 azimuth_dev
NovatelInsExtendedSolutionStatus extended_solution_status
uint16 time_since_update

