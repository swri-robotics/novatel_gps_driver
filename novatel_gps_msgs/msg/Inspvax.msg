# message 1465

std_msgs/Header header

NovatelMessageHeader novatel_msg_header

# Table 29 in the SPAN on OEM6 manual:
# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=121
string ins_status


# Table 30 in the SPAN on OEM6 manual:
# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=124
string position_type


float64 latitude
float64 longitude
float64 altitude

float32 undulation

float64 north_velocity
float64 east_velocity
float64 up_velocity

float64 roll
float64 pitch
float64 azimuth

float32 latitude_std
float32 longitude_std
float32 altitude_std

float32 north_velocity_std
float32 east_velocity_std
float32 up_velocity_std

float32 roll_std
float32 pitch_std
float32 azimuth_std

NovatelExtendedSolutionStatus extended_status

uint16 seconds_since_update
