# Parsed Best Position in the WGS-84 ECEF frame from Novatel receiver
std_msgs/Header header

NovatelMessageHeader novatel_msg_header

# Position Data (m)
string solution_status
string position_type

float64 x
float64 y
float64 z

# Position Standard Deviation (m)
float32 x_sigma
float32 y_sigma
float32 z_sigma

# Velocity Data
string velocity_solution_status
string velocity_type

float64 x_vel
float64 y_vel
float64 z_vel

# Velocity Standard Deviation (m/s)
float32 x_vel_sigma
float32 y_vel_sigma
float32 z_vel_sigma

string base_station_id
float32 velocity_latency

# Data Ages (sec)
float32 diff_age
float32 solution_age

# Satellite Usage
uint8 num_satellites_tracked
uint8 num_satellites_used_in_solution
uint8 num_gps_and_glonass_l1_used_in_solution
uint8 num_gps_and_glonass_l1_and_l2_used_in_solution

# Extended Solution Status
NovatelExtendedSolutionStatus extended_solution_status

# Signal Masks
NovatelSignalMask signal_mask