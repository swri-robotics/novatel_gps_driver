# Parsed Heading East of North from Novatel receiver. Only ALIGN capable recievers can publish this
std_msgs/Header header

NovatelMessageHeader novatel_msg_header

# Solution Status
string solution_status
string position_type

# Baseline length (m)
float32 baseline_length

# Heading in degrees [0,360)
float32 heading

# Pitch in degrees +- 90
float32 pitch

# Orientation Standard Deviations (deg)
float32 heading_sigma
float32 pitch_sigma

# Station ids
string rover_station_id
string master_station_id

# Satellite Usage
uint8 num_satellites_tracked
uint8 num_satellites_used_in_solution
uint8 num_satellites_above_elevation_mask_angle
uint8 num_satellites_above_elevation_mask_angle_l2

# Enum for solution source
uint8 solution_source

uint8 SOURCE_PRIMARY_ANTENNA=0
uint8 SOURCE_SECONDARY_ANTENNA=1

# Extended Solution Status
NovatelExtendedSolutionStatus extended_solution_status

# Signal Masks
NovatelSignalMask signal_mask