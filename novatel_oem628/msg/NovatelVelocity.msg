# Parsed Best Position or Omnistar XP or HP pos data from Novatel OEM6 receiver
Header header

NovatelMessageHeader novatel_msg_header

string solution_status
string velocity_type

# A measure of the latency in the velocity time tag in seconds.
float32 latency

# Differential age in seconds
float32 age

# Horizontal speed over ground, meters/second
float64 horizontal_speed

# Direction of motion over ground with respect to true North, degrees
float64 track_ground

# Vertical speed, where positive values correspond to increasing altitude, meters/second
float64 vertical_speed