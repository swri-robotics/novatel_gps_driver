# Parsed Best UTM data from Novatel OEM6 receiver
Header header

NovatelMessageHeader novatel_msg_header

string solution_status
string position_type

# Position Data
uint32 lon_zone_number
string lat_zone_letter
float64 northing
float64 easting
float64 height

float32 undulation
string datum_id

# Accuracy Statistics (units?)
float32 northing_sigma
float32 easting_sigma
float32 height_sigma
string base_station_id
float32 diff_age
float32 solution_age
uint8 num_satellites_tracked
uint8 num_satellites_used_in_solution
uint8 num_gps_and_glonass_l1_used_in_solution
uint8 num_gps_and_glonass_l1_and_l2_used_in_solution

NovatelExtendedSolutionStatus extended_solution_status

NovatelSignalMask signal_mask