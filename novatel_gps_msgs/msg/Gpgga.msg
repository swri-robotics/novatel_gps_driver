# Message from GPGGA NMEA String
# Based on https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm
Header header

string message_id

# UTC seconds from midnight
float64 utc_seconds

float64 lat
float64 lon

string lat_dir
string lon_dir

uint32 GPS_QUAL_INVALID=0
uint32 GPS_QUAL_SINGLE_POINT=1
uint32 GPS_QUAL_PSEUDORANGE_DIFFERENTIAL=2
uint32 GPS_QUAL_RTK_FIXED_AMBIGUITY_SOLUTION=4
uint32 GPS_QUAL_RTK_FLOATING_AMBIGUITY_SOLUTION=5
uint32 GPS_QUAL_DEAD_RECKONING_MODE=6
uint32 GPS_QUAL_MANUAL_INPUT_MODE=7
uint32 GPS_QUAL_SIMULATION_MODE=8
uint32 GPS_QUAL_WASS=9
uint32 gps_qual

uint32 num_sats
float32 hdop 
float32 alt
string altitude_units

float32 undulation
string undulation_units
uint32 diff_age
string station_id

