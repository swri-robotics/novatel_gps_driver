# Parsed Best Position or Omnistar XP or HP pos data from Novatel OEM6 receiver
std_msgs/Header header

# Clock model status
# see Table 65 on pg 322 of the OEM6 Family Firmware Reference Manual, Rev3
string clock_status

# Receiver clock offset, in seconds, from GPS reference time. A positive offset
# implies that the receiver clock is ahead of GPS reference time. To derive
# GPS reference time, use the following formula:
# GPS reference time = receiver time - offset
float64 offset

# Standard deviation of the offset
float64 offset_std

# The offset of the GPS reference time from UTC time, computed using almanac
# parameters. UTC time is GPS reference time plus the current UTC offset plus
# the receiver clock offset:
# UTC time = GPS reference time + offset + UTC offset
float64 utc_offset

uint32 utc_year
uint8 utc_month
uint8 utc_day
uint8 utc_hour
uint8 utc_minute
uint32 utc_millisecond

string utc_status