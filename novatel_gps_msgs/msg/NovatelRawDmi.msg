# Raw wheel sensor (Distance Measurement Instrument) input, from the RAWDMI log.
# Based on https://docs.novatel.com/OEM7/Content/SPAN_Logs/RAWDMI.htm
std_msgs/Header header

NovatelMessageHeader novatel_msg_header

# Input value for each of the receiver's four DMI inputs.  How the value is
# interpreted (cumulative ticks or ticks since the last update) depends on the
# DMICONFIG source.  Only DMI1 is currently supported by the receiver.
int32[4] dmi

# Bit n is set if DMI input n+1 is active.
uint32 mask
