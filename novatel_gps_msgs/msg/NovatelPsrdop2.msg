# PSR Dilution of Precision (DOP) values
std_msgs/Header header

NovatelMessageHeader novatel_msg_header

# Geometry dilution of precision
float32 gdop
# Position dilution of precision
float32 pdop
# Horizontal dilution of precision
float32 hdop
# Vertical dilution of precision
float32 vdop

# Time dilution of precision for each GNSS system
NovatelPsrdop2System[] systems
