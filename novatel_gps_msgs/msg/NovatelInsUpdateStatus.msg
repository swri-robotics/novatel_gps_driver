# Which updates the INS filter used at its last update epoch, from the
# INSUPDATESTATUS log.
# Based on https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSUPDATESTATUS.htm
std_msgs/Header header

NovatelMessageHeader novatel_msg_header

# Type of GNSS solution used for the last INS filter update, e.g. "INS_PSRSP"
string position_type

# Number of raw pseudorange, phase, and Doppler observations used
int32 num_psr
int32 num_adr
int32 num_dop

# Wheel sensor (DMI) status at the last update:
#   INACTIVE      - no wheel sensor connected
#   ACTIVE        - connected, but not used in the INS solution
#   USED          - used in the INS solution
#   BAD_MISC      - rejected due to a bad misclosure
#   HIGH_ROTATION - not used because of high rotation
string dmi_update_status

# ALIGN (dual antenna heading) status at the last update: INACTIVE, ACTIVE,
# USED, or BAD_MISC
string align_update_status

# The INS extended solution status word, left undecoded.  It uses the SPAN
# extended solution status bit table linked from the INSUPDATESTATUS reference
# above, which differs from the GNSS one in NovatelExtendedSolutionStatus.
uint32 extended_solution_status

# Bit mask of the updates enabled in the INS filter.
uint32 ins_enabled_updates
uint32 INS_UPDATE_POSITION = 1
uint32 INS_UPDATE_PHASE = 2
uint32 INS_UPDATE_ZERO_VELOCITY = 4
uint32 INS_UPDATE_WHEEL_SENSOR = 8
uint32 INS_UPDATE_ALIGN = 16
uint32 INS_UPDATE_EXTERNAL_POSITION = 32
uint32 INS_UPDATE_DOPPLER = 128
uint32 INS_UPDATE_PSEUDORANGE = 256
uint32 INS_UPDATE_VELOCITY = 512
uint32 INS_UPDATE_DEAD_RECKONING = 2048
uint32 INS_UPDATE_PHASE_WIND_UP = 4096
uint32 INS_UPDATE_COURSE_OVER_GROUND = 8192
uint32 INS_UPDATE_EXTERNAL_VELOCITY = 16384
uint32 INS_UPDATE_EXTERNAL_ATTITUDE = 32768
uint32 INS_UPDATE_EXTERNAL_HEADING = 65536
uint32 INS_UPDATE_EXTERNAL_HEIGHT = 131072
