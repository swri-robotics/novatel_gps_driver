# The extended solution status of an INS log (INSPVAX, INSSTDEV, INSUPDATESTATUS).
# Based on the Extended solution status table in
# https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSATTX.htm
# This is a different bit table than the one GNSS logs use, which is decoded
# into NovatelExtendedSolutionStatus.
uint32 original_mask

# Whether each type of update was used in the INS solution
bool position_update
bool phase_update
bool zero_velocity_update
bool wheel_sensor_update
bool align_update
bool external_position_update
bool doppler_update
bool pseudorange_update
bool velocity_update
bool dead_reckoning_update
bool phase_wind_up_update
bool course_over_ground_update
bool external_velocity_update
bool external_attitude_update

bool ins_solution_converged
bool secondary_ins_solution_used
bool turn_on_biases_estimated
bool alignment_direction_verified

# How the INS was aligned, from bits 26-28: one of INCOMPLETE, STATIC,
# KINEMATIC, DUAL_ANTENNA, USER_COMMAND, NVM_SEED, or UNKNOWN
string alignment_type

# Status of the INS seed from non-volatile memory, from bits 29-31: one of
# INVALID, FAILED_VALIDATION, PENDING_VALIDATION, INJECTED, or UNKNOWN.
# NovAtel doesn't document a value of 0, which is reported as UNKNOWN.
string nvm_seed_status
