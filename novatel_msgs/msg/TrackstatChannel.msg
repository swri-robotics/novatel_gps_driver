# A submessage of Trackstat that contains all of the data about a single hardware channel

# Satellite PRN number
int16 prn

# GLONASS Frequency +7
int16 glofreq

# Channel tracking status
uint32 ch_tr_status

# Pseudorange (m)
float64 psr

# Doppler frequency (Hz)
float32 doppler

# Carrier to noise density ratio (dB-Hz)
float32 c_no

# Number of seconds of continuous tracking (no cycle slips)
float32 locktime

# Pseudorange residual from pseudorange filter (m)
float32 psr_res

# Range reject code from pseudorange filter
string reject

# Pseudorange filter weighting
float32 psr_weight