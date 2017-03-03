#Satellite Range information structure used in range messages

#Satelite PRN number of range measurement
uint16 prn_number

#GLONASS Frequency
uint16 glofreq

#Pseudorange measurement(m)
float64 psr

#Pseudorange measurement standard deviation(m)
float32 psr_std

#Carrier phase, in cycles
float64 adr

#Estimated carrier phase standard deviation(cycles)
float32 adr_std

#Instantaneous carrier Doppler frequency(Hz)
float32 dopp

#Carrier to noise density ratio
float32 noise_density_ratio

## of seconds of continous tracking
float32 locktime

#Tracking status
uint32 tracking_status
