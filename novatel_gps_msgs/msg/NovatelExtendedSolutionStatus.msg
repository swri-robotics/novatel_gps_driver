# Bit    Mask      Description
#  0     0x01      Advance RTK verified (0 = not verified, 1 = verified)
# 1-3    0x0E      Pseudorange Ionosphere Correction
#                    0 = unknown
#                    1 = Klobuchar Broadcast
#                    2 = SBAS Broadcast
#                    3 = Multi-frequency Computed
#                    4 = PSRDiff Correction
#                    5 = NovaTel Blended Ionosphere Value
# 4-7  0xF0        <Reserved>
uint32 original_mask
bool advance_rtk_verified
string psuedorange_iono_correction
