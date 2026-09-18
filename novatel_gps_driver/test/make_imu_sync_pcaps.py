#!/usr/bin/env python3
"""Generates synthetic pcap fixtures for tests that need specific log combinations.

The receivers that exposed https://github.com/swri-robotics/novatel_gps_driver/issues/127
log a corrected-IMU message paired with an INS position/velocity/attitude message,
which is the combination NovatelGps::GenerateImuMessages needs before it will emit a
sensor_msgs/Imu. Neither of the recorded captures in this directory contains such a
pair, so these files are synthesized instead of captured from hardware.

This also covers https://github.com/swri-robotics/novatel_gps_driver/issues/101,
which needs a capture with BESTPOS, BESTVEL, and INSPVAX all present so a test can
check which one GetFixMessages() actually used for GPSFix::track.

It also covers https://github.com/swri-robotics/novatel_gps_driver/issues/2, where
GetFixMessages() stopped publishing GPSFix messages entirely if BESTVEL logs arrived
too far behind their BESTPOS logs, so there are captures where BESTVEL lags behind
BESTPOS or goes missing.

For https://github.com/swri-robotics/novatel_gps_driver/issues/14 there's a capture
of the wheel sensor logs, RAWDMI and INSUPDATESTATUS, in both ASCII and binary.

Each NovAtel log is placed in its own TCP segment on port 3001, matching the framing
NovatelGps::ReadData expects from a pcap connection.

Run from this directory to regenerate the fixtures:

    python3 make_imu_sync_pcaps.py
"""

import struct

WEEK = 1820
START_SECONDS = 160205.900
IMU_PERIOD_S = 0.010          # 100 Hz, the rate reported in issue #127
PAIR_COUNT = 10

CORRIMUDATA_ID = 812
CORRIMUS_ID = 2264
INSPVA_ID = 507
INSPVAS_ID = 508
INSSTDEV_ID = 2051
INSCOV_ID = 264
RAWDMI_ID = 2269
INSUPDATESTATUS_ID = 1825

TIME_STATUS_FINESTEERING = 180
INS_SOLUTION_GOOD = 3

# Distinctive values so the tests can assert on what came out the far end.
PITCH_RATE, ROLL_RATE, YAW_RATE = 0.001, 0.002, 0.003
LATERAL_ACC, LONGITUDINAL_ACC, VERTICAL_ACC = 0.01, 0.02, 0.03
ROLL_DEG, PITCH_DEG, AZIMUTH_DEG = 1.0, 2.0, 3.0
ROLL_DEV, PITCH_DEV, AZIMUTH_DEV = 1.0, 2.0, 3.0
# INSCOV reports variances rather than standard deviations, in deg^2.
ROLL_VAR, PITCH_VAR, AZIMUTH_VAR = 0.25, 1.0, 2.25


# --- NovAtel framing -------------------------------------------------------

def crc32_value(i):
    """Mirrors NovatelMessageExtractor::CRC32Value."""
    crc = i
    for _ in range(8):
        crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc


def block_crc32(buf):
    """Mirrors NovatelMessageExtractor::CalculateBlockCRC32."""
    crc = 0
    for byte in buf:
        crc = ((crc >> 8) & 0x00FFFFFF) ^ crc32_value((crc ^ byte) & 0xFF)
    return crc


def long_message(message_id, gps_ms, payload):
    """A standard 28-byte-header NovAtel binary log."""
    header = struct.pack(
        '<BBBBHbBHHBBHIIHH',
        0xAA, 0x44, 0x12, 28,      # sync bytes + header length
        message_id, 0, 32,         # id, type (binary), port address
        len(payload), 0,           # message length, sequence
        0, TIME_STATUS_FINESTEERING,
        WEEK, gps_ms,
        0, 0, 0,                   # receiver status, reserved, sw version
    )
    body = header + payload
    return body + struct.pack('<I', block_crc32(body))


def short_message(message_id, gps_ms, payload):
    """A 12-byte-header NovAtel binary log, used by the "S" variants."""
    header = struct.pack('<BBBBHHI', 0xAA, 0x44, 0x13, len(payload),
                         message_id, WEEK, gps_ms)
    body = header + payload
    return body + struct.pack('<I', block_crc32(body))


# --- Log payloads ----------------------------------------------------------

def corrimudata_payload(seconds):
    return struct.pack('<Id6d', WEEK, seconds, PITCH_RATE, ROLL_RATE, YAW_RATE,
                       LATERAL_ACC, LONGITUDINAL_ACC, VERTICAL_ACC)


def corrimus_payload():
    # CORRIMUS carries an IMU sample count where CORRIMUDATA carries week/seconds;
    # its time comes from the short header instead.
    payload = struct.pack('<I6d', 1, PITCH_RATE, ROLL_RATE, YAW_RATE,
                          LATERAL_ACC, LONGITUDINAL_ACC, VERTICAL_ACC)
    return payload.ljust(60, b'\x00')   # CorrImusParser expects 60 bytes


def inspva_payload(seconds):
    return struct.pack('<Id9dI', WEEK, seconds,
                       29.443917634921949, -98.614755510637181, 250.0,
                       0.0, 0.0, 0.0,
                       ROLL_DEG, PITCH_DEG, AZIMUTH_DEG,
                       INS_SOLUTION_GOOD)


def insstdev_payload():
    payload = struct.pack('<9fIH', 0.1, 0.1, 0.1, 0.01, 0.01, 0.01,
                          ROLL_DEV, PITCH_DEV, AZIMUTH_DEV, 0, 0)
    return payload.ljust(52, b'\x00')   # InsstdevParser expects 52 bytes


def inscov_payload(seconds):
    def diagonal(x, y, z):
        return [x, 0.0, 0.0, 0.0, y, 0.0, 0.0, 0.0, z]

    covariances = (diagonal(1.0, 1.0, 1.0) +
                   diagonal(ROLL_VAR, PITCH_VAR, AZIMUTH_VAR) +
                   diagonal(0.01, 0.01, 0.01))
    # 4 + 8 + 27 * 8 = 228 bytes, the length InscovParser expects.
    return struct.pack('<Id27d', WEEK, seconds, *covariances)


def ascii_message(log_name, seconds, body_fields, port='COM1', sequence=0,
                   idle_time=50.0, receiver_status='00000000', reserved=0, sw_version=0):
    """A NovAtel ASCII log: '#NAME,header;body*crc32\\r\\n'."""
    header = '%s,%s,%d,%.1f,FINESTEERING,%d,%.3f,%s,%d,%d' % (
        log_name, port, sequence, idle_time, WEEK, seconds, receiver_status, reserved, sw_version)
    sentence = header + ';' + ','.join(body_fields)
    checksum = block_crc32(sentence.encode('ascii'))
    return ('#' + sentence + '*%08x\r\n' % checksum).encode('ascii')


# BESTVEL's track_ground is Doppler-derived and gets noisy at low speed; INSPVAX's
# azimuth is the SPAN filter's true direction of travel and doesn't have that
# problem. These are deliberately different so the GPSFix track test can tell
# which one GetFixMessages() actually used.
BESTVEL_TRACK_DEG = 45.0
INSPVAX_AZIMUTH_DEG = 270.0


def bestpos_fields():
    return ['SOL_COMPUTED', 'SINGLE', '29.443917634921949', '-98.614755510637181',
            '250.0000', '-26.0000', 'WGS84', '0.0200', '0.0200', '0.0300', '""',
            '0.000', '0.000', '8', '8', '8', '8', '0', '06', '00', '03']


def bestvel_fields():
    return ['SOL_COMPUTED', 'DOPPLER_VELOCITY', '0.250', '0.000', '0.0500',
            '%.4f' % BESTVEL_TRACK_DEG, '0.0000', '00000000']


def inspvax_fields():
    return ['INS_SOLUTION_GOOD', 'INS_PSRSP', '29.443917634921949', '-98.614755510637181',
            '250.0000', '-26.0000', '0.0300', '-0.0300', '0.0000', '1.0000', '2.0000',
            '%.4f' % INSPVAX_AZIMUTH_DEG, '0.0200', '0.0200', '0.0300', '0.0100',
            '0.0100', '0.0200', '0.0500', '0.0500', '0.1000', '00000000', '0']


# BESTPOS/BESTVEL captures for the GPSFix sync tests: 20 Hz, like the driver's
# default bestposa/bestvela log period.
FIX_PERIOD_S = 0.05
FIX_COUNT = 40


def lagged_fix_messages(lag):
    """BESTPOS for every epoch, with each BESTVEL arriving `lag` epochs late."""
    messages = []
    for i in range(FIX_COUNT + lag):
        if i < FIX_COUNT:
            messages.append(ascii_message('BESTPOSA', START_SECONDS + i * FIX_PERIOD_S,
                                          bestpos_fields()))
        if i >= lag:
            messages.append(ascii_message('BESTVELA', START_SECONDS + (i - lag) * FIX_PERIOD_S,
                                          bestvel_fields()))
    return messages


def dropped_fix_messages(dropped):
    """BESTPOS and BESTVEL for every epoch, except for a missing BESTVEL at `dropped`."""
    messages = []
    for i in range(FIX_COUNT):
        seconds = START_SECONDS + i * FIX_PERIOD_S
        if i != dropped:
            messages.append(ascii_message('BESTVELA', seconds, bestvel_fields()))
        messages.append(ascii_message('BESTPOSA', seconds, bestpos_fields()))
    return messages


# Wheel sensor logs.  The ASCII ones are the examples from NovAtel's RAWDMI and
# INSUPDATESTATUS references; the binary ones carry different values so a test
# can tell which is which.
RAWDMI_ASCII = (b'#RAWDMIA,COM1,0,24.0,FINESTEERING,2048,427043.137,02004048,b411,32768;'
                b'2297,0,0,0,00000001*61b727c0\r\n')
INSUPDATESTATUS_ASCII = (b'#INSUPDATESTATUSA,COM3,0,49.0,FINESTEERING,2117,416218.000,02004020,78f1,32768;'
                         b'INS_PSRSP,0,22,24,INACTIVE,USED,0b0020c3,007ff3bf,0,0*c1d6e8bc\r\n')
RAWDMI_BINARY_TICKS = 4096
INS_PSRSP = 53
DMI_USED = 2


def rawdmi_payload(ticks):
    return struct.pack('<4iI', ticks, 0, 0, 0, 1)


def insupdatestatus_payload(dmi_status):
    return struct.pack('<I3iII4I', INS_PSRSP, 7, 22, 24, dmi_status, 0,
                       0x0b0020c3, 0x007ff3bf, 0, 0)


# --- pcap / TCP framing ----------------------------------------------------

SRC_IP = bytes((192, 168, 74, 10))
DST_IP = bytes((192, 168, 74, 1))
SRC_PORT, DST_PORT = 49152, 3001        # the driver filters on tcp dst port 3001


def ones_complement_sum(data):
    if len(data) % 2:
        data += b'\x00'
    total = sum(struct.unpack('>%dH' % (len(data) // 2), data))
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def tcp_packet(payload, seq):
    tcp = struct.pack('>HHIIBBHHH', SRC_PORT, DST_PORT, seq, 1,
                      5 << 4, 0x18, 8192, 0, 0)
    pseudo = SRC_IP + DST_IP + struct.pack('>BBH', 0, 6, len(tcp) + len(payload))
    tcp = tcp[:16] + struct.pack('>H', ones_complement_sum(pseudo + tcp + payload)) + tcp[18:]

    ip = struct.pack('>BBHHHBBH', 0x45, 0, 20 + len(tcp) + len(payload),
                     0, 0x4000, 64, 6, 0) + SRC_IP + DST_IP
    ip = ip[:10] + struct.pack('>H', ones_complement_sum(ip)) + ip[12:]

    ethernet = b'\x02\x00\x00\x00\x00\x01\x02\x00\x00\x00\x00\x02\x08\x00'
    return ethernet + ip + tcp + payload


def write_pcap(path, messages):
    """One NovAtel log per TCP segment, so each read yields one complete message."""
    out = [struct.pack('<IHHiIII', 0xA1B2C3D4, 2, 4, 0, 0, 65535, 1)]
    seq = 1
    for i, message in enumerate(messages):
        packet = tcp_packet(message, seq)
        out.append(struct.pack('<IIII', 1424000000 + i // 100, (i % 100) * 10000,
                               len(packet), len(packet)))
        out.append(packet)
        seq += len(message)
    with open(path, 'wb') as handle:
        handle.write(b''.join(out))
    print('wrote %s (%d messages)' % (path, len(messages)))


def main():
    start_ms = int(round(START_SECONDS * 1000))
    long_msgs = [long_message(INSSTDEV_ID, start_ms, insstdev_payload())]
    short_msgs = list(long_msgs)
    # A receiver logging INSCOV as well as INSSTDEV; GenerateImuMessages prefers
    # INSCOV, so this capture exercises the other covariance branch.
    cov_msgs = [long_message(INSCOV_ID, start_ms, inscov_payload(START_SECONDS))]

    for i in range(PAIR_COUNT):
        seconds = START_SECONDS + i * IMU_PERIOD_S
        gps_ms = int(round(seconds * 1000))
        long_msgs.append(long_message(CORRIMUDATA_ID, gps_ms, corrimudata_payload(seconds)))
        long_msgs.append(long_message(INSPVA_ID, gps_ms, inspva_payload(seconds)))
        short_msgs.append(short_message(CORRIMUS_ID, gps_ms, corrimus_payload()))
        short_msgs.append(short_message(INSPVAS_ID, gps_ms, inspva_payload(seconds)))
    cov_msgs.extend(long_msgs[1:])

    write_pcap('corrimudata-inspva-sync.pcap', long_msgs)
    write_pcap('corrimus-inspvas-sync.pcap', short_msgs)
    write_pcap('corrimudata-inspva-inscov.pcap', cov_msgs)

    track_msgs = [
        ascii_message('INSPVAXA', START_SECONDS, inspvax_fields()),
        ascii_message('BESTVELA', START_SECONDS, bestvel_fields()),
        ascii_message('BESTPOSA', START_SECONDS, bestpos_fields()),
    ]
    write_pcap('bestpos-bestvel-inspvax-sync.pcap', track_msgs)

    # Within the default 1 s sync timeout and the 10-message sync buffer.
    write_pcap('bestpos-bestvel-lag5.pcap', lagged_fix_messages(5))
    # Farther behind than the sync buffer can hold.
    write_pcap('bestpos-bestvel-lag15.pcap', lagged_fix_messages(15))
    write_pcap('bestpos-bestvel-dropped.pcap', dropped_fix_messages(10))

    start_ms = int(round(START_SECONDS * 1000))
    write_pcap('rawdmi-insupdatestatus.pcap', [
        RAWDMI_ASCII,
        INSUPDATESTATUS_ASCII,
        long_message(RAWDMI_ID, start_ms, rawdmi_payload(RAWDMI_BINARY_TICKS)),
        long_message(INSUPDATESTATUS_ID, start_ms, insupdatestatus_payload(DMI_USED)),
    ])


if __name__ == '__main__':
    main()
