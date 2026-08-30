#!/usr/bin/env python3
"""Generates the synthetic pcap fixtures used by the IMU synchronization tests.

The receivers that exposed https://github.com/swri-robotics/novatel_gps_driver/issues/127
log a corrected-IMU message paired with an INS position/velocity/attitude message,
which is the combination NovatelGps::GenerateImuMessages needs before it will emit a
sensor_msgs/Imu. Neither of the recorded captures in this directory contains such a
pair, so these files are synthesized instead of captured from hardware.

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

TIME_STATUS_FINESTEERING = 180
INS_SOLUTION_GOOD = 3

# Distinctive values so the tests can assert on what came out the far end.
PITCH_RATE, ROLL_RATE, YAW_RATE = 0.001, 0.002, 0.003
LATERAL_ACC, LONGITUDINAL_ACC, VERTICAL_ACC = 0.01, 0.02, 0.03
ROLL_DEG, PITCH_DEG, AZIMUTH_DEG = 1.0, 2.0, 3.0
ROLL_DEV, PITCH_DEV, AZIMUTH_DEV = 1.0, 2.0, 3.0


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
    long_msgs = [long_message(INSSTDEV_ID, int(round(START_SECONDS * 1000)),
                              insstdev_payload())]
    short_msgs = list(long_msgs)

    for i in range(PAIR_COUNT):
        seconds = START_SECONDS + i * IMU_PERIOD_S
        gps_ms = int(round(seconds * 1000))
        long_msgs.append(long_message(CORRIMUDATA_ID, gps_ms, corrimudata_payload(seconds)))
        long_msgs.append(long_message(INSPVA_ID, gps_ms, inspva_payload(seconds)))
        short_msgs.append(short_message(CORRIMUS_ID, gps_ms, corrimus_payload()))
        short_msgs.append(short_message(INSPVAS_ID, gps_ms, inspva_payload(seconds)))

    write_pcap('corrimudata-inspva-sync.pcap', long_msgs)
    write_pcap('corrimus-inspvas-sync.pcap', short_msgs)


if __name__ == '__main__':
    main()
