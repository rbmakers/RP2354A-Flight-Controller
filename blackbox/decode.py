#!/usr/bin/env python3
"""
RP2354A Blackbox Decoder
========================
Reads a raw binary blackbox dump from the W25Q128 flash and outputs
a CSV file compatible with standard analysis tools (Excel, Python/pandas,
or Betaflight Blackbox Explorer with minor column remapping).

Usage:
    python3 decode.py <input.bin> [output.csv]

Dump the flash content first:
    # Using pyserial (add a "DUMP_BB" USB command to the firmware):
    python3 -c "
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
s.write(b'DUMP_BB\n')
data = b''
while True:
    chunk = s.read(4096)
    if not chunk: break
    data += chunk
open('flash_dump.bin', 'wb').write(data)
print(f'Saved {len(data)} bytes')
"

Frame layout (32 bytes, little-endian):
  uint32  timestamp_us
  int16   roll_x10         (degrees × 10  → divide by 10 for degrees)
  int16   pitch_x10
  int16   yaw_x10
  int16   gx_x100          (rad/s × 100   → divide by 100 for rad/s)
  int16   gy_x100
  int16   gz_x100
  uint16  m1, m2, m3, m4   (0–1000        → divide by 10 for %)
  uint16  altitude_cm      (cm            → divide by 100 for m)
  uint16  voltage_mV       (mV            → divide by 1000 for V)
  uint8   armed_mode       (bit7=armed, [1:0]=mode)
  uint8   flags            (bit0=altHold, bit1=calibrating)
  uint16  loop_time_us
"""

import struct
import sys
import os
import csv

FRAME_SIZE   = 32
FRAME_FORMAT = '<I6h4H2HBBh'   # Note: last field int16 but stored uint16
FRAME_FORMAT = '<IhhhhhHHHHHHHBBH'
BB_MAGIC     = 0xBB354A01

COLUMNS = [
    'timestamp_us', 'roll_deg', 'pitch_deg', 'yaw_deg',
    'gx_rads', 'gy_rads', 'gz_rads',
    'm1_pct', 'm2_pct', 'm3_pct', 'm4_pct',
    'altitude_m', 'voltage_V',
    'armed', 'mode', 'alt_hold', 'calibrating',
    'loop_time_us'
]


def unpack_frame(data: bytes):
    """Unpack one 32-byte frame into a dict. Returns None if invalid."""
    if len(data) < FRAME_SIZE:
        return None

    (ts, roll10, pitch10, yaw10,
     gx100, gy100, gz100,
     m1, m2, m3, m4,
     alt_cm, volt_mv,
     armed_mode, flags, loop_us) = struct.unpack_from('<IhhhhhHHHHHHHBBH', data)

    if ts == 0 and flags == 0xFF:
        return None   # end-of-session sentinel

    return {
        'timestamp_us': ts,
        'roll_deg':     round(roll10  / 10.0,  2),
        'pitch_deg':    round(pitch10 / 10.0,  2),
        'yaw_deg':      round(yaw10   / 10.0,  2),
        'gx_rads':      round(gx100   / 100.0, 4),
        'gy_rads':      round(gy100   / 100.0, 4),
        'gz_rads':      round(gz100   / 100.0, 4),
        'm1_pct':       round(m1 / 10.0, 1),
        'm2_pct':       round(m2 / 10.0, 1),
        'm3_pct':       round(m3 / 10.0, 1),
        'm4_pct':       round(m4 / 10.0, 1),
        'altitude_m':   round(alt_cm  / 100.0, 3),
        'voltage_V':    round(volt_mv / 1000.0, 3),
        'armed':        int(bool(armed_mode & 0x80)),
        'mode':         int(armed_mode & 0x03),
        'alt_hold':     int(bool(flags & 0x01)),
        'calibrating':  int(bool(flags & 0x02)),
        'loop_time_us': loop_us,
    }


def find_sessions(data: bytes):
    """Scan index area (offset 4096) for valid session entries."""
    sessions = []
    idx_offset = 4096   # sector 1 = BB_INDEX_SECTOR
    for i in range(16):
        base = idx_offset + i * 16
        if base + 16 > len(data):
            break
        magic, start, nbytes, ts = struct.unpack_from('<IIII', data, base)
        if magic == BB_MAGIC and nbytes > 0:
            sessions.append({'id': i, 'start': start, 'bytes': nbytes, 'ts': ts})
            print(f"  Session {i}: start=0x{start:06X}  bytes={nbytes}  ts={ts} µs")
    return sessions


def decode_session(data: bytes, start: int, length: int):
    """Decode frames from a session region. Yields row dicts."""
    end = min(start + length, len(data))
    pos = start
    frame_count = 0
    while pos + FRAME_SIZE <= end:
        row = unpack_frame(data[pos:pos + FRAME_SIZE])
        pos += FRAME_SIZE
        if row is None:
            break
        frame_count += 1
        yield row
    print(f"    → {frame_count} frames decoded")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    in_path  = sys.argv[1]
    out_base = sys.argv[2] if len(sys.argv) > 2 else in_path.replace('.bin', '')

    print(f"Reading: {in_path}")
    with open(in_path, 'rb') as f:
        data = f.read()
    print(f"  {len(data)} bytes ({len(data)/1024:.1f} KB)")

    sessions = find_sessions(data)
    if not sessions:
        print("No sessions found in index.")
        print("Attempting linear scan from offset 0x20000 (data area)...")
        # Fallback: linear scan from data area
        sessions = [{'id': 0, 'start': 0x20000,
                     'bytes': len(data) - 0x20000, 'ts': 0}]

    for sess in sessions:
        out_path = f"{out_base}_session{sess['id']}.csv"
        print(f"\nDecoding session {sess['id']} → {out_path}")

        rows = list(decode_session(data, sess['start'], sess['bytes']))
        if not rows:
            print("  No valid frames.")
            continue

        with open(out_path, 'w', newline='') as csvf:
            writer = csv.DictWriter(csvf, fieldnames=COLUMNS)
            writer.writeheader()
            writer.writerows(rows)

        print(f"  Saved {len(rows)} rows to {out_path}")

        # Print summary statistics
        ts_end  = rows[-1]['timestamp_us']
        ts_start = rows[0]['timestamp_us']
        duration = (ts_end - ts_start) / 1e6
        avg_loop = sum(r['loop_time_us'] for r in rows) / len(rows)
        print(f"  Duration: {duration:.1f} s   Avg loop: {avg_loop:.0f} µs")


if __name__ == '__main__':
    main()
