#!/usr/bin/env python3
"""
dump_flash.py — RP2354A FC Blackbox Flash Dump Tool
====================================================
Captures the raw 16 MiB W25Q128 flash dump streamed by the
flight controller over USB serial.

Usage:
    python3 dump_flash.py <port> <output.bin> [baud]

Examples:
    python3 dump_flash.py /dev/ttyACM0 flash.bin
    python3 dump_flash.py COM3 flash.bin 921600

Then decode:
    python3 blackbox/decode.py flash.bin

Dependencies:
    pip install pyserial
"""

import sys
import time
import struct
import serial

CHUNK_SIZE  = 256
TOTAL_BYTES = 16 * 1024 * 1024   # 16 MiB

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    port     = sys.argv[1]
    out_path = sys.argv[2]
    baud     = int(sys.argv[3]) if len(sys.argv) > 3 else 115200

    print(f"Connecting to {port} at {baud} baud...")

    with serial.Serial(port, baud, timeout=5) as ser:
        # Wait for FC to be ready
        time.sleep(0.5)

        print("Sending DUMP_BB command...")
        ser.write(b"DUMP_BB\n")
        ser.flush()

        # Wait for BB_DUMP_START:<total>
        while True:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line.startswith("BB_DUMP_START:"):
                total = int(line.split(":")[1])
                print(f"Dump started: {total:,} bytes ({total/1024/1024:.1f} MB)")
                break
            elif line:
                print(f"  [debug] {line}")

        # Receive chunks
        data = bytearray(total)
        received  = 0
        errors    = 0
        t_start   = time.time()

        with open(out_path, 'wb') as f:
            while received < total:
                # Each frame: 4-byte offset + 256 data bytes + 1 checksum
                frame_size = 4 + CHUNK_SIZE + 1
                frame = ser.read(frame_size)

                if len(frame) < frame_size:
                    print(f"\nTimeout at offset {received}")
                    break

                offset = struct.unpack_from('<I', frame, 0)[0]
                chunk  = frame[4:4 + CHUNK_SIZE]
                csum_rx = frame[4 + CHUNK_SIZE]

                # Verify checksum
                csum_calc = 0
                for b in frame[:4 + CHUNK_SIZE]:
                    csum_calc ^= b
                csum_calc &= 0xFF

                if csum_calc != csum_rx:
                    print(f"\nChecksum error at offset 0x{offset:06X}")
                    errors += 1

                # Write chunk at correct offset
                if offset + CHUNK_SIZE <= total:
                    data[offset:offset + CHUNK_SIZE] = chunk

                received += CHUNK_SIZE

                # Progress display
                pct  = received * 100 // total
                rate = received / (time.time() - t_start) / 1024
                bar  = '█' * (pct // 5) + '░' * (20 - pct // 5)
                print(f"\r  [{bar}] {pct:3d}%  {received//1024:6d}/{total//1024} KB  "
                      f"{rate:.0f} KB/s  errs={errors}", end='', flush=True)

                # Check for end marker in buffered line data (non-blocking)
                # The FC prints BB_DUMP_END as a text line after the binary stream.

        print()   # newline after progress bar

        # Drain any remaining text from serial
        time.sleep(0.5)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line: print(f"  [FC] {line}")

        # Write output file
        with open(out_path, 'wb') as f:
            f.write(data)

        elapsed = time.time() - t_start
        print(f"\nSaved {len(data):,} bytes to {out_path}")
        print(f"Time: {elapsed:.1f} s   Errors: {errors}")
        print(f"\nDecode with:  python3 blackbox/decode.py {out_path}")


if __name__ == '__main__':
    main()
