#pragma once
#include <Arduino.h>

// ================================================================
// Blackbox USB Dump
// ================================================================
// Streams the entire W25Q128 flash content over USB serial so it
// can be saved as a binary file on the host PC and decoded with
// blackbox/decode.py.
//
// Protocol:
//   FC receives:  "DUMP_BB\n"
//   FC responds:
//     "BB_DUMP_START:<total_bytes>\n"
//     <total_bytes> bytes of raw binary data in 256-byte chunks,
//     each chunk preceded by a 4-byte little-endian offset header
//     and followed by a 1-byte XOR checksum.
//     "BB_DUMP_END\n"
//
// Host-side capture (Linux/Mac):
//   python3 tools/dump_flash.py /dev/ttyACM0 flash.bin
//
// Host-side capture (Windows):
//   python3 tools/dump_flash.py COM3 flash.bin
//
// The dump covers the entire 16 MiB flash.  At 115 200 baud this
// takes ~1200 s (~20 min), which is impractical.  At 921 600 baud
// (arduino-pico supports this on USB CDC) it takes ~150 s.
// For faster transfers, set Serial.begin(921600) in setup().
//
// Call bbDumpStart() when "DUMP_BB" is received, then call
// bbDumpUpdate() every loop tick until bbDumpActive() is false.
// The dump blocks the flight loop but is only intended for bench use.
// ================================================================

void bbDumpStart();
void bbDumpUpdate();   // call every loop while bbDumpActive()
bool bbDumpActive();
