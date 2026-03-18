#pragma once
#include <Arduino.h>

// ================================================================
// DShot600 ESC Driver — bit-bang, cycle-accurate timing
// ================================================================
// Pins: ESC1=GPIO1, ESC2=GPIO4, ESC3=GPIO10, ESC4=GPIO14
//
// DShot600 electrical specification:
//   Bit rate   : 600 kBit/s  → bit period = 1.667 µs
//   Logic-1    : T1H=1.250 µs, T1L=0.417 µs
//   Logic-0    : T0H=0.625 µs, T0L=1.042 µs
//   Frame      : 16 bits (11-bit value, 1 telem bit, 4-bit CRC)
//   Inter-frame: ≥ 2 µs gap
//
// NOTE: Timing constants in config.h are calibrated for 150 MHz
// RP2354A.  Re-tune DSHOT600_T*_CYCLES if you change CPU clock.
//
// For production use, replace with a PIO-based driver for
// jitter-free output without disabling interrupts.
// ================================================================

void dshotInit();
// value : 0 = disarm, 48–2047 = throttle range
void dshotWriteMotors(float m1, float m2, float m3, float m4);
// Raw packet send (used internally)
void dshotSendPacket(uint8_t pin, uint16_t value, bool telemetry);
