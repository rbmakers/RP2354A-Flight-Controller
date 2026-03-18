#pragma once
#include <Arduino.h>

// ================================================================
// DShot600 PIO Driver  — RP2354A / RP2350
// ================================================================
// Uses the RP2350 Programmable IO (PIO) state machines to generate
// cycle-accurate DShot600 waveforms with ZERO CPU blocking and
// ZERO interrupt jitter.
//
// Architecture
// ────────────
//   PIO0, SM0  → ESC1 (GPIO  1)
//   PIO0, SM1  → ESC2 (GPIO  4)
//   PIO0, SM2  → ESC3 (GPIO 10)
//   PIO0, SM3  → ESC4 (GPIO 14)
//
//   All four state machines share the same PIO program and are
//   clocked identically, so all four ESCs receive their frames
//   simultaneously (within one PIO clock cycle ≈ 6.7 ns at 150 MHz).
//
// DShot600 electrical specification
// ──────────────────────────────────
//   Bit rate  : 600 kbit/s  → bit period  = 1666.7 ns
//   Logic-1   : T1H = 1250 ns HIGH, T1L =  417 ns LOW
//   Logic-0   : T0H =  625 ns HIGH, T0L = 1042 ns LOW
//
//   At 150 MHz CPU the PIO clock divider is set to give
//   a PIO tick of exactly 41.667 ns (÷4 from 600 kHz * 16 counts).
//   Each bit is encoded as 16 PIO ticks.
//       Logic-1 HIGH = 30 ticks (1250 ns), LOW =  6 ticks (250 ns)
//       Logic-0 HIGH = 15 ticks ( 625 ns), LOW = 21 ticks (875 ns)
//   (Small rounding; within the ±5 % tolerance of DShot spec.)
//
// DShot frame
// ───────────
//   16 bits, MSB first:
//     [15:5]  11-bit throttle  (0 = disarm, 48–2047 = throttle)
//     [4]      1-bit telemetry request
//     [3:0]    4-bit CRC = XOR of upper three nibbles of [15:4]
//
// Bidirectional DShot (DSHOT_BIDIR)
// ───────────────────────────────────
//   When DSHOT_BIDIR is defined the GPIO is switched to input after
//   the 16-bit frame and the PIO captures the eRPM telemetry frame
//   returned by the ESC (BLHeli32 / BLHeli_S bidirectional mode).
//   The eRPM value is decoded and made available via
//   dshotPioGetRPM(motor).
// ================================================================

// Uncomment to enable bidirectional DShot eRPM telemetry
//#define DSHOT_BIDIR

void  dshotPioInit();

// Send throttle to all four motors simultaneously.
// m1..m4: normalised 0.0–1.0  (0 = disarm)
void  dshotPioWriteMotors(float m1, float m2, float m3, float m4);

// Send a raw 11-bit value to one motor (0 = disarm, 48–2047 = throttle,
// 1–47 are special commands per DShot command protocol).
void  dshotPioSendRaw(uint8_t motor, uint16_t value, bool telemetry);

#ifdef DSHOT_BIDIR
// Returns the last decoded electrical RPM for the given motor (0–3).
// Returns 0 if no valid frame has been received yet.
uint32_t dshotPioGetRPM(uint8_t motor);
#endif
