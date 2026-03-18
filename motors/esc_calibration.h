#pragma once
#include <Arduino.h>

// ================================================================
// ESC PWM Calibration Routine
// ================================================================
// Most BLHeli/KISS/AM32 ESCs need to learn the throttle range
// of the flight controller before first use.  This module performs
// the standard "high-then-low" ESC calibration sequence.
//
// *** SAFETY WARNING ***
//   REMOVE PROPELLERS BEFORE CALIBRATING ESCs.
//   This routine spins the motors at full throttle briefly!
//
// Calibration sequence (standard for most ESCs):
//   1. FC sends MAXIMUM throttle signal
//   2. ESC powers on → hears max signal → beeps twice
//   3. FC sends MINIMUM throttle signal within 2 seconds
//   4. ESC stores the range → beeps confirmation
//   5. ESC is calibrated and ready
//
// Only applies when MOTOR_TYPE_BLDC is selected AND the brushed
// PWM / servo-style ESC is used (i.e., not DShot).
// DShot ESCs do NOT require range calibration (digital protocol).
//
// Usage:
//   escCalStart()  — called via GUI "ESC_CAL_START" command
//   escCalUpdate() — call every loop tick during calibration
//   escCalDone()   — returns true when sequence is complete
// ================================================================

typedef enum {
    ESC_CAL_IDLE      = 0,
    ESC_CAL_WAIT_PWR  = 1,   // outputting max throttle, wait for ESC power-on
    ESC_CAL_SEND_MAX  = 2,   // sending max for 3 s
    ESC_CAL_SEND_MIN  = 3,   // sending min for 2 s
    ESC_CAL_COMPLETE  = 4,
    ESC_CAL_ABORTED   = 5
} ESCCalState;

void        escCalStart();         // refuses if armed
void        escCalAbort();
ESCCalState escCalUpdate(float &m1out, float &m2out,
                         float &m3out, float &m4out);  // returns current state
bool        escCalDone();
bool        escCalActive();
const char* escCalStateName(ESCCalState s);
