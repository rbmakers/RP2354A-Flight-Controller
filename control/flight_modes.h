#pragma once
#include <Arduino.h>

// ================================================================
// Flight Modes and ARM / DISARM Logic
// ================================================================
//
// All gain constants (OUTER_LOOP_KP, MAX_ANGLE_DEG, MAX_RATE_DPS,
// ACRO_EXPO) are defined in config.h alongside the inner-loop PID
// values so every tunable parameter lives in one place.
//
// ACRO mode (Rate mode):
//   RC sticks command angular rates (deg/s × ACRO_EXPO shaping).
//   PID input  = desired rate (from stick)
//   PID feedback = gyro rate
//
// ANGLE mode (Self-level):
//   RC sticks command a desired tilt angle (deg).
//   Outer P loop: angle error × OUTER_LOOP_KP → rate setpoint (deg/s)
//   Inner PID: rate setpoint vs. gyro — identical to Acro inner loop.
//
// ARM / DISARM (Betaflight stick commands):
//   ARM    : throttle < ARM_THROTTLE_MAX  AND  yaw stick full RIGHT
//   DISARM : throttle < ARM_THROTTLE_MAX  AND  yaw stick full LEFT
//   Switch arm: CH5 > 1700 µs → armed,  CH5 < 1300 µs → disarmed
// ================================================================

typedef enum
{
    MODE_ACRO  = 0,
    MODE_ANGLE = 1
} FlightMode;

void        flightModeSet(FlightMode m);
FlightMode  flightModeGet();

void  motorArm();
void  motorDisarm();
bool  isArmed();

// Acro: stick (−1…+1) → rate setpoint (deg/s), with optional expo
float acroRateSetpoint(float stick);

// Angle: stick (−1…+1) + current angle (deg) → rate setpoint (deg/s)
float angleModeSetpoint(float stick, float currentAngleDeg);

void  checkArmDisarm(float throttle, float yaw);
void  checkArmSwitch(uint16_t ch5us);
void  checkModeSwitch(uint16_t ch6us);

