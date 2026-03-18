#pragma once
#include <Arduino.h>

// ================================================================
// USB Telemetry — bidirectional serial at ~100 Hz
// ================================================================
// TX packet (FC → GUI), comma-separated, newline-terminated:
//
//  roll,pitch,yaw,m1,m2,m3,m4,kP,kI,kD,alt,armed,mode,
//  volt_mV,curr_cA,bat_pct,r1rpm,r2rpm,r3rpm,r4rpm,
//  loop_us,fs_level
//
//  [0]  roll        degrees × 10 as int16  (divide by 10 → °)
//  [1]  pitch       degrees × 10 as int16
//  [2]  yaw         degrees × 10 as int16
//  [3-6] m1..m4     0–1000 (motor output × 1000)
//  [7]  kP          float (3 decimals)
//  [8]  kI          float
//  [9]  kD          float
//  [10] alt         cm as uint16 (divide by 100 → m)
//  [11] armed       0 or 1
//  [12] mode        0=ACRO 1=ANGLE
//  [13] volt_mV     battery voltage in mV
//  [14] curr_cA     current in centi-Amps (0 if no sensor)
//  [15] bat_pct     0–100
//  [16-19] r1..r4   ESC RPM (0 if not available)
//  [20] loop_us     actual control loop period in µs
//  [21] fs_level    0=NONE 1=WARN 2=LAND 3=KILL
//
// RX commands (GUI → FC), one per line:
//   SET_KP:<val>    SET_KI:<val>    SET_KD:<val>
//   ARM             DISARM          CALIB
//   MODE_ACRO       MODE_ANGLE
//   AUTOTUNE_START
//   MOTOR_TEST:<motor(0-3)>:<throttle_pct(0-15)>
//   MOTOR_TEST_STOP
//   ERASE_BB        DUMP_BB
// ================================================================

void usbTelemetrySend(
    float roll, float pitch, float yaw,
    float m1, float m2, float m3, float m4,
    float kp, float ki, float kd,
    float altitude,
    bool  armed, int mode,
    uint16_t volt_mV, int16_t curr_cA, uint8_t bat_pct,
    uint16_t r1rpm, uint16_t r2rpm, uint16_t r3rpm, uint16_t r4rpm,
    uint32_t loop_us, uint8_t fs_level);

// Parse and dispatch an incoming GUI command line.
// Call once per loop() after checking Serial.available().
void usbTelemetryParseCmd();

