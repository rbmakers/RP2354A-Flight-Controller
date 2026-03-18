#pragma once
#include <Arduino.h>

// ================================================================
// MSP v1 (MultiWii Serial Protocol)  — Betaflight Configurator
// ================================================================
// Implementing a subset of MSP v1 allows the official Betaflight
// Configurator (Chrome app or desktop) to connect to this FC and
// display real-time attitude, motor output, RC channels, and
// battery data — using the Configurator's own polished UI.
//
// Protocol framing (MSP v1):
//   '$'  'M'  direction  size  cmd  [payload]  checksum
//
//   direction: '<' = request  (configurator → FC)
//              '>' = response (FC → configurator)
//              '!' = error
//
//   checksum: XOR of size, cmd, and all payload bytes
//
// Implemented MSP commands (sufficient for Configurator tabs):
//   MSP_IDENT         (100) — firmware type, version, capability flags
//   MSP_STATUS        (101) — armed, flight mode, cycle time, sensors present
//   MSP_RAW_IMU       (102) — raw accelerometer + gyro + magnetometer
//   MSP_RC            (105) — 8-channel RC input (µs units)
//   MSP_MOTOR         (104) — motor values (1000–2000)
//   MSP_ATTITUDE      (108) — roll/pitch (°×10), yaw (°)
//   MSP_ALTITUDE      (109) — altitude (cm), vario (cm/s)
//   MSP_ANALOG        (110) — battery voltage (0.1 V), mAh, rssi, current
//   MSP_SET_PID       (202) — receive PID gains from Configurator
//
// Usage:
//   mspInit()        — call in setup(), uses Serial (USB)
//   mspUpdate()      — call every loop() tick
//
// Note: MSP and the custom CSV telemetry share the same USB serial
// port.  This module detects the '$' MSP preamble and handles the
// frame; all other bytes are forwarded to usbTelemetryParseCmd().
// ================================================================

void mspInit();
void mspUpdate();
