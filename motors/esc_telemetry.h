#pragma once
#include <Arduino.h>

// ================================================================
// ESC Telemetry Manager
// ================================================================
// Aggregates per-motor eRPM data from bidirectional DShot and
// feeds it into the rpm_notch filter bank.
//
// Two sources of eRPM are supported:
//
//  1. DSHOT_BIDIR  — eRPM decoded from the ESC's GCR response
//                    frame returned on the DShot data line after
//                    each forward packet.
//                    Enable: #define DSHOT_BIDIR  in dshot_pio.h
//                    Requires: BLHeli32 or BLHeli_S bidir mode.
//
//  2. BLHeli serial telemetry — ESC sends a 10-byte frame at
//                    115 200 baud on a separate RX pin.
//                    This is the fallback when bidir DShot is not
//                    used (e.g. older ESCs or single-wire setups).
//
// The motor_rpm[] array is always kept in mechanical RPM (not eRPM)
// so that rpmFilterUpdateRPM() receives a sensor-independent value.
//
// Usage:
//   escTelemetryInit();          // call in setup()
//   escTelemetryUpdate();        // call every control loop (1 kHz)
//   uint16_t rpm = escGetRPM(0); // motor 0 mechanical RPM
// ================================================================

#define ESC_TELEM_MOTORS  4

void     escTelemetryInit();
void     escTelemetryUpdate();           // poll & decode, feed rpm filter
uint16_t escGetRPM(uint8_t motor);       // mechanical RPM (0 if unknown)
uint16_t escGetVoltage_mV(uint8_t motor);
uint16_t escGetCurrent_cA(uint8_t motor);
uint16_t escGetTemp_C(uint8_t motor);
bool     escTelemetryValid(uint8_t motor);
