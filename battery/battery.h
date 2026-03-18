#pragma once
#include <Arduino.h>

// ================================================================
// Battery Monitor
// ================================================================
// Measures pack voltage and (optionally) current draw using the
// RP2354A's internal 12-bit ADC.
//
// Hardware connections (add to config.h if different):
//   Voltage divider → ADC0 (GPIO26)
//     Pack+ → R1 (100 kΩ) → ADC pin → R2 (10 kΩ) → GND
//     Divider ratio = R2 / (R1 + R2) = 10/(100+10) ≈ 0.0909
//     For 4S LiPo (max 16.8 V):  ADC sees max 1.527 V < 3.3 V ✓
//
//   Current sensor → ADC1 (GPIO27)  (optional)
//     ACS712-30A: 66 mV/A, zero-current output = Vcc/2 = 1.65 V
//     Shunt + op-amp: configure CURRENT_SENSOR_MV_PER_A below.
//
// Filtering:
//   Both channels are low-pass filtered (RC α = 0.05) to reject
//   switching noise from motors.
//
// Low-voltage warning:
//   When voltage drops below BAT_WARN_MV a flag is set.
//   At BAT_CRITICAL_MV motors are throttled down to a safe landing.
// ================================================================

// Pin assignments — override in config.h if needed
#ifndef BAT_VOLTAGE_PIN
#define BAT_VOLTAGE_PIN    26    // ADC0
#endif
#ifndef BAT_CURRENT_PIN
#define BAT_CURRENT_PIN    27    // ADC1  (comment out if no sensor)
#endif

// Voltage divider: set for your resistor values
// V_bat = V_adc * (R1 + R2) / R2
#define BAT_DIVIDER_R1_K   100.0f   // kΩ
#define BAT_DIVIDER_R2_K    10.0f   // kΩ
#define BAT_DIVIDER_RATIO  ((BAT_DIVIDER_R1_K + BAT_DIVIDER_R2_K) / BAT_DIVIDER_R2_K)
// 3.3 V reference, 12-bit ADC → LSB = 3300 / 4096 mV
#define BAT_ADC_MV_PER_LSB  (3300.0f / 4096.0f)

// Current sensor: ACS712-30A (change for your sensor)
#define CURRENT_SENSOR_MV_PER_A  66.0f    // mV per Amp
#define CURRENT_ZERO_MV        1650.0f    // output at 0 A (Vcc/2)

// Thresholds (millivolts) — adjust for your battery chemistry/cell count
// Default: 4S LiPo
#define BAT_FULL_MV       16800    // 4.20 V/cell × 4
#define BAT_WARN_MV       14400    // 3.60 V/cell × 4  (land soon)
#define BAT_CRITICAL_MV   13200    // 3.30 V/cell × 4  (forced land)

// LPF alpha (0 = static, 1 = no filter)
#define BAT_LPF_ALPHA  0.05f

// ---- Public API ------------------------------------------------

void  batInit();
void  batUpdate();           // call at ~50 Hz (every 20 ms) from loop

uint16_t batGetVoltage_mV();    // filtered pack voltage in mV
int16_t  batGetCurrent_cA();    // filtered current in centi-Amps (may be 0)
uint8_t  batGetPercent();       // 0–100 % (simple linear on full..critical range)
bool     batIsLow();            // true if voltage ≤ BAT_WARN_MV
bool     batIsCritical();       // true if voltage ≤ BAT_CRITICAL_MV
