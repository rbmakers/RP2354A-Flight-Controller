#pragma once

// ================================================================
// RB-RP2354A Flight Controller — Central Configuration
// ================================================================

// ---------------------------------------------------------------
// SPI Bus  (BMI088 IMU)
// ---------------------------------------------------------------
#define PIN_MOSI        19
#define PIN_MISO        20
#define PIN_SCK         22
#define CS_ACCEL        21
#define CS_GYRO         23
#define GYRO_INT_PIN    24

// ---------------------------------------------------------------
// I2C0  (BMP580 Barometer)
// ---------------------------------------------------------------
#define PIN_SDA         16
#define PIN_SCL         17
#define BMP580_I2C_ADDR 0x47   // SDO tied to VDD

// ---------------------------------------------------------------
// Brushed Motor PWM  (coreless motors via MOSFET)
// ---------------------------------------------------------------
#define M1_PIN  29
#define M2_PIN  11
#define M3_PIN  18
#define M4_PIN  25

// ---------------------------------------------------------------
// BLDC ESC  (DShot600)
// ---------------------------------------------------------------
#define ESC1_PIN  1
#define ESC2_PIN  4
#define ESC3_PIN  10
#define ESC4_PIN  14

// ---------------------------------------------------------------
// ELRS Receiver — UART0 CRSF @ 420 kBaud
// ---------------------------------------------------------------
#define ELRS_TX_PIN  12
#define ELRS_RX_PIN  13
#define ELRS_BAUD    420000

// ---------------------------------------------------------------
// Motor type — uncomment the one you are using
// ---------------------------------------------------------------
#define MOTOR_TYPE_BRUSHED
//#define MOTOR_TYPE_BLDC

// ---------------------------------------------------------------
// Loop rates (Hz)
// ---------------------------------------------------------------
#define IMU_RATE_HZ         2000   // Gyro DRDY interrupt rate
#define FLIGHT_LOOP_US      1000   // 1 kHz control loop period (µs)
#define BARO_DIVIDER        20     // Read baro every N control loops → 50 Hz
#define TELEMETRY_DIVIDER   10     // Send telemetry every N control loops → 100 Hz

// ---------------------------------------------------------------
// SPI
// ---------------------------------------------------------------
#define SPI_CLOCK_HZ  10000000     // 10 MHz — safe max for BMI088

// ---------------------------------------------------------------
// BMI088 scales
// ---------------------------------------------------------------
#define GYRO_FS_DPS   2000.0f      // ±2000 °/s  (register 0x00 on gyro)
#define ACCEL_FS_G    24.0f        // ±24 g       (register 0x03 on accel)

// Derived scale factors to physical units
#define GYRO_SCALE    (GYRO_FS_DPS  / 32768.0f * DEG_TO_RAD)  // LSB → rad/s
#define ACCEL_SCALE   (ACCEL_FS_G   / 32768.0f)                // LSB → g

// ---------------------------------------------------------------
// Madgwick AHRS
// ---------------------------------------------------------------
#define MADGWICK_BETA      0.04f   // Fusion gain (higher = faster, noisier)
#define MADGWICK_RATE_HZ   (float)IMU_RATE_HZ

// ---------------------------------------------------------------
// Gyro biquad low-pass filter
// ---------------------------------------------------------------
#define GYRO_LPF_CUTOFF_HZ  150.0f

// ---------------------------------------------------------------
// RPM filter  — motor pole pairs
// ---------------------------------------------------------------
#define MOTOR_POLE_PAIRS  7        // Typical for 1407/2306 motors

// ---------------------------------------------------------------
// DShot600 timing — cycles at 150 MHz CPU
// GPIO set/clear via gpio_put() costs ~3 cycles overhead each side.
// Adjusted values measured empirically; tune if ESC does not respond.
// ---------------------------------------------------------------
#define DSHOT600_T1H_CYCLES  180   // ~1.25 µs  (logic-1 high time)
#define DSHOT600_T1L_CYCLES   55   // ~0.42 µs  (logic-1 low  time)
#define DSHOT600_T0H_CYCLES   84   // ~0.63 µs  (logic-0 high time)
#define DSHOT600_T0L_CYCLES  148   // ~1.04 µs  (logic-0 low  time)

// ---------------------------------------------------------------
// PID Gain Presets
// ---------------------------------------------------------------
// Select ONE preset block by uncommenting it, or define your own.
//
// Unit conventions:
//   Inner-loop sensor/setpoint : rad/s  (BMI088 gyro after GYRO_SCALE)
//   Inner-loop output          : dimensionless correction ±PID_OUTPUT_LIMIT
//   Outer-loop input           : degrees (from Madgwick Euler)
//   Outer-loop output          : deg/s rate setpoint → inner loop
//
// Weight-based scaling rule (linear, 40–180 g):
//   Kp  ≈ 2.0 + (mass_g − 40) × 0.033
//   Kd  ≈ Kp × 0.008  (brushed)  /  Kp × 0.009  (BLDC)
//   Ki  ≈ Kp × 0.015
//   Kff ≈ Kp × 0.012  (brushed)  /  Kp × 0.010  (BLDC)
//   Kp_outer ≈ 6.0 + (mass_g − 40) × 0.025
// ---------------------------------------------------------------

// ---- PRESET A : 1S–2S Brushed Coreless  30–60 g  (65 mm TinyWhoop) ----
// Activate: uncomment this block, comment out PRESET B

#define PID_ROLL_KP    2.0f
#define PID_ROLL_KI    0.04f
#define PID_ROLL_KD    0.015f
#define PID_ROLL_KFF   0.024f   // Feedforward: fires on stick movement (Kp × 0.012)

#define PID_PITCH_KP   2.0f
#define PID_PITCH_KI   0.04f
#define PID_PITCH_KD   0.015f
#define PID_PITCH_KFF  0.024f

#define PID_YAW_KP     2.5f
#define PID_YAW_KI     0.04f
#define PID_YAW_KD     0.00f
#define PID_YAW_KFF    0.00f    // Yaw FF usually 0 — causes heading drift on brushed

#define OUTER_LOOP_KP  6.0f    // Outer P: 10° error → 60 deg/s rate command
#define MAX_ANGLE_DEG  30.0f   // Tighter tilt limit for indoor micro flying
#define MAX_RATE_DPS   400.0f  // Acro rate limit

// I-term Relax: suppress integrator when setpoint rate exceeds this threshold (rad/s²)
// Lower value = more aggressive suppression.
// Rule of thumb: ≈ MAX_RATE_DPS × DEG_TO_RAD × 0.5
#define ITERM_RELAX_CUTOFF   3.5f   // rad/s per second  (400°/s × 0.0175 × 0.5 ≈ 3.5)

// TPA (Throttle PID Attenuation): scale Kp/Kd down above this throttle level.
// Prevents high-throttle oscillation on faster motors.
#define TPA_BREAKPOINT   0.50f  // throttle (0–1) above which attenuation begins
#define TPA_RATE         0.30f  // max reduction factor (30% at full throttle)
// Effect: at throttle=1.0, Kp and Kd are multiplied by (1 − 0.30) = 0.70
// Yaw is NOT attenuated — it needs full authority at all throttle levels.

// Airmode: keep PID authority at zero throttle by redistributing motor outputs.
// When any motor would go below zero, shift all motors up to preserve the
// differential (control authority) without changing the net thrust command.
// Set to 1 to enable, 0 to disable.
#define AIRMODE_ENABLED  0      // Disable for beginners; enable for Acro flying


// ---- PRESET B : 2S–3S BLDC  80–180 g  (2"–3" toothpick / micro-racer) ----
// Activate: uncomment this block, comment out PRESET A

//#define PID_ROLL_KP    4.0f
//#define PID_ROLL_KI    0.06f
//#define PID_ROLL_KD    0.035f
//#define PID_ROLL_KFF   0.040f   // Feedforward (Kp × 0.010)
//
//#define PID_PITCH_KP   4.0f
//#define PID_PITCH_KI   0.06f
//#define PID_PITCH_KD   0.035f
//#define PID_PITCH_KFF  0.040f
//
//#define PID_YAW_KP     5.0f
//#define PID_YAW_KI     0.06f
//#define PID_YAW_KD     0.00f
//#define PID_YAW_KFF    0.00f
//
//#define OUTER_LOOP_KP  10.0f
//#define MAX_ANGLE_DEG  45.0f
//#define MAX_RATE_DPS   600.0f
//
//#define ITERM_RELAX_CUTOFF   5.2f   // 600°/s × 0.0175 × 0.5 ≈ 5.2
//
//#define TPA_BREAKPOINT   0.40f   // BLDC responds faster, start TPA earlier
//#define TPA_RATE         0.40f   // Up to 40% reduction at full throttle
//
//#define AIRMODE_ENABLED  1       // Enable for Acro / freestyle BLDC flying

// ---------------------------------------------------------------
// PID limits (shared by both presets — rarely need changing)
// ---------------------------------------------------------------
#define PID_INTEGRAL_LIMIT  200.0f
#define PID_OUTPUT_LIMIT    500.0f

// ---------------------------------------------------------------
// Altitude hold PID
// ---------------------------------------------------------------
#define ALT_KP  1.5f
#define ALT_KI  0.3f
#define ALT_KD  0.8f
#define ALT_INTEGRAL_LIMIT   5.0f
#define ALT_OUTPUT_LIMIT     0.4f   // max throttle correction (0–1 scale)

// ---------------------------------------------------------------
// ARM / DISARM stick commands (normalized ±1.0)
// ---------------------------------------------------------------
#define ARM_THROTTLE_MAX   0.05f
#define ARM_YAW_THRESHOLD  0.90f

// ---------------------------------------------------------------
// Battery Monitor  (ADC)
// ---------------------------------------------------------------
#define BAT_VOLTAGE_PIN   26    // ADC0 — voltage divider (100k / 10k)
#define BAT_CURRENT_PIN   27    // ADC1 — ACS712 or shunt amp (optional)

// ---------------------------------------------------------------
// Blackbox logger  (W25Q128 SPI flash, shares SPI bus with BMI088)
// ---------------------------------------------------------------
#define BB_FLASH_CS_PIN   28    // dedicated CS for W25Q128

// ---------------------------------------------------------------
// Optional modules — uncomment to enable
// ---------------------------------------------------------------
//#define USE_FFT_ANALYSIS   // Vibration analysis (Core 1)
                             //   Uses CMSIS-DSP arm_rfft_fast_f32 if
                             //   available (arduino-pico ≥ 3.x), otherwise
                             //   falls back to Goertzel selective DFT.
                             //   Automatically feeds dynamic notch filter.

//#define USE_RPM_FILTER     // Per-motor RPM notch filter bank.
                             //   Requires ESC telemetry (either DSHOT_BIDIR
                             //   or BLHeli serial on GPIO26).
                             //   Feed path: esc_telemetry → rpm_notch.

//#define USE_BLACKBOX       // Blackbox flight data logger.
                             //   Writes 32-byte frames at 1 kHz to
                             //   W25Q128 SPI flash on BB_FLASH_CS_PIN.
                             //   Decode:  python3 blackbox/decode.py dump.bin

//#define USE_BATTERY_MON    // Battery voltage/current monitor via ADC.
                             //   Required for low-voltage failsafe and
                             //   battery % display in GUI.

// ---------------------------------------------------------------
// BLDC motor driver selection (only when MOTOR_TYPE_BLDC is set)
// ---------------------------------------------------------------
// Default: dshot_pio.cpp  (PIO state machine — zero CPU blocking,
//                          cycle-accurate, all 4 ESCs synchronous)
//
// To use bidirectional DShot eRPM telemetry instead of BLHeli serial,
// also uncomment this in motors/dshot_pio.h:
//#define DSHOT_BIDIR         // Requires BLHeli32/BLHeli_S bidir mode
                              // enabled in ESC configurator.
                              // When disabled, BLHeli serial telemetry
                              // is used instead (separate RX pin GPIO26).
