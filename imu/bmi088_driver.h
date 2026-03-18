#pragma once
#include <Arduino.h>

// ================================================================
// BMI088 IMU Driver  — SPI, interrupt-driven gyro at 2 kHz
// ================================================================
// Hardware:  MOSI=19, MISO=20, SCK=22, CS_ACC=21, CS_GYR=23
//            GYRO_INT3 → GPIO 24
//
// Protocol notes:
//   Accel reads require one dummy byte after the register address.
//   Gyro reads do NOT require a dummy byte.
// ================================================================

// ------ Public data type ----------------------------------------
typedef struct
{
    volatile float gx, gy, gz;   // rad/s
    volatile float ax, ay, az;   // g
    volatile uint32_t timestamp; // micros() at last update
} IMURaw;

// ------ Public API ----------------------------------------------
void   bmi088Init();

// Returns a snapshot copy of the latest ISR data (atomic copy).
IMURaw bmi088GetLatest();

// Chip-ID verification (call after init; returns true if sensors found).
bool   bmi088CheckIDs();
