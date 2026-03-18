#pragma once
#include <Arduino.h>

// ================================================================
// System Self-Test
// ================================================================
// Runs at boot (before the flight loop) to verify that all hardware
// is present, responding correctly, and within safe operating bounds.
//
// Tests performed:
//   1. BMI088 accelerometer chip ID  (expect 0x1E)
//   2. BMI088 gyroscope  chip ID     (expect 0x0F)
//   3. BMP580 barometer  chip ID     (expect 0x50)
//   4. W25Q128 flash JEDEC ID        (expect 0xEF4018)
//   5. Battery voltage reasonable    (4S: expect 12.0–17.0 V)
//   6. CRSF link received within 3 s (optional — may be flying standalone)
//   7. IMU stationary check          (gyro RMS < threshold after calibration)
//   8. Accel magnitude check         (should be ~1.0 g on flat surface)
//
// Output: coloured pass/fail table to Serial.
//
// Usage:
//   selfTestRun();        // call at end of setup(), after all inits
//   bool ok = selfTestAllPassed();
//   if (!ok) selfTestPrintReport();
// ================================================================

typedef struct
{
    const char *name;
    bool        passed;
    const char *detail;   // short message, max 40 chars
} SelfTestResult;

#define SELF_TEST_COUNT  8

void selfTestRun();
bool selfTestAllPassed();
void selfTestPrintReport();
const SelfTestResult* selfTestGetResults();   // array of SELF_TEST_COUNT items
