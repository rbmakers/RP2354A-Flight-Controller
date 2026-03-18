#pragma once
#include <Arduino.h>

// ================================================================
// IMU Calibration — gyro bias + accelerometer offset
// ================================================================
// Usage:
//   1. Place the board perfectly level and still.
//   2. Call imuCalibStart().
//   3. Call imuCalibUpdate() every IMU sample until
//      imuCalibDone() returns true.
//   4. Call imuCalibGetBias() and subtract from raw readings.
// ================================================================

typedef struct
{
    float gyroBias[3];   // rad/s
    float accelBias[3];  // g  (Z already corrected for gravity)
    bool  valid;
} IMUCalibration;

void          imuCalibStart();
bool          imuCalibRunning();
bool          imuCalibDone();
void          imuCalibUpdate(float gx, float gy, float gz,
                             float ax, float ay, float az);
IMUCalibration imuCalibGet();
void          imuCalibReset();
