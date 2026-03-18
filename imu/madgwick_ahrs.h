#pragma once
#include <Arduino.h>

// ================================================================
// Madgwick AHRS
// S. Madgwick, "An efficient orientation filter for inertial and
// inertial/magnetic sensor arrays", 2010.
// ================================================================

typedef struct { float q0, q1, q2, q3; } Quaternion;

void       madgwickInit(float sampleRateHz, float beta);
void       madgwickUpdate(float gx, float gy, float gz,
                          float ax, float ay, float az);
Quaternion madgwickGetQ();
void       madgwickGetEuler(float *rollDeg, float *pitchDeg, float *yawDeg);
