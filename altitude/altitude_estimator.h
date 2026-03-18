#pragma once
#include <Arduino.h>

// ================================================================
// Altitude Estimator — barometer + accelerometer complementary filter
// ================================================================
// Fast dynamics come from the accelerometer (integrated to velocity/
// position).  Long-term drift is corrected by the barometer.
//
// Call altEstInit() once, then altEstUpdate() every control loop at
// FLIGHT_LOOP_HZ.  The barometer is read externally and passed in
// as the pressurePa argument.
// ================================================================

void  altEstInit(float firstPressurePa);
void  altEstUpdate(float accZg,            // Z accel in g (positive up)
                   float pressurePa,       // from BMP580
                   float dt);              // loop dt in seconds
float altEstGetAltitude();                 // metres AGL
float altEstGetVerticalSpeed();            // m/s positive up
