#pragma once
#include <Arduino.h>

// ================================================================
// Altitude Hold — PID on altitude estimate
// ================================================================

void  altHoldInit();
void  altHoldSetTarget(float altMetres);
float altHoldUpdate(float throttleIn, float dt);  // returns corrected throttle
float altHoldGetTarget();
