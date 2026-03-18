#pragma once
#include <Arduino.h>

// ================================================================
// Brushed Motor Driver — 32 kHz PWM via analogWrite()
// ================================================================
// Pins: M1=GPIO29, M2=GPIO11, M3=GPIO18, M4=GPIO25
// Drives coreless motors through single-MOSFET stages.
// 32 kHz keeps switching noise above human hearing and above
// the BMI088 accelerometer passband.
// ================================================================

void brushedInit();
void brushedWriteMotors(float m1, float m2, float m3, float m4);
void brushedStopAll();
