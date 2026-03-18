#pragma once
#include <Arduino.h>

// ================================================================
// BMP580 Barometer Driver — I2C0, continuous mode @ 50 Hz
// ================================================================
// Hardware:  SDA=16, SCL=17, I2C address 0x47 (SDO=VDD)
// Output:    Pressure in Pa (24-bit, 1/64 Pa resolution)
//            Temperature in °C (24-bit signed, 1/65536 °C resolution)
// ================================================================

bool  bmp580Init();            // returns false if chip not found
float bmp580ReadPressure();    // Pa
float bmp580ReadTemperature(); // °C
float bmp580PressureToAltitude(float pressurePa,
                                float seaLevelPa = 101325.0f);
