#include "brushed_pwm.h"
#include "../config.h"

void brushedInit()
{
    analogWriteFreq(32000);   // 32 kHz, above audio band

    // RP2040/RP2350 analogWrite resolution: default 8-bit (0–255)
    // Increasing to 10-bit gives finer throttle steps.
    analogWriteResolution(10);

    const uint8_t pins[4] = { M1_PIN, M2_PIN, M3_PIN, M4_PIN };
    for (int i = 0; i < 4; i++)
    {
        pinMode(pins[i], OUTPUT);
        analogWrite(pins[i], 0);
    }
}

void brushedWriteMotors(float m1, float m2, float m3, float m4)
{
    // With 10-bit resolution, full scale = 1023
    const int maxPWM = 1023;
    auto toPWM = [&](float v) -> int
    {
        return (int)(constrain(v, 0.0f, 1.0f) * maxPWM);
    };

    analogWrite(M1_PIN, toPWM(m1));
    analogWrite(M2_PIN, toPWM(m2));
    analogWrite(M3_PIN, toPWM(m3));
    analogWrite(M4_PIN, toPWM(m4));
}

void brushedStopAll()
{
    brushedWriteMotors(0, 0, 0, 0);
}
