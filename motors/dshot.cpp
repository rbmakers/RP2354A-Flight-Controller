#include "dshot.h"
#include "../config.h"
#include "hardware/gpio.h"

// ================================================================
// DShot600 frame encoder
// ================================================================
// Frame layout (MSB first):
//   [15:5]  11-bit throttle value (0=disarm, 48–2047=throttle)
//   [4]     1-bit telemetry request
//   [3:0]   4-bit CRC = XOR of nibbles [15:4]

static uint16_t dshotEncode(uint16_t throttle, bool telemetry)
{
    uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);

    // CRC: XOR of the three 4-bit nibbles of packet[15:4]
    uint16_t csum = 0;
    uint16_t v    = packet;
    for (int i = 0; i < 3; i++) { csum ^= v; v >>= 4; }
    csum &= 0x0F;

    return (packet << 4) | csum;
}

// ================================================================
// Bit-bang transmit
// ================================================================
// Uses nop loops for timing at 150 MHz.
// gpio_put() is used for direct, fast GPIO toggling.
// Interrupts are disabled during the frame to prevent glitches.
// Replace with RP235x PIO for production-grade output.

static inline void nops(uint32_t n)
{
    for (volatile uint32_t i = 0; i < n; i++) __asm__ volatile("nop");
}

void dshotSendPacket(uint8_t pin, uint16_t value, bool telemetry)
{
    uint16_t frame = dshotEncode(value, telemetry);

    noInterrupts();

    for (int i = 15; i >= 0; i--)
    {
        bool bit = (frame >> i) & 1;

        if (bit)
        {
            gpio_put(pin, 1);
            nops(DSHOT600_T1H_CYCLES);
            gpio_put(pin, 0);
            nops(DSHOT600_T1L_CYCLES);
        }
        else
        {
            gpio_put(pin, 1);
            nops(DSHOT600_T0H_CYCLES);
            gpio_put(pin, 0);
            nops(DSHOT600_T0L_CYCLES);
        }
    }

    interrupts();

    // Inter-frame gap ≥ 2 µs
    delayMicroseconds(3);
}

// ================================================================
// Public API
// ================================================================

void dshotInit()
{
    const uint8_t pins[4] = { ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN };
    for (int i = 0; i < 4; i++)
    {
        pinMode(pins[i], OUTPUT);
        gpio_put(pins[i], 0);
    }

    // Send 100 ms of zero-throttle (disarm) frames at startup
    // so ESCs recognize the controller and arm their power stage.
    uint32_t tEnd = millis() + 100;
    while (millis() < tEnd)
    {
        for (int i = 0; i < 4; i++)
            dshotSendPacket(pins[i], 0, false);
        delayMicroseconds(200);
    }
}

// m1..m4: normalised throttle 0.0–1.0
void dshotWriteMotors(float m1, float m2, float m3, float m4)
{
    // DShot throttle range: 48 (min) to 2047 (max).
    // Value 0 = disarm command.  Values 1–47 are reserved commands.
    auto toDS = [](float v) -> uint16_t
    {
        v = constrain(v, 0.0f, 1.0f);
        if (v < 0.001f) return 0;                       // disarm
        return (uint16_t)(48 + v * (2047 - 48));
    };

    dshotSendPacket(ESC1_PIN, toDS(m1), false);
    dshotSendPacket(ESC2_PIN, toDS(m2), false);
    dshotSendPacket(ESC3_PIN, toDS(m3), false);
    dshotSendPacket(ESC4_PIN, toDS(m4), false);
}
