#pragma once
#include <Arduino.h>

// ================================================================
// ExpressLRS / CRSF Receiver Parser
// ================================================================
// UART0 @ 420 000 baud:  TX=GPIO12, RX=GPIO13
//
// CRSF frame format:
//   Byte 0  : Sync    = 0xC8
//   Byte 1  : Length  = payload_len + 2  (type + payload + CRC)
//   Byte 2  : Type
//   Byte 3..N: Payload
//   Last byte: CRC8 (DVB-S2 polynomial 0xD5)
//
// RC_CHANNELS_PACKED (type 0x16):
//   22-byte payload, 16 channels packed at 11 bits each.
//   Channel range: 172–1811 (mid = 992)
//   Normalised output: −1.0 to +1.0 for sticks, 0.0/1.0 for switches.
// ================================================================

#define CRSF_CHANNELS  16

typedef struct
{
    uint16_t raw[CRSF_CHANNELS];   // 172–1811 µs equivalent
    float    norm[CRSF_CHANNELS];  // normalised −1.0 to +1.0
    bool     valid;                // true after first good frame
    uint32_t lastFrameMs;          // millis() of last good frame
} CRSFData;

void       crsfInit();             // call in Core1 setup
void       crsfTask();             // call repeatedly in Core1 loop
CRSFData   crsfGet();              // atomic snapshot for Core0
bool       crsfIsLinkUp();         // false if no frame in last 500 ms

// Convenience accessors (normalised −1…+1, or 0…1 for throttle)
float crsfRoll();      // CH1
float crsfPitch();     // CH2
float crsfThrottle();  // CH3  (0.0…1.0)
float crsfYaw();       // CH4
uint16_t crsfCH5();    // arm switch (raw µs)
uint16_t crsfCH6();    // mode switch (raw µs)
uint16_t crsfCH7();    // alt-hold switch (raw µs)
