#include "esc_telemetry.h"
#include "../config.h"
#include "../filters/rpm_notch.h"

// ================================================================
// Choose telemetry source at compile time
// ================================================================
//   DSHOT_BIDIR    defined in dshot_pio.h → use bidir DShot eRPM
//   otherwise      use BLHeli serial 115200 baud (UART2)
// ================================================================

#if defined(DSHOT_BIDIR)
  #include "dshot_pio.h"
  #define TELEM_SOURCE_BIDIR 1
#else
  #define TELEM_SOURCE_BIDIR 0
#endif

// ================================================================
// Per-motor telemetry state
// ================================================================

typedef struct
{
    uint16_t rpm;         // mechanical RPM
    uint16_t voltage_mV;
    uint16_t current_cA;  // centi-Amps (×10 = mA)
    uint16_t temp_C;
    bool     valid;
    uint32_t lastUpdateMs;
} ESCData;

static ESCData _esc[ESC_TELEM_MOTORS];

// Telemetry valid if updated within last 500 ms
#define TELEM_TIMEOUT_MS  500

// ================================================================
// Convert electrical RPM → mechanical RPM
// ================================================================

static uint16_t erpmToMechanical(uint32_t erpm)
{
    if (MOTOR_POLE_PAIRS == 0) return 0;
    return (uint16_t)(erpm / (uint32_t)MOTOR_POLE_PAIRS);
}

// ================================================================
// PATH A: Bidirectional DShot eRPM
// ================================================================

#if TELEM_SOURCE_BIDIR

static void updateFromBidir()
{
    for (uint8_t m = 0; m < ESC_TELEM_MOTORS; m++)
    {
        uint32_t erpm = dshotPioGetRPM(m);
        if (erpm == 0 && !_esc[m].valid) continue;

        _esc[m].rpm           = erpmToMechanical(erpm);
        _esc[m].valid         = true;
        _esc[m].lastUpdateMs  = millis();
        // Voltage/current/temp not available from bidir DShot
        _esc[m].voltage_mV    = 0;
        _esc[m].current_cA    = 0;
        _esc[m].temp_C        = 0;
    }
}

// ================================================================
// PATH B: BLHeli Serial Telemetry @ 115200 baud
// ================================================================
// BLHeli32 and BLHeli_S ESCs (when configured with telemetry output
// enabled) send a 10-byte frame on a dedicated serial pin at
// 115 200 8N1.
//
// Frame layout (BLHeli32):
//   Byte  0    : Temperature (°C, or 0xFF if unavailable)
//   Bytes 1–2  : Voltage (mV, big-endian)
//   Bytes 3–4  : Current (cA, big-endian)
//   Bytes 5–6  : Consumption (mAh, big-endian) — ignored here
//   Bytes 7–8  : eRPM × 100 (big-endian) — divide by 100 to get eRPM
//   Byte  9    : CRC8 of bytes 0–8
//
// All four ESCs share the same RX pin via a diode-OR wiring (common
// BLHeli_S practice).  Each ESC sends in round-robin triggered by
// the DShot telemetry-request bit.  We poll one motor per request.
//
// UART: We use Serial2 (adapt to whichever UART is free on your board).
// The arduino-pico core supports Serial2.setRX() / setTX().
//
// NOTE: BLHeli_S telemetry pin is separate from the DShot data line.
//       Wire one free GPIO to the ESC telemetry pad.
//       For this design we use GPIO26 as the telemetry RX.
// ================================================================

#else   // !TELEM_SOURCE_BIDIR

#define BLHELI_TELEM_RX_PIN  26
#define BLHELI_TELEM_BAUD    115200
#define BLHELI_FRAME_LEN     10

// CRC8 (DVB-S2) used by BLHeli serial telemetry
static uint8_t crc8_dvbs2(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

// Round-robin: request telemetry from one motor per call.
static uint8_t _requestMotor = 0;
static uint8_t _frameBuf[BLHELI_FRAME_LEN];
static uint8_t _frameIdx = 0;
static bool    _frameActive = false;

static void requestNextMotorTelemetry()
{
    // Send a DShot packet to the next motor with telem bit set.
    // The ESC will respond with a 10-byte BLHeli telemetry frame.
    // (Only works when NOT using bidir DShot; with bidir, use PATH A.)
    // This is sent via the normal dshot.cpp bit-bang driver.
    // dshot.cpp is included in the brushed/bldc switch; for BLDC
    // builds the caller ensures dshot.cpp is in use (not dshot_pio.cpp).
    _requestMotor = (_requestMotor + 1) % ESC_TELEM_MOTORS;
    // The telem request is handled externally by the motor writer.
    // Here we just track which motor we expect data from.
    _frameIdx    = 0;
    _frameActive = true;
}

static bool parseBlheliFrame()
{
    uint8_t crcCalc = crc8_dvbs2(_frameBuf, BLHELI_FRAME_LEN - 1);
    if (crcCalc != _frameBuf[BLHELI_FRAME_LEN - 1]) return false;

    uint8_t m = _requestMotor;

    _esc[m].temp_C    = _frameBuf[0];
    _esc[m].voltage_mV= (uint16_t)((_frameBuf[1] << 8) | _frameBuf[2]);
    _esc[m].current_cA= (uint16_t)((_frameBuf[3] << 8) | _frameBuf[4]);
    // bytes 5–6 = mAh consumption (skipped)
    uint32_t erpm100  = (uint32_t)((_frameBuf[7] << 8) | _frameBuf[8]);
    uint32_t erpm     = erpm100 * 100u;

    _esc[m].rpm           = erpmToMechanical(erpm);
    _esc[m].valid         = true;
    _esc[m].lastUpdateMs  = millis();
    return true;
}

static void pollBLHeliSerial()
{
    if (!_frameActive) return;

    while (Serial2.available() && _frameIdx < BLHELI_FRAME_LEN)
        _frameBuf[_frameIdx++] = (uint8_t)Serial2.read();

    if (_frameIdx >= BLHELI_FRAME_LEN)
    {
        parseBlheliFrame();
        _frameActive = false;
    }
}

#endif  // TELEM_SOURCE_BIDIR

// ================================================================
// Public API
// ================================================================

void escTelemetryInit()
{
    memset(_esc, 0, sizeof(_esc));

#if !TELEM_SOURCE_BIDIR
    Serial2.setRX(BLHELI_TELEM_RX_PIN);
    Serial2.begin(BLHELI_TELEM_BAUD);
    Serial.print("[ESC Telem] BLHeli serial RX GPIO");
    Serial.println(BLHELI_TELEM_RX_PIN);
#else
    Serial.println("[ESC Telem] Bidirectional DShot eRPM");
#endif
}

// Call every control loop tick (1 kHz)
void escTelemetryUpdate()
{
#if TELEM_SOURCE_BIDIR
    updateFromBidir();
#else
    // Request telemetry once per motor per ~4 ms (every 4 control ticks)
    static uint8_t divider = 0;
    if (++divider >= 4)
    {
        divider = 0;
        requestNextMotorTelemetry();
    }
    pollBLHeliSerial();
#endif

    // Feed updated RPM into the notch filter bank
#ifdef USE_RPM_FILTER
    for (uint8_t m = 0; m < ESC_TELEM_MOTORS; m++)
    {
        if (_esc[m].valid)
        {
            // rpmFilterUpdateRPM expects electrical RPM
            uint32_t erpm = (uint32_t)_esc[m].rpm * MOTOR_POLE_PAIRS;
            rpmFilterUpdateRPM(m, (uint16_t)min(erpm, (uint32_t)65535u));
        }
    }
#endif
}

uint16_t escGetRPM(uint8_t m)
{
    return (m < ESC_TELEM_MOTORS) ? _esc[m].rpm : 0;
}

uint16_t escGetVoltage_mV(uint8_t m)
{
    return (m < ESC_TELEM_MOTORS) ? _esc[m].voltage_mV : 0;
}

uint16_t escGetCurrent_cA(uint8_t m)
{
    return (m < ESC_TELEM_MOTORS) ? _esc[m].current_cA : 0;
}

uint16_t escGetTemp_C(uint8_t m)
{
    return (m < ESC_TELEM_MOTORS) ? _esc[m].temp_C : 0;
}

bool escTelemetryValid(uint8_t m)
{
    return (m < ESC_TELEM_MOTORS) &&
           _esc[m].valid &&
           (millis() - _esc[m].lastUpdateMs < TELEM_TIMEOUT_MS);
}
