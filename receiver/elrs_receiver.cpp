#include "elrs_receiver.h"
#include "../config.h"

// ================================================================
// CRC8 DVB-S2  (polynomial 0xD5)
// ================================================================

static const uint8_t crc8_dvb_s2_table[256] = {
    0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54,0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
    0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06,0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
    0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0,0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
    0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2,0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
    0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9,0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
    0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B,0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
    0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D,0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
    0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F,0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
    0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB,0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
    0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9,0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
    0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F,0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
    0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D,0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
    0x72,0xA7,0x0D,0xD8,0x8C,0x59,0xF3,0x26,0x5B,0x8E,0x24,0xF1,0xA5,0x70,0xDA,0x0F,
    0x20,0xF5,0x5F,0x8A,0xDE,0x0B,0xA1,0x74,0x09,0xDC,0x76,0xA3,0xF7,0x22,0x88,0x5D,
    0xD6,0x03,0xA9,0x7C,0x28,0xFD,0x57,0x82,0xFF,0x2A,0x80,0x55,0x01,0xD4,0x7E,0xAB,
    0x84,0x51,0xFB,0x2E,0x7A,0xAF,0x05,0xD0,0xAD,0x78,0xD2,0x07,0x53,0x86,0x2C,0xF9
};

static uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
        crc = crc8_dvb_s2_table[crc ^ data[i]];
    return crc;
}

// ================================================================
// CRSF constants
// ================================================================

#define CRSF_SYNC_BYTE          0xC8
#define CRSF_TYPE_RC_CHANNELS   0x16
#define CRSF_MAX_FRAME_LEN      64
#define CRSF_LINK_TIMEOUT_MS    500

// CRSF channel range
#define CRSF_CH_MIN   172
#define CRSF_CH_MID   992
#define CRSF_CH_MAX  1811

// ================================================================
// Parser state machine
// ================================================================

static enum { S_WAIT_SYNC, S_WAIT_LEN, S_PAYLOAD, S_CRC } _state = S_WAIT_SYNC;
static uint8_t  _buf[CRSF_MAX_FRAME_LEN];
static uint8_t  _frameLen  = 0;
static uint8_t  _byteIndex = 0;

// Shared data (written by Core1, read by Core0 via crsfGet())
static volatile CRSFData _rcData = {};
static volatile CRSFData _rcShadow = {};   // double-buffer

// ================================================================
// Channel unpacking — 16 channels × 11 bits = 176 bits = 22 bytes
// ================================================================

static void unpackChannels(const uint8_t *payload)
{
    // Unpack 16 × 11-bit values from 22 bytes (little-endian bit stream)
    uint16_t ch[CRSF_CHANNELS];

    const uint8_t *p = payload;
    ch[ 0] = ((uint16_t)p[ 0]       | (uint16_t)p[ 1] << 8) & 0x07FF;
    ch[ 1] = ((uint16_t)p[ 1] >>  3 | (uint16_t)p[ 2] << 5) & 0x07FF;
    ch[ 2] = ((uint16_t)p[ 2] >>  6 | (uint16_t)p[ 3] << 2
                                     | (uint16_t)p[ 4] <<10) & 0x07FF;
    ch[ 3] = ((uint16_t)p[ 4] >>  1 | (uint16_t)p[ 5] << 7) & 0x07FF;
    ch[ 4] = ((uint16_t)p[ 5] >>  4 | (uint16_t)p[ 6] << 4) & 0x07FF;
    ch[ 5] = ((uint16_t)p[ 6] >>  7 | (uint16_t)p[ 7] << 1
                                     | (uint16_t)p[ 8] << 9) & 0x07FF;
    ch[ 6] = ((uint16_t)p[ 8] >>  2 | (uint16_t)p[ 9] << 6) & 0x07FF;
    ch[ 7] = ((uint16_t)p[ 9] >>  5 | (uint16_t)p[10] << 3) & 0x07FF;
    ch[ 8] = ((uint16_t)p[11]       | (uint16_t)p[12] << 8) & 0x07FF;
    ch[ 9] = ((uint16_t)p[12] >>  3 | (uint16_t)p[13] << 5) & 0x07FF;
    ch[10] = ((uint16_t)p[13] >>  6 | (uint16_t)p[14] << 2
                                     | (uint16_t)p[15] <<10) & 0x07FF;
    ch[11] = ((uint16_t)p[15] >>  1 | (uint16_t)p[16] << 7) & 0x07FF;
    ch[12] = ((uint16_t)p[16] >>  4 | (uint16_t)p[17] << 4) & 0x07FF;
    ch[13] = ((uint16_t)p[17] >>  7 | (uint16_t)p[18] << 1
                                     | (uint16_t)p[19] << 9) & 0x07FF;
    ch[14] = ((uint16_t)p[19] >>  2 | (uint16_t)p[20] << 6) & 0x07FF;
    ch[15] = ((uint16_t)p[20] >>  5 | (uint16_t)p[21] << 3) & 0x07FF;

    CRSFData d;
    d.valid       = true;
    d.lastFrameMs = millis();

    for (int i = 0; i < CRSF_CHANNELS; i++)
    {
        d.raw[i] = ch[i];
        // Normalise: sticks → −1 … +1, centre = CRSF_CH_MID
        d.norm[i] = (float)(ch[i] - CRSF_CH_MID) / (float)(CRSF_CH_MAX - CRSF_CH_MID);
        d.norm[i] = constrain(d.norm[i], -1.0f, 1.0f);
    }

    // CH3 (throttle, index 2): remap to 0…1
    d.norm[2] = (float)(ch[2] - CRSF_CH_MIN) / (float)(CRSF_CH_MAX - CRSF_CH_MIN);
    d.norm[2] = constrain(d.norm[2], 0.0f, 1.0f);

    // Atomic double-buffer swap
    noInterrupts();
    _rcData = d;
    interrupts();
}

// ================================================================
// Byte-level state machine
// ================================================================

static void crsfParseByte(uint8_t b)
{
    switch (_state)
    {
    case S_WAIT_SYNC:
        if (b == CRSF_SYNC_BYTE) { _buf[0] = b; _state = S_WAIT_LEN; }
        break;

    case S_WAIT_LEN:
        if (b < 2 || b > CRSF_MAX_FRAME_LEN - 2)
        {
            _state = S_WAIT_SYNC;   // invalid length
            break;
        }
        _frameLen  = b + 2;   // total = sync(1) + len(1) + payload+type+crc
        _buf[1]    = b;
        _byteIndex = 2;
        _state     = S_PAYLOAD;
        break;

    case S_PAYLOAD:
        _buf[_byteIndex++] = b;
        if (_byteIndex >= _frameLen - 1) _state = S_CRC;
        break;

    case S_CRC:
        _buf[_byteIndex] = b;

        // CRC covers bytes [2 .. frameLen-2] (type + payload, NOT sync/len/crc)
        uint8_t calcCRC = crc8(&_buf[2], _frameLen - 3);

        if (calcCRC == b && _buf[2] == CRSF_TYPE_RC_CHANNELS)
            unpackChannels(&_buf[3]);

        _state = S_WAIT_SYNC;
        break;
    }
}

// ================================================================
// Public API
// ================================================================

void crsfInit()
{
    Serial1.setTX(ELRS_TX_PIN);
    Serial1.setRX(ELRS_RX_PIN);
    Serial1.begin(ELRS_BAUD);
}

void crsfTask()
{
    while (Serial1.available())
        crsfParseByte((uint8_t)Serial1.read());
}

CRSFData crsfGet()
{
    noInterrupts();
    CRSFData copy = _rcData;
    interrupts();
    return copy;
}

bool crsfIsLinkUp()
{
    return _rcData.valid &&
           (millis() - _rcData.lastFrameMs < CRSF_LINK_TIMEOUT_MS);
}

// ---- Convenience accessors -----

float crsfRoll()      { return _rcData.norm[0]; }
float crsfPitch()     { return _rcData.norm[1]; }
float crsfThrottle()  { return _rcData.norm[2]; }   // 0…1
float crsfYaw()       { return _rcData.norm[3]; }
uint16_t crsfCH5()    { return _rcData.raw[4];  }
uint16_t crsfCH6()    { return _rcData.raw[5];  }
uint16_t crsfCH7()    { return _rcData.raw[6];  }
