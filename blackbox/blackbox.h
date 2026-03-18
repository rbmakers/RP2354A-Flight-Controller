#pragma once
#include <Arduino.h>

// ================================================================
// Blackbox Flight Data Logger
// ================================================================
// Records a compact binary log frame every control-loop tick to
// an on-board SPI flash (W25Q128JV, 16 MiB) or micro-SD card.
//
// Storage selection (compile-time):
//   #define BB_STORAGE_FLASH   — W25Q128 on SPI (recommended)
//   #define BB_STORAGE_SD      — SD card via SPI + SD library
//
// Default: BB_STORAGE_FLASH
//
// SPI pins (shared with IMU bus — CS pin is separate):
//   MOSI = GPIO19, MISO = GPIO20, SCK = GPIO22
//   CS   = GPIO26  (BB_FLASH_CS_PIN in config.h)
//
// Flash memory layout (W25Q128, 16 MiB = 16 777 216 bytes):
//   Sector 0         : header (magic, firmware version, session count)
//   Sectors 1–16     : session index (up to 16 sessions)
//   Sector 32 onward : log data, consecutive 32-byte frames
//
// Each log frame (32 bytes, packed):
//   uint32_t  timestamp_us       4
//   int16_t   roll_x10           2   (degrees × 10)
//   int16_t   pitch_x10          2
//   int16_t   yaw_x10            2
//   int16_t   gx_x100            2   (rad/s × 100)
//   int16_t   gy_x100            2
//   int16_t   gz_x100            2
//   uint16_t  m1,m2,m3,m4        8   (0–1000 per motor)
//   uint16_t  altitude_cm        2   (centimetres)
//   uint16_t  voltage_mV         2
//   uint8_t   armed_mode         1   (bit7=armed, bits[1:0]=mode)
//   uint8_t   flags              1   (bit0=altHold, bit1=calibrating)
//   uint16_t  loop_time_us       2   (actual control loop period)
//                               ──
//                               32 bytes total
//
// Capacity at 1 kHz: 32 bytes × 1000 frames/s = 32 KB/s
//   W25Q128 (16 MiB) ÷ 32 KB/s ≈ 524 seconds of flight per session.
//
// Blackbox decode:
//   A Python decode script is provided in blackbox/decode.py.
//   Output is CSV compatible with Betaflight Blackbox Explorer
//   (with minor column remapping).
// ================================================================

// Enable blackbox in config.h:
//   #define USE_BLACKBOX

#define BB_FRAME_SIZE   32      // bytes per log frame (must stay 32)
#define BB_MAX_SESSIONS 16

typedef struct __attribute__((packed))
{
    uint32_t timestamp_us;
    int16_t  roll_x10;
    int16_t  pitch_x10;
    int16_t  yaw_x10;
    int16_t  gx_x100;
    int16_t  gy_x100;
    int16_t  gz_x100;
    uint16_t m1, m2, m3, m4;    // 0–1000
    uint16_t altitude_cm;
    uint16_t voltage_mV;
    uint8_t  armed_mode;         // bit7=armed, [1:0]=mode
    uint8_t  flags;              // bit0=altHold, bit1=calibrating
    uint16_t loop_time_us;
} BBFrame;                       // 32 bytes

static_assert(sizeof(BBFrame) == BB_FRAME_SIZE,
              "BBFrame must be exactly 32 bytes");

// ---- Public API ------------------------------------------------

// Init flash/SD, mount existing log index.
// Returns false if storage device not found.
bool  bbInit();

// Start a new log session (erases one flash region).
// Called on ARM.
void  bbSessionStart();

// Close current session (write end marker, flush).
// Called on DISARM.
void  bbSessionEnd();

// Write one frame to the log.
// Call every 1 kHz loop tick when armed.
// Internally buffers 16 frames before a flash page-write to avoid
// blocking the flight loop (page write ≈ 700 µs on W25Q128).
void  bbWrite(const BBFrame *frame);

// True if storage is ready and a session is open.
bool  bbReady();

// Returns total bytes written in current session.
uint32_t bbBytesWritten();

// Erase all sessions (called from GUI "ERASE LOG" command).
void  bbEraseAll();
