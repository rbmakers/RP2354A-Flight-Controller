#include "blackbox.h"
#include "../config.h"
#include <SPI.h>

// ================================================================
// Compile-time storage selection
// ================================================================

#if defined(BB_STORAGE_SD)
  #include <SD.h>
  #define BB_SD_CS_PIN  BB_FLASH_CS_PIN   // reuse same CS define
#else
  #define BB_STORAGE_FLASH                 // default: W25Q128
#endif

// ================================================================
// W25Q128 Command Set
// ================================================================
#define W25_CMD_WRITE_EN      0x06
#define W25_CMD_WRITE_DIS     0x04
#define W25_CMD_READ_SR1      0x05   // status register 1
#define W25_CMD_READ_SR2      0x35
#define W25_CMD_CHIP_ERASE    0xC7
#define W25_CMD_SECTOR_ERASE  0x20   // 4 KB
#define W25_CMD_PAGE_PROGRAM  0x02   // write up to 256 bytes
#define W25_CMD_READ          0x03
#define W25_CMD_JEDEC_ID      0x9F

#define W25_SR1_BUSY  0x01   // bit 0 of status register 1

#define W25_PAGE_SIZE    256
#define W25_SECTOR_SIZE  4096
#define W25_TOTAL_BYTES  (16 * 1024 * 1024)   // 16 MiB

// Flash layout constants
#define BB_HEADER_SECTOR    0
#define BB_INDEX_SECTOR     1                   // 1 sector = 4 KB for index
#define BB_DATA_START_ADDR  (32 * W25_SECTOR_SIZE)  // 128 KB reserved for header+index

#define BB_MAGIC  0xBB354A01

// ================================================================
// Module state
// ================================================================

static SPISettings bbSPI(8000000, MSBFIRST, SPI_MODE0);  // 8 MHz

static bool     _ready        = false;
static bool     _sessionOpen  = false;
static uint32_t _writeAddr    = BB_DATA_START_ADDR;
static uint32_t _bytesWritten = 0;
static uint8_t  _sessionId    = 0;

// Page-write buffer: accumulate 8 frames (256 bytes = 1 flash page)
// so we write in complete pages for maximum throughput.
#define BB_PAGE_FRAMES  8      // 8 × 32 = 256 bytes = 1 page
static BBFrame  _pageBuf[BB_PAGE_FRAMES];
static uint8_t  _pageBufIdx = 0;

// ================================================================
// SPI flash low-level helpers
// ================================================================

static inline void flashCS(bool low)
{
    digitalWrite(BB_FLASH_CS_PIN, low ? LOW : HIGH);
}

static void flashWaitReady()
{
    SPI.beginTransaction(bbSPI);
    flashCS(true);
    SPI.transfer(W25_CMD_READ_SR1);
    uint32_t tOut = millis() + 3000;   // 3 s timeout
    while ((SPI.transfer(0x00) & W25_SR1_BUSY) && millis() < tOut)
        ; // busy-wait
    flashCS(false);
    SPI.endTransaction();
}

static void flashWriteEnable()
{
    SPI.beginTransaction(bbSPI);
    flashCS(true);
    SPI.transfer(W25_CMD_WRITE_EN);
    flashCS(false);
    SPI.endTransaction();
}

static void flashSectorErase(uint32_t addr)
{
    flashWriteEnable();
    SPI.beginTransaction(bbSPI);
    flashCS(true);
    SPI.transfer(W25_CMD_SECTOR_ERASE);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >>  8) & 0xFF);
    SPI.transfer( addr        & 0xFF);
    flashCS(false);
    SPI.endTransaction();
    flashWaitReady();
}

// Write one full 256-byte page.  addr must be page-aligned.
static void flashPageWrite(uint32_t addr, const uint8_t *data)
{
    flashWriteEnable();
    SPI.beginTransaction(bbSPI);
    flashCS(true);
    SPI.transfer(W25_CMD_PAGE_PROGRAM);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >>  8) & 0xFF);
    SPI.transfer( addr        & 0xFF);
    for (uint16_t i = 0; i < W25_PAGE_SIZE; i++)
        SPI.transfer(data[i]);
    flashCS(false);
    SPI.endTransaction();
    flashWaitReady();
}

static void flashRead(uint32_t addr, uint8_t *buf, uint16_t len)
{
    SPI.beginTransaction(bbSPI);
    flashCS(true);
    SPI.transfer(W25_CMD_READ);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >>  8) & 0xFF);
    SPI.transfer( addr        & 0xFF);
    for (uint16_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    flashCS(false);
    SPI.endTransaction();
}

static uint32_t flashJEDECID()
{
    SPI.beginTransaction(bbSPI);
    flashCS(true);
    SPI.transfer(W25_CMD_JEDEC_ID);
    uint32_t id = ((uint32_t)SPI.transfer(0) << 16) |
                  ((uint32_t)SPI.transfer(0) <<  8) |
                   (uint32_t)SPI.transfer(0);
    flashCS(false);
    SPI.endTransaction();
    return id;
}

// ================================================================
// Flash-backed session index
// ================================================================
// Session index lives in sector 1 (4 KB).
// Each entry is 16 bytes:
//   uint32_t magic    (0xBB354A01)
//   uint32_t start_addr
//   uint32_t byte_count
//   uint32_t timestamp_us (of first frame)

#define BB_IDX_ENTRY_SIZE  16

static void indexWriteSession(uint8_t sid, uint32_t start,
                               uint32_t bytes, uint32_t ts)
{
    uint32_t addr = BB_INDEX_SECTOR * W25_SECTOR_SIZE
                  + sid * BB_IDX_ENTRY_SIZE;

    // We need a full 256-byte page, so read the page, patch, rewrite
    // (simplified: just write directly, flash cells will be 0xFF→value)
    flashWriteEnable();
    SPI.beginTransaction(bbSPI);
    flashCS(true);
    SPI.transfer(W25_CMD_PAGE_PROGRAM);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >>  8) & 0xFF);
    SPI.transfer( addr        & 0xFF);
    // Encode 16-byte entry
    uint32_t words[4] = { BB_MAGIC, start, bytes, ts };
    for (int w = 0; w < 4; w++)
        for (int b = 0; b < 4; b++)
            SPI.transfer((words[w] >> (8*b)) & 0xFF);
    flashCS(false);
    SPI.endTransaction();
    flashWaitReady();
}

// Scan index to find next unused session slot + write pointer.
static void indexScan()
{
    _sessionId   = 0;
    _writeAddr   = BB_DATA_START_ADDR;
    _bytesWritten = 0;

    for (uint8_t i = 0; i < BB_MAX_SESSIONS; i++)
    {
        uint32_t addr = BB_INDEX_SECTOR * W25_SECTOR_SIZE
                      + i * BB_IDX_ENTRY_SIZE;
        uint8_t entry[16];
        flashRead(addr, entry, 16);

        uint32_t magic = ((uint32_t)entry[3] << 24) | ((uint32_t)entry[2] << 16)
                       | ((uint32_t)entry[1] <<  8) |  (uint32_t)entry[0];

        if (magic == BB_MAGIC)
        {
            uint32_t start = ((uint32_t)entry[7] << 24) | ((uint32_t)entry[6] << 16)
                           | ((uint32_t)entry[5] <<  8) |  (uint32_t)entry[4];
            uint32_t bytes = ((uint32_t)entry[11] << 24) | ((uint32_t)entry[10] << 16)
                           | ((uint32_t)entry[9]  <<  8) |  (uint32_t)entry[8];

            _sessionId  = i + 1;
            _writeAddr  = start + bytes;
            // Align to next page boundary
            _writeAddr  = (_writeAddr + W25_PAGE_SIZE - 1) & ~(W25_PAGE_SIZE - 1);
        }
    }

    // Wrap around if flash is full
    if (_writeAddr + W25_SECTOR_SIZE >= W25_TOTAL_BYTES)
    {
        Serial.println("[BB] Flash full — wrapping to start");
        _writeAddr  = BB_DATA_START_ADDR;
        _sessionId  = 0;
    }

    Serial.print("[BB] Next session: "); Serial.print(_sessionId);
    Serial.print("  write @ 0x"); Serial.println(_writeAddr, HEX);
}

// ================================================================
// Page buffer flush
// ================================================================

static void flushPageBuf()
{
    if (_pageBufIdx == 0) return;

    // Pad remainder of page with 0xFF (erased state = no-op)
    static uint8_t page[W25_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, _pageBuf, _pageBufIdx * BB_FRAME_SIZE);

    flashPageWrite(_writeAddr, page);
    _writeAddr   += W25_PAGE_SIZE;
    _bytesWritten += _pageBufIdx * BB_FRAME_SIZE;
    _pageBufIdx   = 0;
}

// ================================================================
// Public API
// ================================================================

bool bbInit()
{
    // SPI bus already started by IMU; just configure our CS pin.
    pinMode(BB_FLASH_CS_PIN, OUTPUT);
    digitalWrite(BB_FLASH_CS_PIN, HIGH);

    delay(5);

    uint32_t jedec = flashJEDECID();
    Serial.print("[BB] Flash JEDEC ID: 0x");
    Serial.println(jedec, HEX);

    // W25Q128JV JEDEC: 0xEF4018
    if ((jedec & 0xFFFF00) != 0xEF4000)
    {
        Serial.println("[BB] W25Q128 not found — blackbox disabled");
        return false;
    }

    indexScan();
    _ready = true;
    Serial.println("[BB] Blackbox ready");
    return true;
}

void bbSessionStart()
{
    if (!_ready) return;
    if (_sessionOpen) bbSessionEnd();

    // Erase enough sectors for a ~30 s session at 1 kHz
    // 30 s × 1000 Hz × 32 B = 960 KB → 240 sectors.
    // We erase lazily (only the first sector) and rely on the fact
    // that the flash region was erased during bbEraseAll() or wrap.
    // For a more robust implementation erase the full session region here.
    flashSectorErase(_writeAddr);

    _sessionOpen   = true;
    _pageBufIdx    = 0;
    _bytesWritten  = 0;

    Serial.print("[BB] Session "); Serial.print(_sessionId);
    Serial.print(" started @ 0x"); Serial.println(_writeAddr, HEX);
}

void bbSessionEnd()
{
    if (!_ready || !_sessionOpen) return;

    flushPageBuf();   // flush partial page

    // Write a sentinel frame (all-zero timestamp = end marker)
    BBFrame sentinel = {};
    sentinel.flags = 0xFF;
    bbWrite(&sentinel);
    flushPageBuf();

    indexWriteSession(_sessionId % BB_MAX_SESSIONS,
                      _writeAddr - _bytesWritten,
                      _bytesWritten,
                      micros());

    _sessionOpen = false;
    _sessionId++;

    Serial.print("[BB] Session ended. Bytes: ");
    Serial.println(_bytesWritten);
}

void bbWrite(const BBFrame *frame)
{
    if (!_ready || !_sessionOpen) return;

    _pageBuf[_pageBufIdx++] = *frame;

    if (_pageBufIdx >= BB_PAGE_FRAMES)
        flushPageBuf();
}

bool     bbReady()        { return _ready && _sessionOpen; }
uint32_t bbBytesWritten() { return _bytesWritten; }

void bbEraseAll()
{
    Serial.println("[BB] Erasing all flash...");
    // Chip erase: ~40 seconds for W25Q128 — only do on bench.
    flashWriteEnable();
    SPI.beginTransaction(bbSPI);
    flashCS(true);
    SPI.transfer(W25_CMD_CHIP_ERASE);
    flashCS(false);
    SPI.endTransaction();
    flashWaitReady();
    _sessionId  = 0;
    _writeAddr  = BB_DATA_START_ADDR;
    _bytesWritten = 0;
    Serial.println("[BB] Erase complete");
}
