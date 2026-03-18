#include "bb_dump.h"
#include "../config.h"
#include <SPI.h>

// W25Q128 read command (same as blackbox.cpp — no dependency needed)
#define BB_DUMP_READ_CMD   0x03
#define BB_DUMP_CHUNK_SIZE 256
#define BB_DUMP_TOTAL      (16 * 1024 * 1024)   // 16 MiB

static bool     _active      = false;
static uint32_t _dumpOffset  = 0;

static SPISettings _bbDumpSPI(8000000, MSBFIRST, SPI_MODE0);

static void readFlashChunk(uint32_t addr, uint8_t *buf, uint16_t len)
{
    SPI.beginTransaction(_bbDumpSPI);
    digitalWrite(BB_FLASH_CS_PIN, LOW);
    SPI.transfer(BB_DUMP_READ_CMD);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >>  8) & 0xFF);
    SPI.transfer( addr        & 0xFF);
    for (uint16_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    digitalWrite(BB_FLASH_CS_PIN, HIGH);
    SPI.endTransaction();
}

void bbDumpStart()
{
    _active     = true;
    _dumpOffset = 0;

    Serial.print("BB_DUMP_START:");
    Serial.println(BB_DUMP_TOTAL);
    Serial.flush();

    Serial.println("[BB_DUMP] Streaming 16 MiB flash — do not disconnect");
}

void bbDumpUpdate()
{
    if (!_active) return;

    static uint8_t chunk[BB_DUMP_CHUNK_SIZE];

    if (_dumpOffset >= BB_DUMP_TOTAL)
    {
        Serial.println("BB_DUMP_END");
        Serial.flush();
        _active = false;
        Serial.println("[BB_DUMP] Complete");
        return;
    }

    readFlashChunk(_dumpOffset, chunk, BB_DUMP_CHUNK_SIZE);

    // Frame: 4-byte offset (LE) + 256 data bytes + 1 XOR checksum
    uint8_t header[4] = {
        (uint8_t)(_dumpOffset & 0xFF),
        (uint8_t)((_dumpOffset >>  8) & 0xFF),
        (uint8_t)((_dumpOffset >> 16) & 0xFF),
        (uint8_t)((_dumpOffset >> 24) & 0xFF)
    };
    Serial.write(header, 4);
    Serial.write(chunk, BB_DUMP_CHUNK_SIZE);

    uint8_t csum = 0;
    for (int i = 0; i < 4; i++)              csum ^= header[i];
    for (int i = 0; i < BB_DUMP_CHUNK_SIZE; i++) csum ^= chunk[i];
    Serial.write(&csum, 1);
    Serial.flush();

    _dumpOffset += BB_DUMP_CHUNK_SIZE;

    // Progress every 64 KB
    if ((_dumpOffset & 0xFFFF) == 0)
    {
        Serial.print("[BB_DUMP] ");
        Serial.print(_dumpOffset / 1024);
        Serial.print(" / ");
        Serial.print(BB_DUMP_TOTAL / 1024);
        Serial.println(" KB");
    }
}

bool bbDumpActive() { return _active; }
