#include "bmp580_driver.h"
#include "../config.h"
#include <Wire.h>
#include <math.h>

// ================================================================
// BMP580 / BMP5 Register Map  (Bosch BMP5-Sensor-API reference)
// ================================================================

#define BMP5_REG_CHIP_ID         0x01   // Expected: 0x50
#define BMP5_REG_REV_ID          0x02
#define BMP5_REG_CHIP_STATUS     0x11
#define BMP5_REG_INT_STATUS      0x27
#define BMP5_REG_STATUS          0x28
#define BMP5_REG_TEMP_DATA_XLSB  0x1D   // 3 bytes: XLSB, LSB, MSB (signed 24-bit)
#define BMP5_REG_PRESS_DATA_XLSB 0x20   // 3 bytes: XLSB, LSB, MSB (unsigned 24-bit)
#define BMP5_REG_DSP_IIR         0x31   // IIR filter coefficients
#define BMP5_REG_OSR_CONF        0x36   // oversampling config
#define BMP5_REG_ODR_CONFIG      0x37   // ODR + power mode
#define BMP5_REG_CMD             0x7E   // soft-reset = 0xB6

// ODR_CONFIG power mode bits [6:5]
#define BMP5_PWR_STANDBY    0x00
#define BMP5_PWR_NORMAL     0x03    // continuous measurement

// ODR 50 Hz
#define BMP5_ODR_50HZ       0x0F

// OSR_CONF bits:
//   [2:0] osr_t  — temperature oversampling
//   [5:3] osr_p  — pressure oversampling
//   [6]   press_en
#define BMP5_OSR_T_2X       0x01
#define BMP5_OSR_P_16X      0x04
#define BMP5_PRESS_ENABLE   (1 << 6)

// IIR filter coefficients (set_iir_t[2:0], set_iir_p[5:3])
#define BMP5_IIR_T_COEFF_1  0x01
#define BMP5_IIR_P_COEFF_7  0x03    // 0x03 << 3

// ================================================================
// I2C helpers
// ================================================================

static bool i2cWriteReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(BMP580_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static bool i2cReadBuf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(BMP580_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)BMP580_I2C_ADDR, len);
    for (uint8_t i = 0; i < len; i++)
    {
        if (!Wire.available()) return false;
        buf[i] = Wire.read();
    }
    return true;
}

// ================================================================
// Public API
// ================================================================

bool bmp580Init()
{
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.setClock(400000);   // 400 kHz fast mode
    Wire.begin();

    delay(5);

    // 1. Soft reset
    i2cWriteReg(BMP5_REG_CMD, 0xB6);
    delay(10);   // NVM reload time

    // 2. Check chip ID
    uint8_t id;
    if (!i2cReadBuf(BMP5_REG_CHIP_ID, &id, 1) || id != 0x50)
    {
        Serial.print("[BMP580] Chip ID mismatch: 0x");
        Serial.println(id, HEX);
        return false;
    }

    // 3. IIR filter: temp=1, pressure=7 coefficients
    uint8_t iirVal = (BMP5_IIR_P_COEFF_7 << 3) | BMP5_IIR_T_COEFF_1;
    i2cWriteReg(BMP5_REG_DSP_IIR, iirVal);

    // 4. Oversampling: temp 2x, pressure 16x, enable pressure output
    uint8_t osrVal = BMP5_PRESS_ENABLE
                   | ((uint8_t)BMP5_OSR_P_16X << 3)
                   | BMP5_OSR_T_2X;
    i2cWriteReg(BMP5_REG_OSR_CONF, osrVal);

    // 5. ODR = 50 Hz, normal (continuous) mode
    uint8_t odrVal = ((uint8_t)BMP5_PWR_NORMAL << 5) | BMP5_ODR_50HZ;
    i2cWriteReg(BMP5_REG_ODR_CONFIG, odrVal);

    delay(5);
    Serial.println("[BMP580] Init OK");
    return true;
}

float bmp580ReadPressure()
{
    uint8_t d[3];
    if (!i2cReadBuf(BMP5_REG_PRESS_DATA_XLSB, d, 3)) return 0.0f;

    uint32_t raw = ((uint32_t)d[2] << 16) |
                   ((uint32_t)d[1] <<  8) |
                    (uint32_t)d[0];

    // BMP580 output: Pa * 64  → divide by 64.0 to get Pa
    return (float)raw / 64.0f;
}

float bmp580ReadTemperature()
{
    uint8_t d[3];
    if (!i2cReadBuf(BMP5_REG_TEMP_DATA_XLSB, d, 3)) return 0.0f;

    // Sign-extend 24-bit → 32-bit
    int32_t raw = (int32_t)(((uint32_t)d[2] << 16) |
                             ((uint32_t)d[1] <<  8) |
                              (uint32_t)d[0]);
    if (raw & 0x00800000) raw |= 0xFF000000;   // sign extend

    // BMP580 output: °C * 65536 → divide by 65536 to get °C
    return (float)raw / 65536.0f;
}

float bmp580PressureToAltitude(float pressurePa, float seaLevelPa)
{
    if (pressurePa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - powf(pressurePa / seaLevelPa, 0.1903f));
}
