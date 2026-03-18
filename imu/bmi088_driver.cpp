#include "bmi088_driver.h"
#include "../config.h"
#include <SPI.h>

// ================================================================
// BMI088 Register Map
// ================================================================

// --- Accelerometer (separate die, needs dummy byte on SPI read) --
#define ACC_REG_CHIP_ID    0x00   // Expected: 0x1E
#define ACC_REG_DATA       0x12   // 6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
#define ACC_REG_CONF       0x40   // [6:4]=BWP, [3:0]=ODR
#define ACC_REG_RANGE      0x41   // 0x00=3g,0x01=6g,0x02=12g,0x03=24g
#define ACC_REG_PWR_CONF   0x7C   // 0x00=active, 0x03=suspend
#define ACC_REG_PWR_CTRL   0x7D   // 0x04=on, 0x00=off
#define ACC_REG_SOFTRESET  0x7E   // write 0xB6

// Accel ODR + BWP
// BWP[6:4]: 0x0=OSR4, 0x1=OSR2, 0x2=normal
// ODR[3:0]: 0x05=12.5Hz … 0x0C=1600Hz
#define ACC_CONF_1600HZ_NORMAL  0x2C   // BWP=normal(0x2<<4) | ODR=1600Hz(0x0C)
#define ACC_RANGE_24G           0x03

// --- Gyroscope (separate die, standard SPI) ----------------------
#define GYR_REG_CHIP_ID    0x00   // Expected: 0x0F
#define GYR_REG_DATA       0x02   // 6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
#define GYR_REG_RANGE      0x0F   // 0x00=2000dps … 0x04=125dps
#define GYR_REG_BANDWIDTH  0x10   // 0x00=2000Hz/532Hz … 0x07=100Hz/32Hz
#define GYR_REG_LPM1       0x11   // 0x00=normal
#define GYR_REG_INT_CTRL   0x15   // bit7=1 → enable DRDY interrupt
#define GYR_REG_INT34_CONF 0x16   // INT3: bit1=LVL(1=hi), bit0=OD(0=PP)
#define GYR_REG_INT34_MAP  0x18   // bit0=1 → DRDY on INT3

// Gyro config: 2000 Hz ODR, 532 Hz filter BW
#define GYR_BW_2000HZ    0x00
#define GYR_RANGE_2000   0x00
#define GYR_INT3_ACTIVE_HIGH_PP  0x02  // push-pull, active high
#define GYR_INT3_MAP_DRDY        0x01

// ================================================================
// Module state
// ================================================================

static SPISettings spiSettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0);
static volatile IMURaw imuLatest;

// ================================================================
// SPI helpers (not called from ISR during init, safe)
// ================================================================

static void accelWriteReg(uint8_t reg, uint8_t val)
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(CS_ACCEL, LOW);
    SPI.transfer(reg & 0x7F);   // write: MSB = 0
    SPI.transfer(val);
    digitalWrite(CS_ACCEL, HIGH);
    SPI.endTransaction();
}

static void accelReadBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(CS_ACCEL, LOW);
    SPI.transfer(reg | 0x80);   // read: MSB = 1
    SPI.transfer(0x00);          // BMI088 accel mandatory dummy byte
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    digitalWrite(CS_ACCEL, HIGH);
    SPI.endTransaction();
}

static uint8_t accelReadReg(uint8_t reg)
{
    uint8_t val;
    accelReadBurst(reg, &val, 1);
    return val;
}

static void gyroWriteReg(uint8_t reg, uint8_t val)
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(CS_GYRO, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(val);
    digitalWrite(CS_GYRO, HIGH);
    SPI.endTransaction();
}

static void gyroReadBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(CS_GYRO, LOW);
    SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    digitalWrite(CS_GYRO, HIGH);
    SPI.endTransaction();
}

// ================================================================
// ISR — called on every Gyro DRDY (2 kHz)
// Reads gyro every call, accel at 1/10 rate (200 Hz).
// SPI is owned entirely by this ISR, so no locking needed.
// ================================================================

static void IRAM_ATTR gyroISR()
{
    uint8_t g[6];
    static uint8_t accelDiv = 0;

    // --- Gyro read (always) ---
    gyroReadBurst(GYR_REG_DATA, g, 6);

    int16_t gxRaw = (int16_t)((g[1] << 8) | g[0]);
    int16_t gyRaw = (int16_t)((g[3] << 8) | g[2]);
    int16_t gzRaw = (int16_t)((g[5] << 8) | g[4]);

    imuLatest.gx = gxRaw * GYRO_SCALE;
    imuLatest.gy = gyRaw * GYRO_SCALE;
    imuLatest.gz = gzRaw * GYRO_SCALE;

    // --- Accel read every 10 ISR calls → 200 Hz ---
    if (++accelDiv >= 10)
    {
        accelDiv = 0;
        uint8_t a[6];
        accelReadBurst(ACC_REG_DATA, a, 6);

        int16_t axRaw = (int16_t)((a[1] << 8) | a[0]);
        int16_t ayRaw = (int16_t)((a[3] << 8) | a[2]);
        int16_t azRaw = (int16_t)((a[5] << 8) | a[4]);

        imuLatest.ax = axRaw * ACCEL_SCALE;
        imuLatest.ay = ayRaw * ACCEL_SCALE;
        imuLatest.az = azRaw * ACCEL_SCALE;
    }

    imuLatest.timestamp = micros();
}

// ================================================================
// Public API
// ================================================================

bool bmi088CheckIDs()
{
    // First accel read may return garbage; do it twice.
    accelReadReg(ACC_REG_CHIP_ID);
    delayMicroseconds(500);
    uint8_t accID = accelReadReg(ACC_REG_CHIP_ID);

    uint8_t buf;
    gyroReadBurst(GYR_REG_CHIP_ID, &buf, 1);
    uint8_t gyrID = buf;

    Serial.print("[BMI088] ACC chip ID: 0x");
    Serial.print(accID, HEX);
    Serial.print("  (expect 0x1E)   GYR chip ID: 0x");
    Serial.print(gyrID, HEX);
    Serial.println("  (expect 0x0F)");

    return (accID == 0x1E) && (gyrID == 0x0F);
}

void bmi088Init()
{
    // --- SPI bus ---
    SPI.setTX(PIN_MOSI);
    SPI.setRX(PIN_MISO);
    SPI.setSCK(PIN_SCK);
    SPI.begin();

    pinMode(CS_ACCEL, OUTPUT); digitalWrite(CS_ACCEL, HIGH);
    pinMode(CS_GYRO,  OUTPUT); digitalWrite(CS_GYRO,  HIGH);
    pinMode(GYRO_INT_PIN, INPUT);

    delay(10);

    // ---- Accelerometer init ----------------------------------------
    // 1. Soft reset
    accelWriteReg(ACC_REG_SOFTRESET, 0xB6);
    delay(50);   // wait for NVM reload

    // 2. Dummy read to switch from I2C to SPI mode
    accelReadReg(ACC_REG_CHIP_ID);
    delay(5);

    // 3. Wake from suspend
    accelWriteReg(ACC_REG_PWR_CONF, 0x00); // active
    delay(10);

    // 4. Enable accelerometer
    accelWriteReg(ACC_REG_PWR_CTRL, 0x04);
    delay(10);

    // 5. ODR = 1600 Hz, normal BWP
    accelWriteReg(ACC_REG_CONF, ACC_CONF_1600HZ_NORMAL);

    // 6. Range = ±24 g
    accelWriteReg(ACC_REG_RANGE, ACC_RANGE_24G);
    delay(5);

    // ---- Gyroscope init --------------------------------------------
    // 1. Range = ±2000 dps
    gyroWriteReg(GYR_REG_RANGE, GYR_RANGE_2000);

    // 2. ODR = 2000 Hz, filter BW = 532 Hz
    gyroWriteReg(GYR_REG_BANDWIDTH, GYR_BW_2000HZ);

    // 3. Normal power mode
    gyroWriteReg(GYR_REG_LPM1, 0x00);
    delay(5);

    // 4. INT3 → push-pull, active high
    gyroWriteReg(GYR_REG_INT34_CONF, GYR_INT3_ACTIVE_HIGH_PP);

    // 5. Map DRDY → INT3
    gyroWriteReg(GYR_REG_INT34_MAP, GYR_INT3_MAP_DRDY);

    // 6. Enable DRDY interrupt
    gyroWriteReg(GYR_REG_INT_CTRL, 0x80);
    delay(5);

    // ---- Attach interrupt ------------------------------------------
    attachInterrupt(digitalPinToInterrupt(GYRO_INT_PIN), gyroISR, RISING);
}

IMURaw bmi088GetLatest()
{
    // Atomic snapshot: disable interrupt for the copy
    noInterrupts();
    IMURaw copy = imuLatest;
    interrupts();
    return copy;
}
