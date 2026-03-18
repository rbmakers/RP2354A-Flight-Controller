#include "self_test.h"
#include "../config.h"
#include "../imu/bmi088_driver.h"
#include "../altitude/bmp580_driver.h"
#include "../receiver/elrs_receiver.h"
#include <SPI.h>
#include <math.h>

#ifdef USE_BATTERY_MON
  #include "../battery/battery.h"
#endif

// ================================================================
// Result storage
// ================================================================

static SelfTestResult _results[SELF_TEST_COUNT];
static int            _resultCount = 0;
static bool           _allPassed   = true;

// Short fixed-length detail string buffer
static char _detail[SELF_TEST_COUNT][48];

static void recordResult(const char *name, bool passed, const char *detail)
{
    int i = _resultCount;
    if (i >= SELF_TEST_COUNT) return;

    _results[i].name   = name;
    _results[i].passed = passed;

    // Copy detail into stable storage
    strncpy(_detail[i], detail, 47);
    _detail[i][47] = '\0';
    _results[i].detail = _detail[i];

    if (!passed) _allPassed = false;
    _resultCount++;
}

// ================================================================
// Individual tests
// ================================================================

static void testBMI088Accel()
{
    // bmi088CheckIDs() already called in setup; we do a direct SPI register read.
    SPISettings s(1000000, MSBFIRST, SPI_MODE0);
    SPI.beginTransaction(s);
    digitalWrite(CS_ACCEL, LOW);
    SPI.transfer(0x00 | 0x80);   // read chip ID register
    SPI.transfer(0x00);           // dummy byte (accel requires it)
    uint8_t id = SPI.transfer(0x00);
    digitalWrite(CS_ACCEL, HIGH);
    SPI.endTransaction();

    bool ok = (id == 0x1E);
    char buf[32];
    snprintf(buf, sizeof(buf), "chip_id=0x%02X (expect 0x1E)", id);
    recordResult("BMI088 Accel", ok, buf);
}

static void testBMI088Gyro()
{
    SPISettings s(1000000, MSBFIRST, SPI_MODE0);
    SPI.beginTransaction(s);
    digitalWrite(CS_GYRO, LOW);
    SPI.transfer(0x00 | 0x80);
    uint8_t id = SPI.transfer(0x00);
    digitalWrite(CS_GYRO, HIGH);
    SPI.endTransaction();

    bool ok = (id == 0x0F);
    char buf[32];
    snprintf(buf, sizeof(buf), "chip_id=0x%02X (expect 0x0F)", id);
    recordResult("BMI088 Gyro", ok, buf);
}

static void testBMP580()
{
    // Read BMP5 chip ID over I2C
    Wire.beginTransmission(BMP580_I2C_ADDR);
    Wire.write(0x01);   // REG_CHIP_ID
    bool ackOk = (Wire.endTransmission(false) == 0);

    uint8_t id = 0;
    if (ackOk)
    {
        Wire.requestFrom((uint8_t)BMP580_I2C_ADDR, (uint8_t)1);
        if (Wire.available()) id = Wire.read();
    }

    bool ok = ackOk && (id == 0x50);
    char buf[40];
    snprintf(buf, sizeof(buf), "chip_id=0x%02X %s",
             id, ackOk ? "" : "(no I2C ack)");
    recordResult("BMP580 Baro", ok, buf);
}

static void testFlash()
{
#ifdef USE_BLACKBOX
    // JEDEC ID: W25Q128JV = 0xEF4018
    SPISettings s(8000000, MSBFIRST, SPI_MODE0);
    SPI.beginTransaction(s);
    digitalWrite(BB_FLASH_CS_PIN, LOW);
    SPI.transfer(0x9F);
    uint32_t jedec = ((uint32_t)SPI.transfer(0) << 16) |
                     ((uint32_t)SPI.transfer(0) <<  8) |
                      (uint32_t)SPI.transfer(0);
    digitalWrite(BB_FLASH_CS_PIN, HIGH);
    SPI.endTransaction();

    bool ok = ((jedec & 0xFFFF00) == 0xEF4000);
    char buf[32];
    snprintf(buf, sizeof(buf), "JEDEC=0x%06lX (expect 0xEF4018)", jedec);
    recordResult("W25Q128 Flash", ok, buf);
#else
    recordResult("W25Q128 Flash", true, "SKIPPED (USE_BLACKBOX not set)");
#endif
}

static void testBattery()
{
#ifdef USE_BATTERY_MON
    uint16_t v = batGetVoltage_mV();
    // Reasonable range for any LiPo 1S–6S: 3.0 V – 25.2 V
    bool ok = (v >= 3000 && v <= 25200);
    char buf[32];
    snprintf(buf, sizeof(buf), "%.2f V %s",
             v / 1000.0f, ok ? "" : "(OUT OF RANGE)");
    recordResult("Battery Voltage", ok, buf);
#else
    recordResult("Battery Voltage", true, "SKIPPED (USE_BATTERY_MON not set)");
#endif
}

static void testCRSFLink()
{
    // Wait up to 2 s for first CRSF frame
    uint32_t tEnd = millis() + 2000;
    while (!crsfIsLinkUp() && millis() < tEnd)
        delay(10);

    bool ok = crsfIsLinkUp();
    recordResult("CRSF Link", ok,
                 ok ? "link up" : "no ELRS signal (check Tx, binding)");
}

static void testGyroStationary()
{
    // Collect 200 ms of gyro samples and check RMS < 0.05 rad/s
    // (board should be completely still during this test)
    float rmsSum = 0;
    int   n = 0;
    uint32_t tEnd = millis() + 200;
    while (millis() < tEnd)
    {
        IMURaw imu = bmi088GetLatest();
        rmsSum += imu.gx * imu.gx + imu.gy * imu.gy + imu.gz * imu.gz;
        n++;
        delay(1);
    }
    float rms = (n > 0) ? sqrtf(rmsSum / n) : 9999.0f;
    bool ok = (rms < 0.05f);  // 0.05 rad/s ≈ 2.9 °/s
    char buf[40];
    snprintf(buf, sizeof(buf), "gyro RMS=%.4f rad/s (limit 0.05)", rms);
    recordResult("Gyro Stationary", ok, buf);
}

static void testAccelMagnitude()
{
    // Read 50 samples, check |accel| = 0.90–1.10 g (board roughly level)
    float magSum = 0;
    for (int i = 0; i < 50; i++)
    {
        IMURaw imu = bmi088GetLatest();
        float mag = sqrtf(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
        magSum += mag;
        delay(2);
    }
    float avg = magSum / 50.0f;
    bool ok = (avg > 0.90f && avg < 1.10f);
    char buf[40];
    snprintf(buf, sizeof(buf), "|a|=%.3f g (expect ~1.000)", avg);
    recordResult("Accel Magnitude", ok, buf);
}

// ================================================================
// Public API
// ================================================================

void selfTestRun()
{
    _resultCount = 0;
    _allPassed   = true;

    Serial.println("\n[SELF-TEST] Running hardware verification...");
    Serial.println("─────────────────────────────────────────────");

    testBMI088Accel();
    testBMI088Gyro();
    testBMP580();
    testFlash();
    testBattery();
    testCRSFLink();
    testGyroStationary();
    testAccelMagnitude();

    selfTestPrintReport();
}

void selfTestPrintReport()
{
    Serial.println("\n[SELF-TEST] Results:");
    Serial.println("─────────────────────────────────────────────");

    for (int i = 0; i < _resultCount; i++)
    {
        Serial.print(_results[i].passed ? "  [PASS] " : "  [FAIL] ");
        Serial.print(_results[i].name);
        Serial.print(": ");
        Serial.println(_results[i].detail);
    }

    Serial.println("─────────────────────────────────────────────");
    if (_allPassed)
        Serial.println("  ALL TESTS PASSED — safe to arm\n");
    else
        Serial.println("  SOME TESTS FAILED — investigate before flying!\n");
}

bool selfTestAllPassed() { return _allPassed; }

const SelfTestResult* selfTestGetResults() { return _results; }
