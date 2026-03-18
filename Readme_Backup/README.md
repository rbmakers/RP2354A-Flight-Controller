# RP2354A Flight Controller Firmware  v4
## Complete Betaflight-Class Stack — RB-RP2354A

---

## Hardware Pinout

| Signal         | GPIO | Notes                                    |
|----------------|------|------------------------------------------|
| SPI MOSI       |  19  | BMI088 + W25Q128 (shared SPI bus)        |
| SPI MISO       |  20  |                                          |
| SPI SCK        |  22  |                                          |
| Accel CS       |  21  | BMI088 accelerometer                     |
| Gyro  CS       |  23  | BMI088 gyroscope                         |
| Gyro DRDY      |  24  | Rising-edge ISR, 2 kHz                   |
| I2C SDA        |  16  | BMP580 barometer                         |
| I2C SCL        |  17  |                                          |
| Brushed M1     |  29  | MOSFET PWM 32 kHz, 10-bit                |
| Brushed M2     |  11  |                                          |
| Brushed M3     |  18  |                                          |
| Brushed M4     |  25  |                                          |
| BLDC ESC1      |   1  | PIO DShot600                             |
| BLDC ESC2      |   4  |                                          |
| BLDC ESC3      |  10  |                                          |
| BLDC ESC4      |  14  |                                          |
| ELRS TX        |  12  | UART0, 420 000 baud CRSF                 |
| ELRS RX        |  13  |                                          |
| Battery Vsense |  26  | ADC0, resistor divider 100 kΩ / 10 kΩ   |
| Current sense  |  27  | ADC1, ACS712-30A (optional)              |
| Flash CS       |  28  | W25Q128 SPI flash (blackbox)             |
| BLHeli Telem   |  26* | UART2 RX — *only when not using bidir DShot |

---

## Complete Module Map  (v4 — 63 files)

```
RP2354A_FC/
├── RP2354A_FC.ino           Master sketch, dual-core, 508 lines
├── config.h                 All pins, rates, gains, feature flags
│
├── imu/
│   ├── bmi088_driver        SPI DRDY ISR @ 2 kHz, correct BMI088 init
│   │                        (dummy byte, full register sequence)
│   ├── imu_calibration      Blocking 2-second gyro+accel bias cal
│   └── madgwick_ahrs        Corrected gradient descent AHRS
│
├── altitude/
│   ├── bmp580_driver        I2C0, correct BMP5 register map + IIR config
│   ├── altitude_estimator   Baro+accel complementary fusion
│   └── altitude_hold        PID altitude hold with integral anti-windup
│
├── filters/
│   ├── biquad_filter        LPF + notch, Direct Form II transposed
│   └── rpm_notch            Per-motor notch bank (4 motors × 2 harmonics)
│
├── motors/
│   ├── brushed_pwm          32 kHz PWM, 10-bit resolution
│   ├── dshot                Bit-bang DShot600 (reference / fallback)
│   ├── dshot_pio            PIO DShot600: all 4 ESCs simultaneous,
│   │                        zero CPU blocking; GCR decode for bidir eRPM
│   ├── esc_telemetry        Bidir DShot eRPM or BLHeli serial telemetry;
│   │                        feeds rpm_notch automatically
│   └── esc_calibration      PWM range calibration state machine
│                            (high→low sequence, timed, safety interlocked)
│
├── control/
│   ├── pid_controller       Per-axis PID + Ziegler-Nichols auto-tune
│   └── flight_modes         ARM/DISARM (stick+switch), Acro, Angle,
│                            outer P-loop angle → rate setpoint
│
├── receiver/
│   └── elrs_receiver        Full 16-channel CRSF parser, CRC8 DVB-S2,
│                            byte-level state machine, atomic double-buffer
│
├── receiver_cal/
│   └── rc_calibration       Per-channel min/mid/max sweep, two-sided
│                            normalisation, expo curve, deadband
│
├── safety/
│   └── failsafe             4-level failsafe (NONE/WARN/LAND/KILL);
│                            motor test mode with 5 s auto-timeout
│
├── battery/
│   └── battery              ADC voltage+current, 12-bit 8-sample avg,
│                            LPF, cell-count auto-detection
│
├── blackbox/
│   ├── blackbox             W25Q128 SPI flash, 32-byte packed frames,
│   │                        page-write buffered, session index
│   ├── bb_dump              Chunked binary USB dump with XOR checksum
│   ├── decode.py            Python 3 decoder → CSV per session
│   └── dump_flash.py        Host-side capture tool (pyserial)
│
├── msp/
│   └── msp_protocol         MSP v1 subset for Betaflight Configurator:
│                            IDENT, STATUS, RAW_IMU, RC, MOTOR,
│                            ATTITUDE, ALTITUDE, ANALOG, SET_PID
│
├── system/
│   └── self_test            8-point hardware verification at boot
│                            (chip IDs, battery range, CRSF link,
│                             gyro noise, accel magnitude)
│
├── telemetry/
│   └── usb_telemetry        22-field bidirectional CSV at 100 Hz;
│                            full command parser (23 commands)
│
├── fft/
│   ├── fft_analyzer         Original O(N²) DFT (reference)
│   └── fft_cmsis            CMSIS-DSP arm_rfft_fast_f32 (O(N log N));
│                            Goertzel fallback if library unavailable
│
└── gui/
    └── mini_bf_config.pde   Processing 4 GUI (683 lines), 4 tabs:
                             • Flight   — 3D cube, motors, battery, FFT
                             • PID      — Roll/Pitch/Yaw sliders + tips
                             • RC Cal   — Expo/deadband per channel
                             • Self-Test — Live hardware verification
```

---

## Feature Flags  (config.h)

| Define              | Effect                                      | CPU cost  |
|---------------------|---------------------------------------------|-----------|
| `MOTOR_TYPE_BRUSHED`| Brushed PWM driver (default)                | —         |
| `MOTOR_TYPE_BLDC`   | PIO DShot600 driver                         | ~0 %      |
| `USE_RPM_FILTER`    | Per-motor notch from ESC telemetry          | ~3 %      |
| `USE_FFT_ANALYSIS`  | CMSIS-DSP vibration spectrum (Core 1)       | ~0.2 %    |
| `USE_BLACKBOX`      | W25Q128 flash logger @ 1 kHz               | ~0.5 %    |
| `USE_BATTERY_MON`   | ADC voltage/current + failsafe              | ~0.1 %    |
| `DSHOT_BIDIR`       | Bidir DShot eRPM (in dshot_pio.h)          | ~0 %      |

---

## Build Instructions

### Arduino Firmware

1. **IDE**: Arduino 2.3.8
2. **Board package**: `earlephilhower/arduino-pico` ≥ 3.x
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
3. Board: **Raspberry Pi Pico 2**  (RP2354A = A-variant of RP2350)
4. CPU speed: **150 MHz**
5. Edit `config.h` — select `MOTOR_TYPE_BRUSHED` or `MOTOR_TYPE_BLDC`
6. Enable optional modules as needed
7. Compile and upload

### Processing GUI

1. Download [Processing 4](https://processing.org/download)
2. Open `gui/mini_bf_config.pde`
3. If auto-connect picks the wrong port, change `Serial.list()[0]`
4. Run — the GUI opens with 4 tabs

### Betaflight Configurator (MSP mode)

The firmware also speaks MSP v1, so the official Betaflight Configurator
can connect to it for real-time graphs, motor test, and PID editing.
Use the same USB serial port. The MSP handler is active alongside the
CSV telemetry — the '$M' preamble identifies MSP frames automatically.

---

## USB Command Reference  (23 commands)

| Command                     | Effect                                    |
|-----------------------------|-------------------------------------------|
| `ARM`                       | Arm motors                                |
| `DISARM`                    | Disarm motors                             |
| `MODE_ACRO`                 | Switch to Acro (rate) mode                |
| `MODE_ANGLE`                | Switch to Angle (self-level) mode         |
| `CALIB`                     | Start 2 s IMU calibration (disarmed only) |
| `SET_KP:<val>`              | Set roll+pitch Kp                         |
| `SET_KI:<val>`              | Set roll+pitch Ki                         |
| `SET_KD:<val>`              | Set roll+pitch Kd                         |
| `SET_YAW_KP/KI/KD:<val>`   | Set yaw gains independently               |
| `RC_CAL_START`              | Begin RC sweep calibration                |
| `RC_CAL_END`                | Finish RC calibration                     |
| `RC_CAL_PRINT`              | Print calibration table to Serial         |
| `SET_EXPO:<ch>:<val>`       | Set expo curve (0.0–1.0) for channel      |
| `SET_DEADBAND:<ch>:<val>`   | Set deadband (0.0–0.20) for channel       |
| `ESC_CAL_START`             | Start ESC PWM range calibration           |
| `ESC_CAL_ABORT`             | Abort ESC calibration                     |
| `MOTOR_TEST:<m>:<pct>`      | Spin motor m (0–3) at pct % (max 15 %)   |
| `MOTOR_TEST_ALL`            | Spin all motors at 10 %                   |
| `MOTOR_TEST_STOP`           | Stop motor test                           |
| `AUTOTUNE_START`            | (Acknowledged; manual PID recommended)    |
| `SELF_TEST`                 | Run hardware self-test, stream results    |
| `DUMP_BB`                   | Stream flash contents over serial         |
| `ERASE_BB`                  | Chip-erase W25Q128 (disarmed only)        |

---

## Telemetry Packet Format  (FC → GUI, 100 Hz)

```
roll×10, pitch×10, yaw×10,
m1, m2, m3, m4,        (0–1000)
kP, kI, kD,
alt_cm, armed, mode,
volt_mV, curr_cA, bat_pct,
r1rpm, r2rpm, r3rpm, r4rpm,
loop_us, fs_level
```

---

## Self-Test Results Format  (FC → GUI)

```
SELF_TEST_BEGIN
ST:PASS:BMI088 Accel:chip_id=0x1E (expect 0x1E)
ST:PASS:BMI088 Gyro:chip_id=0x0F (expect 0x0F)
ST:PASS:BMP580 Baro:chip_id=0x50
ST:PASS:W25Q128 Flash:JEDEC=0xEF4018
ST:PASS:Battery Voltage:16.75 V
ST:PASS:CRSF Link:link up
ST:PASS:Gyro Stationary:gyro RMS=0.0021 rad/s (limit 0.05)
ST:PASS:Accel Magnitude:|a|=1.002 g (expect ~1.000)
SELF_TEST_END
```

---

## Boot Sequence

```
[RP2354A-FC v4] =====================
[FC] Motor: BLDC DShot600 PIO
[FC] IMU calibration — keep board still for 2 s...
[FC] Calibration done
[SELF-TEST] Running hardware verification...
  [PASS] BMI088 Accel: chip_id=0x1E ...
  [PASS] BMI088 Gyro:  chip_id=0x0F ...
  [PASS] BMP580 Baro:  chip_id=0x50
  [PASS] W25Q128 Flash: JEDEC=0xEF4018
  [PASS] Battery Voltage: 16.75 V
  [PASS] CRSF Link: link up
  [PASS] Gyro Stationary: RMS=0.0021 rad/s
  [PASS] Accel Magnitude: |a|=1.002 g
  ALL TESTS PASSED — safe to arm
[FC] Flight loop running
[FC] =====================
```

---

## ELRS Channel Map

| CH | Function     | Range                              |
|----|--------------|------------------------------------|
|  1 | Roll         | ±1.0 (after calibration + expo)    |
|  2 | Pitch        | ±1.0                               |
|  3 | Throttle     | 0.0–1.0                            |
|  4 | Yaw          | ±1.0                               |
|  5 | Arm switch   | >1700 µs = ARM, <1300 µs = DISARM  |
|  6 | Mode switch  | >1500 µs = ANGLE, ≤1500 = ACRO     |
|  7 | Alt hold     | >1500 µs = altitude hold ON        |

---

## Blackbox Workflow

```bash
# 1. Dump flash (FC disarmed, USB connected)
python3 blackbox/dump_flash.py /dev/ttyACM0 flash.bin

# 2. Decode all sessions to CSV
python3 blackbox/decode.py flash.bin

# 3. Analyse (example: plot roll angle in Python)
import pandas as pd, matplotlib.pyplot as plt
df = pd.read_csv("flash_session0.csv")
plt.plot(df.timestamp_us / 1e6, df.roll_deg); plt.show()

# 4. Erase before next flight session
# (GUI → ERASE BB, or send "ERASE_BB\n" over serial)
```

---

## CPU Budget  (all features, 150 MHz Cortex-M33)

| Task                   | Rate     | Max cost   |
|------------------------|----------|------------|
| BMI088 DRDY ISR        | 2 kHz    | ~15 µs     |
| Gyro filters (8 biquad)| 2 kHz    | ~26 µs     |
| Madgwick AHRS          | 1 kHz    | ~20 µs     |
| PID (3 axes)           | 1 kHz    | ~12 µs     |
| Altitude fusion + hold | 1 kHz    | ~8 µs      |
| DShot PIO write        | 1 kHz    | ~2 µs      |
| Blackbox page write    | ~125 Hz  | ~700 µs    |
| Telemetry TX           | 100 Hz   | ~15 µs     |
| CRSF parser  (Core 1)  | async    | Core 1     |
| CMSIS-DSP FFT (Core 1) | ~8 Hz    | Core 1     |
| **Total Core 0**       |          | **< 15 %** |
| **Total Core 1**       |          | **< 5 %**  |

---

## Hardware Verification Checklist  (before first flight)

- [ ] All chip IDs pass self-test
- [ ] Battery voltage reads correctly (check divider resistors)
- [ ] CRSF link up, all 7 channels moving in correct direction
- [ ] RC calibration complete (all sticks swept)
- [ ] Gyro calibration complete (board held still for 2 s at boot)
- [ ] Motor directions correct (Quad-X: FR/RL CW, FL/RR CCW)
- [ ] Motor test at 10 % — correct motor spins for each M1–M4 button
- [ ] Angle mode: roll/pitch cube follows board rotation correctly
- [ ] Altitude hold: barometer reads reasonable value at bench
- [ ] Blackbox: test write/read with dump_flash.py before flight

---

## License

MIT — free for educational and personal use.

Reference: Yang, C.-S. et al.,
*Design and Development of the RB-RP2354 UAV Flight Controller*,
Department of Electrical Engineering, Chung Yuan Christian University, 2025.
