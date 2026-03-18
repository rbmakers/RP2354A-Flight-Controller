#include "imu_calibration.h"

// Collect 2000 samples @ 1 kHz = 2 seconds of data.
#define CALIB_SAMPLES  2000

static IMUCalibration calib   = {{0,0,0},{0,0,0},false};
static int            count   = 0;
static bool           running = false;
static double         gSum[3] = {0,0,0};
static double         aSum[3] = {0,0,0};

void imuCalibStart()
{
    count   = 0;
    running = true;
    calib.valid = false;
    for (int i = 0; i < 3; i++) { gSum[i] = 0.0; aSum[i] = 0.0; }
}

bool imuCalibRunning() { return running; }
bool imuCalibDone()    { return calib.valid; }

void imuCalibUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az)
{
    if (!running) return;

    gSum[0] += gx; gSum[1] += gy; gSum[2] += gz;
    aSum[0] += ax; aSum[1] += ay; aSum[2] += az;
    count++;

    if (count >= CALIB_SAMPLES)
    {
        calib.gyroBias[0] = (float)(gSum[0] / CALIB_SAMPLES);
        calib.gyroBias[1] = (float)(gSum[1] / CALIB_SAMPLES);
        calib.gyroBias[2] = (float)(gSum[2] / CALIB_SAMPLES);

        calib.accelBias[0] = (float)(aSum[0] / CALIB_SAMPLES);
        calib.accelBias[1] = (float)(aSum[1] / CALIB_SAMPLES);
        // Z axis: subtract expected 1 g (board flat, Z points up)
        calib.accelBias[2] = (float)(aSum[2] / CALIB_SAMPLES) - 1.0f;

        running    = false;
        calib.valid = true;
    }
}

IMUCalibration imuCalibGet()  { return calib; }

void imuCalibReset()
{
    calib.valid = false;
    running = false;
    count   = 0;
}
