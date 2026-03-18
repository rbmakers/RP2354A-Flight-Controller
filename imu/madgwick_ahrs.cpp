#include "madgwick_ahrs.h"
#include <math.h>

static float _beta;
static float _dt;
static Quaternion _q = {1.0f, 0.0f, 0.0f, 0.0f};

void madgwickInit(float sampleRateHz, float beta)
{
    _beta = beta;
    _dt   = 1.0f / sampleRateHz;
    _q.q0 = 1.0f; _q.q1 = 0.0f; _q.q2 = 0.0f; _q.q3 = 0.0f;
}

void madgwickUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az)
{
    float q1 = _q.q0, q2 = _q.q1, q3 = _q.q2, q4 = _q.q3;

    // Normalize accelerometer
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;
    ax /= norm; ay /= norm; az /= norm;

    // Auxiliary variables
    float _2q1 = 2.0f*q1, _2q2 = 2.0f*q2;
    float _2q3 = 2.0f*q3, _2q4 = 2.0f*q4;
    float _4q1 = 4.0f*q1, _4q2 = 4.0f*q2, _4q3 = 4.0f*q3;
    float _8q2 = 8.0f*q2, _8q3 = 8.0f*q3;
    float q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3, q4q4 = q4*q4;

    // Gradient descent algorithm: objective function & Jacobian
    float f1 = _2q2*q4 - _2q1*q3 - ax;
    float f2 = _2q1*q2 + _2q3*q4 - ay;
    float f3 = 1.0f - _4q2*q2 - _4q3*q3 - az;   // note: 1 - 4q2² - 4q3²

    float J_11or24 = _2q3;
    float J_12or23 = _2q4;
    float J_13or22 = _2q1;
    float J_14or21 = _2q2;
    float J_32     = _4q2;        // was 2*J_14or21
    float J_33     = _4q3;        // was 2*J_11or24

    float s1 = -J_14or21*f2 + J_11or24*f1;
    float s2 =  J_12or23*f2 + J_13or22*f1 - J_32*f3;
    float s3 = -J_13or22*f2 + J_12or23*f1 - J_33*f3;   // note sign on J_13or22
    float s4 = -J_11or24*f2 - J_14or21*f1;

    // Normalize step
    norm = sqrtf(s1*s1 + s2*s2 + s3*s3 + s4*s4);
    if (norm < 1e-10f) norm = 1.0f;
    s1 /= norm; s2 /= norm; s3 /= norm; s4 /= norm;

    // Rate of change of quaternion from gyroscope
    float qDot1 = 0.5f*(-q2*gx - q3*gy - q4*gz) - _beta*s1;
    float qDot2 = 0.5f*( q1*gx + q3*gz - q4*gy) - _beta*s2;
    float qDot3 = 0.5f*( q1*gy - q2*gz + q4*gx) - _beta*s3;
    float qDot4 = 0.5f*( q1*gz + q2*gy - q3*gx) - _beta*s4;

    // Integrate
    q1 += qDot1 * _dt;
    q2 += qDot2 * _dt;
    q3 += qDot3 * _dt;
    q4 += qDot4 * _dt;

    // Normalise quaternion
    norm = sqrtf(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    if (norm < 1e-10f) { _q = {1,0,0,0}; return; }
    _q.q0 = q1/norm; _q.q1 = q2/norm;
    _q.q2 = q3/norm; _q.q3 = q4/norm;
}

Quaternion madgwickGetQ() { return _q; }

void madgwickGetEuler(float *rollDeg, float *pitchDeg, float *yawDeg)
{
    float q0 = _q.q0, q1 = _q.q1, q2 = _q.q2, q3 = _q.q3;
    *rollDeg  = atan2f(2.0f*(q0*q1 + q2*q3),
                       1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
    *pitchDeg = asinf (2.0f*(q0*q2 - q3*q1))         * RAD_TO_DEG;
    *yawDeg   = atan2f(2.0f*(q0*q3 + q1*q2),
                       1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
}
