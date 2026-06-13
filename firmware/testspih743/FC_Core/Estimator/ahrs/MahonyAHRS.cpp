#include "MahonyAHRS.h"

// Math constants
#define Kp 2.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f		// integral gain governs rate of convergence of gyroscope biases

static float invSqrt(float x) {
    // Fast inverse square root
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

MahonyAHRS::MahonyAHRS() {
    _twoKp = 2.0f * Kp;
    _twoKi = 2.0f * Ki;
    _q0 = 1.0f;
    _q1 = 0.0f;
    _q2 = 0.0f;
    _q3 = 0.0f;
    _integralFBx = 0.0f;
    _integralFBy = 0.0f;
    _integralFBz = 0.0f;
    _roll = 0.0f;
    _pitch = 0.0f;
    _yaw = 0.0f;
}

void MahonyAHRS::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        updateIMU(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = _q0 * _q0;
        q0q1 = _q0 * _q1;
        q0q2 = _q0 * _q2;
        q0q3 = _q0 * _q3;
        q1q1 = _q1 * _q1;
        q1q2 = _q1 * _q2;
        q1q3 = _q1 * _q3;
        q2q2 = _q2 * _q2;
        q2q3 = _q2 * _q3;
        q3q3 = _q3 * _q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if(_twoKi > 0.0f) {
            _integralFBx += _twoKi * halfex * dt;
            _integralFBy += _twoKi * halfey * dt;
            _integralFBz += _twoKi * halfez * dt;
            gx += _integralFBx;	// apply integral feedback
            gy += _integralFBy;
            gz += _integralFBz;
        }
        else {
            _integralFBx = 0.0f;	// prevent integral windup
            _integralFBy = 0.0f;
            _integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += _twoKp * halfex;
        gy += _twoKp * halfey;
        gz += _twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = _q0;
    qb = _q1;
    qc = _q2;
    _q0 += (-qb * gx - qc * gy - _q3 * gz);
    _q1 += (qa * gx + qc * gz - _q3 * gy);
    _q2 += (qa * gy - qb * gz + _q3 * gx);
    _q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 *= recipNorm;
    _q1 *= recipNorm;
    _q2 *= recipNorm;
    _q3 *= recipNorm;
    
    computeAngles();
}

void MahonyAHRS::updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = _q1 * _q3 - _q0 * _q2;
        halfvy = _q0 * _q1 + _q2 * _q3;
        halfvz = _q0 * _q0 - 0.5f + _q3 * _q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if(_twoKi > 0.0f) {
            _integralFBx += _twoKi * halfex * dt;
            _integralFBy += _twoKi * halfey * dt;
            _integralFBz += _twoKi * halfez * dt;
            gx += _integralFBx;
            gy += _integralFBy;
            gz += _integralFBz;
        } else {
            _integralFBx = 0.0f;
            _integralFBy = 0.0f;
            _integralFBz = 0.0f;
        }

        gx += _twoKp * halfex;
        gy += _twoKp * halfey;
        gz += _twoKp * halfez;
    }

    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = _q0;
    qb = _q1;
    qc = _q2;
    _q0 += (-qb * gx - qc * gy - _q3 * gz);
    _q1 += (qa * gx + qc * gz - _q3 * gy);
    _q2 += (qa * gy - qb * gz + _q3 * gx);
    _q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 *= recipNorm;
    _q1 *= recipNorm;
    _q2 *= recipNorm;
    _q3 *= recipNorm;

    computeAngles();
}

void MahonyAHRS::computeAngles() {
    _roll  = atan2f(2.0f * (_q0 * _q1 + _q2 * _q3), 1.0f - 2.0f * (_q1 * _q1 + _q2 * _q2));
    _pitch = asinf(2.0f * (_q0 * _q2 - _q3 * _q1));
    _yaw   = atan2f(2.0f * (_q0 * _q3 + _q1 * _q2), 1.0f - 2.0f * (_q2 * _q2 + _q3 * _q3));
}
