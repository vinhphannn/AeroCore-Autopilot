#include "AttitudeEstimator.h"
#include "stm32h7xx_hal.h"

AttitudeEstimator g_estimator;

AttitudeEstimator::AttitudeEstimator() : 
    _imu_sub(orb_vehicle_imu, 0),
    _mag_sub(orb_sensor_mag, 0),
    _att_pub(orb_vehicle_attitude),
    _last_update_us(0)
{
    _prev_gyro[0] = _prev_gyro[1] = _prev_gyro[2] = 0.0f;
    // AlphaFilter cho angular_accel: cutoff 30Hz (đúng PX4 vehicle_angular_velocity module)
    for (int i = 0; i < 3; i++) {
        _accel_filter[i].setCutoffFreq(100.0f, 30.0f);
    }
}

uint64_t AttitudeEstimator::get_time_us() {
    return (uint64_t)HAL_GetTick() * 1000;
}

void AttitudeEstimator::init() {
    _last_update_us = get_time_us();
}

void AttitudeEstimator::update() {
    vehicle_imu_s imu;
    
    if (_imu_sub.update(imu)) {
        uint64_t now = get_time_us();
        float dt = (now - _last_update_us) / 1000000.0f;
        if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;
        _last_update_us = now;

        // Lấy data Mag mới nhất
        sensor_mag_s mag;
        _mag_sub.copy(mag);

        // Cập nhật Mahony AHRS
        _ahrs.update(imu.gyro_rad_s[0], imu.gyro_rad_s[1], imu.gyro_rad_s[2],
                     imu.accel_m_s2[0], imu.accel_m_s2[1], imu.accel_m_s2[2],
                     mag.mag_x_ut, mag.mag_y_ut, mag.mag_z_ut, dt);

        vehicle_attitude_s att = {0};
        att.timestamp_us = now;
        _ahrs.getQuaternion(att.q);
        att.roll_rad  = _ahrs.getRoll();
        att.pitch_rad = _ahrs.getPitch();
        att.yaw_rad   = _ahrs.getYaw();

        att.rollspeed_rad_s  = imu.gyro_rad_s[0];
        att.pitchspeed_rad_s = imu.gyro_rad_s[1];
        att.yawspeed_rad_s   = imu.gyro_rad_s[2];

        // Gia tốc góc đã lọc PT1 (AlphaFilter - đúng PX4 vehicle_angular_velocity module)
        // Được dùng làm D-term input cho RateControl, thay vì d(error)/dt
        for (int i = 0; i < 3; i++) {
            float raw_accel = (imu.gyro_rad_s[i] - _prev_gyro[i]) / dt;
            att.angular_accel_rad_s2[i] = _accel_filter[i].update(raw_accel);
            _prev_gyro[i] = imu.gyro_rad_s[i];
        }

        _att_pub.publish(att);
    }
}
