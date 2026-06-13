#include "SensorHub.h"
#include "../Parameters/param.h"

SensorHub g_sensor_hub;

void SensorHub::init() {
    // Đọc tham số Board Rotation từ Flash
    int32_t rot_val = 0;
    param_get(PARAM_SENS_BOARD_ROT, &rot_val);
    _board_rotation = (BoardRotation)rot_val;

    // Khởi tạo bộ lọc Gyro (LPF bậc 2 + Notch nếu cần)
    // 100Hz = tần số lấy mẫu của task SensorsTask
    // 30Hz  = tần số cắt cho Gyro (khớp chuẩn PX4 mặc định IMU_GYRO_CUTOFF)
    // 0.0f  = tắt Notch, sau này người dùng có thể bật qua IMU_GYRO_NF0_FRQ
    _gyro_filter.init(100.0f, 30.0f, 0.0f, 20.0f);

    // Khởi tạo bộ lọc Accel PT1 (AlphaFilter)
    // 100Hz lấy mẫu, 30Hz cutoff
    for (int i = 0; i < 3; i++) {
        _accel_filter[i].setCutoffFreq(100.0f, 30.0f);
    }
}

void SensorHub::rotate_3d(float vec[3], BoardRotation rot) {
    float x = vec[0], y = vec[1], z = vec[2];
    switch (rot) {
        case ROTATION_NONE:                                                     break;
        case ROTATION_YAW_90:         vec[0]= y;  vec[1]=-x;                   break;
        case ROTATION_YAW_180:        vec[0]=-x;  vec[1]=-y;                   break;
        case ROTATION_YAW_270:        vec[0]=-y;  vec[1]= x;                   break;
        case ROTATION_ROLL_180:                   vec[1]=-y; vec[2]=-z;        break;
        case ROTATION_ROLL_180_YAW_90:vec[0]=-y;  vec[1]=-x; vec[2]=-z;       break;
        case ROTATION_PITCH_180:      vec[0]=-x;             vec[2]=-z;        break;
        case ROTATION_ROLL_180_YAW_270:vec[0]=y;  vec[1]= x; vec[2]=-z;       break;
    }
    (void)x; (void)y; (void)z; // suppress unused warning
}

void SensorHub::update() {
    sensor_imu_s raw_imu;

    if (_imu_sub.update(raw_imu)) {
        // 1. Lọc Gyro (LowPassFilter2p bậc 2 + Notch) - đúng như PX4 VehicleIMU.cpp
        float gyro_filtered[3];
        _gyro_filter.apply(raw_imu.gyro_rad_s, gyro_filtered);

        // 2. Lọc Accel (AlphaFilter PT1) - nhẹ hơn vì Accel dùng ở tần số thấp hơn
        float accel_filtered[3];
        for (int i = 0; i < 3; i++) {
            accel_filtered[i] = _accel_filter[i].update(raw_imu.accel_m_s2[i]);
        }

        // 3. Xoay theo SENS_BOARD_ROT (Board Frame -> Body Frame)
        rotate_3d(gyro_filtered,  _board_rotation);
        rotate_3d(accel_filtered, _board_rotation);

        // 4. Publish vehicle_imu (dữ liệu đã lọc và xoay chuẩn Body Frame)
        vehicle_imu_s calibrated_imu;
        calibrated_imu.timestamp_us = raw_imu.timestamp_us;
        for (int i = 0; i < 3; i++) {
            calibrated_imu.accel_m_s2[i] = accel_filtered[i];
            calibrated_imu.gyro_rad_s[i]  = gyro_filtered[i];
        }
        _vehicle_imu_pub.publish(calibrated_imu);
    }
}
