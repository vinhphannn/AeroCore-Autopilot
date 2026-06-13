#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

// Dữ liệu IMU sau khi đã được xoay chuẩn (Board Rotation) và bù sai số (Calibration)
// Tương đương `vehicle_imu` trong PX4 (Hệ quy chiếu NED của máy bay)
struct vehicle_imu_s {
    uint64_t timestamp_us;     // Thời gian lấy mẫu (Microseconds)
    float accel_m_s2[3];       // Gia tốc X, Y, Z (m/s^2) theo Body Frame
    float gyro_rad_s[3];       // Vận tốc góc X, Y, Z (rad/s) theo Body Frame
};

extern uORB::Topic<vehicle_imu_s> orb_vehicle_imu;
