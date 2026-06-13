#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

// 1. Định nghĩa cấu trúc dữ liệu (Giống PX4 msg)
struct sensor_imu_s {
    uint64_t timestamp_us;     // Thời gian lấy mẫu (Microseconds)
    float accel_m_s2[3];       // Gia tốc X, Y, Z (m/s^2)
    float gyro_rad_s[3];       // Vận tốc góc X, Y, Z (rad/s)
    float temperature_c;       // Nhiệt độ (Độ C)
};

// 2. Khai báo Topic dưới dạng biến Toàn cục (Global)
// Từ khóa extern báo cho compiler biết biến này sẽ nằm ở file topics.cpp
extern uORB::Topic<sensor_imu_s> orb_sensor_imu;

// Để thêm một topic mới như GPS, bạn chỉ cần tạo struct sensor_gps_s và khai báo extern ở đây.
