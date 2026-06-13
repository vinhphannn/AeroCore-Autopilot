#pragma once

#include <stdint.h>
#include "../uORB_lite.h"

/**
 * @brief Bản tin chứa dữ liệu góc quay không gian (Attitude)
 * Giống với cấu trúc vehicle_attitude của PX4
 */
struct vehicle_attitude_s {
    uint64_t timestamp_us;
    
    // Quaternion thể hiện góc xoay: [w, x, y, z] (chuẩn Hamilton)
    float q[4]; 
    
    // Tùy chọn: Góc Euler (Radian) để dễ theo dõi và debug
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    
    // Tốc độ góc (Radian/s) - từ Gyro đã lọc LPF2p
    float rollspeed_rad_s;
    float pitchspeed_rad_s;
    float yawspeed_rad_s;

    // Gia tốc góc (Radian/s^2) - đạo hàm của angular velocity đã lọc
    // PX4 dùng field này làm D-term trong RateControl (tránh khuếch đại nhiễu)
    float angular_accel_rad_s2[3];
};

extern uORB::Topic<vehicle_attitude_s> orb_vehicle_attitude;
