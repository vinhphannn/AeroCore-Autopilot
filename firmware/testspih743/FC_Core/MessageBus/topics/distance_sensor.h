#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

struct distance_sensor_s {
    uint64_t timestamp_us;     // Thời gian lấy mẫu
    float current_distance_m;  // Khoảng cách đo được (mét)
    float min_distance_m;      // Giới hạn dưới của cảm biến
    float max_distance_m;      // Giới hạn trên của cảm biến
    float variance;            // Phương sai (độ tin cậy)
    uint8_t type;              // 0: Laser, 1: Ultrasound, 2: Infrared, 3: Radar
    uint8_t device_id;
};

extern uORB::Topic<distance_sensor_s> orb_distance_sensor;
