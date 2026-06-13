#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

struct sensor_mag_s {
    uint64_t timestamp_us;     // Thời gian lấy mẫu
    float mag_x_ut;            // Từ trường trục X (Microtesla)
    float mag_y_ut;            // Từ trường trục Y
    float mag_z_ut;            // Từ trường trục Z
    float heading_deg;         // Góc Heading tính toán thô (tùy chọn)
    float temperature_c;       // Nhiệt độ nếu có
    uint8_t device_id;         // Phân biệt nhiều la bàn
};

extern uORB::Topic<sensor_mag_s> orb_sensor_mag;
