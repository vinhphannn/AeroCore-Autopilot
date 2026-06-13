#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

struct sensor_optical_flow_s {
    uint64_t timestamp_us;     // Thời gian lấy mẫu
    int16_t pixel_flow_x_integral; // Tổng lưu lượng pixel trục X
    int16_t pixel_flow_y_integral; // Tổng lưu lượng pixel trục Y
    uint32_t integration_timespan_us; // Khoảng thời gian tích lũy
    uint8_t quality;           // Chất lượng ảnh (0-255)
    uint8_t device_id;
};

extern uORB::Topic<sensor_optical_flow_s> orb_sensor_optical_flow;
