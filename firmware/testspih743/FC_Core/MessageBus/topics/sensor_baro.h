#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

struct sensor_baro_s {
    uint64_t timestamp_us;     // Thời gian lấy mẫu (Microseconds)
    float pressure_mbar;       // Áp suất (mbar / hPa)
    float temperature_c;       // Nhiệt độ (Độ C)
    uint8_t device_id;         // Để phân biệt nếu hệ thống có nhiều Baro (vd: 0x76, 0x77)
};

// Khai báo Topic dưới dạng biến Toàn cục (Global)
extern uORB::Topic<sensor_baro_s> orb_sensor_baro;
