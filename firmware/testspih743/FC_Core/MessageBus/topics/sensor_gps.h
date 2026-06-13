#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

struct sensor_gps_s {
    uint64_t timestamp_us;     // Thời gian lấy mẫu
    uint64_t time_utc_usec;    // Thời gian GPS UTC
    int32_t lat;               // Vĩ độ (độ * 1E7)
    int32_t lon;               // Kinh độ (độ * 1E7)
    int32_t alt;               // Cao độ so với mặt nước biển (mm)
    int32_t alt_ellipsoid;     // Cao độ so với WGS84 (mm)
    uint8_t fix_type;          // 0: Không có, 2: 2D, 3: 3D, 4: DGPS, 5: RTK float, 6: RTK fixed
    uint8_t satellites_used;   // Số vệ tinh
    float eph;                 // Sai số vị trí ngang (m)
    float epv;                 // Sai số vị trí dọc (m)
    float hdop;                // Horizontal dilution of precision
    float vdop;                // Vertical dilution of precision
    float vel_m_s;             // Vận tốc mặt đất (m/s)
    float vel_n_m_s;           // Vận tốc trục Bắc (m/s)
    float vel_e_m_s;           // Vận tốc trục Đông (m/s)
    float vel_d_m_s;           // Vận tốc trục Xuống (m/s)
    float cog_rad;             // Course over ground (Radian)
    uint8_t device_id;
};

extern uORB::Topic<sensor_gps_s> orb_sensor_gps;
