#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

struct vehicle_local_position_s {
    uint64_t timestamp_us;     // Thời gian lấy mẫu (Microseconds)
    
    // Vị trí (NED frame, m)
    float x; // North position
    float y; // East position
    float z; // Down position (độ cao h = -z)
    
    // Vận tốc (NED frame, m/s)
    float vx; // North velocity
    float vy; // East velocity
    float vz; // Down velocity
    
    // Flag báo tính hợp lệ của dữ liệu ước lượng
    bool xy_valid;
    bool z_valid;
    bool v_xy_valid;
    bool v_z_valid;
};

extern uORB::Topic<vehicle_local_position_s> orb_vehicle_local_position;
