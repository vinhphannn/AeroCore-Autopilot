#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

struct manual_control_setpoint_s {
    uint64_t timestamp;
    float roll;     // -1.0 to 1.0
    float pitch;    // -1.0 to 1.0
    float yaw;      // -1.0 to 1.0
    float throttle; // 0.0 to 1.0
    
    // Các công tắc
    uint8_t switches; // Bitmask trạng thái các công tắc (Ví dụ: switch arm)
};

extern uORB::Topic<manual_control_setpoint_s> orb_manual_control_setpoint;
