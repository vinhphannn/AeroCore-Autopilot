#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../uORB_lite.h"

struct actuator_armed_s {
    uint64_t timestamp;
    bool armed;
    bool prearmed; // Dùng để quay idle chậm khi chuẩn bị cất cánh
    bool force_failsafe;
};

extern uORB::Topic<actuator_armed_s> orb_actuator_armed;
