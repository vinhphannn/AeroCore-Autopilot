#pragma once

#include <stdint.h>
#include "../uORB_lite.h"

struct vehicle_command_s {
    uint64_t timestamp;
    uint16_t command;
    float param1;
    float param2;
    float param3;
    float param4;
    double param5;
    double param6;
    float param7;
};

extern uORB::Topic<vehicle_command_s> orb_vehicle_command;
