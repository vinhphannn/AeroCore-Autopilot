#pragma once

#include <stdint.h>
#include "../uORB_lite.h"

#define SETPOINT_TYPE_POSITION 0
#define SETPOINT_TYPE_VELOCITY 1
#define SETPOINT_TYPE_LOITER   2
#define SETPOINT_TYPE_TAKEOFF  3
#define SETPOINT_TYPE_LAND     4

struct position_setpoint_s {
    bool valid;
    float x;    // m (NED Local Position X)
    float y;    // m (NED Local Position Y)
    float z;    // m (NED Local Position Z)
    float vx;   // m/s (NED Velocity X)
    float vy;   // m/s (NED Velocity Y)
    float vz;   // m/s (NED Velocity Z)
    float yaw;  // rad
    uint8_t type;
};

struct position_setpoint_triplet_s {
    uint64_t timestamp;
    position_setpoint_s previous;
    position_setpoint_s current;
    position_setpoint_s next;
};

extern uORB::Topic<position_setpoint_triplet_s> orb_position_setpoint_triplet;
