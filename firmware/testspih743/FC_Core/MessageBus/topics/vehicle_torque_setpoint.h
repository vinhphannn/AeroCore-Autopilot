#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

struct vehicle_torque_setpoint_s {
    uint64_t timestamp;
    float xyz[3]; // Torque setpoint X (Roll), Y (Pitch), Z (Yaw) (-1.0 to 1.0)
};

extern uORB::Topic<vehicle_torque_setpoint_s> orb_vehicle_torque_setpoint;
