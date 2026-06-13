#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

struct vehicle_thrust_setpoint_s {
    uint64_t timestamp;
    float xyz[3]; // Thrust setpoint X, Y, Z. Dành cho Multicopter Z sẽ là 0.0 -> 1.0 (Lực nâng)
};

extern uORB::Topic<vehicle_thrust_setpoint_s> orb_vehicle_thrust_setpoint;
