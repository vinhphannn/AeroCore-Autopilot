#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../uORB_lite.h"

// Trạng thái Arming của máy bay
#define ARMING_STATE_INIT 0
#define ARMING_STATE_STANDBY 1
#define ARMING_STATE_ARMED 2
#define ARMING_STATE_STANDBY_ERROR 3

// Chế độ bay (Navigation State)
#define NAVIGATION_STATE_MANUAL 0
#define NAVIGATION_STATE_ALTCTL 1
#define NAVIGATION_STATE_POSCTL 2
#define NAVIGATION_STATE_AUTO_MISSION 3
#define NAVIGATION_STATE_ACRO 4
#define NAVIGATION_STATE_STAB 5
#define NAVIGATION_STATE_AUTO_LAND 6
#define NAVIGATION_STATE_AUTO_TAKEOFF 7
#define NAVIGATION_STATE_AUTO_RTL 8

struct vehicle_status_s {
    uint64_t timestamp;
    uint8_t nav_state;
    uint8_t arming_state;
    bool failsafe;
    bool rc_signal_lost;
};

extern uORB::Topic<vehicle_status_s> orb_vehicle_status;
