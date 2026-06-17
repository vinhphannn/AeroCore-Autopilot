#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../uORB_lite.h"

#ifdef __cplusplus
extern "C" {
#endif

// Topic ID
#define ORB_ID_LANDING_TARGET 25

typedef struct {
    uint64_t timestamp_us;     // Thời gian nhận dạng (microseconds)
    bool valid;                // Cờ báo tìm thấy mục tiêu (ArUco/Landing Pad) hay chưa
    float rel_pos_x;           // Sai lệch X trong hệ NED local (m)
    float rel_pos_y;           // Sai lệch Y trong hệ NED local (m)
    float rel_pos_z;           // Sai lệch Z trong hệ NED local (m)
} landing_target_s;

#ifdef __cplusplus
}

extern uORB::Topic<landing_target_s> orb_landing_target;

#endif
