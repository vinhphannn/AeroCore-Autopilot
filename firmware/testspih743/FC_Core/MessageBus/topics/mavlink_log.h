#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ORB_ID_MAVLINK_LOG 11

// Các mức độ cảnh báo chuẩn MAVLink
#define FC_LOG_EMERGENCY 0
#define FC_LOG_ALERT     1
#define FC_LOG_CRITICAL  2
#define FC_LOG_ERROR     3
#define FC_LOG_WARNING   4
#define FC_LOG_NOTICE    5
#define FC_LOG_INFO      6
#define FC_LOG_DEBUG     7

typedef struct {
    uint64_t timestamp;
    uint8_t severity;
    char text[127]; // Chuỗi text tối đa 127 ký tự (Chuẩn MAVLink STATUSTEXT)
} mavlink_log_s;

extern mavlink_log_s g_mavlink_log_data;

#ifdef __cplusplus
extern uORB::Topic<mavlink_log_s> orb_mavlink_log;
#endif

#ifdef __cplusplus
}
#endif
