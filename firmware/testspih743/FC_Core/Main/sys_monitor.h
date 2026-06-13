#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mô phỏng lệnh `top` của PX4
void px4_top(void);

// Task chạy ngầm giống cpuload của PX4
void cpuload_monitor_task(void);

// Lấy CPU Load cho MAVLink
uint8_t cpuload_get_percentage(void);

#ifdef __cplusplus
}
#endif
