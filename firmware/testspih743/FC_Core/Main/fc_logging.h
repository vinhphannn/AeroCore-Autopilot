#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include "../MessageBus/topics/mavlink_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// Hàm core xử lý chuỗi và publish lên uORB
void fc_log_print(uint8_t severity, const char *fmt, ...);

// Các Macro y hệt PX4 để gọi cho ngắn gọn ở bất cứ file nào
#define FC_INFO(fmt, ...)  fc_log_print(FC_LOG_INFO, fmt, ##__VA_ARGS__)
#define FC_WARN(fmt, ...)  fc_log_print(FC_LOG_WARNING, fmt, ##__VA_ARGS__)
#define FC_ERR(fmt, ...)   fc_log_print(FC_LOG_ERROR, fmt, ##__VA_ARGS__)
#define FC_CRIT(fmt, ...)  fc_log_print(FC_LOG_CRITICAL, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif
