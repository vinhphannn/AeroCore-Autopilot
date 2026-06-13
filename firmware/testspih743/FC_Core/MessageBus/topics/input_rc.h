#pragma once
#include <stdint.h>
#include "../uORB_lite.h"

#ifdef __cplusplus
extern "C" {
#endif

// Topic id
#define ORB_ID_INPUT_RC 10

// Cấu trúc mô phỏng đúng chuẩn PX4 input_rc_s
typedef struct {
    uint64_t timestamp;             // Thời gian cập nhật
    uint64_t timestamp_last_signal; // Thời gian có tín hiệu cuối cùng
    uint8_t channel_count;          // Số kênh RC (Thường là 16 với CRSF)
    int32_t rssi;                   // Cường độ sóng (Link Quality)
    uint8_t rc_failsafe;            // 1 = Mất sóng tay cầm
    uint8_t rc_lost;                // 1 = Không nhận được frame nào
    uint16_t values[16];            // Giá trị các kênh (Tương tự PWM từ 1000 đến 2000)
} input_rc_s;

extern input_rc_s g_input_rc_data;

#ifdef __cplusplus
}

extern uORB::Topic<input_rc_s> orb_input_rc;

#endif
