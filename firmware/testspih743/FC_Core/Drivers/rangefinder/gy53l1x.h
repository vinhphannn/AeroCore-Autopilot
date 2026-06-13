/**
 * @file    gy53l1x.h
 * @brief   Driver cho cảm biến laser TOF GY-53-L1X (VL53L1X + STM32)
 * @note    Module xuất dữ liệu qua UART 8 byte, baud mặc định 9600
 */

#ifndef __GY53L1X_H
#define __GY53L1X_H

#include "main.h"
#include <stdint.h>

/* Cấu hình mặc định (có thể thay đổi tùy module) */
#define GY53_UART_BAUD      9600          // Baudrate mặc định của module
#define GY53_FRAME_LEN      8             // Số byte trong 1 frame
#define GY53_TIMEOUT_MS     50            // Timeout đọc 1 frame (ms)

/* Cấu trúc dữ liệu cảm biến */
typedef struct {
    uint16_t distance_mm;                 // Khoảng cách (mm) nếu valid = 1
    uint8_t  valid;                       // 1 nếu frame hợp lệ
    char     raw_str[64];                 // Chuỗi debug thô (hex)
    uint8_t  raw_bytes[GY53_FRAME_LEN];  // Mảng byte thô cuối cùng đọc được
    uint8_t  raw_len;                     // Số byte thực đọc
} GY53L1X_Data_t;

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- API ---------- */
void GY53L1X_Init(UART_HandleTypeDef *huart);
void GY53L1X_Update(UART_HandleTypeDef *huart, GY53L1X_Data_t *data);
void GY53L1X_PrintLog(const GY53L1X_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __GY53L1X_H */
