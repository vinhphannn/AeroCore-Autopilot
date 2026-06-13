/**
 * @file    gy53l1x.c
 * @brief   Thực thi driver GY-53-L1X
 */

#include "gy53l1x.h"
#include "usbd_cdc_if.h"   // Nếu có CDC, để in debug
#include <stdio.h>
#include <string.h>
#include "../../MessageBus/topics/distance_sensor.h"
#include "../../Main/fc_logging.h"

// Biến pub của uORB
static uORB::PublicationMulti<distance_sensor_s> tof_pub(orb_distance_sensor);

/* Macro hỗ trợ debug CDC (có thể bỏ nếu không dùng) */
#define CDC_Transmit_FS   CDC_Transmit_FS   // Giữ nguyên như project của bạn

/* ---------- Hàm nội bộ ---------- */
/**
 * @brief  Xóa tất cả cờ lỗi UART để tránh treo.
 * @param  huart: con trỏ UART handle
 */
static void GY53_ClearUARTFlags(UART_HandleTypeDef *huart) {
    __HAL_UART_CLEAR_FLAG(huart,
        UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
}

/**
 * @brief  Đọc một byte từ UART (blocking, chờ 1ms nếu không có)
 * @param  huart: UART handle
 * @param  byte: con trỏ lưu byte đọc được
 * @param  timeout_ms: thời gian chờ tối đa
 * @return 1 nếu đọc thành công, 0 nếu timeout
 */
static uint8_t GY53_ReadByte(UART_HandleTypeDef *huart, uint8_t *byte, uint32_t timeout_ms) {
    uint32_t tickstart = HAL_GetTick();
    while ((HAL_GetTick() - tickstart) < timeout_ms) {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
            *byte = (uint8_t)(huart->Instance->RDR & 0xFF);
            return 1;
        }
    }
    return 0;
}

/* ---------- API chính ---------- */

/**
 * @brief  Khởi tạo chân GPIO và UART cho GY-53.
 * @note   Sử dụng UART7 (PE7=RX, PE8=TX) như code gốc.
 *         Có thể dễ dàng thay đổi huart và chân.
 */
void GY53L1X_Init(UART_HandleTypeDef *huart) {
    /* Bật clock cho UART7 và GPIOE (theo phần cứng của bạn) */
    __HAL_RCC_UART7_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* Cấu hình GPIO PE7 (RX) và PE8 (TX) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;      // RX cần pull-up
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_UART7;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Cấu hình UART */
    huart->Instance          = UART7;
    huart->Init.BaudRate     = GY53_UART_BAUD;
    huart->Init.WordLength   = UART_WORDLENGTH_8B;
    huart->Init.StopBits     = UART_STOPBITS_1;
    huart->Init.Parity       = UART_PARITY_NONE;
    huart->Init.Mode         = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(huart);

    /* Đảm bảo cờ sạch trước khi bắt đầu */
    GY53_ClearUARTFlags(huart);
    
    FC_INFO("GY53L1X Init OK on USART2");
}

/**
 * @brief  Đọc 1 frame từ GY-53 và trích xuất khoảng cách.
 * @param  huart: UART handle
 * @param  data: con trỏ lưu kết quả
 */
void GY53L1X_Update(UART_HandleTypeDef *huart, GY53L1X_Data_t *data) {
    if (!data) return;

    /* Reset struct */
    memset(data, 0, sizeof(GY53L1X_Data_t));
    data->raw_len = 0;

    /* Xóa cờ lỗi trước khi đọc (tránh khóa UART trên H7) */
    GY53_ClearUARTFlags(huart);

    uint32_t t_start = HAL_GetTick();
    uint8_t  byte;
    uint8_t  idx = 0;

    /* Vòng lặp đọc tối đa GY53_FRAME_LEN byte hoặc hết timeout */
    while ((HAL_GetTick() - t_start) < GY53_TIMEOUT_MS && idx < GY53_FRAME_LEN) {
        if (GY53_ReadByte(huart, &byte, 1)) {   // Chờ tối đa 1ms cho mỗi byte
            /* Đồng bộ header 0x5A 0x5A */
            if (idx == 0) {
                if (byte == 0x5A) {
                    data->raw_bytes[idx++] = byte;
                }
                // Nếu không đúng 0x5A, bỏ qua và đợi byte tiếp (không reset idx)
            }
            else if (idx == 1) {
                if (byte == 0x5A) {
                    data->raw_bytes[idx++] = byte;
                } else {
                    idx = 0;  // Sai header -> quay lại tìm 0x5A đầu tiên
                }
            }
            else {
                data->raw_bytes[idx++] = byte;
            }
        }
    }

    data->raw_len = idx;

    /* Tạo chuỗi hex để debug */
    char *p = data->raw_str;
    for (uint8_t i = 0; i < idx; i++) {
        p += sprintf(p, "%02X ", data->raw_bytes[i]);
    }
    // Nếu không có byte nào, gán thông báo
    if (idx == 0) {
        strcpy(data->raw_str, "NO SIGNAL");
    }

    /* Parse frame nếu đủ 8 byte và header đúng */
    if (idx == GY53_FRAME_LEN &&
        data->raw_bytes[0] == 0x5A &&
        data->raw_bytes[1] == 0x5A) {
        // Theo datasheet GY-53: Byte4 (High) + Byte5 (Low) = Khoảng cách mm
        data->distance_mm = (uint16_t)((data->raw_bytes[4] << 8) | data->raw_bytes[5]);
        data->valid = 1;

        // Publish lên uORB
        distance_sensor_s msg;
        msg.timestamp_us = HAL_GetTick() * 1000ULL;
        msg.current_distance_m = data->distance_mm / 1000.0f;
        msg.min_distance_m = 0.05f; // Theo datasheet VL53L1X
        msg.max_distance_m = 4.0f;
        msg.variance = 0.0f;
        msg.type = 0; // Laser
        msg.device_id = 0;
        tof_pub.publish(msg);
    }
}

/**
 * @brief  In dữ liệu cảm biến qua USB CDC (tiện debug).
 * @param  data: dữ liệu đã cập nhật
 */
void GY53L1X_PrintLog(const GY53L1X_Data_t *data) {
    if (!data) return;
    char buf[128];
    if (data->valid) {
        snprintf(buf, sizeof(buf), "[ToF] Dist: %d mm\r\n", data->distance_mm);
    } else {
        snprintf(buf, sizeof(buf), "[ToF] %s\r\n", data->raw_str);
    }
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
}
