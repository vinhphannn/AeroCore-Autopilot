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

// Ring Buffer & State Machine variables
#define GY53_RX_BUF_SIZE 128
static uint8_t gy53_rx_buffer[GY53_RX_BUF_SIZE];
static volatile uint16_t gy53_rx_head = 0;
static uint16_t gy53_rx_tail = 0;
static uint8_t gy53_rx_byte;
static UART_HandleTypeDef *gy53_huart = NULL;

static void GY53_ClearUARTFlags(UART_HandleTypeDef *huart) {
    __HAL_UART_CLEAR_FLAG(huart,
        UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
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
    
    gy53_huart = huart;
    HAL_UART_Receive_IT(huart, &gy53_rx_byte, 1);
    
    FC_INFO("GY53L1X Init OK (Interrupt Mode)");
}

/**
 * @brief  Callback ngắt nhận UART (cần được gọi từ HAL_UART_RxCpltCallback trong main.c/it.c)
 */
void GY53L1X_RxCallback(UART_HandleTypeDef *huart) {
    if (huart == gy53_huart) {
        gy53_rx_buffer[gy53_rx_head] = gy53_rx_byte;
        gy53_rx_head = (gy53_rx_head + 1) % GY53_RX_BUF_SIZE;
        HAL_UART_Receive_IT(huart, &gy53_rx_byte, 1);
    }
}

/**
 * @brief  Đọc data từ Ring Buffer (Non-blocking) và phân tích frame GY-53.
 * @param  huart: UART handle (không còn block)
 * @param  data: con trỏ lưu kết quả
 */
void GY53L1X_Update(UART_HandleTypeDef *huart, GY53L1X_Data_t *data) {
    if (!data || huart != gy53_huart) return;

    static uint8_t frame_buf[GY53_FRAME_LEN];
    static uint8_t frame_idx = 0;

    // Lấy tất cả byte đang có trong Ring Buffer
    while (gy53_rx_tail != gy53_rx_head) {
        uint8_t byte = gy53_rx_buffer[gy53_rx_tail];
        gy53_rx_tail = (gy53_rx_tail + 1) % GY53_RX_BUF_SIZE;

        // State Machine phân tích Frame [0x5A, 0x5A, type, len, dataHigh, dataLow, mode, chksum]
        if (frame_idx == 0) {
            if (byte == 0x5A) frame_buf[frame_idx++] = byte;
        } else if (frame_idx == 1) {
            if (byte == 0x5A) frame_buf[frame_idx++] = byte;
            else frame_idx = 0;
        } else {
            frame_buf[frame_idx++] = byte;
        }

        // Đã nhận đủ 1 frame hợp lệ
        if (frame_idx == GY53_FRAME_LEN) {
            // Có thể kiểm tra Checksum ở đây (Checksum = tổng các byte trừ header)
            
            // Lấy khoảng cách: Byte4 (High) + Byte5 (Low) = Khoảng cách mm
            data->distance_mm = (uint16_t)((frame_buf[4] << 8) | frame_buf[5]);
            data->valid = 1;
            
            // Tạo chuỗi hex để debug
            char *p = data->raw_str;
            for (uint8_t i = 0; i < GY53_FRAME_LEN; i++) {
                p += sprintf(p, "%02X ", frame_buf[i]);
            }

            // Publish lên uORB ngay lập tức
            distance_sensor_s msg;
            msg.timestamp_us = HAL_GetTick() * 1000ULL;
            msg.current_distance_m = data->distance_mm / 1000.0f;
            msg.min_distance_m = 0.05f; // Theo datasheet VL53L1X
            msg.max_distance_m = 4.0f;
            msg.variance = 0.0f;
            msg.type = 0; // Laser
            msg.device_id = 0;
            tof_pub.publish(msg);

            // Reset frame index để đọc frame tiếp theo
            frame_idx = 0;
        }
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
