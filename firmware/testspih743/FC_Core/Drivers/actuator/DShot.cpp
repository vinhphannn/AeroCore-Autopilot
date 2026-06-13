#include "DShot.h"

extern TIM_HandleTypeDef htim5;
DShotDriver g_dshot(&htim5); // Cấu hình TIM5

DShotDriver::DShotDriver(TIM_HandleTypeDef* htim) : _htim(htim) {
    for (int i = 0; i < 4; i++) _throttles[i] = 0;
}

void DShotDriver::init() {
    // Không làm gì nhiều, CubeMX đã Init Timer
}

void DShotDriver::set_throttle(uint8_t motor_index, uint16_t throttle) {
    if (motor_index < 4) {
        if (throttle > 2047) throttle = 2047;
        _throttles[motor_index] = throttle;
    }
}

void DShotDriver::prepare_packet(uint16_t throttle, uint32_t* buffer) {
    uint16_t packet = (throttle << 1); // 11-bit throttle + 1-bit telemetry request (0)
    
    // Tính CRC 4-bit
    uint16_t csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    packet = (packet << 4) | csum;

    // Chuyển thành duty cycle cho PWM
    for (int i = 0; i < 16; i++) {
        buffer[i] = (packet & (0x8000 >> i)) ? DSHOT_1_TIMING : DSHOT_0_TIMING;
    }
    
    // Reset bit (bắt buộc = 0 để ESC nhận biết hết gói)
    buffer[16] = 0;
    buffer[17] = 0;
}

void DShotDriver::update() {
    prepare_packet(_throttles[0], _dma_buffer_m1);
    prepare_packet(_throttles[1], _dma_buffer_m2);
    prepare_packet(_throttles[2], _dma_buffer_m3);
    prepare_packet(_throttles[3], _dma_buffer_m4);

    // BẮT BUỘC: Xả Cache L1 xuống RAM để DMA đọc đúng data (M-7 Rule)
    SCB_CleanDCache_by_Addr((uint32_t*)_dma_buffer_m1, sizeof(_dma_buffer_m1));
    SCB_CleanDCache_by_Addr((uint32_t*)_dma_buffer_m2, sizeof(_dma_buffer_m2));
    SCB_CleanDCache_by_Addr((uint32_t*)_dma_buffer_m3, sizeof(_dma_buffer_m3));
    SCB_CleanDCache_by_Addr((uint32_t*)_dma_buffer_m4, sizeof(_dma_buffer_m4));

    // Kích hoạt DMA Burst
    // PHẢI gọi Stop_DMA trước để reset state machine của thư viện HAL
    // (Vì chúng ta không xài ngắt DMA Transfer Complete nên HAL sẽ bị kẹt ở trạng thái BUSY vĩnh viễn)
    HAL_TIM_PWM_Stop_DMA(_htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop_DMA(_htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop_DMA(_htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop_DMA(_htim, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start_DMA(_htim, TIM_CHANNEL_1, _dma_buffer_m1, DSHOT_FRAME_SIZE);
    HAL_TIM_PWM_Start_DMA(_htim, TIM_CHANNEL_2, _dma_buffer_m2, DSHOT_FRAME_SIZE);
    HAL_TIM_PWM_Start_DMA(_htim, TIM_CHANNEL_3, _dma_buffer_m3, DSHOT_FRAME_SIZE);
    HAL_TIM_PWM_Start_DMA(_htim, TIM_CHANNEL_4, _dma_buffer_m4, DSHOT_FRAME_SIZE);
}
