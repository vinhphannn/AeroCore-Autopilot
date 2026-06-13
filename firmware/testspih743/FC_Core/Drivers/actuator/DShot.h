#pragma once
#include <stdint.h>
#include "stm32h7xx_hal.h"

#define DSHOT_BIT_LENGTH 16
#define DSHOT_FRAME_SIZE 18 // 16 bit data + 2 bit reset (PWM = 0)

// DShot600 timing values for Period = 399
#define DSHOT_0_TIMING 150
#define DSHOT_1_TIMING 300

class DShotDriver {
public:
    DShotDriver(TIM_HandleTypeDef* htim);
    ~DShotDriver() = default;

    void init();
    
    /**
     * @brief Set motor throttle
     * @param motor_index 0 to 3 for Motor 1 to 4
     * @param throttle 0 to 2000 (0 = off, 48-2047 = speed)
     */
    void set_throttle(uint8_t motor_index, uint16_t throttle);
    
    /**
     * @brief Encode throttle into DMA buffers and trigger DMA transfer
     * MUST CALL SCB_CleanDCache_by_Addr to clear D-Cache before DMA.
     */
    void update();

private:
    TIM_HandleTypeDef* _htim;
    
    uint16_t _throttles[4];

    // Các buffer này BẮT BUỘC phải được ép xuống vùng nhớ hỗ trợ DMA và căn lề 32 byte
    // Cortex-M7 D-Cache rule
    uint32_t _dma_buffer_m1[DSHOT_FRAME_SIZE] __attribute__((aligned(32)));
    uint32_t _dma_buffer_m2[DSHOT_FRAME_SIZE] __attribute__((aligned(32)));
    uint32_t _dma_buffer_m3[DSHOT_FRAME_SIZE] __attribute__((aligned(32)));
    uint32_t _dma_buffer_m4[DSHOT_FRAME_SIZE] __attribute__((aligned(32)));

    void prepare_packet(uint16_t throttle, uint32_t* buffer);
};

extern DShotDriver g_dshot;
