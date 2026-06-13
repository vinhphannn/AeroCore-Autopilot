#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

// Semaphores for DMA completion
SemaphoreHandle_t sem_spi1_dma = NULL;
SemaphoreHandle_t sem_spi4_dma = NULL;
SemaphoreHandle_t sem_i2c1_dma = NULL;
SemaphoreHandle_t sem_i2c2_dma = NULL;

extern "C" void Init_Sensor_Semaphores() {
    sem_spi1_dma = xSemaphoreCreateBinary();
    sem_spi4_dma = xSemaphoreCreateBinary();
    sem_i2c1_dma = xSemaphoreCreateBinary();
    sem_i2c2_dma = xSemaphoreCreateBinary();
}

// SPI DMA Complete Callback
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hspi->Instance == SPI1 && sem_spi1_dma != NULL) {
        xSemaphoreGiveFromISR(sem_spi1_dma, &xHigherPriorityTaskWoken);
    } else if (hspi->Instance == SPI4 && sem_spi4_dma != NULL) {
        xSemaphoreGiveFromISR(sem_spi4_dma, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// I2C DMA Complete Callback
extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hi2c->Instance == I2C1 && sem_i2c1_dma != NULL) {
        xSemaphoreGiveFromISR(sem_i2c1_dma, &xHigherPriorityTaskWoken);
    } else if (hi2c->Instance == I2C2 && sem_i2c2_dma != NULL) {
        xSemaphoreGiveFromISR(sem_i2c2_dma, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
