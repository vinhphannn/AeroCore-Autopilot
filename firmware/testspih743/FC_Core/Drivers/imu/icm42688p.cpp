#include "icm42688p.h"
#include <math.h>
#include "../../Main/fc_logging.h"

extern SPI_HandleTypeDef hspi1;
#define ICM_CS_PORT  GPIOA
#define ICM_CS_PIN   GPIO_PIN_15

// Khởi tạo instance (singleton)
ICM42688P g_icm42688p;

ICM42688P::ICM42688P() : _imu_pub(orb_sensor_imu) {
    // Không thực hiện giao tiếp phần cứng trong constructor
}

void ICM42688P::cs_delay() {
    for(volatile uint32_t i = 0; i < 500; i++) __NOP();
}

uint64_t ICM42688P::get_timestamp_us() {
    // Tạm thời lấy ms của HAL nhân lên 1000 để ra us
    // Bạn nên cập nhật Timer (như TIM2 32-bit) để có microsecond chính xác.
    return (uint64_t)HAL_GetTick() * 1000ULL;
}

bool ICM42688P::init() {
    FC_INFO("Starting ICM42688P on SPI1...");
    __HAL_RCC_SPI1_FORCE_RESET();
    HAL_Delay(10);
    __HAL_RCC_SPI1_RELEASE_RESET();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Chân SPI1 */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM; 
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6; // MISO
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Chân CS (PA15) */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    /* Cấu hình SPI */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    HAL_SPI_Init(&hspi1);

    HAL_Delay(100);

    /* Đọc ID để kiểm tra sự tồn tại của chip */
    uint8_t tx[2] = {0x75 | 0x80, 0x00};
    uint8_t rx[2] = {0, 0};
    
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
    cs_delay();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
    cs_delay();
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
    
    if (rx[1] == 0x47 || rx[1] == 0xDB) {
        // Bật cảm biến (Thoát chế độ sleep)
        uint8_t pwr_cmd[2] = {ICM_REG_PWR_MGMT0, 0x0F}; 
        HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, pwr_cmd, 2, 10);
        HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
        FC_INFO("ICM42688P Init OK (WhoAmI = 0x%02X)", rx[1]);
        return true;
    }
    
    FC_ERR("ICM42688P Init FAIL! (WhoAmI = 0x%02X)", rx[1]);
    return false;
}

void ICM42688P::run() {
    uint8_t tx[15] = {ICM_REG_ACCEL_X_H | 0x80, 0};
    uint8_t rx[15] = {0}; 
    
    /* Giao tiếp SPI đọc Burst 14 bytes (Accel X,Y,Z + Gyro X,Y,Z + Temperature) */
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
    cs_delay();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 15, 10);
    cs_delay();
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);

    /* Parse Raw data (High byte trước, Low byte sau) */
    int16_t raw_ax = (int16_t)((rx[1] << 8) | rx[2]);
    int16_t raw_ay = (int16_t)((rx[3] << 8) | rx[4]);
    int16_t raw_az = (int16_t)((rx[5] << 8) | rx[6]);
    
    int16_t raw_gx = (int16_t)((rx[7] << 8) | rx[8]);
    int16_t raw_gy = (int16_t)((rx[9] << 8) | rx[10]);
    int16_t raw_gz = (int16_t)((rx[11] << 8) | rx[12]);
    
    int16_t raw_temp = (int16_t)((rx[13] << 8) | rx[14]);

    /* Chuẩn bị gói dữ liệu */
    sensor_imu_s msg;
    msg.timestamp_us = get_timestamp_us();

    /* 1. Đổi Accel ra m/s^2 (Gia tốc trọng trường g = 9.80665) */
    msg.accel_m_s2[0] = ((float)raw_ax / ICM_ACCEL_SENS) * 9.80665f;
    msg.accel_m_s2[1] = ((float)raw_ay / ICM_ACCEL_SENS) * 9.80665f;
    msg.accel_m_s2[2] = ((float)raw_az / ICM_ACCEL_SENS) * 9.80665f;

    /* 2. Đổi Gyro ra rad/s (1 độ = 0.0174532925 radian) */
    msg.gyro_rad_s[0] = ((float)raw_gx / ICM_GYRO_SENS) * 0.0174532925f;
    msg.gyro_rad_s[1] = ((float)raw_gy / ICM_GYRO_SENS) * 0.0174532925f;
    msg.gyro_rad_s[2] = ((float)raw_gz / ICM_GYRO_SENS) * 0.0174532925f;

    /* 3. Nhiệt độ (Công thức datasheet ICM42688: Temp = raw/132.48 + 25) */
    msg.temperature_c = ((float)raw_temp / 132.48f) + 25.0f;

    /* PUBLISH: Đẩy dữ liệu vào mạng lưới uORB */
    _imu_pub.publish(msg);
}
