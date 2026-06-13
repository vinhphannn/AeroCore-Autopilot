#include "ms5607.h"
#include "../../Main/fc_logging.h"

extern I2C_HandleTypeDef hi2c1;

// Khởi tạo các instance
MS5607 g_ms5607_1(0x76);
MS5607 g_ms5607_2(0x77);

MS5607::MS5607(uint8_t i2c_addr) : _baro_pub(orb_sensor_baro), _i2c_addr(i2c_addr), _present(false) {
}

uint64_t MS5607::get_timestamp_us() {
    return (uint64_t)HAL_GetTick() * 1000ULL;
}

bool MS5607::init() {
    uint8_t cmd = MS5607_CMD_RESET;
    // Gửi lệnh Reset
    if (HAL_I2C_Master_Transmit(&hi2c1, (_i2c_addr << 1), &cmd, 1, 50) == HAL_OK) {
        HAL_Delay(50); // Chờ cảm biến reset xong (Datasheet yêu cầu tối thiểu 3ms)
        
        // Đọc 6 hằng số hiệu chuẩn từ PROM (C1 - C6)
        for (int c = 1; c <= 6; c++) {
            uint8_t prom_cmd = MS5607_CMD_PROM_READ + (c * 2);
            uint8_t d[2];
            HAL_I2C_Master_Transmit(&hi2c1, (_i2c_addr << 1), &prom_cmd, 1, 50);
            HAL_I2C_Master_Receive(&hi2c1, (_i2c_addr << 1), d, 2, 50);
            _calib[c] = (d[0] << 8) | d[1];
        }
        _present = true;
        FC_INFO("MS5607 Init OK on I2C addr 0x%02X", _i2c_addr);
        return true;
    }
    FC_ERR("MS5607 Init FAIL on I2C addr 0x%02X", _i2c_addr);
    return false;
}

void MS5607::run() {
    if (!_present) return;

    uint8_t d1_cmd = MS5607_CMD_D1_4096;
    uint8_t d2_cmd = MS5607_CMD_D2_4096;
    uint8_t read_cmd = MS5607_CMD_ADC_READ;
    uint8_t raw[3];

    // Đo Áp suất (D1)
    if (HAL_I2C_Master_Transmit(&hi2c1, (_i2c_addr << 1), &d1_cmd, 1, 20) != HAL_OK) return;
    
    // CẢNH BÁO MỨC ĐỘ THIẾT KẾ: 
    // Việc dùng HAL_Delay(10) trong một hàm run() là TỐI KỴ trong FreeRTOS vì nó block CPU.
    // Tương lai (Phase 3), bạn cần viết lại hàm này dưới dạng State Machine (Máy trạng thái)
    // Tức là: State 1 (Gửi lệnh đo) -> Thoát hàm -> OS tự động đợi 10ms -> State 2 (Quay lại lấy data).
    // Ở Phase hiện tại (Bare-metal), chúng ta tạm giữ nguyên logic cũ.
    HAL_Delay(10); 
    
    HAL_I2C_Master_Transmit(&hi2c1, (_i2c_addr << 1), &read_cmd, 1, 20);
    HAL_I2C_Master_Receive(&hi2c1, (_i2c_addr << 1), raw, 3, 20);
    uint32_t D1 = (raw[0] << 16) | (raw[1] << 8) | raw[2];

    // Đo Nhiệt độ (D2)
    HAL_I2C_Master_Transmit(&hi2c1, (_i2c_addr << 1), &d2_cmd, 1, 20);
    HAL_Delay(10); // Lại chặn CPU 10ms
    HAL_I2C_Master_Transmit(&hi2c1, (_i2c_addr << 1), &read_cmd, 1, 20);
    HAL_I2C_Master_Receive(&hi2c1, (_i2c_addr << 1), raw, 3, 20);
    uint32_t D2 = (raw[0] << 16) | (raw[1] << 8) | raw[2];

    /* TÍNH TOÁN BÙ TRỪ TOÁN HỌC (Công thức chuẩn của MS5607) */
    int64_t dT = (int64_t)D2 - ((int64_t)_calib[5] << 8);
    int32_t TEMP = 2000 + ((dT * (int64_t)_calib[6]) >> 23);
    int64_t OFF = ((int64_t)_calib[2] << 17) + (((int64_t)_calib[4] * dT) >> 6);
    int64_t SENS = ((int64_t)_calib[1] << 16) + (((int64_t)_calib[3] * dT) >> 7);
    int32_t P = (int32_t)(((((int64_t)D1 * SENS) >> 21) - OFF) >> 15);

    /* ĐẨY DỮ LIỆU VÀO MẠNG LƯỚI uORB */
    sensor_baro_s msg;
    msg.timestamp_us = get_timestamp_us();
    msg.pressure_mbar = (float)P / 100.0f;
    msg.temperature_c = (float)TEMP / 100.0f;
    msg.device_id = _i2c_addr;

    _baro_pub.publish(msg);
}
