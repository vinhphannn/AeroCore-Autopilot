#pragma once

#include "main.h"
#include "i2c.h"
#include <stdint.h>

// Import thư viện cấu trúc MessageBus (uORB_lite)
#include "../../MessageBus/topics/sensor_baro.h"
#include "../../MessageBus/uORB_lite.h"

/* Commands */
#define MS5607_CMD_RESET      0x1E
#define MS5607_CMD_PROM_READ  0xA0   /* +c*2 for coefficients 1..6 */
#define MS5607_CMD_D1_4096    0x48   /* Pressure OSR=4096 */
#define MS5607_CMD_D2_4096    0x58   /* Temperature OSR=4096 */
#define MS5607_CMD_ADC_READ   0x00

/**
 * @brief Lớp Driver cho MS5607 theo kiến trúc PX4
 */
class MS5607 {
public:
    /**
     * @param i2c_addr Địa chỉ I2C của Baro (Thường là 0x76 hoặc 0x77)
     */
    MS5607(uint8_t i2c_addr);
    ~MS5607() = default;

    /**
     * @brief Khởi tạo cảm biến (Reset và đọc tham số hiệu chuẩn PROM)
     * @return true nếu tìm thấy cảm biến và đọc PROM thành công
     */
    bool init();

    /**
     * @brief Kích hoạt đo đạc, đọc ADC, tính toán áp suất/nhiệt độ theo công thức
     *        sau đó xuất (publish) dữ liệu lên mạng lưới uORB.
     */
    void run();

private:
    uORB::PublicationMulti<sensor_baro_s> _baro_pub;

    uint8_t  _i2c_addr;
    bool     _present;
    uint16_t _calib[8]; // Các hệ số bù trừ C1-C6

    uint64_t get_timestamp_us();
};

// Khai báo sẵn 2 biến toàn cục nếu hệ thống có 2 cảm biến MS5607
extern MS5607 g_ms5607_1;
extern MS5607 g_ms5607_2;
