#pragma once

#include "main.h"
#include "spi.h"
#include <stdint.h>

// Import thư viện cấu trúc MessageBus (uORB_lite)
#include "../../MessageBus/topics/sensor_imu.h"
#include "../../MessageBus/uORB_lite.h"

/* ---- Thanh ghi ---- */
#define ICM_REG_WHO_AM_I     0x75   /* Expected: 0x47 */
#define ICM_REG_PWR_MGMT0    0x4E
#define ICM_REG_ACCEL_CONFIG 0x4F
#define ICM_REG_GYRO_CONFIG  0x50
#define ICM_REG_ACCEL_X_H    0x1F   /* Burst read: ACCEL X/Y/Z + GYRO X/Y/Z */

/* ---- Full-scale / sensitivity ---- */
/* ACCEL_CONFIG0: 0x66 → ±2g   → 16384 LSB/g */
/* GYRO_CONFIG0:  0x66 → ±2000°/s → 16.4 LSB/(°/s) */
#define ICM_ACCEL_SENS    16384.0f  /* LSB/g   for ±2g  */
#define ICM_GYRO_SENS     16.4f     /* LSB/dps for ±2000dps */

/**
 * @brief Lớp Driver cho ICM42688P theo kiến trúc PX4
 */
class ICM42688P {
public:
    ICM42688P();
    ~ICM42688P() = default;

    /**
     * @brief Khởi tạo phần cứng SPI và bật IMU
     * @return true nếu đọc đúng chip_id, false nếu lỗi
     */
    bool init();

    /**
     * @brief Đọc giá trị raw, quy đổi ra đơn vị chuẩn và đẩy (publish) lên uORB
     */
    void run();

private:
    // Kênh phát dữ liệu (Publisher) vào topic orb_sensor_imu
    uORB::PublicationMulti<sensor_imu_s> _imu_pub;

    // Các hàm nội bộ
    void cs_delay();
    uint64_t get_timestamp_us();
};

// Khai báo biến toàn cục (Singleton) để mọi người đều có thể truy cập nếu cần thiết
extern ICM42688P g_icm42688p;
