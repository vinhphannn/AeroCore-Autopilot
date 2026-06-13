#pragma once

#include <stdint.h>
#include <stdbool.h>

// Import cấu trúc uORB để nhận data nội bộ
#include "../MessageBus/topics/sensor_imu.h"
#include "../MessageBus/topics/sensor_baro.h"
#include "../MessageBus/topics/sensor_mag.h"
#include "../MessageBus/topics/sensor_optical_flow.h"
#include "../MessageBus/topics/sensor_gps.h"
#include "../MessageBus/topics/distance_sensor.h"
#include "../MessageBus/topics/vehicle_attitude.h"
#include "../MessageBus/topics/mavlink_log.h"
#include "../MessageBus/topics/actuator_armed.h"
#include "../MessageBus/topics/vehicle_status.h"
#include "../MessageBus/topics/input_rc.h"
#include "../MessageBus/uORB_lite.h"

// Cấu hình ID cho hệ thống (System ID) và Component (Component ID)
#define MAVLINK_SYSTEM_ID     1     // ID của Drone (Mặc định là 1)
#define MAVLINK_COMPONENT_ID  1     // ID của Mạch Autopilot (Mặc định là 1)

#define MAVLINK_RX_BUF_SIZE 512
#define MAVLINK_TX_BUF_SIZE 2048

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "mavlink_lib/paparazzi/mavlink.h"
#pragma GCC diagnostic pop

class MavlinkManager {
public:
    MavlinkManager();
    ~MavlinkManager() = default;

    /**
     * @brief Khởi tạo hệ thống Mavlink (Subscribe các uORB topic)
     */
    void init();

    /**
     * @brief Vòng lặp chính của MAVLink. 
     * Phải được gọi liên tục trong một FreeRTOS Task (VD: vMavlinkTask).
     * Quản lý việc parse data từ RX, ném dữ liệu lên TX và flush ra USB.
     */
    void run();

    /**
     * @brief Lưu trữ Byte nhận được từ USB/UART vào mảng đệm (RX Ring Buffer).
     * Hàm này an toàn và ĐƯỢC PHÉP gọi trong ngắt (ISR).
     */
    void receive_char(uint8_t c);

private:
    // Các hàm đóng gói và gửi
    void send_raw_imu(const sensor_imu_s& imu_data, const sensor_mag_s& mag_data);
    void send_scaled_pressure(const sensor_baro_s& baro_data);
    void send_optical_flow(const sensor_optical_flow_s& opt_data);
    void send_distance_sensor(const distance_sensor_s& dist_data);
    void send_gps(const sensor_gps_s& gps_data);
    void send_attitude(const vehicle_attitude_s& att_data);
    void send_rc_channels(const input_rc_s& rc_data);
    void send_log(const mavlink_log_s& log_data);

    // uORB Subscriptions
    uORB::SubscriptionMulti<sensor_imu_s> _imu_sub;
    uORB::SubscriptionMulti<sensor_baro_s> _baro_sub;
    uORB::SubscriptionMulti<sensor_mag_s> _mag_sub;
    uORB::SubscriptionMulti<sensor_optical_flow_s> _opt_sub;
    uORB::SubscriptionMulti<distance_sensor_s> _dist_sub;
    uORB::SubscriptionMulti<sensor_gps_s> _gps_sub;
    uORB::SubscriptionMulti<vehicle_attitude_s> _att_sub;
    uORB::SubscriptionMulti<mavlink_log_s> _log_sub;
    uORB::SubscriptionMulti<actuator_armed_s> _armed_sub;
    uORB::SubscriptionMulti<vehicle_status_s> _status_sub;
    uORB::SubscriptionMulti<input_rc_s> _rc_sub;
    
    uint64_t _last_heartbeat_us;
    uint64_t _last_imu_send_us;

    // Bộ đệm RX Ring Buffer
    uint8_t _rx_buffer[MAVLINK_RX_BUF_SIZE];
    volatile uint16_t _rx_head;
    volatile uint16_t _rx_tail;

    // Bộ đệm TX Ring Buffer
    uint8_t _tx_buffer[MAVLINK_TX_BUF_SIZE];
    volatile uint16_t _tx_head;
    volatile uint16_t _tx_tail;

    // Các hàm quản lý buffer nội bộ
    bool rx_push(uint8_t c);
    bool rx_pop(uint8_t& c);
    bool tx_push(const uint8_t* data, uint16_t len);
    void flush_tx_buffer();

    // Các hàm đóng gói và gửi
    void send_heartbeat();

    // Xử lý các bản tin đặc biệt từ QGC
    void handle_message(mavlink_message_t* msg);
    void handle_param_request_list(mavlink_message_t* msg);
    void handle_param_set(mavlink_message_t* msg);

    // Thời gian tính bằng microsecond theo chuẩn PX4
    uint64_t get_time_us();
};

extern MavlinkManager g_mavlink;
