#pragma once
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "../../MessageBus/topics/input_rc.h"

// Giao thức CRSF
#define CRSF_SYNC_BYTE          0xC8
#define CRSF_FRAMETYPE_RC       0x16
#define CRSF_FRAMETYPE_LINK     0x14
#define CRSF_FRAMETYPE_BATTERY  0x08
#define CRSF_FRAMETYPE_ATTITUDE 0x1E

// Cấu trúc Link Statistics (Frame 0x14)
typedef struct {
    uint8_t uplink_rssi_1;
    uint8_t uplink_rssi_2;
    uint8_t uplink_lq;
    int8_t  uplink_snr;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    uint8_t downlink_rssi;
    uint8_t downlink_lq;
    int8_t  downlink_snr;
} crsf_link_statistics_t;

enum CRSF_State {
    CRSF_STATE_SYNC = 0,
    CRSF_STATE_LEN,
    CRSF_STATE_TYPE,
    CRSF_STATE_PAYLOAD,
    CRSF_STATE_CRC
};

class CRSFParser {
public:
    CRSFParser(UART_HandleTypeDef* huart);
    ~CRSFParser() = default;

    void init();
    void update(); // Gọi trong RTOS task (ví dụ 100Hz)
    
    // Gửi Telemetry ngược về tay cầm (TX)
    void send_telemetry_battery(uint16_t voltage_0_1v, uint16_t current_0_1a, uint32_t capacity_mah, uint8_t remaining_pct);
    void send_telemetry_attitude(int16_t pitch_rad_10000, int16_t roll_rad_10000, int16_t yaw_rad_10000);

    // Gọi trong USART6 IDLE Interrupt
    void handle_idle_interrupt();

private:
    UART_HandleTypeDef* _huart;
    
    uint8_t _rx_buffer[256];
    uint16_t _last_dma_pos;
    
    crsf_link_statistics_t _link_stats;

    // State machine variables
    CRSF_State _state;
    uint8_t _frame_len;
    uint8_t _frame_type;
    uint8_t _payload_len;
    uint8_t _payload_idx;
    uint8_t _payload_buf[64];

    // uORB Publisher
    uORB::PublicationMulti<input_rc_s> _rc_pub;

    void parse_byte(uint8_t b);
    void unpack_channels(uint8_t* payload);
    void parse_link_statistics(uint8_t* payload);
    void check_failsafe();
    
    uint8_t calc_crc8(uint8_t *data, uint8_t len);
    void send_frame(uint8_t type, uint8_t* payload, uint8_t len);
};

extern CRSFParser g_crsf;
