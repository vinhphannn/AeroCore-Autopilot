#include "crsf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "../../Main/fc_logging.h"

extern UART_HandleTypeDef huart6;

CRSFParser g_crsf(&huart6);
input_rc_s g_input_rc_data;

CRSFParser::CRSFParser(UART_HandleTypeDef* huart) : _huart(huart), _last_dma_pos(0), _state(CRSF_STATE_SYNC), _rc_pub(orb_input_rc) {
    g_input_rc_data.channel_count = 16;
    g_input_rc_data.rc_lost = 1;
    g_input_rc_data.rc_failsafe = 1;
}

void CRSFParser::init() {
    // Kích hoạt ngắt IDLE Line để phát hiện khoảng nghỉ giữa các gói tin
    __HAL_UART_ENABLE_IT(_huart, UART_IT_IDLE);
    
    // Kích hoạt DMA nhận xoay vòng liên tục
    HAL_UART_Receive_DMA(_huart, _rx_buffer, sizeof(_rx_buffer));
}

void CRSFParser::update() {
    // Tìm vị trí ghi hiện tại của DMA
    uint16_t curr_dma_pos = sizeof(_rx_buffer) - __HAL_DMA_GET_COUNTER(_huart->hdmarx);
    
    // Đọc dần dần từng byte từ last_pos đến curr_pos
    while (_last_dma_pos != curr_dma_pos) {
        parse_byte(_rx_buffer[_last_dma_pos]);
        
        _last_dma_pos++;
        if (_last_dma_pos >= sizeof(_rx_buffer)) {
            _last_dma_pos = 0;
        }
    }
    
    // Kiểm tra Timeout / Failsafe (Giống PX4)
    check_failsafe();
}

void CRSFParser::handle_idle_interrupt() {
    // Ngắt này sẽ tự động được gọi trong UART ISR, giúp báo hiệu có packet mới ngay lập tức
    if (__HAL_UART_GET_FLAG(_huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(_huart);
        // Có thể gọi BaseType_t xHigherPriorityTaskWoken và xTaskNotifyFromISR tại đây
        // nếu muốn đánh thức task RC ngay lập tức thay vì chờ poll 100Hz.
    }
}

void CRSFParser::parse_byte(uint8_t b) {
    switch (_state) {
        case CRSF_STATE_SYNC:
            if (b == CRSF_SYNC_BYTE) { // 0xC8
                _state = CRSF_STATE_LEN;
            }
            break;
            
        case CRSF_STATE_LEN:
            _frame_len = b;
            if (_frame_len > 64 || _frame_len < 2) { 
                _state = CRSF_STATE_SYNC;
            } else {
                _payload_len = _frame_len - 2; // Loại trừ Type và CRC
                _payload_idx = 0;
                _state = CRSF_STATE_TYPE;
            }
            break;
            
        case CRSF_STATE_TYPE:
            _frame_type = b;
            _payload_buf[_payload_idx++] = b; // Gộp chung vào mảng payload để dễ tính CRC
            _state = CRSF_STATE_PAYLOAD;
            break;
            
        case CRSF_STATE_PAYLOAD:
            _payload_buf[_payload_idx++] = b;
            if (_payload_idx == _payload_len + 1) { // Lấy đủ payload (tính cả Type)
                _state = CRSF_STATE_CRC;
            }
            break;
            
        case CRSF_STATE_CRC:
            uint8_t crc = calc_crc8(_payload_buf, _payload_len + 1);
            if (crc == b) { // CRC đúng
                if (_frame_type == CRSF_FRAMETYPE_RC) {
                    unpack_channels(&_payload_buf[1]); // Dữ liệu kênh bắt đầu sau byte Type
                } else if (_frame_type == CRSF_FRAMETYPE_LINK) {
                    parse_link_statistics(&_payload_buf[1]);
                }
            }
            _state = CRSF_STATE_SYNC; // Xong một gói
            break;
    }
}

// Bóc tách 16 kênh RC (11 bit / kênh)
void CRSFParser::unpack_channels(uint8_t* payload) {
    uint32_t bits = 0;
    uint8_t bits_available = 0;
    uint8_t payload_idx = 0;

    for (int i = 0; i < 16; i++) {
        while (bits_available < 11) {
            bits |= (uint32_t)payload[payload_idx++] << bits_available;
            bits_available += 8;
        }
        
        uint16_t val = bits & 0x07FF; // Lấy 11 bit thấp nhất
        
        // Scale từ CRSF (172 - 1811) sang tiêu chuẩn (1000 - 2000 us)
        g_input_rc_data.values[i] = ((val - 172) * 1000 / 1639) + 1000; 
        
        bits >>= 11;
        bits_available -= 11;
    }
    
    if (g_input_rc_data.rc_failsafe) {
        FC_INFO("CRSF Signal Recovered!");
    }
    
    g_input_rc_data.timestamp = xTaskGetTickCount();
    g_input_rc_data.rc_lost = 0;
    g_input_rc_data.rc_failsafe = 0;
    
    _rc_pub.publish(g_input_rc_data);
}

// Bóc tách Link Statistics (Giống PX4)
void CRSFParser::parse_link_statistics(uint8_t* payload) {
    _link_stats.uplink_rssi_1   = payload[0];
    _link_stats.uplink_rssi_2   = payload[1];
    _link_stats.uplink_lq       = payload[2];
    _link_stats.uplink_snr      = (int8_t)payload[3];
    _link_stats.active_antenna  = payload[4];
    _link_stats.rf_mode         = payload[5];
    _link_stats.uplink_tx_power = payload[6];
    _link_stats.downlink_rssi   = payload[7];
    _link_stats.downlink_lq     = payload[8];
    _link_stats.downlink_snr    = (int8_t)payload[9];

    // Lấy RSSI mạnh nhất gán vào input_rc_s
    uint8_t best_rssi = (_link_stats.uplink_rssi_1 < _link_stats.uplink_rssi_2) ? _link_stats.uplink_rssi_1 : _link_stats.uplink_rssi_2;
    g_input_rc_data.rssi = -(int32_t)best_rssi;
}

// Kiểm tra Failsafe (Giống PX4)
void CRSFParser::check_failsafe() {
    uint32_t now = xTaskGetTickCount();
    if (now - g_input_rc_data.timestamp > pdMS_TO_TICKS(100)) { // 100ms không có frame RC
        if (g_input_rc_data.rc_failsafe == 0) {
            FC_ERR("CRSF Signal Lost! Failsafe Triggered!");
        }
        g_input_rc_data.rc_lost = 1;
        g_input_rc_data.rc_failsafe = 1;
        _rc_pub.publish(g_input_rc_data);
    }
}

// Hàm gửi Frame Telemetry chung
void CRSFParser::send_frame(uint8_t type, uint8_t* payload, uint8_t len) {
    uint8_t tx_buf[64];
    tx_buf[0] = CRSF_SYNC_BYTE;
    tx_buf[1] = len + 2; // Type + Payload + CRC
    tx_buf[2] = type;
    
    for(int i = 0; i < len; i++) {
        tx_buf[3 + i] = payload[i];
    }
    
    // Tính CRC bao gồm cả Type và Payload
    tx_buf[3 + len] = calc_crc8(&tx_buf[2], len + 1);
    
    // Bắn qua UART (Nên dùng DMA TX nếu rảnh)
    HAL_UART_Transmit(_huart, tx_buf, len + 4, 10);
}

// Bắn Battery Telemetry (Mã 0x08)
void CRSFParser::send_telemetry_battery(uint16_t voltage_0_1v, uint16_t current_0_1a, uint32_t capacity_mah, uint8_t remaining_pct) {
    uint8_t payload[8];
    payload[0] = (voltage_0_1v >> 8) & 0xFF;
    payload[1] = voltage_0_1v & 0xFF;
    payload[2] = (current_0_1a >> 8) & 0xFF;
    payload[3] = current_0_1a & 0xFF;
    payload[4] = (capacity_mah >> 16) & 0xFF;
    payload[5] = (capacity_mah >> 8) & 0xFF;
    payload[6] = capacity_mah & 0xFF;
    payload[7] = remaining_pct;
    
    send_frame(CRSF_FRAMETYPE_BATTERY, payload, 8);
}

// Bắn Attitude Telemetry (Mã 0x1E)
void CRSFParser::send_telemetry_attitude(int16_t pitch_rad_10000, int16_t roll_rad_10000, int16_t yaw_rad_10000) {
    uint8_t payload[6];
    payload[0] = (pitch_rad_10000 >> 8) & 0xFF;
    payload[1] = pitch_rad_10000 & 0xFF;
    payload[2] = (roll_rad_10000 >> 8) & 0xFF;
    payload[3] = roll_rad_10000 & 0xFF;
    payload[4] = (yaw_rad_10000 >> 8) & 0xFF;
    payload[5] = yaw_rad_10000 & 0xFF;
    
    send_frame(CRSF_FRAMETYPE_ATTITUDE, payload, 6);
}

// Hàm tính CRC8 chuẩn của chuẩn Crossfire
uint8_t CRSFParser::calc_crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
