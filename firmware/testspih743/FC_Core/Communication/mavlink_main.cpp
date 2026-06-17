#include "mavlink_main.h"
#include "usbd_cdc_if.h" // Dùng USB CDC để giao tiếp với QGroundControl
#include "../Parameters/param.h" // Để đổi PID qua MAVLink
#include "../Navigation/Navigator.h"
#include "../Main/fc_logging.h"

// Đặt toàn bộ object MavlinkManager vào vùng SRAM hỗ trợ DMA để an toàn cho USB/UART TX
__attribute__((section(".dma_buffer"))) __attribute__((aligned(32))) MavlinkManager g_mavlink;

MavlinkManager::MavlinkManager() : 
    _imu_sub(orb_sensor_imu, 0),
    _baro_sub(orb_sensor_baro, 0),
    _mag_sub(orb_sensor_mag, 0),
    _opt_sub(orb_sensor_optical_flow, 0),
    _dist_sub(orb_distance_sensor, 0),
    _gps_sub(orb_sensor_gps, 0),
    _att_sub(orb_vehicle_attitude, 0),
    _log_sub(orb_mavlink_log, 0),
    _armed_sub(orb_actuator_armed, 0),
    _status_sub(orb_vehicle_status, 0),
    _rc_sub(orb_input_rc, 0),
    _cmd_pub(orb_vehicle_command),
    _last_heartbeat_us(0),
    _last_imu_send_us(0),
    _rx_head(0), _rx_tail(0),
    _tx_head(0), _tx_tail(0),
    _incoming_mission_count(0),
    _incoming_mission_index(0)
{}

uint64_t MavlinkManager::get_time_us() {
    // Tạm thời dùng HAL_GetTick() quy đổi, tương lai nên dùng hrt_absolute_time()
    return (uint64_t)HAL_GetTick() * 1000;
}

void MavlinkManager::init() {
    // Khởi tạo các cờ, bộ đệm nếu cần
}

// ================= RING BUFFER (RX) =================

bool MavlinkManager::rx_push(uint8_t c) {
    uint16_t next = (_rx_head + 1) % MAVLINK_RX_BUF_SIZE;
    if (next == _rx_tail) return false; // Full
    _rx_buffer[_rx_head] = c;
    _rx_head = next;
    return true;
}

bool MavlinkManager::rx_pop(uint8_t& c) {
    if (_rx_head == _rx_tail) return false; // Empty
    c = _rx_buffer[_rx_tail];
    _rx_tail = (_rx_tail + 1) % MAVLINK_RX_BUF_SIZE;
    return true;
}

// Gọi hàm này bên trong CDC_Receive_FS hoặc ngắt UART Rx
// TUYỆT ĐỐI KHÔNG PARSE TRONG NÀY NỮA
void MavlinkManager::receive_char(uint8_t c) {
    rx_push(c);
}

// ================= RING BUFFER (TX) =================

bool MavlinkManager::tx_push(const uint8_t* data, uint16_t len) {
    // Kiểm tra dung lượng còn lại
    uint16_t available;
    if (_tx_head >= _tx_tail) {
        available = MAVLINK_TX_BUF_SIZE - _tx_head + _tx_tail - 1;
    } else {
        available = _tx_tail - _tx_head - 1;
    }

    if (available < len) return false; // Không đủ chỗ

    for (uint16_t i = 0; i < len; i++) {
        _tx_buffer[_tx_head] = data[i];
        _tx_head = (_tx_head + 1) % MAVLINK_TX_BUF_SIZE;
    }
    return true;
}

void MavlinkManager::flush_tx_buffer() {
    if (_tx_head == _tx_tail) return; // Nothing to send

    // Lấy trạng thái của USB để tránh gọi CDC_Transmit_FS khi nó đang BUSY
    extern USBD_HandleTypeDef hUsbDeviceFS; 
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc != nullptr && hcdc->TxState != 0) {
        return; // USB đang bận gửi, đợi lần sau
    }

    // Nếu không bận, tính toán số byte có thể gửi liền mạch (đến cuối mảng hoặc đến tail)
    uint16_t len = 0;
    uint16_t current_tail = _tx_tail; // Copy để thao tác an toàn
    
    if (_tx_head > current_tail) {
        len = _tx_head - current_tail;
    } else {
        // Gửi phần từ tail đến cuối mảng trước (wrap around)
        len = MAVLINK_TX_BUF_SIZE - current_tail;
    }

    if (len > 0) {
        if (CDC_Transmit_FS((uint8_t*)&_tx_buffer[current_tail], len) == USBD_OK) {
            // Chỉ cập nhật _tx_tail khi đã xác nhận hàm gọi thành công (đang gửi dở)
            // Ngắt TX Complete có thể gọi lại flush, nhưng ở đây thiết kế đơn giản:
            // Hàm truyền xong thì mới tính là free
            _tx_tail = (current_tail + len) % MAVLINK_TX_BUF_SIZE;
        }
    }
}

// Định nghĩa Custom Mode theo chuẩn PX4
union px4_custom_mode {
    struct {
        uint16_t reserved;
        uint8_t main_mode;
        uint8_t sub_mode;
    };
    uint32_t data;
    float data_float;
};

enum PX4_CUSTOM_MAIN_MODE {
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
    PX4_CUSTOM_MAIN_MODE_ALTCTL,
    PX4_CUSTOM_MAIN_MODE_POSCTL,
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_ACRO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_CUSTOM_MAIN_MODE_STABILIZED,
};

enum PX4_CUSTOM_SUB_MODE_AUTO {
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3,
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6,
};

// ================= MAVLINK SENDER =================

void MavlinkManager::send_heartbeat() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Lấy trạng thái hiện tại
    actuator_armed_s armed;
    vehicle_status_s status;
    _armed_sub.copy(armed);
    _status_sub.copy(status);

    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (armed.armed) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    
    uint8_t system_status = MAV_STATE_STANDBY;
    if (armed.armed) {
        system_status = MAV_STATE_ACTIVE;
    }
    if (status.failsafe) {
        system_status = MAV_STATE_CRITICAL;
    }

    union px4_custom_mode custom_mode;
    custom_mode.data = 0;

    switch (status.nav_state) {
        case NAVIGATION_STATE_MANUAL:
            custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
            break;
        case NAVIGATION_STATE_ALTCTL:
            custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
            break;
        case NAVIGATION_STATE_POSCTL:
            custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
            break;
        case NAVIGATION_STATE_ACRO:
            custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
            break;
        case NAVIGATION_STATE_AUTO_TAKEOFF:
            custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
            break;
        case NAVIGATION_STATE_AUTO_MISSION:
            custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
            break;
        case NAVIGATION_STATE_AUTO_RTL:
            custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
            break;
        case NAVIGATION_STATE_AUTO_LAND:
            custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
            break;
        case NAVIGATION_STATE_STAB:
        default:
            custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
            break;
    }

    // Gửi Heartbeat: Máy bay Quadrotor, Autopilot PX4 (MAV_AUTOPILOT_PX4)
    mavlink_msg_heartbeat_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, 
                               MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 
                               base_mode, custom_mode.data, system_status);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    tx_push(buf, len); // Đẩy vào RingBuffer thay vì gửi trực tiếp
}

void MavlinkManager::send_raw_imu(const sensor_imu_s& imu_data, const sensor_mag_s& mag_data) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_raw_imu_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                             imu_data.timestamp_us,
                             (int16_t)(imu_data.accel_m_s2[0] * 1000.0f),
                             (int16_t)(imu_data.accel_m_s2[1] * 1000.0f),
                             (int16_t)(imu_data.accel_m_s2[2] * 1000.0f),
                             (int16_t)(imu_data.gyro_rad_s[0] * 1000.0f),
                             (int16_t)(imu_data.gyro_rad_s[1] * 1000.0f),
                             (int16_t)(imu_data.gyro_rad_s[2] * 1000.0f),
                             (int16_t)(mag_data.mag_x_ut * 10.0f), // RAW_IMU mag tính bằng milliGauss (1uT = 10mG)
                             (int16_t)(mag_data.mag_y_ut * 10.0f),
                             (int16_t)(mag_data.mag_z_ut * 10.0f),
                             0,       // id (IMU instance)
                             (int16_t)(imu_data.temperature_c * 100.0f));

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    tx_push(buf, len);
}

void MavlinkManager::send_scaled_pressure(const sensor_baro_s& baro_data) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_scaled_pressure_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                                     (uint32_t)(baro_data.timestamp_us / 1000), // time_boot_ms
                                     baro_data.pressure_mbar, // press_abs (hPa = mbar)
                                     0.0f, // press_diff
                                     (int16_t)(baro_data.temperature_c * 100.0f), // temperature (cdegC)
                                     0); // temperature_press_diff (cdegC)
                                     
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    tx_push(buf, len);
}

void MavlinkManager::send_optical_flow(const sensor_optical_flow_s& opt_data) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_optical_flow_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                                  opt_data.timestamp_us, opt_data.device_id,
                                  opt_data.pixel_flow_x_integral, opt_data.pixel_flow_y_integral,
                                  0.0f, 0.0f, opt_data.quality, 0.0f, 0.0f, 0.0f);
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    tx_push(buf, len);
}

void MavlinkManager::send_distance_sensor(const distance_sensor_s& dist_data) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    const float q[4] = {0, 0, 0, 0};
    mavlink_msg_distance_sensor_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                                     (uint32_t)(dist_data.timestamp_us / 1000), 
                                     (uint16_t)(dist_data.min_distance_m * 100.0f), 
                                     (uint16_t)(dist_data.max_distance_m * 100.0f), 
                                     (uint16_t)(dist_data.current_distance_m * 100.0f), 
                                     dist_data.type, dist_data.device_id, 25 /* MAV_SENSOR_ROTATION_PITCH_270 (Down) */, 
                                     (uint8_t)dist_data.variance, 0.0f, 0.0f, q, 0);
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    tx_push(buf, len);
}

void MavlinkManager::send_gps(const sensor_gps_s& gps_data) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_gps_raw_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                                 gps_data.timestamp_us, gps_data.fix_type,
                                 gps_data.lat, gps_data.lon, gps_data.alt,
                                 (uint16_t)(gps_data.hdop * 100.0f), (uint16_t)(gps_data.vdop * 100.0f),
                                 (uint16_t)(gps_data.vel_m_s * 100.0f), (uint16_t)(gps_data.cog_rad * 18000.0f / 3.14159f),
                                 gps_data.satellites_used, 
                                 0, 0, 0, 0, 0, 0);
                                 
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    tx_push(buf, len);
}

void MavlinkManager::send_attitude(const vehicle_attitude_s& att_data) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_attitude_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                              (uint32_t)(att_data.timestamp_us / 1000), // time_boot_ms
                              att_data.roll_rad,
                              att_data.pitch_rad,
                              att_data.yaw_rad,
                              att_data.rollspeed_rad_s,
                              att_data.pitchspeed_rad_s,
                              att_data.yawspeed_rad_s);
                              
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    tx_push(buf, len);
}

void MavlinkManager::send_rc_channels(const input_rc_s& rc_data) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_rc_channels_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                                 (uint32_t)(rc_data.timestamp / 1000),
                                 rc_data.channel_count,
                                 rc_data.values[0], rc_data.values[1], rc_data.values[2], rc_data.values[3],
                                 rc_data.values[4], rc_data.values[5], rc_data.values[6], rc_data.values[7],
                                 rc_data.values[8], rc_data.values[9], rc_data.values[10], rc_data.values[11],
                                 rc_data.values[12], rc_data.values[13], rc_data.values[14], rc_data.values[15],
                                 rc_data.values[16], rc_data.values[17],
                                 rc_data.rssi);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    tx_push(buf, len);
}

void MavlinkManager::send_log(const mavlink_log_s& log_data) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_statustext_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                                log_data.severity,
                                log_data.text,
                                0, 0); // MAVLink 2 fields

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    tx_push(buf, len);
}

// ================= MAIN LOOP & HANDLER =================

void MavlinkManager::run() {
    uint64_t now = get_time_us();

    // 1. Nhận data từ RX Buffer và parse
    uint8_t c;
    mavlink_message_t msg;
    mavlink_status_t status;
    while (rx_pop(c)) {
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            handle_message(&msg);
        }
    }

    // 2. Phát Heartbeat ở tần số 1Hz
    if (now - _last_heartbeat_us >= 1000000) {
        _last_heartbeat_us = now;
        send_heartbeat();
    }

    // 3. Đẩy dữ liệu IMU / Cảm biến (Khoảng 50Hz)
    if (now - _last_imu_send_us >= 20000) { // 50Hz
        sensor_imu_s imu_data = {0};
        sensor_mag_s mag_data = {0};
        sensor_baro_s baro_data = {0};
        sensor_optical_flow_s opt_data = {0};
        distance_sensor_s dist_data = {0};
        sensor_gps_s gps_data = {0};
        vehicle_attitude_s att_data = {0};
        input_rc_s rc_data = {0};

        // Lấy data mới nhất (kể cả cũ thì copy() vẫn lấy giá trị an toàn)
        _imu_sub.copy(imu_data);
        _mag_sub.copy(mag_data);
        _baro_sub.copy(baro_data);
        
        send_raw_imu(imu_data, mag_data);
        send_scaled_pressure(baro_data);

        if (_opt_sub.update(opt_data)) send_optical_flow(opt_data);
        if (_dist_sub.update(dist_data)) send_distance_sensor(dist_data);
        if (_gps_sub.update(gps_data)) send_gps(gps_data);
        if (_att_sub.update(att_data)) send_attitude(att_data);
        if (_rc_sub.update(rc_data)) send_rc_channels(rc_data);

        _last_imu_send_us = now;
    }
    
    // 3.5. Kiểm tra và gửi Log Text (Không bị giới hạn rate)
    // Tạm thời vì chưa xong uORB_lite, ta đọc trực tiếp biến toàn cục
    if (g_mavlink_log_data.timestamp > 0) {
        send_log(g_mavlink_log_data);
        g_mavlink_log_data.timestamp = 0; // Đã gửi xong
    }

    // 4. Xả TX Buffer ra phần cứng (nếu không bận)
    flush_tx_buffer();
}

void MavlinkManager::handle_message(mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            handle_param_request_list(msg);
            break;
        case MAVLINK_MSG_ID_PARAM_SET:
            handle_param_set(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_COUNT:
            handle_mission_count(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
            handle_mission_item_int(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM:
            handle_mission_item(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
            handle_mission_request_list(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
            handle_mission_request_int(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST:
            handle_mission_request(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
            handle_mission_clear_all(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
            handle_mission_set_current(msg);
            break;
        case MAVLINK_MSG_ID_SET_MODE:
            handle_set_mode(msg);
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handle_command_long(msg);
            break;
        default:
            break;
    }
}

void MavlinkManager::handle_param_request_list(mavlink_message_t* msg) {
    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        mavlink_message_t tx_msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        union param_value_u val;
        param_get(i, &val);

        float param_val_f = 0.0f;
        uint8_t mav_type = MAV_PARAM_TYPE_REAL32;

        if (param_type(i) == PARAM_TYPE_INT32) {
            param_val_f = (float)val.i;
            mav_type = MAV_PARAM_TYPE_INT32;
        } else if (param_type(i) == PARAM_TYPE_FLOAT) {
            param_val_f = val.f;
            mav_type = MAV_PARAM_TYPE_REAL32;
        }

        char name_buf[16] = {0};
        strncpy(name_buf, param_name(i), 16);

        mavlink_msg_param_value_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                     name_buf, param_val_f, mav_type, PARAM_COUNT, i);

        uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
        tx_push(buf, len);
    }
}

void MavlinkManager::handle_param_set(mavlink_message_t* msg) {
    mavlink_param_set_t set_packet;
    mavlink_msg_param_set_decode(msg, &set_packet);

    if (set_packet.target_system == MAVLINK_SYSTEM_ID && set_packet.target_component == MAVLINK_COMPONENT_ID) {
        char param_id[16] = {0};
        strncpy(param_id, set_packet.param_id, 16);

        param_t param = param_find(param_id);
        if (param != PARAM_INVALID) {
            union param_value_u val;
            if (param_type(param) == PARAM_TYPE_INT32) {
                val.i = (int32_t)set_packet.param_value;
                param_set(param, &val.i);
            } else if (param_type(param) == PARAM_TYPE_FLOAT) {
                val.f = set_packet.param_value;
                param_set(param, &val.f);
            }

            // Gửi lại PARAM_VALUE xác nhận thay đổi thành công cho QGC
            mavlink_message_t tx_msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];

            float param_val_f = 0.0f;
            uint8_t mav_type = MAV_PARAM_TYPE_REAL32;
            if (param_type(param) == PARAM_TYPE_INT32) {
                param_val_f = (float)val.i;
                mav_type = MAV_PARAM_TYPE_INT32;
            } else if (param_type(param) == PARAM_TYPE_FLOAT) {
                param_val_f = val.f;
                mav_type = MAV_PARAM_TYPE_REAL32;
            }

            mavlink_msg_param_value_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                         param_id, param_val_f, mav_type, PARAM_COUNT, param);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
            tx_push(buf, len);

            // Lưu trực tiếp xuống bộ nhớ Flash
            param_save_default();
        }
    }
}

void MavlinkManager::handle_set_mode(mavlink_message_t* msg) {
    mavlink_set_mode_t set_mode;
    mavlink_msg_set_mode_decode(msg, &set_mode);

    if (set_mode.target_system == MAVLINK_SYSTEM_ID) {
        union px4_custom_mode custom_mode;
        custom_mode.data = set_mode.custom_mode;

        vehicle_command_s cmd_msg{};
        cmd_msg.timestamp = get_time_us();
        cmd_msg.command = 176; // MAV_CMD_DO_SET_MODE
        cmd_msg.param1 = (float)set_mode.base_mode;
        cmd_msg.param2 = (float)custom_mode.main_mode;
        cmd_msg.param3 = (float)custom_mode.sub_mode;
        _cmd_pub.publish(cmd_msg);
    }
}

void MavlinkManager::handle_command_long(mavlink_message_t* msg) {
    mavlink_command_long_t cmd_long;
    mavlink_msg_command_long_decode(msg, &cmd_long);

    if (cmd_long.target_system == MAVLINK_SYSTEM_ID) {
        vehicle_command_s cmd_msg{};
        cmd_msg.timestamp = get_time_us();
        cmd_msg.command = cmd_long.command;
        cmd_msg.param1 = cmd_long.param1;
        cmd_msg.param2 = cmd_long.param2;
        cmd_msg.param3 = cmd_long.param3;
        cmd_msg.param4 = cmd_long.param4;
        cmd_msg.param5 = cmd_long.param5;
        cmd_msg.param6 = cmd_long.param6;
        cmd_msg.param7 = cmd_long.param7;
        _cmd_pub.publish(cmd_msg);

        // Gửi COMMAND_ACK về cho GCS
        mavlink_message_t ack_msg;
        uint8_t ack_buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_command_ack_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &ack_msg,
                                     cmd_long.command, MAV_RESULT_ACCEPTED, 0, 0,
                                     msg->sysid, msg->compid);
        uint16_t ack_len = mavlink_msg_to_send_buffer(ack_buf, &ack_msg);
        tx_push(ack_buf, ack_len);
    }
}

// ================= GIAO THỨC MAVLINK MISSION =================

void MavlinkManager::handle_mission_count(mavlink_message_t* msg) {
    mavlink_mission_count_t count_packet;
    mavlink_msg_mission_count_decode(msg, &count_packet);

    if (count_packet.target_system == MAVLINK_SYSTEM_ID && count_packet.target_component == MAVLINK_COMPONENT_ID) {
        g_navigator.clear_waypoints();
        _incoming_mission_count = count_packet.count;
        _incoming_mission_index = 0;

        FC_INFO("[Mavlink] Upload started: expecting %d waypoints", _incoming_mission_count);

        if (_incoming_mission_count > 0) {
            // Yêu cầu waypoint đầu tiên (seq = 0)
            mavlink_message_t tx_msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_mission_request_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                                 msg->sysid, msg->compid, 0, MAV_MISSION_TYPE_MISSION);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
            tx_push(buf, len);
        } else {
            // Acknowledge rỗng
            mavlink_message_t tx_msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_mission_ack_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                         msg->sysid, msg->compid, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION, 0);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
            tx_push(buf, len);
        }
    }
}

void MavlinkManager::handle_mission_item_int(mavlink_message_t* msg) {
    mavlink_mission_item_int_t item;
    mavlink_msg_mission_item_int_decode(msg, &item);

    if (item.target_system == MAVLINK_SYSTEM_ID && item.target_component == MAVLINK_COMPONENT_ID) {
        if (item.seq == _incoming_mission_index && _incoming_mission_index < _incoming_mission_count) {
            position_setpoint_s wp{};
            wp.valid = true;
            wp.type = SETPOINT_TYPE_POSITION;

            // Hỗ trợ cả FRAME_LOCAL_NED và GLOBAL_RELATIVE_ALT
            if (item.frame == MAV_FRAME_LOCAL_NED) {
                wp.x = (float)item.x; // local X trong mét
                wp.y = (float)item.y; // local Y trong mét
                wp.z = (float)item.z; // local Z trong mét (NED âm)
            } else {
                // Mặc định quy đổi Flat Earth từ GPS
                sensor_gps_s gps{};
                _gps_sub.copy(gps);

                float home_lat = (gps.fix_type >= 3) ? (float)gps.lat : 0.0f;
                float home_lon = (gps.fix_type >= 3) ? (float)gps.lon : 0.0f;
                float home_alt = (gps.fix_type >= 3) ? (float)gps.alt * 0.001f : 0.0f;

                float lat_val = (float)item.x;
                float lon_val = (float)item.y;

                wp.x = (lat_val - home_lat) * 1e-7f * 111132.954f;
                wp.y = (lon_val - home_lon) * 1e-7f * 111132.954f * cosf(home_lat * 1e-7f * 0.0174532925f);
                
                if (item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT) {
                    wp.z = -item.z; // Z âm đi lên
                } else {
                    wp.z = -(item.z - home_alt);
                }
            }
            
            wp.yaw = item.param4; // Yaw ở góc radian
            g_navigator.add_waypoint(wp);
            _incoming_mission_index++;

            if (_incoming_mission_index < _incoming_mission_count) {
                // Yêu cầu item tiếp theo
                mavlink_message_t tx_msg;
                uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                mavlink_msg_mission_request_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                                     msg->sysid, msg->compid, _incoming_mission_index, MAV_MISSION_TYPE_MISSION);
                uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
                tx_push(buf, len);
            } else {
                // Hoàn thành sứ mệnh nạp
                g_navigator.save_mission_to_flash();
                mavlink_message_t tx_msg;
                uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                mavlink_msg_mission_ack_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                             msg->sysid, msg->compid, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION, 0);
                uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
                tx_push(buf, len);
                FC_INFO("[Mavlink] Mission upload complete: %d waypoints saved", _incoming_mission_count);
            }
        }
    }
}

void MavlinkManager::handle_mission_item(mavlink_message_t* msg) {
    mavlink_mission_item_t item;
    mavlink_msg_mission_item_decode(msg, &item);

    if (item.target_system == MAVLINK_SYSTEM_ID && item.target_component == MAVLINK_COMPONENT_ID) {
        if (item.seq == _incoming_mission_index && _incoming_mission_index < _incoming_mission_count) {
            position_setpoint_s wp{};
            wp.valid = true;
            wp.type = SETPOINT_TYPE_POSITION;

            if (item.frame == MAV_FRAME_LOCAL_NED) {
                wp.x = item.x;
                wp.y = item.y;
                wp.z = item.z;
            } else {
                sensor_gps_s gps{};
                _gps_sub.copy(gps);

                float home_lat = (gps.fix_type >= 3) ? (float)gps.lat : 0.0f;
                float home_lon = (gps.fix_type >= 3) ? (float)gps.lon : 0.0f;
                float home_alt = (gps.fix_type >= 3) ? (float)gps.alt * 0.001f : 0.0f;

                wp.x = (item.x - home_lat * 1e-7f) * 111132.954f;
                wp.y = (item.y - home_lon * 1e-7f) * 111132.954f * cosf(home_lat * 1e-7f * 0.0174532925f);
                
                if (item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT) {
                    wp.z = -item.z;
                } else {
                    wp.z = -(item.z - home_alt);
                }
            }
            
            wp.yaw = item.param4;
            g_navigator.add_waypoint(wp);
            _incoming_mission_index++;

            if (_incoming_mission_index < _incoming_mission_count) {
                mavlink_message_t tx_msg;
                uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                mavlink_msg_mission_request_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                                 msg->sysid, msg->compid, _incoming_mission_index, MAV_MISSION_TYPE_MISSION);
                uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
                tx_push(buf, len);
            } else {
                g_navigator.save_mission_to_flash();
                mavlink_message_t tx_msg;
                uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                mavlink_msg_mission_ack_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                             msg->sysid, msg->compid, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION, 0);
                uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
                tx_push(buf, len);
                FC_INFO("[Mavlink] Mission upload complete (float format): %d waypoints saved", _incoming_mission_count);
            }
        }
    }
}

void MavlinkManager::handle_mission_request_list(mavlink_message_t* msg) {
    mavlink_mission_request_list_t request;
    mavlink_msg_mission_request_list_decode(msg, &request);

    if (request.target_system == MAVLINK_SYSTEM_ID && request.target_component == MAVLINK_COMPONENT_ID) {
        uint8_t count = g_navigator.get_waypoint_count();
        mavlink_message_t tx_msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_mission_count_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                       msg->sysid, msg->compid, count, MAV_MISSION_TYPE_MISSION, 0);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
        tx_push(buf, len);
    }
}

void MavlinkManager::handle_mission_request_int(mavlink_message_t* msg) {
    mavlink_mission_request_int_t request;
    mavlink_msg_mission_request_int_decode(msg, &request);

    if (request.target_system == MAVLINK_SYSTEM_ID && request.target_component == MAVLINK_COMPONENT_ID) {
        uint8_t count = g_navigator.get_waypoint_count();
        if (request.seq < count) {
            position_setpoint_s wp = g_navigator.get_waypoint(request.seq);

            sensor_gps_s gps{};
            _gps_sub.copy(gps);

            float home_lat = (gps.fix_type >= 3) ? (float)gps.lat : 0.0f;
            float home_lon = (gps.fix_type >= 3) ? (float)gps.lon : 0.0f;

            // Flat earth inverse
            int32_t lat_val = (int32_t)(home_lat + (wp.x / 111132.954f) * 1e7f);
            int32_t lon_val = (int32_t)(home_lon + (wp.y / (111132.954f * cosf(home_lat * 1e-7f * 0.0174532925f))) * 1e7f);
            float alt_val = -wp.z; // relative to Home

            mavlink_message_t tx_msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_mission_item_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                              msg->sysid, msg->compid, request.seq,
                                              MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT,
                                              0, 1, 0.0f, 1.0f, 0.0f, wp.yaw,
                                              lat_val, lon_val, alt_val, MAV_MISSION_TYPE_MISSION);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
            tx_push(buf, len);
        }
    }
}

void MavlinkManager::handle_mission_request(mavlink_message_t* msg) {
    mavlink_mission_request_t request;
    mavlink_msg_mission_request_decode(msg, &request);

    if (request.target_system == MAVLINK_SYSTEM_ID && request.target_component == MAVLINK_COMPONENT_ID) {
        uint8_t count = g_navigator.get_waypoint_count();
        if (request.seq < count) {
            position_setpoint_s wp = g_navigator.get_waypoint(request.seq);

            sensor_gps_s gps{};
            _gps_sub.copy(gps);

            float home_lat = (gps.fix_type >= 3) ? (float)gps.lat : 0.0f;
            float home_lon = (gps.fix_type >= 3) ? (float)gps.lon : 0.0f;

            float lat_val = home_lat * 1e-7f + (wp.x / 111132.954f);
            float lon_val = home_lon * 1e-7f + (wp.y / (111132.954f * cosf(home_lat * 1e-7f * 0.0174532925f)));
            float alt_val = -wp.z;

            mavlink_message_t tx_msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_mission_item_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                          msg->sysid, msg->compid, request.seq,
                                          MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT,
                                          0, 1, 0.0f, 1.0f, 0.0f, wp.yaw,
                                          lat_val, lon_val, alt_val, MAV_MISSION_TYPE_MISSION);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
            tx_push(buf, len);
        }
    }
}

void MavlinkManager::handle_mission_clear_all(mavlink_message_t* msg) {
    mavlink_mission_clear_all_t request;
    mavlink_msg_mission_clear_all_decode(msg, &request);

    if (request.target_system == MAVLINK_SYSTEM_ID && request.target_component == MAVLINK_COMPONENT_ID) {
        g_navigator.clear_waypoints();
        g_navigator.save_mission_to_flash();
        FC_INFO("[Mavlink] All waypoints cleared and saved to Flash");

        mavlink_message_t tx_msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_mission_ack_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                     msg->sysid, msg->compid, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION, 0);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
        tx_push(buf, len);
    }
}

void MavlinkManager::handle_mission_set_current(mavlink_message_t* msg) {
    mavlink_mission_set_current_t request;
    mavlink_msg_mission_set_current_decode(msg, &request);

    if (request.target_system == MAVLINK_SYSTEM_ID && request.target_component == MAVLINK_COMPONENT_ID) {
        mavlink_message_t tx_msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_mission_current_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &tx_msg,
                                         request.seq, 0, 0, 0, 0, 0, 0);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &tx_msg);
        tx_push(buf, len);
    }
}

