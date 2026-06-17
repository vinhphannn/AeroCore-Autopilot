#include "Commander.h"
#include "stm32h7xx_hal.h"
#include "../Main/fc_logging.h"
#include <math.h>

Commander g_commander;

Commander::Commander() : 
    _status_pub(orb_vehicle_status),
    _armed_pub(orb_actuator_armed),
    _manual_pub(orb_manual_control_setpoint),
    _local_pos_sub(orb_vehicle_local_position, 0),
    _att_sp_sub(orb_vehicle_attitude_setpoint, 0),
    _dist_sensor_sub(orb_distance_sensor, 0),
    _cmd_sub(orb_vehicle_command, 0),
    _last_update_us(0),
    _land_detect_counter(0),
    _takeoff_complete(false),
    _last_rc_mode_state(NAVIGATION_STATE_MANUAL),
    _rc_mode_initialized(false)
{
    _status.nav_state = NAVIGATION_STATE_MANUAL;
    _status.arming_state = ARMING_STATE_INIT;
    _status.failsafe = false;
    _status.rc_signal_lost = true;

    _armed.armed = false;
    _armed.prearmed = false;
    _armed.force_failsafe = false;

    _manual_sp.roll = 0.0f;
    _manual_sp.pitch = 0.0f;
    _manual_sp.yaw = 0.0f;
    _manual_sp.throttle = 0.0f;
}

uint64_t Commander::get_time_us() {
    return (uint64_t)HAL_GetTick() * 1000;
}

void Commander::init() {
    set_arming_state(ARMING_STATE_STANDBY);
}

void Commander::set_arming_state(uint8_t new_state) {
    if (_status.arming_state == new_state) return;

    if (new_state == ARMING_STATE_ARMED) {
        _armed.armed = true;
        FC_INFO(">>> ARMED <<<");
    } else if (new_state == ARMING_STATE_STANDBY) {
        _armed.armed = false;
        _takeoff_complete = false;
        FC_INFO(">>> DISARMED <<<");
    }

    _status.arming_state = new_state;
    _status.timestamp = get_time_us();
    _armed.timestamp = _status.timestamp;

    _status_pub.publish(_status);
    _armed_pub.publish(_armed);
}

// Map 1000-2000us thành [-1.0, 1.0] với Deadband ở giữa
static float apply_rc_deadband(uint16_t pwm, uint16_t mid, uint16_t deadband) {
    if (pwm > mid - deadband && pwm < mid + deadband) return 0.0f;
    if (pwm >= mid + deadband) {
        return (float)(pwm - (mid + deadband)) / (2000.0f - (mid + deadband));
    } else {
        return (float)(pwm - (mid - deadband)) / ((mid - deadband) - 1000.0f); // Trả về âm
    }
}

void Commander::map_rc_to_manual_control() {
    if (g_input_rc_data.rc_lost || g_input_rc_data.rc_failsafe) {
        // Failsafe: Trả mọi thứ về an toàn
        _manual_sp.roll = 0.0f;
        _manual_sp.pitch = 0.0f;
        _manual_sp.yaw = 0.0f;
        _manual_sp.throttle = 0.0f;
        return;
    }

    // Roll, Pitch, Yaw: Center ở 1500, deadband 20, map ra [-1.0, 1.0]
    _manual_sp.roll  = apply_rc_deadband(g_input_rc_data.values[RC_CH_ROLL], RC_MID, RC_DEADBAND);
    // Pitch trên stick RC thường là đẩy lên = PWM thấp (nhưng ta muốn setpoint dương là mũi ngóc lên)
    // Tùy cấu hình tay cầm, ở đây giả định đẩy lên = PWM cao = Pitch dương
    _manual_sp.pitch = apply_rc_deadband(g_input_rc_data.values[RC_CH_PITCH], RC_MID, RC_DEADBAND);
    _manual_sp.yaw   = apply_rc_deadband(g_input_rc_data.values[RC_CH_YAW], RC_MID, RC_DEADBAND);

    // Throttle: Không có center, map thẳng từ 1000-2000 ra [0.0, 1.0]
    uint16_t thr_pwm = g_input_rc_data.values[RC_CH_THROTTLE];
    if (thr_pwm < RC_MIN + RC_DEADBAND) thr_pwm = RC_MIN + RC_DEADBAND;
    if (thr_pwm > RC_MAX) thr_pwm = RC_MAX;
    _manual_sp.throttle = (float)(thr_pwm - (RC_MIN + RC_DEADBAND)) / (float)(RC_MAX - (RC_MIN + RC_DEADBAND));

    // Giới hạn an toàn
    if (_manual_sp.roll < -1.0f) _manual_sp.roll = -1.0f;
    if (_manual_sp.roll > 1.0f) _manual_sp.roll = 1.0f;

    if (_manual_sp.pitch < -1.0f) _manual_sp.pitch = -1.0f;
    if (_manual_sp.pitch > 1.0f) _manual_sp.pitch = 1.0f;

    if (_manual_sp.yaw < -1.0f) _manual_sp.yaw = -1.0f;
    if (_manual_sp.yaw > 1.0f) _manual_sp.yaw = 1.0f;

    if (_manual_sp.throttle < 0.0f) _manual_sp.throttle = 0.0f;
    if (_manual_sp.throttle > 1.0f) _manual_sp.throttle = 1.0f;
}

void Commander::handle_arming_logic() {
    _status.rc_signal_lost = g_input_rc_data.rc_lost || g_input_rc_data.rc_failsafe;

    // 1. FAILSAFE NGAY LẬP TỨC NẾU MẤT SÓNG
    if (_status.rc_signal_lost) {
        if (_status.arming_state == ARMING_STATE_ARMED) {
            vehicle_local_position_s local_pos;
            local_pos.timestamp_us = 0;
            _local_pos_sub.copy(local_pos);
            bool local_pos_valid = (local_pos.timestamp_us > 0) && local_pos.z_valid;

            if (local_pos_valid) {
                if (_status.nav_state != NAVIGATION_STATE_AUTO_LAND) {
                    _status.nav_state = NAVIGATION_STATE_AUTO_LAND;
                    _status.failsafe = true;
                    _land_detect_counter = 0;
                    FC_WARN("FAILSAFE: RC Lost! Initiating Auto Land...");
                }
            } else {
                // Không có ước lượng độ cao -> Disarm khẩn cấp để tránh bay mất kiểm soát
                FC_ERR("FAILSAFE: RC Lost with no Position Estimator! Emergency Disarm!");
                set_arming_state(ARMING_STATE_STANDBY);
            }
        }
        return;
    }

    _status.failsafe = false;

    // 2. LOGIC ARM / DISARM bằng công tắc (CH5 > 1500 = Arm)
    bool is_arm_switch_on = (g_input_rc_data.values[RC_CH_ARM] > 1500);

    if (is_arm_switch_on && _status.arming_state == ARMING_STATE_STANDBY) {
        // KHÓA AN TOÀN: Chỉ cho phép Arm nếu cần Ga đang hạ thấp nhất (Throttle < 5%)
        if (_manual_sp.throttle < 0.05f) {
            set_arming_state(ARMING_STATE_ARMED);
        } else {
            // Không log liên tục để tránh rác
            static uint32_t last_warn = 0;
            if (get_time_us() - last_warn > 1000000) {
                FC_WARN("Lỗi Arming: Vui lòng hạ cần Ga (Throttle) xuống 0!");
                last_warn = get_time_us();
            }
        }
    } 
    else if (!is_arm_switch_on && _status.arming_state == ARMING_STATE_ARMED) {
        // Disarm
        set_arming_state(ARMING_STATE_STANDBY);
    }
}

void Commander::update() {
    uint64_t now = get_time_us();
    
    // 50Hz Loop
    if (now - _last_update_us >= 20000) {
        _last_update_us = now;

        // 1. Chuyển đổi tín hiệu tay cầm
        map_rc_to_manual_control();

        // 2. Cập nhật Arming State Machine
        handle_arming_logic();
        // 3. Đọc công tắc chuyển chế độ bay (CH6 / Aux2)
        if (!g_input_rc_data.rc_lost && !g_input_rc_data.rc_failsafe) {
            uint16_t mode_pwm = g_input_rc_data.values[RC_CH_MODE];
            uint8_t new_nav_state = NAVIGATION_STATE_MANUAL;
            if (mode_pwm < 1300) {
                new_nav_state = NAVIGATION_STATE_MANUAL;
            } else if (mode_pwm >= 1300 && mode_pwm <= 1700) {
                new_nav_state = NAVIGATION_STATE_ALTCTL;
            } else {
                new_nav_state = NAVIGATION_STATE_POSCTL;
            }

            // Chỉ cập nhật từ tay cầm khi tay cầm thay đổi công tắc chuyển chế độ (RC override)
            if (!_rc_mode_initialized) {
                _last_rc_mode_state = new_nav_state;
                _rc_mode_initialized = true;
                _status.nav_state = new_nav_state;
            } else if (new_nav_state != _last_rc_mode_state) {
                _last_rc_mode_state = new_nav_state;
                _status.nav_state = new_nav_state;
                FC_INFO("Chế độ bay thay đổi bằng RC Switch: %s", 
                    (new_nav_state == NAVIGATION_STATE_MANUAL) ? "MANUAL (STAB)" :
                    (new_nav_state == NAVIGATION_STATE_ALTCTL) ? "ALTCTL (ALT HOLD)" : "POSCTL (POS HOLD)");
            }

            // Logic cất cánh tự động (Auto Takeoff)
            if (_status.arming_state == ARMING_STATE_ARMED && !_takeoff_complete) {
                if (_status.nav_state == NAVIGATION_STATE_MANUAL) {
                    // Abort cất cánh tự động
                    _takeoff_complete = true;
                } else if (_status.nav_state == NAVIGATION_STATE_AUTO_TAKEOFF) {
                    // Chờ đạt độ cao 1.5m
                    vehicle_local_position_s local_pos;
                    _local_pos_sub.copy(local_pos);
                    if (local_pos.timestamp_us > 0 && local_pos.z <= -1.5f) {
                        FC_INFO("Auto Takeoff Complete - Hovering.");
                        _takeoff_complete = true;
                        _status.nav_state = new_nav_state; // Quay lại chế độ mong muốn ban đầu
                    }
                } else {
                    // ALTCTL/POSCTL + Throttle > 60% -> Kích hoạt Auto Takeoff
                    if (_manual_sp.throttle > 0.6f && (new_nav_state == NAVIGATION_STATE_ALTCTL || new_nav_state == NAVIGATION_STATE_POSCTL)) {
                        FC_INFO("Auto Takeoff Triggered!");
                        _status.nav_state = NAVIGATION_STATE_AUTO_TAKEOFF;
                    }
                }
            }
        }

        // 3.5. Xử lý lệnh từ MAVLink GCS
        vehicle_command_s cmd;
        if (_cmd_sub.update(cmd)) {
            if (cmd.command == 176) { // MAV_CMD_DO_SET_MODE
                uint8_t main_mode = (uint8_t)cmd.param2;
                uint8_t sub_mode = (uint8_t)cmd.param3;
                if (main_mode == 1) { // MANUAL
                    _status.nav_state = NAVIGATION_STATE_MANUAL;
                } else if (main_mode == 2) { // ALTCTL
                    _status.nav_state = NAVIGATION_STATE_ALTCTL;
                } else if (main_mode == 3) { // POSCTL
                    _status.nav_state = NAVIGATION_STATE_POSCTL;
                } else if (main_mode == 5) { // ACRO
                    _status.nav_state = NAVIGATION_STATE_ACRO;
                } else if (main_mode == 7) { // STABILIZED
                    _status.nav_state = NAVIGATION_STATE_STAB;
                } else if (main_mode == 4) { // AUTO
                    if (sub_mode == 2) { // TAKEOFF
                        _status.nav_state = NAVIGATION_STATE_AUTO_TAKEOFF;
                    } else if (sub_mode == 4) { // MISSION
                        _status.nav_state = NAVIGATION_STATE_AUTO_MISSION;
                    } else if (sub_mode == 5) { // RTL
                        _status.nav_state = NAVIGATION_STATE_AUTO_RTL;
                    } else if (sub_mode == 6) { // LAND
                        _status.nav_state = NAVIGATION_STATE_AUTO_LAND;
                    }
                }
                FC_INFO("Chế độ bay thay đổi bằng GCS Command: nav_state=%d", _status.nav_state);
            } else if (cmd.command == 400) { // MAV_CMD_COMPONENT_ARM_DISARM
                if (cmd.param1 == 1.0f) {
                    if (_manual_sp.throttle < 0.10f || _status.nav_state == NAVIGATION_STATE_AUTO_MISSION) {
                        set_arming_state(ARMING_STATE_ARMED);
                    } else {
                        FC_WARN("GCS Arm từ chối: Cần ga còn đang cao!");
                    }
                } else if (cmd.param1 == 0.0f) {
                    set_arming_state(ARMING_STATE_STANDBY);
                }
            } else if (cmd.command == 22) { // MAV_CMD_NAV_TAKEOFF
                _status.nav_state = NAVIGATION_STATE_AUTO_TAKEOFF;
                FC_INFO("GCS Command: AUTO TAKEOFF");
            } else if (cmd.command == 21) { // MAV_CMD_NAV_LAND
                _status.nav_state = NAVIGATION_STATE_AUTO_LAND;
                FC_INFO("GCS Command: AUTO LAND");
            } else if (cmd.command == 20) { // MAV_CMD_NAV_RETURN_TO_LAUNCH
                _status.nav_state = NAVIGATION_STATE_AUTO_RTL;
                FC_INFO("GCS Command: AUTO RTL");
            }
        }
        
        // 4. Kiểm tra tự động Disarm khi chạm đất ở chế độ AUTO_LAND
        if (_status.arming_state == ARMING_STATE_ARMED && _status.nav_state == NAVIGATION_STATE_AUTO_LAND) {
            vehicle_local_position_s local_pos;
            vehicle_attitude_setpoint_s att_sp;
            distance_sensor_s dist;

            _local_pos_sub.copy(local_pos);
            att_sp.timestamp = 0;
            _att_sp_sub.copy(att_sp);
            dist.timestamp_us = 0;
            _dist_sensor_sub.copy(dist);
            
            // Ngưỡng phát hiện chạm đất:
            // 1. Vận tốc đi xuống rất nhỏ: |vz| < 0.25 m/s (NED Z-down)
            // 2. Lực đẩy ga rất thấp: thrust_z < 0.20 (tức là bộ điều khiển đã giảm tối đa ga vì drone bị đất cản lại)
            // 3. Hoặc nếu có ToF hợp lệ, khoảng cách tới đất < 0.12 m
            float thrust_z = (att_sp.timestamp > 0) ? -att_sp.thrust_body[2] : 0.5f;

            bool is_on_ground = false;
            if (local_pos.timestamp_us > 0) {
                if (fabsf(local_pos.vz) < 0.25f) {
                    if (thrust_z < 0.20f) {
                        is_on_ground = true;
                    }
                    if (dist.timestamp_us > 0 && dist.current_distance_m > dist.min_distance_m && dist.current_distance_m < 0.12f) {
                        is_on_ground = true;
                    }
                }
            }

            if (is_on_ground) {
                _land_detect_counter++;
                if (_land_detect_counter >= 100) { // 100 * 20ms = 2 giây liên tục
                    FC_INFO("Failsafe Land Touchdown Detected - Disarming.");
                    set_arming_state(ARMING_STATE_STANDBY);
                    _status.nav_state = NAVIGATION_STATE_MANUAL; // Reset mode
                    _land_detect_counter = 0;
                }
            } else {
                _land_detect_counter = 0;
            }
        }
        
        // 5. Publish liên tục
        _status.timestamp = now;
        _armed.timestamp = now;
        _manual_sp.timestamp = now;
        
        _status_pub.publish(_status);
        _armed_pub.publish(_armed);
        _manual_pub.publish(_manual_sp);
    }
}
