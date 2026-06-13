#include "Commander.h"
#include "stm32h7xx_hal.h"
#include "../Main/fc_logging.h"
#include <math.h>

Commander g_commander;

Commander::Commander() : 
    _status_pub(orb_vehicle_status),
    _armed_pub(orb_actuator_armed),
    _manual_pub(orb_manual_control_setpoint),
    _last_update_us(0)
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
            _status.failsafe = true;
            // Ở chế độ Manual bay tay, Failsafe = Disarm thả rơi luôn để an toàn (ko RTH được vì ko có GPS định vị)
            FC_ERR("FAILSAFE TRIGGERED - DISARMING");
            set_arming_state(ARMING_STATE_STANDBY);
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
        
        // 3. Publish liên tục
        _status.timestamp = now;
        _armed.timestamp = now;
        _manual_sp.timestamp = now;
        
        _status_pub.publish(_status);
        _armed_pub.publish(_armed);
        _manual_pub.publish(_manual_sp);
    }
}
