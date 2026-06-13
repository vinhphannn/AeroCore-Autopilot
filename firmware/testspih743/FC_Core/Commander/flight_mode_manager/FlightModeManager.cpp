#include "FlightModeManager.h"
#include "stm32h7xx_hal.h"

FlightModeManager g_flight_mode_mgr;

FlightModeManager::FlightModeManager() :
    _status_sub(orb_vehicle_status, 0),
    _manual_sub(orb_manual_control_setpoint, 0),
    _att_sub(orb_vehicle_attitude, 0),
    _att_sp_pub(orb_vehicle_attitude_setpoint)
{}

uint64_t FlightModeManager::get_time_us() {
    return (uint64_t)HAL_GetTick() * 1000;
}

void FlightModeManager::init() {
    _last_update_us = get_time_us();
}

void FlightModeManager::run_stabilized_mode(const manual_control_setpoint_s& manual, const vehicle_attitude_s& att) {
    vehicle_attitude_setpoint_s att_sp;
    att_sp.timestamp = get_time_us();

    // PX4 STABILIZED Mode: 
    // - Roll/Pitch RC map thẳng thành Góc.
    // - Yaw RC map thành Tốc độ xoay (Yaw Rate).
    // - Throttle RC map thành Lực đẩy (Thrust Z).

    const float max_angle_rad = 0.523f; // 30 degrees góc nghiêng tối đa
    
    // Tính toán Quaternion mong muốn từ RC
    Quatf qd = Quatf::from_euler(
        manual.roll  * max_angle_rad,  
        manual.pitch * max_angle_rad,  
        att.yaw_rad  // Giữ nguyên góc Yaw hiện tại (để chống xoay vòng)
    );

    att_sp.q_d[0] = qd.w;
    att_sp.q_d[1] = qd.x;
    att_sp.q_d[2] = qd.y;
    att_sp.q_d[3] = qd.z;

    // Yaw Feedforward (Tốc độ xoay rad/s)
    att_sp.yaw_sp_move_rate = manual.yaw * 1.5708f; // Tối đa 90 deg/s

    // Thrust (PX4 dùng -Z hướng lên trên)
    att_sp.thrust_body[0] = 0.0f;
    att_sp.thrust_body[1] = 0.0f;
    att_sp.thrust_body[2] = -manual.throttle; // -1.0 là nâng tối đa

    // Đẩy xuống cho AttitudeController
    _att_sp_pub.publish(att_sp);
}

void FlightModeManager::update() {
    uint64_t now = get_time_us();
    if (now - _last_update_us >= 10000) { // 100Hz
        _last_update_us = now;

        vehicle_status_s status;
        _status_sub.copy(status);
        if (status.timestamp == 0) return; // Không có status thì không chạy

        manual_control_setpoint_s manual;
        vehicle_attitude_s att;
        _manual_sub.copy(manual);
        _att_sub.copy(att);

        // Theo lý thuyết của PX4, dựa vào nav_state để chọn thuật toán
        switch (status.nav_state) {
            case NAVIGATION_STATE_MANUAL:
            case NAVIGATION_STATE_STAB:
            case NAVIGATION_STATE_ACRO: // Tạm gộp
                run_stabilized_mode(manual, att);
                break;
                
            case NAVIGATION_STATE_ALTCTL:
                // TODO: Chạy PID Độ cao (lấy Baro) -> Tính ra Thrust và Góc
                break;

            case NAVIGATION_STATE_POSCTL:
                // TODO: Chạy PID Vị trí (lấy GPS) -> Tính ra Góc
                break;

            default:
                // Mặc định an toàn: Dừng lại
                break;
        }
    }
}
