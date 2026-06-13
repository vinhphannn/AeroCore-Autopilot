#include "AttitudeController.h"
#include "../../Parameters/param.h"
#include "stm32h7xx_hal.h"

AttitudeController g_att_control;

AttitudeController::AttitudeController() :
    _att_sub(orb_vehicle_attitude, 0),
    _att_sp_sub(orb_vehicle_attitude_setpoint, 0),
    _armed_sub(orb_actuator_armed, 0),
    _torque_pub(orb_vehicle_torque_setpoint),
    _thrust_pub(orb_vehicle_thrust_setpoint)
{}

uint64_t AttitudeController::get_time_us() {
    return (uint64_t)HAL_GetTick() * 1000;
}

void AttitudeController::init() {
    _last_update_us = get_time_us();
    update_parameters();
}

void AttitudeController::update_parameters() {
    float roll_p, roll_i, roll_d;
    float pitch_p, pitch_i, pitch_d;
    float yaw_p, yaw_i, yaw_d;

    param_get(PARAM_MC_ROLL_P,  &roll_p);
    param_get(PARAM_MC_ROLL_I,  &roll_i);
    param_get(PARAM_MC_ROLL_D,  &roll_d);
    param_get(PARAM_MC_PITCH_P, &pitch_p);
    param_get(PARAM_MC_PITCH_I, &pitch_i);
    param_get(PARAM_MC_PITCH_D, &pitch_d);
    param_get(PARAM_MC_YAW_P,   &yaw_p);
    param_get(PARAM_MC_YAW_I,   &yaw_i);
    param_get(PARAM_MC_YAW_D,   &yaw_d);

    // ---- Outer Loop: AttitudeControl (P-only, đúng PX4) ----
    // PX4 mc_att_control params: MC_ROLL_P, MC_PITCH_P, MC_YAW_P
    // Đây là Outer P-gain, KHÔNG phải Rate PID gain
    _att_control.setProportionalGain({roll_p, pitch_p, yaw_p}, 0.4f /* yaw_weight */);
    _att_control.setRateLimit({
        3.1415f,  // Max roll rate: ~180 deg/s
        3.1415f,  // Max pitch rate
        1.5708f   // Max yaw rate: ~90 deg/s
    });

    // ---- Inner Loop: RateControl (PID, đúng PX4 mc_rate_control) ----
    // PX4 param tên: MC_ROLLRATE_P, MC_ROLLRATE_I, MC_ROLLRATE_D
    // Ở đây ta tạm dùng chung PARAM_MC_ROLL_P vì chưa có param riêng cho Rate
    // TODO: Thêm PARAM_MC_ROLLRATE_P/I/D vào param.h sau
    _rate_control.setPidGains(
        {roll_p * 0.15f,  pitch_p * 0.15f,  yaw_p * 0.2f},  // P_rate
        {roll_i * 0.05f,  pitch_i * 0.05f,  yaw_i * 0.1f},  // I_rate
        {roll_d * 0.003f, pitch_d * 0.003f, yaw_d * 0.0f}   // D_rate
    );
    _rate_control.setIntegratorLimit({0.3f, 0.3f, 0.3f});
    _rate_control.setFeedForwardGain({0.0f, 0.0f, 0.0f}); // FF=0 cho Multicopter
}

void AttitudeController::update() {
    uint64_t now = get_time_us();
    float dt = (now - _last_update_us) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
    _last_update_us = now;

    // Cập nhật trạng thái Armed
    actuator_armed_s armed;
    if (_armed_sub.update(armed)) {
        _armed = armed.armed;
        _landed = !armed.armed; // Đơn giản: Nếu disarmed = đang ở đất
    }

    if (!_armed) {
        // Reset integral khi disarmed (Anti-windup)
        _rate_control.resetIntegral();
        vehicle_torque_setpoint_s torque; torque.timestamp = now;
        torque.xyz[0] = torque.xyz[1] = torque.xyz[2] = 0.0f;
        vehicle_thrust_setpoint_s thrust; thrust.timestamp = now;
        thrust.xyz[0] = thrust.xyz[1] = thrust.xyz[2] = 0.0f;
        _torque_pub.publish(torque);
        _thrust_pub.publish(thrust);
        return;
    }

    // === Đọc dữ liệu ===
    vehicle_attitude_s att;
    vehicle_attitude_setpoint_s att_sp;
    _att_sub.copy(att);
    
    // Nếu chưa nhận được lệnh điều khiển góc nào thì không làm gì cả
    _att_sp_sub.copy(att_sp);
    if (att_sp.timestamp == 0) return;

    // === OUTER LOOP: AttitudeControl (Quaternion-based P-controller) ===
    Quatf qd(att_sp.q_d[0], att_sp.q_d[1], att_sp.q_d[2], att_sp.q_d[3]);
    float yaw_rate_ff = att_sp.yaw_sp_move_rate;

    Quatf q_current(att.q[0], att.q[1], att.q[2], att.q[3]);
    _att_control.setAttitudeSetpoint(qd, yaw_rate_ff);
    Vector3f rate_sp = _att_control.update(q_current);

    // === INNER LOOP: RateControl (PID với D-term từ angular_accel) ===
    Vector3f rate_current(att.rollspeed_rad_s, att.pitchspeed_rad_s, att.yawspeed_rad_s);
    Vector3f angular_accel(att.angular_accel_rad_s2[0],
                           att.angular_accel_rad_s2[1],
                           att.angular_accel_rad_s2[2]);

    Vector3f torque_out = _rate_control.update(rate_current, rate_sp, angular_accel, dt, _landed);

    // === PUBLISH OUTPUTS ===
    vehicle_torque_setpoint_s torque;
    torque.timestamp = now;
    torque.xyz[0] = torque_out.x;
    torque.xyz[1] = torque_out.y;
    torque.xyz[2] = torque_out.z;

    vehicle_thrust_setpoint_s thrust;
    thrust.timestamp = now;
    thrust.xyz[0] = att_sp.thrust_body[0];
    thrust.xyz[1] = att_sp.thrust_body[1];
    thrust.xyz[2] = att_sp.thrust_body[2]; // Lấy trực tiếp thrust từ setpoint cấp trên

    _torque_pub.publish(torque);
    _thrust_pub.publish(thrust);
}
