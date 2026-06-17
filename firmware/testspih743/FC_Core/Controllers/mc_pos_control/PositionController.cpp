#include "PositionController.h"
#include "stm32h7xx_hal.h"
#include "../../Main/fc_logging.h"
#include <math.h>

PositionController g_pos_control;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

PositionController::PositionController() :
    _local_pos_sub(orb_vehicle_local_position, 0),
    _att_sub(orb_vehicle_attitude, 0),
    _manual_sub(orb_manual_control_setpoint, 0),
    _dist_sensor_sub(orb_distance_sensor, 0),
    _landing_target_sub(orb_landing_target, 0),
    _pos_sp_triplet_sub(orb_position_setpoint_triplet, 0),
    _att_sp_pub(orb_vehicle_attitude_setpoint),
    _hold_pos_xy(false),
    _hold_pos_z(false),
    _pos(0.0f, 0.0f, 0.0f),
    _vel(0.0f, 0.0f, 0.0f),
    _vel_dot(0.0f, 0.0f, 0.0f),
    _yaw(0.0f),
    _pos_sp(NAN, NAN, NAN),
    _vel_sp(NAN, NAN, NAN),
    _acc_sp(NAN, NAN, NAN),
    _thr_sp(0.0f, 0.0f, 0.0f),
    _yaw_sp(0.0f),
    _yawspeed_sp(0.0f),
    _vel_int(0.0f, 0.0f, 0.0f)
{
    // Cấu hình Gains mặc định giống PX4
    _gain_pos_p = Vector3f(1.2f, 1.2f, 1.2f);
    _gain_vel_p = Vector3f(0.15f, 0.15f, 0.20f);
    _gain_vel_i = Vector3f(0.02f, 0.02f, 0.03f);
    _gain_vel_d = Vector3f(0.02f, 0.02f, 0.05f);

    // Cấu hình Limits mặc định giống PX4
    _lim_vel_horizontal = 3.0f;
    _lim_vel_up = 1.5f;
    _lim_vel_down = 1.5f;
    _lim_thr_min = 0.12f;
    _lim_thr_max = 0.9f;
    _lim_thr_xy_margin = 0.3f;
    _lim_tilt = 0.35f;

    _hover_thrust = 0.3f;
    _decouple_horizontal_and_vertical_acceleration = true;
}

uint64_t PositionController::get_time_us() {
    return (uint64_t)HAL_GetTick() * 1000;
}

void PositionController::init() {
    _hold_pos_xy = false;
    _hold_pos_z = false;
    _vel_int.zero();
    _pos_sp = Vector3f(NAN, NAN, NAN);
    _vel_sp = Vector3f(NAN, NAN, NAN);
    _acc_sp = Vector3f(NAN, NAN, NAN);
    _thr_sp.zero();
    _yaw_sp = 0.0f;
    _yawspeed_sp = 0.0f;
}

void PositionController::update(float dt, uint8_t nav_state) {
    vehicle_local_position_s local_pos;
    vehicle_attitude_s att;
    manual_control_setpoint_s manual;
    distance_sensor_s dist;
    landing_target_s landing_target;

    _local_pos_sub.copy(local_pos);
    _att_sub.copy(att);
    _manual_sub.copy(manual);
    dist.timestamp_us = 0;
    _dist_sensor_sub.copy(dist);
    landing_target.valid = false;
    _landing_target_sub.copy(landing_target);

    if (local_pos.timestamp_us == 0 || att.timestamp_us == 0 || manual.timestamp == 0) {
        return;
    }

    // Đọc trạng thái ước lượng hiện tại
    _pos = Vector3f(local_pos.x, local_pos.y, local_pos.z);
    _vel = Vector3f(local_pos.vx, local_pos.vy, local_pos.vz);
    _vel_dot.zero(); // Tạm thời đặt 0
    _yaw = att.yaw_rad;

    // Lưu trữ góc hướng đầu (Yaw Target)
    static bool first_run = true;
    if (first_run) {
        _yaw_sp = _yaw;
        first_run = false;
    }

    // Cập nhật Yaw Target từ tay lái
    if (fabsf(manual.yaw) > 0.05f) {
        _yawspeed_sp = manual.yaw * 1.5708f; // 90 deg/s max
        _yaw_sp += _yawspeed_sp * dt;
        if (_yaw_sp > M_PI)  _yaw_sp -= 2.0f * M_PI;
        if (_yaw_sp < -M_PI) _yaw_sp += 2.0f * M_PI;
    } else {
        _yawspeed_sp = 0.0f;
    }

    // Đặt lại các setpoints mặc định là NAN nếu không ở trạng thái giữ vị trí (Hold)
    if (!_hold_pos_xy) {
        _pos_sp(0) = NAN;
        _pos_sp(1) = NAN;
    }
    if (!_hold_pos_z) {
        _pos_sp(2) = NAN;
    }
    _vel_sp = Vector3f(NAN, NAN, NAN);
    _acc_sp = Vector3f(NAN, NAN, NAN);

    // Đọc setpoint tự động từ Navigator nếu ở chế độ AUTO_MISSION hoặc AUTO_RTL
    if (nav_state == NAVIGATION_STATE_AUTO_MISSION || nav_state == NAVIGATION_STATE_AUTO_RTL) {
        position_setpoint_triplet_s triplet{};
        _pos_sp_triplet_sub.copy(triplet);
        if (triplet.timestamp > 0 && triplet.current.valid) {
            _pos_sp(0) = triplet.current.x;
            _pos_sp(1) = triplet.current.y;
            _pos_sp(2) = triplet.current.z;
            _yaw_sp = triplet.current.yaw;
            _hold_pos_xy = false;
            _hold_pos_z = false;
        }
    }

    // ==========================================
    // 1. ĐIỀU KHIỂN TRỤC DỌC Z (Cho cả ALTCTL và POSCTL)
    // ==========================================
    if (nav_state == NAVIGATION_STATE_AUTO_LAND) {
        float land_speed = 0.5f; // Tốc độ xuống thông thường
        
        // Nếu ToF đo được hợp lệ và cách đất < 1m -> Kích hoạt Land Crawl (hạ cánh nhẹ nhàng 0.2 m/s)
        if (dist.timestamp_us > 0 && dist.current_distance_m > dist.min_distance_m && dist.current_distance_m < 1.0f) {
            land_speed = 0.2f;
        }
        _vel_sp(2) = land_speed; // Đi xuống chậm trong hệ NED
        _hold_pos_z = false;
    } else if (nav_state == NAVIGATION_STATE_AUTO_TAKEOFF) {
        _vel_sp(2) = -0.8f; // Bay lên với vận tốc 0.8 m/s (NED Z-down)
        _hold_pos_z = false;
    } else if (nav_state == NAVIGATION_STATE_AUTO_MISSION || nav_state == NAVIGATION_STATE_AUTO_RTL) {
        // Trục dọc Z được điều khiển bằng vị trí lấy từ triplet nên không ghi đè _vel_sp(2) ở đây.
        // Chỉ cần reset cờ để bộ điều khiển vị trí hoạt động.
        _hold_pos_z = false;
    } else if (local_pos.z_valid) {
        bool stick_in_deadband_z = (manual.throttle > 0.45f && manual.throttle < 0.55f);

        if (stick_in_deadband_z) {
            if (!_hold_pos_z) {
                _pos_sp(2) = _pos(2);
                _hold_pos_z = true;
            }
        } else {
            _hold_pos_z = false;
            // throttle 0.0 -> 1.0, quy đổi 0.5 làm mốc 0 m/s. Đẩy ga lên (throttle > 0.5) -> Z giảm (bay lên)
            float thr_joy = (manual.throttle - 0.5f) * 2.0f;
            _vel_sp(2) = -thr_joy * (thr_joy > 0.f ? _lim_vel_up : _lim_vel_down);
        }
    } else {
        // Dự phòng khi mất cảm biến Z: pass-through ga trực tiếp
        vehicle_attitude_setpoint_s att_sp;
        att_sp.timestamp = get_time_us();
        
        float roll_sp = (nav_state == NAVIGATION_STATE_ALTCTL) ? (manual.roll * _lim_tilt) : 0.0f;
        float pitch_sp = (nav_state == NAVIGATION_STATE_ALTCTL) ? (manual.pitch * _lim_tilt) : 0.0f;
        
        Quatf qd = Quatf::from_euler(roll_sp, pitch_sp, _yaw_sp);
        att_sp.q_d[0] = qd.w;
        att_sp.q_d[1] = qd.x;
        att_sp.q_d[2] = qd.y;
        att_sp.q_d[3] = qd.z;
        att_sp.yaw_sp_move_rate = _yawspeed_sp;
        
        att_sp.thrust_body[0] = 0.0f;
        att_sp.thrust_body[1] = 0.0f;
        att_sp.thrust_body[2] = -manual.throttle;
        
        _att_sp_pub.publish(att_sp);
        return;
    }

    // ==========================================
    // 2. ĐIỀU KHIỂN TRỤC NGANG X-Y
    // ==========================================
    bool control_xy = (nav_state == NAVIGATION_STATE_POSCTL || nav_state == NAVIGATION_STATE_AUTO_LAND || nav_state == NAVIGATION_STATE_AUTO_TAKEOFF || nav_state == NAVIGATION_STATE_AUTO_MISSION || nav_state == NAVIGATION_STATE_AUTO_RTL) && local_pos.xy_valid;

    if (control_xy) {
        bool use_precision_landing = (nav_state == NAVIGATION_STATE_AUTO_LAND) && landing_target.valid && (landing_target.timestamp_us > 0);

        if (use_precision_landing) {
            _hold_pos_xy = false;
            // Precision Landing: Căn chỉnh thẳng tâm thẻ bằng điều khiển tỷ lệ
            float gain = 0.8f; // P-gain cho căn chỉnh tâm (0.8 m/s trên mỗi mét sai lệch)
            _vel_sp(0) = landing_target.rel_pos_x * gain;
            _vel_sp(1) = landing_target.rel_pos_y * gain;

            // Giới hạn tốc độ dịch chuyển ngang khi căn chỉnh để hạ cánh mịn màng
            float max_align_vel = 0.6f; // m/s
            float vel_norm = sqrtf(_vel_sp(0) * _vel_sp(0) + _vel_sp(1) * _vel_sp(1));
            if (vel_norm > max_align_vel) {
                _vel_sp(0) = (_vel_sp(0) / vel_norm) * max_align_vel;
                _vel_sp(1) = (_vel_sp(1) / vel_norm) * max_align_vel;
            }
        } else {
            bool stick_in_deadband_xy = (nav_state == NAVIGATION_STATE_AUTO_LAND) || (nav_state == NAVIGATION_STATE_AUTO_TAKEOFF) || (fabsf(manual.roll) < 0.05f && fabsf(manual.pitch) < 0.05f);

            if (stick_in_deadband_xy) {
                if (!_hold_pos_xy) {
                    _pos_sp(0) = _pos(0);
                    _pos_sp(1) = _pos(1);
                    _hold_pos_xy = true;
                }
            } else {
                _hold_pos_xy = false;
                // Map sticks sang NED velocity setpoint bằng cách xoay góc Yaw
                float vel_sp_x_body = manual.pitch * _lim_vel_horizontal;
                float vel_sp_y_body = manual.roll  * _lim_vel_horizontal;
                
                _vel_sp(0) = vel_sp_x_body * cosf(_yaw) - vel_sp_y_body * sinf(_yaw);
                _vel_sp(1) = vel_sp_x_body * sinf(_yaw) + vel_sp_y_body * cosf(_yaw);
            }
        }
    } else {
        _hold_pos_xy = false;
        _vel_int(0) = 0.0f;
        _vel_int(1) = 0.0f;
    }

    // ==========================================
    // 3. CHẠY THUẬT TOÁN ĐIỀU KHIỂN KIỂU PX4
    // ==========================================
    _positionControl();
    _velocityControl(dt);

    // ==========================================
    // 4. ĐÓNG GÓI ATTITUDE SETPOINT & PUBLISH
    // ==========================================
    vehicle_attitude_setpoint_s att_sp;
    att_sp.timestamp = get_time_us();

    if (nav_state == NAVIGATION_STATE_ALTCTL) {
        // ALTCTL: Lấy Roll/Pitch trực tiếp từ tay lái, Z từ PID độ cao
        float roll_sp = manual.roll * _lim_tilt;
        float pitch_sp = manual.pitch * _lim_tilt;
        
        Quatf qd = Quatf::from_euler(roll_sp, pitch_sp, _yaw_sp);
        att_sp.q_d[0] = qd.w;
        att_sp.q_d[1] = qd.x;
        att_sp.q_d[2] = qd.y;
        att_sp.q_d[3] = qd.z;
        att_sp.yaw_sp_move_rate = _yawspeed_sp;
        
        att_sp.thrust_body[0] = 0.0f;
        att_sp.thrust_body[1] = 0.0f;
        att_sp.thrust_body[2] = _thr_sp(2); // Thrust dọc tính từ _velocityControl
    } else {
        // POSCTL: Góc thái được tính trực tiếp từ vector thrust _thr_sp của bộ PID
        ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, att_sp);
        att_sp.yaw_sp_move_rate = _yawspeed_sp;
    }

    _att_sp_pub.publish(att_sp);
}

void PositionController::_positionControl()
{
    // P-position controller
    Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
    ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
    ControlMath::setZeroIfNanVector3f(vel_sp_position);

    // Constrain horizontal velocity
    _vel_sp.xy(ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal));
    
    // Constrain velocity in z-direction (Z-axis points down in NED)
    _vel_sp(2) = fmaxf(fminf(_vel_sp(2), _lim_vel_down), -_lim_vel_up);
}

void PositionController::_velocityControl(const float dt)
{
    // Constrain vertical velocity integral
    _vel_int(2) = fmaxf(fminf(_vel_int(2), CONSTANTS_ONE_G), -CONSTANTS_ONE_G);

    // PID velocity control
    Vector3f vel_error = _vel_sp - _vel;
    Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

    ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

    _accelerationControl();

    // Integrator anti-windup in vertical direction
    if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
        (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
        vel_error(2) = 0.f;
    }

    // Prioritize vertical control while keeping a horizontal margin
    const Vector2f thrust_sp_xy(_thr_sp.x, _thr_sp.y);
    const float thrust_sp_xy_norm = thrust_sp_xy.norm();
    const float thrust_max_squared = _lim_thr_max * _lim_thr_max;

    // Determine how much vertical thrust is left keeping horizontal margin
    const float allocated_horizontal_thrust = fminf(thrust_sp_xy_norm, _lim_thr_xy_margin);
    const float thrust_z_max_squared = thrust_max_squared - (allocated_horizontal_thrust * allocated_horizontal_thrust);

    // Saturate maximal vertical thrust
    _thr_sp(2) = fmaxf(_thr_sp(2), -sqrtf(thrust_z_max_squared));

    // Determine how much horizontal thrust is left after prioritizing vertical control
    const float thrust_max_xy_squared = thrust_max_squared - (_thr_sp(2) * _thr_sp(2));
    float thrust_max_xy = 0.f;

    if (thrust_max_xy_squared > 0.f) {
        thrust_max_xy = sqrtf(thrust_max_xy_squared);
    }

    // Saturate thrust in horizontal direction
    if (thrust_sp_xy_norm > thrust_max_xy) {
        _thr_sp.xy(thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy);
    }

    // Use tracking Anti-Windup for horizontal direction
    const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp.x, _thr_sp.y) * (CONSTANTS_ONE_G / _hover_thrust);

    if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
        const float arw_gain = 2.f / _gain_vel_p(0);
        const Vector2f acc_sp_xy = _acc_sp.xy();

        Vector2f vel_err_xy = Vector2f(vel_error.x, vel_error.y) - (acc_sp_xy - acc_sp_xy_produced) * arw_gain;
        vel_error.x = vel_err_xy.x;
        vel_error.y = vel_err_xy.y;
    }

    // Make sure integral doesn't get NAN
    ControlMath::setZeroIfNanVector3f(vel_error);
    
    // Update integral part of velocity control
    _vel_int = _vel_int + vel_error.emult(_gain_vel_i) * dt;
}

void PositionController::_accelerationControl()
{
    // Assume standard acceleration due to gravity in vertical direction for attitude generation
    float z_specific_force = -CONSTANTS_ONE_G;

    if (!_decouple_horizontal_and_vertical_acceleration) {
        z_specific_force += _acc_sp(2);
    }

    Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
    ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
    
    // Convert to thrust assuming hover thrust produces standard gravity
    const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
    
    // Project thrust to planned body attitude
    const float cos_ned_body = Vector3f(0, 0, 1).dot(body_z);
    
    // collective_thrust must be at least -_lim_thr_min
    const float collective_thrust = fminf(thrust_ned_z / cos_ned_body, -_lim_thr_min);
    _thr_sp = body_z * collective_thrust;
}
