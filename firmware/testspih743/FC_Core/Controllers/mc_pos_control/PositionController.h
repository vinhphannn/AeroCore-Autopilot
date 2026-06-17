#pragma once

#include <stdint.h>
#include "../../MessageBus/topics/vehicle_local_position.h"
#include "../../MessageBus/topics/vehicle_attitude.h"
#include "../../MessageBus/topics/manual_control_setpoint.h"
#include "../../MessageBus/topics/vehicle_attitude_setpoint.h"
#include "../../MessageBus/topics/vehicle_status.h"
#include "../../MessageBus/topics/distance_sensor.h"
#include "../../MessageBus/topics/landing_target.h"
#include "../../MessageBus/topics/position_setpoint_triplet.h"
#include "../../MessageBus/uORB_lite.h"
#include "../mc_att_control/AttitudeControl.hpp"
#include "ControlMath.hpp"

class PositionController {
public:
    using Vector3f = matrix::Vector3f;
    using Vector2f = matrix::Vector2f;

    PositionController();
    ~PositionController() = default;

    void init();
    
    /**
     * @brief Cập nhật bộ điều khiển vị trí
     * @param dt thời gian bước lặp (s)
     * @param nav_state NAVIGATION_STATE_ALTCTL hoặc NAVIGATION_STATE_POSCTL
     */
    void update(float dt, uint8_t nav_state);

private:
    uint64_t get_time_us();

    void _positionControl();
    void _velocityControl(const float dt);
    void _accelerationControl();

    // uORB Subscriptions
    uORB::SubscriptionMulti<vehicle_local_position_s>   _local_pos_sub;
    uORB::SubscriptionMulti<vehicle_attitude_s>         _att_sub;
    uORB::SubscriptionMulti<manual_control_setpoint_s>  _manual_sub;
    uORB::SubscriptionMulti<distance_sensor_s>          _dist_sensor_sub;
    uORB::SubscriptionMulti<landing_target_s>           _landing_target_sub;
    uORB::SubscriptionMulti<position_setpoint_triplet_s> _pos_sp_triplet_sub;

    // uORB Publication
    uORB::PublicationMulti<vehicle_attitude_setpoint_s> _att_sp_pub;

    // Trạng thái giữ vị trí (Hold States)
    bool _hold_pos_xy;
    bool _hold_pos_z;

    // States
    Vector3f _pos;      // Vị trí hiện tại (m)
    Vector3f _vel;      // Vận tốc hiện tại (m/s)
    Vector3f _vel_dot;  // Gia tốc hiện tại (m/s^2)
    float _yaw;         // Góc Yaw hiện tại (rad)

    // Setpoints
    Vector3f _pos_sp;   // Vị trí mong muốn (m)
    Vector3f _vel_sp;   // Vận tốc mong muốn (m/s)
    Vector3f _acc_sp;   // Gia tốc mong muốn (m/s^2)
    Vector3f _thr_sp;   // Thrust mong muốn (NED)
    float _yaw_sp;      // Yaw đặt (rad)
    float _yawspeed_sp; // Tốc độ Yaw đặt (rad/s)

    // Bộ tích phân (Integrator)
    Vector3f _vel_int;

    // Các hằng số điều khiển (Gains)
    Vector3f _gain_pos_p;
    Vector3f _gain_vel_p;
    Vector3f _gain_vel_i;
    Vector3f _gain_vel_d;

    // Giới hạn (Limits)
    float _lim_vel_horizontal;
    float _lim_vel_up;
    float _lim_vel_down;
    float _lim_thr_min;
    float _lim_thr_max;
    float _lim_thr_xy_margin;
    float _lim_tilt;

    float _hover_thrust;
    bool _decouple_horizontal_and_vertical_acceleration;
};

extern PositionController g_pos_control;
