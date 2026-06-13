/**
 * MulticopterAttitudeController - Kết nối mc_att_control + mc_rate_control của PX4
 *
 * PIPELINE (đúng PX4):
 *
 *   [Tay cầm RC] ---> manual_control_setpoint
 *         |
 *         v
 *   [Chuyển đổi Stick->Quaternion Setpoint]
 *         |
 *         v                         vehicle_attitude (q, gyro, angular_accel)
 *   AttitudeControl::update(q) <---+
 *         | rate_sp [rad/s]
 *         v
 *   RateControl::update(rate, rate_sp, angular_accel, dt, landed)
 *         | torque [-1,1] 3 trục
 *         v
 *   vehicle_torque_setpoint  +  vehicle_thrust_setpoint
 *         |
 *         v
 *   [MIXER -> DShot ESC]
 */
#pragma once

#include <stdint.h>
#include "AttitudeControl.hpp"
#include "../mc_rate_control/RateControl.hpp"
#include "../../MessageBus/topics/vehicle_attitude.h"
#include "../../MessageBus/topics/vehicle_attitude_setpoint.h"
#include "../../MessageBus/topics/vehicle_torque_setpoint.h"
#include "../../MessageBus/topics/vehicle_thrust_setpoint.h"
#include "../../MessageBus/topics/actuator_armed.h"

class AttitudeController {
public:
    AttitudeController();
    ~AttitudeController() = default;

    void init();
    void update();

private:
    uint64_t get_time_us();
    void update_parameters();

    // === PX4 mc_att_control (Outer Loop) ===
    AttitudeControl _att_control;

    // === PX4 mc_rate_control (Inner Loop) ===
    RateControl _rate_control;

    // === uORB Subscriptions ===
    uORB::SubscriptionMulti<vehicle_attitude_s>          _att_sub;
    uORB::SubscriptionMulti<vehicle_attitude_setpoint_s> _att_sp_sub;
    uORB::SubscriptionMulti<actuator_armed_s>            _armed_sub;

    // === uORB Publications ===
    uORB::PublicationMulti<vehicle_torque_setpoint_s>  _torque_pub;
    uORB::PublicationMulti<vehicle_thrust_setpoint_s>  _thrust_pub;

    uint64_t _last_update_us = 0;
    bool _armed = false;
    bool _landed = true; // Bắt đầu từ trạng thái đất
};

extern AttitudeController g_att_control;
