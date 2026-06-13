#pragma once

#include "ControlAllocation.hpp"
#include "../../MessageBus/topics/vehicle_torque_setpoint.h"
#include "../../MessageBus/topics/vehicle_thrust_setpoint.h"
#include "../../MessageBus/topics/actuator_armed.h"
#include "../../Drivers/actuator/DShot.h"

/**
 * Mixer - Tầng kết nối PX4 Control Allocation và DShot Driver
 *
 * Pipeline:
 *   vehicle_torque_setpoint (Roll, Pitch, Yaw [-1,1])
 *   vehicle_thrust_setpoint (Thrust Z [0,1] trong NED -> |thrust_z|)
 *           |
 *           v
 *   ControlAllocation::allocate() -> motor_out[4] [0,1]
 *           |
 *           v
 *   map [0,1] -> DShot [48, 2047]
 *           |
 *           v
 *   DShotDriver::set_throttle() + update()
 *
 * Phản hồi saturation -> RateControl Anti-windup
 */
class Mixer {
public:
    Mixer();
    ~Mixer() = default;

    void init();
    void update();

    // Getter để AttitudeController đọc saturation về
    const MixerSaturation& getSaturation() const { return _last_sat; }

private:
    ControlAllocation _allocator;
    MixerSaturation   _last_sat;

    uORB::SubscriptionMulti<vehicle_torque_setpoint_s>  _torque_sub;
    uORB::SubscriptionMulti<vehicle_thrust_setpoint_s>  _thrust_sub;
    uORB::SubscriptionMulti<actuator_armed_s>           _armed_sub;

    bool _armed = false;

    // DShot range: 48 = min throttle, 2047 = max throttle, 0 = disarm
    static constexpr uint16_t DSHOT_MIN = 48;
    static constexpr uint16_t DSHOT_MAX = 2047;
    static constexpr uint16_t DSHOT_IDLE = 100; // Idle khi armed nhưng throttle=0
};

extern Mixer g_mixer;
