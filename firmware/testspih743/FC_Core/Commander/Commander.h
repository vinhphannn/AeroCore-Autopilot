#pragma once

#include <stdint.h>
#include "../MessageBus/topics/vehicle_status.h"
#include "../MessageBus/topics/actuator_armed.h"
#include "../MessageBus/topics/manual_control_setpoint.h"
#include "../MessageBus/uORB_lite.h"
#include "../Drivers/rc/crsf.h"

class Commander {
public:
    Commander();
    ~Commander() = default;

    void init();
    
    // Gọi định kỳ (50Hz)
    void update();

private:
    uint64_t get_time_us();

    // 1. Chuyển đổi RC thô (1000-2000) -> Setpoint (-1.0 -> 1.0)
    void map_rc_to_manual_control();

    // 2. Logic kiểm tra an toàn và công tắc Arm/Disarm
    void handle_arming_logic();
    void set_arming_state(uint8_t new_state);

    vehicle_status_s   _status;
    actuator_armed_s   _armed;
    manual_control_setpoint_s _manual_sp;

    uORB::PublicationMulti<vehicle_status_s>   _status_pub;
    uORB::PublicationMulti<actuator_armed_s>   _armed_pub;
    uORB::PublicationMulti<manual_control_setpoint_s> _manual_pub;

    uint64_t _last_update_us;

    // Cấu hình kênh RC (AETR = Roll, Pitch, Throttle, Yaw)
    static constexpr int RC_CH_ROLL     = 0; // CH1
    static constexpr int RC_CH_PITCH    = 1; // CH2
    static constexpr int RC_CH_THROTTLE = 2; // CH3
    static constexpr int RC_CH_YAW      = 3; // CH4
    static constexpr int RC_CH_ARM      = 4; // CH5 (Aux1)
    
    static constexpr uint16_t RC_MIN = 1000;
    static constexpr uint16_t RC_MAX = 2000;
    static constexpr uint16_t RC_MID = 1500;
    static constexpr uint16_t RC_DEADBAND = 20; // Khử nhiễu stick ở giữa
};

extern Commander g_commander;
