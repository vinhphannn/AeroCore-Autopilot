#pragma once

#include <stdint.h>
#include "../../MessageBus/topics/vehicle_status.h"
#include "../../MessageBus/topics/vehicle_attitude.h"
#include "../../MessageBus/topics/manual_control_setpoint.h"
#include "../../MessageBus/topics/vehicle_attitude_setpoint.h"
#include "../../MessageBus/uORB_lite.h"
#include "../../Controllers/mc_att_control/AttitudeControl.hpp"

/**
 * FlightModeManager - Mô đun Quản lý Chế độ bay (Giống PX4 FlightTasks)
 * 
 * Nhiệm vụ: Đọc tay cầm (Manual) hoặc GPS/Baro (Auto)
 * để sinh ra setpoint (Mục tiêu Góc + Lực đẩy) cho AttitudeController.
 */
class FlightModeManager {
public:
    FlightModeManager();
    ~FlightModeManager() = default;

    void init();
    void update();

private:
    uint64_t get_time_us();

    // Xử lý các chế độ bay
    void run_stabilized_mode(const manual_control_setpoint_s& manual, const vehicle_attitude_s& att);
    // void run_altitude_mode(...); // Sẽ làm sau
    // void run_position_mode(...); // Sẽ làm sau

    uORB::SubscriptionMulti<vehicle_status_s>          _status_sub;
    uORB::SubscriptionMulti<manual_control_setpoint_s> _manual_sub;
    uORB::SubscriptionMulti<vehicle_attitude_s>        _att_sub;
    
    uORB::PublicationMulti<vehicle_attitude_setpoint_s> _att_sp_pub;

    uint64_t _last_update_us;
};

extern FlightModeManager g_flight_mode_mgr;
