#pragma once

#include <stdint.h>
#include "../uORB_lite.h"

/**
 * @brief Lệnh điều khiển góc nghiêng và lực đẩy (Mục tiêu của AttitudeController)
 * Giống với cấu trúc VehicleAttitudeSetpoint.msg của PX4
 */
struct vehicle_attitude_setpoint_s {
    uint64_t timestamp;
    
    // Quaternion mục tiêu (Từ FlightModeManager, PosControl, hoặc Navigation)
    float q_d[4];
    
    // Tốc độ xoay Yaw mong muốn do người dùng yêu cầu (rad/s) (dùng làm Feedforward)
    float yaw_sp_move_rate;
    
    // Lực đẩy chuẩn hóa [-1, 1] trên trục Z (NED). Lực nâng lên là giá trị ÂM.
    float thrust_body[3];
};

extern uORB::Topic<vehicle_attitude_setpoint_s> orb_vehicle_attitude_setpoint;
