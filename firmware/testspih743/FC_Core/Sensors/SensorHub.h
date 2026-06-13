#pragma once
#include "../MessageBus/topics/sensor_imu.h"
#include "../MessageBus/topics/vehicle_imu.h"
#include "../Modules/Math/GyroFilter.hpp"
#include "../Math/AlphaFilter.hpp"

// Tương đương enum trong PX4: ROTATION_NONE, ROTATION_YAW_90, ...
// Giá trị int phải khớp với PARAM_SENS_BOARD_ROT
enum BoardRotation {
    ROTATION_NONE = 0,
    ROTATION_YAW_90,
    ROTATION_YAW_180,
    ROTATION_YAW_270,
    ROTATION_ROLL_180,
    ROTATION_ROLL_180_YAW_90,
    ROTATION_PITCH_180,
    ROTATION_ROLL_180_YAW_270,
};

class SensorHub {
public:
    SensorHub() : _imu_sub(orb_sensor_imu, 0), _vehicle_imu_pub(orb_vehicle_imu) {}
    ~SensorHub() = default;

    void init();
    void update();

    void set_board_rotation(BoardRotation rot) { _board_rotation = rot; }

private:
    void rotate_3d(float vec[3], BoardRotation rot);

    uORB::SubscriptionMulti<sensor_imu_s> _imu_sub;
    uORB::PublicationMulti<vehicle_imu_s> _vehicle_imu_pub;

    BoardRotation _board_rotation = ROTATION_NONE;

    // Bộ lọc tần số cao cho Gyro (LowPassFilter2p + NotchFilter, đúng chuẩn PX4)
    GyroFilter _gyro_filter;

    // Bộ lọc PT1 cho Accel (dùng AlphaFilter - nhẹ hơn, đủ tốt cho Accel)
    AlphaFilter<float> _accel_filter[3];
};

extern SensorHub g_sensor_hub;
