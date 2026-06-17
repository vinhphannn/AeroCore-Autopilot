#pragma once

#include <stdint.h>
#include "../MessageBus/topics/vehicle_imu.h"
#include "../MessageBus/topics/vehicle_attitude.h"
#include "../MessageBus/topics/sensor_baro.h"
#include "../MessageBus/topics/sensor_gps.h"
#include "../MessageBus/topics/vehicle_local_position.h"
#include "../MessageBus/uORB_lite.h"
#include "../Controllers/mc_rate_control/RateControl.hpp" // For Vector3f definition

class PositionEstimator {
public:
    PositionEstimator();
    ~PositionEstimator() = default;

    void init();
    void update();

private:
    uint64_t get_time_us();

    // uORB Subscriptions
    uORB::SubscriptionMulti<vehicle_imu_s>      _imu_sub;
    uORB::SubscriptionMulti<vehicle_attitude_s> _att_sub;
    uORB::SubscriptionMulti<sensor_baro_s>      _baro_sub;
    uORB::SubscriptionMulti<sensor_gps_s>       _gps_sub;

    // uORB Publication
    uORB::PublicationMulti<vehicle_local_position_s> _local_pos_pub;

    uint64_t _last_update_us;

    // Home Position (GPS)
    bool _home_set;
    int32_t _home_lat;
    int32_t _home_lon;
    float _home_alt_baro;

    // Trạng thái ước lượng
    float _pos[3];  // x, y, z (NED)
    float _vel[3];  // vx, vy, vz (NED)
    float _accel_bias[3]; // x, y, z bias

    // Cấu hình bộ lọc bổ sung (Complementary Filter gains)
    static constexpr float KP_Z = 1.5f;
    static constexpr float KI_Z = 1.5f;
    static constexpr float KD_Z = 0.2f;

    static constexpr float KP_XY = 1.0f;
    static constexpr float KI_XY = 0.5f;
    static constexpr float KV_XY = 1.0f;
    static constexpr float KD_XY = 0.05f;

    // Hằng số chuyển đổi GPS sang mét (Flat Earth)
    static constexpr float CONSTANTS_RADIUS_OF_EARTH = 6371000.0f; // mét
};

extern PositionEstimator g_pos_estimator;
