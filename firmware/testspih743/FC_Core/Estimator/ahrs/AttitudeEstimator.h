#pragma once

#include <stdint.h>
#include "MahonyAHRS.h"
#include "../../MessageBus/topics/vehicle_imu.h"
#include "../../MessageBus/topics/sensor_mag.h"
#include "../../MessageBus/topics/vehicle_attitude.h"
#include "../../MessageBus/uORB_lite.h"
#include "../../Math/AlphaFilter.hpp"

class AttitudeEstimator {
public:
    AttitudeEstimator();
    ~AttitudeEstimator() = default;

    void init();
    void update();

private:
    MahonyAHRS _ahrs;
    
    uORB::SubscriptionMulti<vehicle_imu_s> _imu_sub;
    uORB::SubscriptionMulti<sensor_mag_s>  _mag_sub;
    uORB::PublicationMulti<vehicle_attitude_s> _att_pub;
    
    uint64_t _last_update_us;

    // Dùng để tính angular_accel (D-term cho RateControl)
    float _prev_gyro[3];
    AlphaFilter<float> _accel_filter[3]; // Lọc PT1 cho accel góc

    uint64_t get_time_us();
};

extern AttitudeEstimator g_estimator;
