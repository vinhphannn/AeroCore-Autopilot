#include "PositionEstimator.h"
#include "stm32h7xx_hal.h"
#include "../Main/fc_logging.h"
#include <math.h>

PositionEstimator g_pos_estimator;

PositionEstimator::PositionEstimator() :
    _imu_sub(orb_vehicle_imu, 0),
    _att_sub(orb_vehicle_attitude, 0),
    _baro_sub(orb_sensor_baro, 0),
    _gps_sub(orb_sensor_gps, 0),
    _local_pos_pub(orb_vehicle_local_position),
    _last_update_us(0),
    _home_set(false),
    _home_lat(0),
    _home_lon(0),
    _home_alt_baro(0.0f)
{
    for(int i = 0; i < 3; i++) {
        _pos[i] = 0.0f;
        _vel[i] = 0.0f;
        _accel_bias[i] = 0.0f;
    }
}

uint64_t PositionEstimator::get_time_us() {
    return (uint64_t)HAL_GetTick() * 1000;
}

void PositionEstimator::init() {
    _last_update_us = get_time_us();
    _home_set = false;
    _home_alt_baro = 0.0f;
    for(int i = 0; i < 3; i++) {
        _pos[i] = 0.0f;
        _vel[i] = 0.0f;
        _accel_bias[i] = 0.0f;
    }
}

void PositionEstimator::update() {
    uint64_t now = get_time_us();
    float dt = (now - _last_update_us) / 1000000.0f;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f; // Tần số SensorsTask là 100Hz
    _last_update_us = now;

    vehicle_imu_s imu;
    vehicle_attitude_s att;

    _imu_sub.copy(imu);
    _att_sub.copy(att);

    if (imu.timestamp_us == 0 || att.timestamp_us == 0) return;

    // 1. Chuyển đổi gia tốc từ Body sang Earth (NED) Frame
    float qw = att.q[0];
    float qx = att.q[1];
    float qy = att.q[2];
    float qz = att.q[3];

    float ax_body = imu.accel_m_s2[0];
    float ay_body = imu.accel_m_s2[1];
    float az_body = imu.accel_m_s2[2];

    // Tính ma trận xoay (DCM) từ Quaternion để chuyển Body -> NED
    float r00 = qw*qw + qx*qx - qy*qy - qz*qz;
    float r01 = 2.0f * (qx*qy - qw*qz);
    float r02 = 2.0f * (qx*qz + qw*qy);

    float r10 = 2.0f * (qx*qy + qw*qz);
    float r11 = qw*qw - qx*qx + qy*qy - qz*qz;
    float r12 = 2.0f * (qy*qz - qw*qx);

    float r20 = 2.0f * (qx*qz - qw*qy);
    float r21 = 2.0f * (qy*qz + qw*qx);
    float r22 = qw*qw - qx*qx - qy*qy + qz*qz;

    float ax_earth = r00 * ax_body + r01 * ay_body + r02 * az_body;
    float ay_earth = r10 * ax_body + r11 * ay_body + r12 * az_body;
    float az_earth = r20 * ax_body + r21 * ay_body + r22 * az_body;

    // Bù trừ trọng lực (Trong hệ NED, trục Z hướng xuống nên Trọng lực hướng theo chiều Dương)
    az_earth += 9.80665f;

    // 2. Trừ bias và dự đoán trạng thái mới (Inertial Navigation Prediction)
    float acc_corr[3];
    acc_corr[0] = ax_earth - _accel_bias[0];
    acc_corr[1] = ay_earth - _accel_bias[1];
    acc_corr[2] = az_earth - _accel_bias[2];

    // Cập nhật Vị trí & Vận tốc bằng tích phân gia tốc
    for(int i = 0; i < 3; i++) {
        _pos[i] += _vel[i] * dt + 0.5f * acc_corr[i] * dt * dt;
        _vel[i] += acc_corr[i] * dt;
    }

    // 3. HIỆU CHỈNH TRỤC DỌC (Z) - Sử dụng Barometer
    sensor_baro_s baro;
    bool z_valid = false;
    _baro_sub.copy(baro);
    if (baro.timestamp_us > 0) {
        // Công thức hypsometric tính độ cao từ áp suất
        float alt_m = 44330.0f * (1.0f - powf(baro.pressure_mbar / 1013.25f, 0.190295f));
        
        if (_home_alt_baro == 0.0f) {
            _home_alt_baro = alt_m;
        }

        // Trong NED, Z = -Độ cao tương đối so với Home
        float z_measured = -(alt_m - _home_alt_baro);
        float err_z = z_measured - _pos[2];

        // Áp dụng bộ lọc bổ sung bậc 3 (3rd order complementary filter)
        _pos[2]        += KP_Z * err_z * dt;
        _vel[2]        += KI_Z * err_z * dt;
        _accel_bias[2] -= KD_Z * err_z * dt;
        z_valid = true;
    }

    // 4. HIỆU CHỈNH TRỤC NGANG (X, Y) - Sử dụng GPS
    sensor_gps_s gps;
    bool xy_valid = false;
    _gps_sub.copy(gps);
    if (gps.timestamp_us > 0 && gps.fix_type >= 3) {
        if (!_home_set) {
            _home_lat = gps.lat;
            _home_lon = gps.lon;
            _home_set = true;
            FC_INFO("Home Position Set: Lat=%d, Lon=%d", gps.lat, gps.lon);
        }

        // Chuyển GPS Lat/Lon thành mét trong NED frame (Flat Earth projection)
        float lat_diff = (float)(gps.lat - _home_lat) * 1e-7f;
        float lon_diff = (float)(gps.lon - _home_lon) * 1e-7f;

        float gps_x = lat_diff * 111319.5f;
        float gps_y = lon_diff * 111319.5f * cosf((float)_home_lat * 1e-7f * 0.0174532925f);

        float err_x = gps_x - _pos[0];
        float err_y = gps_y - _pos[1];

        // Lấy vận tốc từ GPS
        float err_vx = gps.vel_n_m_s - _vel[0];
        float err_vy = gps.vel_e_m_s - _vel[1];

        // Bộ lọc bổ sung bậc 3 cho trục X, Y
        _pos[0]        += KP_XY * err_x * dt;
        _pos[1]        += KP_XY * err_y * dt;
        
        _vel[0]        += KI_XY * err_x * dt + KV_XY * err_vx * dt;
        _vel[1]        += KI_XY * err_y * dt + KV_XY * err_vy * dt;

        _accel_bias[0] -= KD_XY * err_x * dt;
        _accel_bias[1] -= KD_XY * err_y * dt;
        xy_valid = true;
    }

    // 5. ĐẨY KẾT QUẢ ĐÃ LỌC LÊN uORB
    vehicle_local_position_s local_pos;
    local_pos.timestamp_us = now;
    local_pos.x = _pos[0];
    local_pos.y = _pos[1];
    local_pos.z = _pos[2];
    local_pos.vx = _vel[0];
    local_pos.vy = _vel[1];
    local_pos.vz = _vel[2];
    local_pos.xy_valid = xy_valid;
    local_pos.z_valid = z_valid;
    local_pos.v_xy_valid = xy_valid;
    local_pos.v_z_valid = z_valid;

    _local_pos_pub.publish(local_pos);
}
