#pragma once

#include <stdint.h>
#include <math.h>

class MahonyAHRS {
public:
    MahonyAHRS();
    ~MahonyAHRS() = default;

    /**
     * @brief Cập nhật bộ lọc với Cả IMU và La bàn (9-DOF)
     * @param gx, gy, gz Tốc độ góc (Radian/s)
     * @param ax, ay, az Gia tốc (m/s^2 hoặc g)
     * @param mx, my, mz Từ trường (Tùy ý, miễn là cùng đơn vị)
     * @param dt Delta time (giây)
     */
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);

    /**
     * @brief Cập nhật bộ lọc chỉ với IMU (6-DOF)
     */
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    
    float getRoll() const { return _roll; }
    float getPitch() const { return _pitch; }
    float getYaw() const { return _yaw; }
    
    void getQuaternion(float* q) const {
        q[0] = _q0; q[1] = _q1; q[2] = _q2; q[3] = _q3;
    }

private:
    float _twoKp;    // 2 * proportional gain
    float _twoKi;    // 2 * integral gain
    
    float _q0, _q1, _q2, _q3; // quaternion of sensor frame relative to auxiliary frame
    float _integralFBx, _integralFBy, _integralFBz; // integral error terms scaled by Ki
    
    float _roll, _pitch, _yaw;
    
    void computeAngles();
};
