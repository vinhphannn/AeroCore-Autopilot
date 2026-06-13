/**
 * Port từ PX4: src/lib/rate_control/rate_control.hpp
 * PX4 RateControl - Bộ điều khiển Vận tốc Góc 3 trục (Inner Loop)
 *
 * INPUT:  rate     [rad/s] - Vận tốc góc thực tế (từ Gyro vehicle_imu)
 *         rate_sp  [rad/s] - Vận tốc góc mong muốn (từ AttitudeControl)
 *         angular_accel [rad/s^2] - Gia tốc góc (dùng cho D-term, lấy từ Gyro đã lọc LPF2p)
 *         dt [s]           - Thời gian bước lặp
 *
 * OUTPUT: torque [normalized -1,1] - Lực xoắn cần xuất ra 3 trục (Roll, Pitch, Yaw)
 *
 * THIẾT KẾ: Khác với PID thông thường:
 *   - D-term dùng gia tốc góc THỰC TẾ (đã lọc sẵn từ IMU), không dùng đạo hàm của error
 *     Lý do: Tránh khuếch đại nhiễu khi setpoint nhảy đột ngột
 *   - Anti-windup theo "i_factor": Giảm I-gain khi error lớn (sau flip/maneuver mạnh)
 *   - Saturation feedback: Khi actuator đã bão hòa, ngừng cộng thêm vào Integral
 */
#pragma once

#include <math.h>
#include <float.h>
#include <stdint.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

// Hàm toán học cần thiết (tương đương lib/mathlib)
namespace math_px4 {
    template<typename T>
    inline T constrain(T val, T min_val, T max_val) {
        if (val < min_val) return min_val;
        if (val > max_val) return max_val;
        return val;
    }
    inline float min(float a, float b) { return a < b ? a : b; }
    inline float max(float a, float b) { return a > b ? a : b; }
    inline float radians(float deg) { return deg * M_PI_F / 180.0f; }
}

struct Vector3f {
    float x, y, z;
    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    Vector3f operator+(const Vector3f &o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vector3f operator-(const Vector3f &o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vector3f operator*(float s)           const { return {x*s,   y*s,   z*s  }; }
    Vector3f emult(const Vector3f &o)     const { return {x*o.x, y*o.y, z*o.z}; } // Element-wise multiply
    float &operator()(int i) { return (&x)[i]; }
    const float &operator()(int i) const { return (&x)[i]; }
    void zero() { x = y = z = 0.0f; }
};

// Trạng thái bão hòa actuator (phản hồi từ Mixer)
struct RateControlSaturation {
    bool positive[3]; // Motor đã đạt Max theo trục [Roll, Pitch, Yaw]
    bool negative[3]; // Motor đã đạt Min theo trục [Roll, Pitch, Yaw]
};

class RateControl {
public:
    RateControl() = default;
    ~RateControl() = default;

    /**
     * Cấu hình PID Gains cho cả 3 trục cùng lúc (đúng API của PX4)
     * @param P [Roll, Pitch, Yaw] Proportional gain
     * @param I [Roll, Pitch, Yaw] Integral gain
     * @param D [Roll, Pitch, Yaw] Derivative gain
     */
    void setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D);

    void setIntegratorLimit(const Vector3f &lim) { _lim_int = lim; }
    void setFeedForwardGain(const Vector3f &FF)  { _gain_ff = FF; }

    // Phản hồi Saturation từ Mixer (Anti-windup chuẩn PX4)
    void setSaturationStatus(const bool sat_pos[3], const bool sat_neg[3]);

    /**
     * Chạy 1 chu kỳ điều khiển (đúng chữ ký của PX4 RateControl::update)
     * @param rate         [rad/s] Vận tốc góc thực tế (từ Gyro)
     * @param rate_sp      [rad/s] Vận tốc góc đặt (từ AttitudeControl)
     * @param angular_accel [rad/s^2] Gia tốc góc (đã lọc LPF2p, dùng làm D-term)
     * @param dt           [s] Delta thời gian
     * @param landed       Máy bay đang đậu đất (tắt cộng thêm integral)
     * @return             [-1,1] Torque 3 trục
     */
    Vector3f update(const Vector3f &rate, const Vector3f &rate_sp,
                    const Vector3f &angular_accel, float dt, bool landed);

    void resetIntegral()          { _rate_int.zero(); }
    void resetIntegral(int axis)  { if (axis < 3) _rate_int(axis) = 0.f; }

    // Lấy trạng thái tích phân để log
    Vector3f getIntegral() const { return _rate_int; }

private:
    // PX4 updateIntegral: Anti-windup với i_factor và saturation check
    void updateIntegral(Vector3f &rate_error, float dt);

    Vector3f _gain_p;  // Proportional gain [Roll, Pitch, Yaw]
    Vector3f _gain_i;  // Integral gain
    Vector3f _gain_d;  // Derivative gain
    Vector3f _lim_int; // Giới hạn Integral
    Vector3f _gain_ff; // Feed forward gain (cho Helicopter, để 0 với MC)

    Vector3f _rate_int; // Trạng thái tích phân hiện tại

    // Phản hồi bão hòa từ Mixer (Anti-windup chuẩn PX4)
    bool _sat_pos[3] = {false, false, false};
    bool _sat_neg[3] = {false, false, false};
};
