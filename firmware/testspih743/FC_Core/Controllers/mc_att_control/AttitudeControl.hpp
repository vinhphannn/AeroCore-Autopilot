/**
 * Port từ PX4: src/modules/mc_att_control/AttitudeControl/
 *
 * AttitudeControl - Bộ điều khiển Góc (Outer Loop, Proportional-only)
 *
 * INPUT:  q [4]      - Quaternion thực tế của máy bay (từ Mahony/EKF)
 *         qd [4]     - Quaternion mong muốn (từ RC stick hoặc Mission)
 *         yawspeed_sp - Feed-forward yaw rate [rad/s]
 *
 * OUTPUT: rate_sp [rad/s] - Target angular rates cho RateControl (Inner Loop)
 *
 * THIẾT KẾ PX4:
 *   - Dùng Quaternion thay vì Euler để tránh Gimbal Lock
 *   - Tách riêng error Roll/Pitch và Yaw để có thể de-prioritize Yaw
 *   - CHỈ là bộ điều khiển tỷ lệ (P-only), KHÔNG có I và D
 *     Lý do: I và D nằm trong RateControl (inner loop), không cần ở outer loop
 */
#pragma once

#include "../mc_rate_control/RateControl.hpp"

#include <math.h>
#include <string.h>

// Quaternion đơn giản [w, x, y, z]
struct Quatf {
    float w, x, y, z;
    Quatf() : w(1), x(0), y(0), z(0) {}
    Quatf(float _w, float _x, float _y, float _z) : w(_w), x(_x), y(_y), z(_z) {}

    // q_inv * q_other (Hamilton product)
    Quatf inversed() const { return {w, -x, -y, -z}; }

    Quatf operator*(const Quatf &b) const {
        return {
            w*b.w - x*b.x - y*b.y - z*b.z,
            w*b.x + x*b.w + y*b.z - z*b.y,
            w*b.y - x*b.z + y*b.w + z*b.x,
            w*b.z + x*b.y - y*b.x + z*b.w
        };
    }

    void normalize() {
        float n = sqrtf(w*w + x*x + y*y + z*z);
        if (n > 1e-6f) { w /= n; x /= n; y /= n; z /= n; }
    }

    // Lấy phần ảo (vector phần của quaternion error) = [x, y, z]
    // Tương đương PX4: 2.f * qe.canonical().imag()
    Vector3f get_attitude_error() const {
        // Đảm bảo w >= 0 (canonical form) để tránh ambiguity ±q = cùng rotation
        float sign = (w >= 0.0f) ? 1.0f : -1.0f;
        return { 2.0f * sign * x, 2.0f * sign * y, 2.0f * sign * z };
    }

    // Hàm tạo từ Roll/Pitch/Yaw (Euler ZYX)
    static Quatf from_euler(float roll, float pitch, float yaw) {
        float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
        float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
        float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);
        return {
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy
        };
    }

    // Lấy cột Z của ma trận xoay (dùng để tính attitude error rút gọn)
    Vector3f dcm_z() const {
        return {
            2.0f * (x*z + w*y),
            2.0f * (y*z - w*x),
            w*w - x*x - y*y + z*z
        };
    }
};

class AttitudeControl {
public:
    AttitudeControl() = default;
    ~AttitudeControl() = default;

    /**
     * Cấu hình P-gain (đúng API PX4: setProportionalGain)
     * @param gain       [Roll_P, Pitch_P, Yaw_P]
     * @param yaw_weight [0,1] Giảm ưu tiên của Yaw so với Roll/Pitch
     *                   PX4 mặc định 0.4 — Yaw có độ trễ chấp nhận được trong maneuver
     */
    void setProportionalGain(const Vector3f &gain, float yaw_weight);
    void setRateLimit(const Vector3f &limit) { _rate_limit = limit; }

    /**
     * Đặt mục tiêu góc quay
     * @param qd quaternion mong muốn
     * @param yawspeed_ff feed-forward yaw rate [rad/s]
     */
    void setAttitudeSetpoint(const Quatf &qd, float yawspeed_ff) {
        _att_sp_q = qd;
        _att_sp_q.normalize();
        _yawspeed_ff = yawspeed_ff;
    }

    /**
     * Chạy 1 chu kỳ (đúng API PX4: update(const Quatf &q))
     * @param  q Quaternion thực tế
     * @return   [rad/s] rate setpoint cho RateControl
     */
    Vector3f update(const Quatf &q) const;

private:
    Vector3f _gain_p;
    Vector3f _rate_limit;
    float _yaw_w = 0.4f;

    Quatf _att_sp_q;
    float _yawspeed_ff = 0.0f;
};
