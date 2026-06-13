/**
 * Port từ PX4: src/lib/rate_control/rate_control.cpp
 */
#include "RateControl.hpp"

void RateControl::setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D) {
    _gain_p = P;
    _gain_i = I;
    _gain_d = D;
}

void RateControl::setSaturationStatus(const bool sat_pos[3], const bool sat_neg[3]) {
    for (int i = 0; i < 3; i++) {
        _sat_pos[i] = sat_pos[i];
        _sat_neg[i] = sat_neg[i];
    }
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp,
                              const Vector3f &angular_accel, float dt, bool landed)
{
    // Sai số vận tốc góc
    Vector3f rate_error = rate_sp - rate;

    // ---------------------------------------------------------
    // CÔNG THỨC CHUẨN PX4 rate_control.cpp dòng 78:
    //   torque = P*rate_error + I_integral - D*angular_accel + FF*rate_sp
    //
    // Quan trọng: D-term dùng ANGULAR_ACCEL (đạo hàm vận tốc góc thực tế)
    // KHÔNG dùng (error - prev_error)/dt
    // Lý do: Khi rate_sp nhảy đột ngột, (d_error/dt) bùng nổ -> giật cục
    //        angular_accel đã được lọc LPF2p từ Gyro -> mượt hơn nhiều
    // ---------------------------------------------------------
    Vector3f torque = _gain_p.emult(rate_error)
                    + _rate_int
                    - _gain_d.emult(angular_accel)
                    + _gain_ff.emult(rate_sp);

    // Chỉ cập nhật integral khi máy bay KHÔNG đang đậu đất
    if (!landed) {
        updateIntegral(rate_error, dt);
    }

    return torque;
}

void RateControl::updateIntegral(Vector3f &rate_error, float dt) {
    for (int i = 0; i < 3; i++) {
        // Anti-windup: Nếu actuator đã bão hòa theo chiều dương -> chỉ cho phép giảm integral
        if (_sat_pos[i]) {
            rate_error(i) = math_px4::min(rate_error(i), 0.f);
        }
        // Anti-windup: Nếu actuator đã bão hòa theo chiều âm -> chỉ cho phép tăng integral
        if (_sat_neg[i]) {
            rate_error(i) = math_px4::max(rate_error(i), 0.f);
        }

        // -------------------------------------------------------
        // PX4 i_factor: Giảm I-gain khi rate_error LỚN
        // Mục đích: Sau khi flip/maneuver mạnh, error đột ngột lớn
        // -> nếu không giảm, integral tích lũy quá lớn -> bounce-back
        // Công thức: i_factor = max(0, 1 - (error / 400deg)^2)
        // -------------------------------------------------------
        float i_factor = rate_error(i) / math_px4::radians(400.0f);
        i_factor = math_px4::max(0.0f, 1.0f - i_factor * i_factor);

        float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

        // Chỉ cập nhật nếu giá trị hợp lệ (không phải NaN/Inf)
        if (rate_i == rate_i && rate_i < 1e6f && rate_i > -1e6f) {
            _rate_int(i) = math_px4::constrain(rate_i, -_lim_int(i), _lim_int(i));
        }
    }
}
