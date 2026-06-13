/**
 * Port từ PX4: src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
 * (Đơn giản hóa phần tách Yaw/Roll-Pitch, giữ nguyên toán học Quaternion Error)
 */
#include "AttitudeControl.hpp"

void AttitudeControl::setProportionalGain(const Vector3f &gain, float yaw_weight) {
    _gain_p = gain;
    _yaw_w  = math_px4::constrain(yaw_weight, 0.0f, 1.0f);

    // Bù ảnh hưởng rescaling của yaw_weight (đúng theo PX4)
    if (_yaw_w > 1e-4f) {
        _gain_p.z /= _yaw_w;
    }
}

Vector3f AttitudeControl::update(const Quatf &q) const {
    Quatf qd = _att_sp_q;

    // ----------------------------------------------------------------
    // PX4 THUẬT TOÁN: "Quaternion reduced attitude" (tách Roll/Pitch khỏi Yaw)
    // Mục đích: Đảm bảo Roll/Pitch được ưu tiên cao hơn Yaw
    //
    // 1. Tính qd_red = Quaternion nhỏ nhất xoay vector Z_current -> Z_desired
    //    (Chỉ sửa Roll/Pitch, Yaw bị bỏ qua trong bước này)
    // 2. Scale phần Yaw error xuống theo yaw_weight
    // 3. Tổng hợp lại: qd = qd_red * qd_dyaw_scaled
    // ----------------------------------------------------------------
    Vector3f e_z  = q.dcm_z();   // Hướng Z thực tế (body frame)
    Vector3f e_zd = qd.dcm_z();  // Hướng Z mong muốn

    // Quaternion xoay ngắn nhất từ e_z -> e_zd
    // q_reduced = [cos(a/2), sin(a/2)*axis] với axis = cross(e_z, e_zd)
    float dot    = math_px4::constrain(e_z.x*e_zd.x + e_z.y*e_zd.y + e_z.z*e_zd.z, -1.0f, 1.0f);
    float angle  = acosf(dot);
    Vector3f cross = { e_z.y*e_zd.z - e_z.z*e_zd.y,
                       e_z.z*e_zd.x - e_z.x*e_zd.z,
                       e_z.x*e_zd.y - e_z.y*e_zd.x };
    float cross_norm = sqrtf(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);

    Quatf qd_red;
    if (cross_norm > 1e-6f && angle > 1e-6f) {
        float s = sinf(angle * 0.5f) / cross_norm;
        qd_red = Quatf(cosf(angle * 0.5f), cross.x*s, cross.y*s, cross.z*s);
        qd_red = qd_red * q; // world frame -> body frame
    } else {
        qd_red = q; // Không cần xoay gì
    }

    // Trích xuất phần Yaw delta
    Quatf q_inv = q.inversed();
    Quatf qd_dyaw = (qd_red.inversed()) * qd;
    // Đảm bảo w >= 0 (canonical)
    if (qd_dyaw.w < 0.0f) {
        qd_dyaw.w = -qd_dyaw.w; qd_dyaw.x = -qd_dyaw.x;
        qd_dyaw.y = -qd_dyaw.y; qd_dyaw.z = -qd_dyaw.z;
    }
    qd_dyaw.w = math_px4::constrain(qd_dyaw.w, -1.0f, 1.0f);
    qd_dyaw.z = math_px4::constrain(qd_dyaw.z, -1.0f, 1.0f);

    // Scale Yaw và tổng hợp lại setpoint
    float yaw_half = _yaw_w * acosf(qd_dyaw.w);
    Quatf qd_final = qd_red * Quatf(cosf(yaw_half), 0.0f, 0.0f, sinf(yaw_half));

    // ----------------------------------------------------------------
    // Tính Quaternion Error: qe = q_inv * qd_final
    // rate_setpoint = 2 * qe.imag() * P_gain  (đúng PX4 dòng 92-95)
    // ----------------------------------------------------------------
    Quatf qe = q.inversed() * qd_final;
    Vector3f eq = qe.get_attitude_error(); // = 2 * sign(w) * [x, y, z]

    // P controller: rate = error * gain
    Vector3f rate_sp = eq.emult(_gain_p);

    // Feed-forward Yaw (từ tay cầm - Yaw stick)
    if (_yawspeed_ff != 0.0f && _yawspeed_ff == _yawspeed_ff) {
        // Chiếu world-Z vào body frame (cột Z của R^T = q.inversed().dcm_z)
        Vector3f body_z = q.inversed().dcm_z();
        rate_sp.x += body_z.x * _yawspeed_ff;
        rate_sp.y += body_z.y * _yawspeed_ff;
        rate_sp.z += body_z.z * _yawspeed_ff;
    }

    // Giới hạn rate setpoint
    rate_sp.x = math_px4::constrain(rate_sp.x, -_rate_limit.x, _rate_limit.x);
    rate_sp.y = math_px4::constrain(rate_sp.y, -_rate_limit.y, _rate_limit.y);
    rate_sp.z = math_px4::constrain(rate_sp.z, -_rate_limit.z, _rate_limit.z);

    return rate_sp;
}
