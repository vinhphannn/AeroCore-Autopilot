/**
 * Port từ PX4: ControlAllocationSequentialDesaturation.cpp
 * Nâng cấp với arm length ratio và airmode support
 */
#include "ControlAllocation.hpp"
#include <string.h>

ControlAllocation::ControlAllocation() {
    // Mặc định: arm_lx = arm_ly = 150mm (Quad vuông đều)
    updateGeometry(150.0f, 150.0f, 0);
}

void ControlAllocation::updateGeometry(float arm_lx, float arm_ly, int airmode) {
    // arm_lx và arm_ly bị bỏ trong hàm này, dùng overload mới dưới đây
    (void)arm_lx; (void)arm_ly;
    _airmode = airmode;
}

void ControlAllocation::updateGeometryPerRotor(
    float px[4], float py[4], float km[4], int airmode)
{
    _airmode = airmode;

    // -------------------------------------------------------------------------
    // XÂY DỰNG GEOMETRY MATRIX từ vị trí và hướng quay TỪNG ROTOR
    // Đúng chuẩn PX4: CA_ROTOR${i}_PX, CA_ROTOR${i}_PY, CA_ROTOR${i}_KM
    //
    // Torque contribution của motor i:
    //   Roll  ∝ -py[i]    (vị trí Y: dương Y -> Roll âm khi thrust+)
    //   Pitch ∝  px[i]    (vị trí X: dương X -> Pitch dương khi thrust+)
    //   Yaw   ∝  km[i]    (hướng quay: +1=CCW tạo Yaw+, -1=CW tạo Yaw-)
    //   Thrust = 1        (tất cả motor đều nâng)
    //
    // Tham chiếu PX4: src/lib/control_allocation/actuator_effectiveness/
    //   ActuatorEffectivenessRotors.cpp :: computeEffectivenessMatrix()
    // -------------------------------------------------------------------------
    for (int i = 0; i < NUM_MOTORS; i++) {
        _B[i][0] = -py[i]; // Roll  contribution
        _B[i][1] =  px[i]; // Pitch contribution
        _B[i][2] =  km[i]; // Yaw   contribution
        _B[i][3] =  1.0f;  // Thrust contribution
    }

    // Chuẩn hóa từng cột (đúng PX4 normalizeControlAllocationMatrix):
    // Roll và Pitch scale bằng max abs để map về [-1, 1]
    float max_rp = FLT_MIN;
    for (int m = 0; m < NUM_MOTORS; m++) {
        float ar = fabsf(_B[m][0]), ap = fabsf(_B[m][1]);
        if (ar > max_rp) max_rp = ar;
        if (ap > max_rp) max_rp = ap;
    }
    float max_yaw = FLT_MIN;
    for (int m = 0; m < NUM_MOTORS; m++) {
        float ay = fabsf(_B[m][2]);
        if (ay > max_yaw) max_yaw = ay;
    }

    if (max_rp > FLT_MIN) {
        for (int m = 0; m < NUM_MOTORS; m++) {
            _B[m][0] /= max_rp; // Roll
            _B[m][1] /= max_rp; // Pitch
        }
    }
    if (max_yaw > FLT_MIN) {
        for (int m = 0; m < NUM_MOTORS; m++) {
            _B[m][2] /= max_yaw; // Yaw
        }
    }
    // Thrust đã chuẩn hóa (= 1.0 cho tất cả motor)
}

float ControlAllocation::computeDesaturationGain(const float vec[NUM_MOTORS],
                                                   const float sp[NUM_MOTORS]) {
    float k_min = 0.f, k_max = 0.f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (fabsf(vec[i]) < 0.2f) continue;
        if (sp[i] < ACTUATOR_MIN) {
            float k = (ACTUATOR_MIN - sp[i]) / vec[i];
            if (k < k_min) k_min = k;
            if (k > k_max) k_max = k;
        }
        if (sp[i] > ACTUATOR_MAX) {
            float k = (ACTUATOR_MAX - sp[i]) / vec[i];
            if (k < k_min) k_min = k;
            if (k > k_max) k_max = k;
        }
    }
    return k_min + k_max;
}

void ControlAllocation::desaturateActuators(float sp[NUM_MOTORS],
                                             const float vec[NUM_MOTORS],
                                             bool increase_only) {
    float gain = computeDesaturationGain(vec, sp);
    if (increase_only && gain < 0.f) return;
    for (int i = 0; i < NUM_MOTORS; i++) sp[i] += gain * vec[i];
    gain = 0.5f * computeDesaturationGain(vec, sp); // Fine correction
    for (int i = 0; i < NUM_MOTORS; i++) sp[i] += gain * vec[i];
}

void ControlAllocation::mixYaw(float sp[NUM_MOTORS], float yaw_delta) {
    float yaw_vec[NUM_MOTORS], thrust_vec[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
        sp[i]        += _B[i][2] * yaw_delta;
        yaw_vec[i]    = _B[i][2];
        thrust_vec[i] = _B[i][3];
    }
    // desaturate Yaw
    float gain_yaw = computeDesaturationGain(yaw_vec, sp);
    for (int i = 0; i < NUM_MOTORS; i++) sp[i] += gain_yaw * yaw_vec[i];
    // Chỉ giảm Thrust
    desaturateActuators(sp, thrust_vec, true);
}

void ControlAllocation::mixAirmodeDisabled(float sp[NUM_MOTORS],
    float roll, float pitch, float yaw, float thrust) {
    // Mix Thrust + Roll + Pitch (không có Yaw)
    float thrust_vec[NUM_MOTORS], roll_vec[NUM_MOTORS], pitch_vec[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
        sp[i]         = _B[i][3]*thrust + _B[i][0]*roll + _B[i][1]*pitch;
        thrust_vec[i] = _B[i][3];
        roll_vec[i]   = _B[i][0];
        pitch_vec[i]  = _B[i][1];
    }
    desaturateActuators(sp, thrust_vec, true); // Chỉ giảm Thrust
    desaturateActuators(sp, roll_vec);
    desaturateActuators(sp, pitch_vec);
    mixYaw(sp, yaw); // Yaw cuối cùng
}

void ControlAllocation::mixAirmodeRP(float sp[NUM_MOTORS],
    float roll, float pitch, float yaw, float thrust) {
    float thrust_vec[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
        sp[i]         = _B[i][3]*thrust + _B[i][0]*roll + _B[i][1]*pitch;
        thrust_vec[i] = _B[i][3];
    }
    desaturateActuators(sp, thrust_vec); // Cho phép tăng/giảm Thrust
    mixYaw(sp, yaw);
}

void ControlAllocation::mixAirmodeRPY(float sp[NUM_MOTORS],
    float roll, float pitch, float yaw, float thrust) {
    float thrust_vec[NUM_MOTORS], yaw_vec[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
        sp[i]         = _B[i][3]*thrust + _B[i][0]*roll + _B[i][1]*pitch + _B[i][2]*yaw;
        thrust_vec[i] = _B[i][3];
        yaw_vec[i]    = _B[i][2];
    }
    desaturateActuators(sp, thrust_vec);
    desaturateActuators(sp, yaw_vec); // Yaw cũng được desaturate
}

void ControlAllocation::detectSaturation(const float sp[NUM_MOTORS], MixerSaturation &sat) {
    sat = {false,false,false,false,false,false};
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (sp[i] >= ACTUATOR_MAX - 0.01f) {
            if (_B[i][0] >  0.1f) sat.roll_pos  = true;
            if (_B[i][0] < -0.1f) sat.roll_neg  = true;
            if (_B[i][1] >  0.1f) sat.pitch_pos = true;
            if (_B[i][1] < -0.1f) sat.pitch_neg = true;
            if (_B[i][2] >  0.1f) sat.yaw_pos   = true;
            if (_B[i][2] < -0.1f) sat.yaw_neg   = true;
        }
        if (sp[i] <= ACTUATOR_MIN + 0.01f) {
            if (_B[i][0] < -0.1f) sat.roll_pos  = true;
            if (_B[i][0] >  0.1f) sat.roll_neg  = true;
            if (_B[i][1] < -0.1f) sat.pitch_pos = true;
            if (_B[i][1] >  0.1f) sat.pitch_neg = true;
            if (_B[i][2] < -0.1f) sat.yaw_pos   = true;
            if (_B[i][2] >  0.1f) sat.yaw_neg   = true;
        }
    }
}

void ControlAllocation::allocate(float roll, float pitch, float yaw, float thrust,
                                  float motor_out[NUM_MOTORS], MixerSaturation &sat) {
    float sp[NUM_MOTORS] = {};

    // Chọn thuật toán theo Airmode param (đúng PX4)
    switch (_airmode) {
        case 1:  mixAirmodeRP(sp,  roll, pitch, yaw, thrust); break;
        case 2:  mixAirmodeRPY(sp, roll, pitch, yaw, thrust); break;
        default: mixAirmodeDisabled(sp, roll, pitch, yaw, thrust); break;
    }

    detectSaturation(sp, sat);

    // Clamp và output
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (sp[i] < ACTUATOR_MIN) sp[i] = ACTUATOR_MIN;
        if (sp[i] > ACTUATOR_MAX) sp[i] = ACTUATOR_MAX;
        motor_out[i] = sp[i];
    }
}
