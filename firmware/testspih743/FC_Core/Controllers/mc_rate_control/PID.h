#pragma once

#include <stdint.h>
#include "../../Modules/Math/LowPassFilter2p.hpp"

/**
 * PID Controller với D-term filter (LowPassFilter2p bậc 2 - đúng chuẩn PX4)
 * PX4 reference: src/modules/mc_att_control/mc_att_control_main.cpp
 *   -> Rate PID dùng LPF bậc 2 cho D-term để chặt nhiễu motor
 */
class PID {
public:
    PID() = default;
    ~PID() = default;

    void set_gains(float kp, float ki, float kd);
    void set_limits(float int_limit, float out_limit);

    /**
     * Cấu hình D-term filter (LowPassFilter2p bậc 2 - Butterworth)
     * @param sample_freq: Tần số vòng lặp PID (Hz)
     * @param cutoff_freq: Tần số cắt D-term (Hz), PX4 mặc định ~20-30Hz
     */
    void set_d_filter(float sample_freq, float cutoff_freq);

    void reset();

    float update(float setpoint, float measurement, float dt);

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _integral = 0.0f;
    float _prev_error = 0.0f;

    float _int_limit = 1.0f;
    float _out_limit = 1.0f;

    // D-term filter: LowPassFilter2p bậc 2 (Butterworth) - đúng chuẩn PX4
    LowPassFilter2p<float> _d_filter;
};
