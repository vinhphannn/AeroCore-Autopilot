#include "PID.h"
#include "../../Modules/Math/math_utils.h"

void PID::set_gains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::set_limits(float int_limit, float out_limit) {
    _int_limit = int_limit;
    _out_limit = out_limit;
}

void PID::set_d_filter(float sample_freq, float cutoff_freq) {
    // LowPassFilter2p bậc 2 - Butterworth, đúng chuẩn PX4 cho D-term
    _d_filter.set_cutoff_frequency(sample_freq, cutoff_freq);
}

void PID::reset() {
    _integral   = 0.0f;
    _prev_error = 0.0f;
}

float PID::update(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return 0.0f;

    const float error = setpoint - measurement;

    // Proportional
    const float p_out = _kp * error;

    // Integral với Anti-Windup
    _integral += _ki * error * dt;
    _integral = math::constrain(_integral, -_int_limit, _int_limit);

    // Derivative - lọc bằng LowPassFilter2p bậc 2 (Butterworth, Direct Form II)
    // PX4 dùng phương pháp tương tự trong mc_rate_control.cpp
    const float derivative = (error - _prev_error) / dt;
    const float d_filtered = _d_filter.apply(derivative);
    const float d_out      = _kd * d_filtered;

    _prev_error = error;

    const float output = p_out + _integral + d_out;
    return math::constrain(output, -_out_limit, _out_limit);
}
