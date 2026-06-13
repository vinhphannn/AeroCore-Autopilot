/**
 * Port từ PX4: src/lib/mathlib/math/filter/NotchFilter.hpp
 * Bộ lọc Notch bậc 2 (Dual Notch / Band-stop)
 * PX4 dùng bộ lọc này để CHẶT ĐỨT các tần số rung cộng hưởng CỤ THỂ
 * Ví dụ: khung carbon cộng hưởng ở 120Hz -> Set notch_freq = 120Hz để lọc bỏ
 *
 * Hệ số (Direct Form I):
 *   alpha = tan(PI * bandwidth / sample_freq)
 *   beta  = -cos(2 * PI * notch_freq / sample_freq)
 *   a0    = alpha + 1
 *   b0 = b2 = 1/a0
 *   b1 = a1 = 2*beta/a0
 *   a2 = (1 - alpha)/a0
 */
#pragma once

#include <math.h>
#include <float.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

template <typename T>
class NotchFilter {
public:
    NotchFilter() { disable(); }
    ~NotchFilter() = default;

    bool setParameters(float sample_freq, float notch_freq, float bandwidth) {
        if ((sample_freq <= 0.f) || (notch_freq <= 0.f) || (bandwidth <= 0.f)
            || (notch_freq >= sample_freq / 2.f)) {
            disable();
            return false;
        }

        const float freq_min = sample_freq * 0.001f;
        _sample_freq = sample_freq;
        _notch_freq  = (notch_freq > freq_min) ? notch_freq : freq_min;
        _bandwidth   = (bandwidth  > freq_min) ? bandwidth  : freq_min;

        const float alpha   = tanf(M_PI_F * _bandwidth   / _sample_freq);
        const float beta    = -cosf(2.f * M_PI_F * _notch_freq / _sample_freq);
        const float a0_inv  = 1.f / (alpha + 1.f);

        _b0 = a0_inv;
        _b1 = 2.f * beta * a0_inv;
        _b2 = a0_inv;
        _a1 = _b1;
        _a2 = (1.f - alpha) * a0_inv;

        if (!isfinite(_b0) || !isfinite(_b1) || !isfinite(_b2) || !isfinite(_a2)) {
            disable();
            return false;
        }

        if (!_initialized) {
            reset(_delay_element_1); // Khởi tạo với giá trị hiện tại
        }
        return true;
    }

    // Áp dụng bộ lọc (Direct Form I - Cho phép cập nhật hệ số online)
    inline T apply(const T &sample) {
        if (!_initialized) {
            reset(sample);
        }

        T output = _b0 * sample + _b1 * _delay_element_1 + _b2 * _delay_element_2
                   - _a1 * _delay_element_out_1 - _a2 * _delay_element_out_2;

        _delay_element_2     = _delay_element_1;
        _delay_element_1     = sample;
        _delay_element_out_2 = _delay_element_out_1;
        _delay_element_out_1 = output;

        return output;
    }

    void reset(const T &sample) {
        const T input = isfinite(sample) ? sample : T{};
        _delay_element_1 = _delay_element_2 = input;
        float denom = 1.f + _a1 + _a2;
        if (fabsf(denom) > FLT_EPSILON) {
            _delay_element_out_1 = _delay_element_out_2 = input * (_b0 + _b1 + _b2) / denom;
        } else {
            _delay_element_out_1 = _delay_element_out_2 = T{};
        }
        _initialized = true;
    }

    void disable() {
        _b0 = 1.f; _b1 = 0.f; _b2 = 0.f;
        _a1 = 0.f; _a2 = 0.f;
        _delay_element_1 = _delay_element_2 = T{};
        _delay_element_out_1 = _delay_element_out_2 = T{};
        _notch_freq = 0.f; _bandwidth = 0.f; _sample_freq = 0.f;
        _initialized = false;
    }

    float getNotchFreq()  const { return _notch_freq; }
    float getBandwidth()  const { return _bandwidth; }
    bool  initialized()   const { return _initialized; }

private:
    T _delay_element_1{};
    T _delay_element_2{};
    T _delay_element_out_1{};
    T _delay_element_out_2{};

    float _a1{0.f}, _a2{0.f};
    float _b0{1.f}, _b1{0.f}, _b2{0.f};

    float _notch_freq{0.f};
    float _bandwidth{0.f};
    float _sample_freq{0.f};

    bool _initialized{false};
};
