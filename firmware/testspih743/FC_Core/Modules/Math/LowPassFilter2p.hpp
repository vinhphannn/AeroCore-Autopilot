/**
 * Port từ PX4: src/lib/mathlib/math/filter/LowPassFilter2p.hpp
 * Bộ lọc Biquad bậc 2 (Butterworth LPF, Direct Form II)
 * Đây là bộ lọc CHÍNH PX4 dùng cho Gyroscope và Accelerometer
 * Ưu điểm: Cắt sắc gấp đôi LPF bậc 1, phase shift ít hơn so với bậc 2 thông thường
 *
 * Hệ số:
 *   fr = sample_freq / cutoff_freq
 *   ohm = tan(PI / fr)
 *   c = 1 + 2*cos(PI/4)*ohm + ohm^2
 *   b0 = ohm^2 / c     b1 = 2*b0      b2 = b0
 *   a1 = 2*(ohm^2-1)/c  a2 = (1-2*cos(PI/4)*ohm+ohm^2)/c
 */
#pragma once

#include <math.h>
#include <float.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

template <typename T>
class LowPassFilter2p {
public:
    LowPassFilter2p() { disable(); }

    LowPassFilter2p(float sample_freq, float cutoff_freq) {
        set_cutoff_frequency(sample_freq, cutoff_freq);
    }

    void set_cutoff_frequency(float sample_freq, float cutoff_freq) {
        if ((sample_freq <= 0.f) || (cutoff_freq <= 0.f) || (cutoff_freq >= sample_freq / 2.f)) {
            disable();
            return;
        }

        // Reset delay elements khi thay đổi tham số
        _delay_element_1 = T{};
        _delay_element_2 = T{};

        _cutoff_freq = (cutoff_freq > sample_freq * 0.001f) ? cutoff_freq : sample_freq * 0.001f;
        _sample_freq = sample_freq;

        const float fr  = _sample_freq / _cutoff_freq;
        const float ohm = tanf(M_PI_F / fr);
        const float c   = 1.f + 2.f * cosf(M_PI_F / 4.f) * ohm + ohm * ohm;

        _b0 = ohm * ohm / c;
        _b1 = 2.f * _b0;
        _b2 = _b0;
        _a1 = 2.f * (ohm * ohm - 1.f) / c;
        _a2 = (1.f - 2.f * cosf(M_PI_F / 4.f) * ohm + ohm * ohm) / c;

        // Kiểm tra NaN/Inf
        if (!isfinite(_b0) || !isfinite(_a1) || !isfinite(_a2)) {
            disable();
        }
    }

    // Áp dụng bộ lọc (Direct Form II)
    inline T apply(const T &sample) {
        T delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
        T output          = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;

        _delay_element_2 = _delay_element_1;
        _delay_element_1 = delay_element_0;

        return output;
    }

    // Reset trạng thái bộ lọc về giá trị mẫu đầu vào
    T reset(const T &sample) {
        const T input = isfinite(sample) ? sample : T{};

        if (fabsf(1.f + _a1 + _a2) > FLT_EPSILON) {
            _delay_element_1 = _delay_element_2 = input / (1.f + _a1 + _a2);
            if (!isfinite(_delay_element_1)) {
                _delay_element_1 = _delay_element_2 = input;
            }
        } else {
            _delay_element_1 = _delay_element_2 = input;
        }
        return apply(input);
    }

    void disable() {
        _sample_freq = 0.f; _cutoff_freq = 0.f;
        _delay_element_1 = _delay_element_2 = T{};
        _b0 = 1.f; _b1 = 0.f; _b2 = 0.f;
        _a1 = 0.f; _a2 = 0.f;
    }

    float get_cutoff_freq() const { return _cutoff_freq; }
    float get_sample_freq() const { return _sample_freq; }

protected:
    T _delay_element_1{}; // z^-1
    T _delay_element_2{}; // z^-2

    float _a1{0.f}, _a2{0.f};          // Hệ số phản hồi
    float _b0{1.f}, _b1{0.f}, _b2{0.f}; // Hệ số tiến

    float _cutoff_freq{0.f};
    float _sample_freq{0.f};
};
