/**
 * Port từ PX4: src/lib/mathlib/math/filter/AlphaFilter.hpp
 * Bộ lọc IIR bậc 1 (PT1 / Leaky Integrator)
 * Công thức: state = state + alpha * (sample - state)
 * Dùng cho: tín hiệu chậm, Barometer, GPS velocity...
 */
#pragma once

#include <math.h>
#include <float.h>

#ifndef M_TWOPI_F
#define M_TWOPI_F 6.28318530717958647692f
#endif

template <typename T>
class AlphaFilter {
public:
    AlphaFilter() = default;
    explicit AlphaFilter(float sample_interval, float time_constant) {
        setParameters(sample_interval, time_constant);
    }
    ~AlphaFilter() = default;

    // Thiết lập theo khoảng thời gian lấy mẫu (giây) và hằng số thời gian (giây)
    void setParameters(float sample_interval, float time_constant) {
        const float denominator = time_constant + sample_interval;
        if (denominator > FLT_EPSILON) {
            _alpha = sample_interval / denominator;
        }
        _time_constant = time_constant;
    }

    // Thiết lập theo tần số lấy mẫu (Hz) và tần số cắt (Hz)
    bool setCutoffFreq(float sample_freq, float cutoff_freq) {
        if ((sample_freq <= 0.f) || (cutoff_freq <= 0.f) || (cutoff_freq >= sample_freq / 2.f)) {
            return false;
        }
        setParameters(1.f / sample_freq, 1.f / (M_TWOPI_F * cutoff_freq));
        return true;
    }

    void setAlpha(float alpha) { _alpha = alpha; }

    // Reset trạng thái bộ lọc về giá trị ban đầu
    void reset(const T &sample) { _filter_state = sample; }

    // Cập nhật bộ lọc với mẫu mới
    const T &update(const T &sample) {
        _filter_state = _filter_state + _alpha * (sample - _filter_state);
        return _filter_state;
    }

    // Cập nhật với dt động (Tự tính lại alpha mỗi lần)
    const T update(const T &sample, float dt) {
        setParameters(dt, _time_constant);
        return update(sample);
    }

    const T &getState() const { return _filter_state; }
    float getCutoffFreq() const {
        return (_time_constant > FLT_EPSILON) ? 1.f / (M_TWOPI_F * _time_constant) : 0.f;
    }

private:
    float _time_constant{0.f};
    float _alpha{0.f};
    T _filter_state{};
};
