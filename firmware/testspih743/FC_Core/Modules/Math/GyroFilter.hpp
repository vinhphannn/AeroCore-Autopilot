/**
 * Adapted from PX4: src/lib/mathlib/math/filter/LowPassFilter2p.hpp
 *
 * Bộ lọc Gyroscope cho SensorHub:
 *   - LowPassFilter2p (Biquad Butterworth 2nd order) lọc nhiễu tần số cao
 *   - NotchFilter (2nd order) chặt đứt tần số cộng hưởng khung máy bay
 */
#pragma once

#include "LowPassFilter2p.hpp"
#include "NotchFilter.hpp"

class GyroFilter {
public:
    GyroFilter() = default;
    ~GyroFilter() = default;

    /**
     * Khởi tạo tất cả bộ lọc
     * @param sample_freq: Tần số vòng lặp (Hz) - ví dụ 1000.0f
     * @param lpf_cutoff:  Tần số cắt LPF (Hz) - ví dụ 80.0f
     * @param notch_freq:  Tần số cộng hưởng cần lọc (Hz) - 0.0f để tắt
     * @param notch_bw:    Độ rộng băng Notch (Hz) - ví dụ 20.0f
     */
    void init(float sample_freq, float lpf_cutoff,
              float notch_freq = 0.0f, float notch_bw = 20.0f) {
        for (int i = 0; i < 3; i++) {
            _lpf[i].set_cutoff_frequency(sample_freq, lpf_cutoff);
        }

        _notch_enabled = (notch_freq > 0.0f);
        if (_notch_enabled) {
            for (int i = 0; i < 3; i++) {
                _notch[i].setParameters(sample_freq, notch_freq, notch_bw);
            }
        }
    }

    /**
     * Áp dụng bộ lọc cho Gyro XYZ
     * @param gyro_in [3]: Dữ liệu thô (rad/s)
     * @param gyro_out[3]: Dữ liệu đã lọc (rad/s)
     */
    void apply(const float gyro_in[3], float gyro_out[3]) {
        for (int i = 0; i < 3; i++) {
            float val = _lpf[i].apply(gyro_in[i]);
            if (_notch_enabled) {
                val = _notch[i].apply(val);
            }
            gyro_out[i] = val;
        }
    }

    // Reset trạng thái bộ lọc
    void reset(const float gyro[3]) {
        for (int i = 0; i < 3; i++) {
            _lpf[i].reset(gyro[i]);
            if (_notch_enabled) {
                _notch[i].reset(gyro[i]);
            }
        }
    }

private:
    LowPassFilter2p<float> _lpf[3];   // Lọc 3 trục X, Y, Z
    NotchFilter<float>     _notch[3]; // Notch cho 3 trục X, Y, Z
    bool _notch_enabled = false;
};
