#pragma once

namespace math {

template<typename T>
inline T constrain(T val, T min_val, T max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

} // namespace math
