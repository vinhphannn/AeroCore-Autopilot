#pragma once

#include <math.h>
#include <float.h>
#include "../../MessageBus/topics/vehicle_attitude_setpoint.h"

// Định nghĩa các macro & hằng số giống PX4
#ifndef PX4_ISFINITE
#define PX4_ISFINITE(x) (isfinite(x))
#endif

#ifndef CONSTANTS_ONE_G
#define CONSTANTS_ONE_G 9.80665f
#endif

namespace matrix {

struct Vector2f {
    float x, y;
    Vector2f() : x(0.0f), y(0.0f) {}
    Vector2f(float _x, float _y) : x(_x), y(_y) {}
    
    float length() const { return sqrtf(x*x + y*y); }
    float norm() const { return length(); }
    float norm_squared() const { return x*x + y*y; }
    
    Vector2f normalized() const {
        float l = length();
        if (l > 1e-6f) return {x/l, y/l};
        return {0.0f, 0.0f};
    }
    
    float dot(const Vector2f &o) const { return x*o.x + y*o.y; }
    
    Vector2f operator+(const Vector2f &o) const { return {x+o.x, y+o.y}; }
    Vector2f operator-(const Vector2f &o) const { return {x-o.x, y-o.y}; }
    Vector2f operator*(float s) const { return {x*s, y*s}; }
    Vector2f operator/(float s) const { return {x/s, y/s}; }
};

struct Vector3f {
    float x, y, z;
    Vector3f() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    Vector3f(const float arr[3]) : x(arr[0]), y(arr[1]), z(arr[2]) {}

    float length() const { return sqrtf(x*x + y*y + z*z); }
    float norm() const { return length(); }
    float norm_squared() const { return x*x + y*y + z*z; }

    Vector3f normalized() const {
        float l = length();
        if (l > 1e-6f) return {x/l, y/l, z/l};
        return {0.0f, 0.0f, 0.0f};
    }
    
    void normalize() {
        float l = length();
        if (l > 1e-6f) { x /= l; y /= l; z /= l; }
    }

    float dot(const Vector3f &o) const { return x*o.x + y*o.y + z*o.z; }

    // Tích có hướng (Cross product) - Operator %
    Vector3f operator%(const Vector3f &o) const {
        return {
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        };
    }

    Vector3f operator+(const Vector3f &o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vector3f operator-(const Vector3f &o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vector3f operator*(float s) const { return {x*s, y*s, z*s}; }
    Vector3f operator/(float s) const { return {x/s, y/s, z/s}; }
    
    Vector3f emult(const Vector3f &o) const { return {x*o.x, y*o.y, z*o.z}; }

    float &operator()(int i) { return (&x)[i]; }
    const float &operator()(int i) const { return (&x)[i]; }

    void zero() { x = y = z = 0.0f; }
    void setZero() { zero(); }

    bool isAllFinite() const { return isfinite(x) && isfinite(y) && isfinite(z); }

    Vector2f xy() const { return {x, y}; }
    void xy(const Vector2f &val) { x = val.x; y = val.y; }

    void copyTo(float arr[3]) const {
        arr[0] = x;
        arr[1] = y;
        arr[2] = z;
    }
};

inline Vector3f operator*(float s, const Vector3f &v) { return v * s; }
inline Vector2f operator*(float s, const Vector2f &v) { return v * s; }

} // namespace matrix

namespace ControlMath
{
    using Vector3f = matrix::Vector3f;
    using Vector2f = matrix::Vector2f;

    void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp);
    void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle);
    void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp);
    Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, const float &max);
    void addIfNotNan(float &setpoint, const float addition);
    void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition);
    void setZeroIfNanVector3f(Vector3f &vector);
}
