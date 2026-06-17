#include "ControlMath.hpp"

using Vector3f = matrix::Vector3f;
using Vector2f = matrix::Vector2f;

namespace ControlMath
{

static void dcm_to_quat(const float R[3][3], float q[4]) {
    float tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0f) {
        float s = sqrtf(tr + 1.0f) * 2.0f; // s = 4 * q0
        q[0] = 0.25f * s;
        q[1] = (R[2][1] - R[1][2]) / s;
        q[2] = (R[0][2] - R[2][0]) / s;
        q[3] = (R[1][0] - R[0][1]) / s;
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
        float s = sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f; // s = 4 * q1
        q[0] = (R[2][1] - R[1][2]) / s;
        q[1] = 0.25f * s;
        q[2] = (R[0][1] + R[1][0]) / s;
        q[3] = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        float s = sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f; // s = 4 * q2
        q[0] = (R[0][2] - R[2][0]) / s;
        q[1] = (R[0][1] + R[1][0]) / s;
        q[2] = 0.25f * s;
        q[3] = (R[1][2] + R[2][1]) / s;
    } else {
        float s = sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f; // s = 4 * q3
        q[0] = (R[1][0] - R[0][1]) / s;
        q[1] = (R[0][2] + R[2][0]) / s;
        q[2] = (R[1][2] + R[2][1]) / s;
        q[3] = 0.25f * s;
    }
    // Chuẩn hóa
    float len = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (len > 1e-6f) {
        q[0] /= len; q[1] /= len; q[2] /= len; q[3] /= len;
    }
}

void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
    bodyzToAttitude(-1.0f * thr_sp, yaw_sp, att_sp);
    att_sp.thrust_body[2] = -thr_sp.length();
}

void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle)
{
    // determine tilt
    const float dot_product_unit = body_unit.dot(world_unit);
    float angle = acosf(dot_product_unit);
    // limit tilt
    if (angle > max_angle) {
        angle = max_angle;
    }
    Vector3f rejection = body_unit - (dot_product_unit * world_unit);

    // corner case exactly parallel vectors
    if (rejection.norm_squared() < FLT_EPSILON) {
        rejection(0) = 1.f;
    }

    body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.normalized();
}

void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
    // zero vector, no direction, set safe level value
    if (body_z.norm_squared() < FLT_EPSILON) {
        body_z(2) = 1.f;
    }

    body_z.normalize();

    // vector of desired yaw direction in XY plane, rotated by PI/2
    const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

    // desired body_x axis, orthogonal to body_z
    Vector3f body_x = y_C % body_z;

    // keep nose to front while inverted upside down
    if (body_z(2) < 0.f) {
        body_x = -1.0f * body_x;
    }

    if (fabsf(body_z(2)) < 0.000001f) {
        // desired thrust is in XY plane, set X downside to construct correct matrix,
        // but yaw component will not be used actually
        body_x.zero();
        body_x(2) = 1.f;
    }

    body_x.normalize();

    // desired body_y axis
    const Vector3f body_y = body_z % body_x;

    float R_sp[3][3];

    // fill rotation matrix
    for (int i = 0; i < 3; i++) {
        R_sp[i][0] = body_x(i);
        R_sp[i][1] = body_y(i);
        R_sp[i][2] = body_z(i);
    }

    // Convert DCM to Quaternion and copy to attitude setpoint
    dcm_to_quat(R_sp, att_sp.q_d);
}

Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, const float &max)
{
    if ((v0 + v1).norm() <= max) {
        // vector does not exceed maximum magnitude
        return v0 + v1;

    } else if (v0.length() >= max) {
        // the magnitude along v0, which has priority, already exceeds maximum.
        return v0.normalized() * max;

    } else if (fabsf((v1 - v0).norm()) < 0.001f) {
        // the two vectors are equal
        return v0.normalized() * max;

    } else if (v0.length() < 0.001f) {
        // the first vector is 0.
        return v1.normalized() * max;

    } else {
        Vector2f u1 = v1.normalized();
        float m = u1.dot(v0);
        float c = v0.dot(v0) - max * max;
        float s = -m + sqrtf(m * m - c);
        return v0 + u1 * s;
    }
}

void addIfNotNan(float &setpoint, const float addition)
{
    if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
        // No NAN, add to the setpoint
        setpoint += addition;

    } else if (!PX4_ISFINITE(setpoint)) {
        // Setpoint NAN, take addition
        setpoint = addition;
    }
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
    for (int i = 0; i < 3; i++) {
        addIfNotNan(setpoint(i), addition(i));
    }
}

void setZeroIfNanVector3f(Vector3f &vector)
{
    addIfNotNanVector3f(vector, Vector3f());
}

} // namespace ControlMath
