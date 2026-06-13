/**
 * ControlAllocation - Port từ PX4 ControlAllocationSequentialDesaturation
 * 
 * Nâng cấp: Hỗ trợ arm length ratio (CA_ARM_LX vs CA_ARM_LY)
 * → Mixer tự bù tỉ lệ cánh tay đòn → Roll P = Pitch P, không cần tune riêng
 *
 * Geometry: Quadrotor X (cứng hóa, tối ưu cho 1 loại khung)
 *
 *        F (Pitch+)
 *        |
 *   M2(CW) ──────── M1(CCW)   ← arm_x (LY, Roll arm)
 *        |    |    |
 *        +----+----+
 *        |         |
 *   M3(CCW) ─────── M4(CW)
 *        |
 *   arm_y (LX, Pitch arm)
 *
 * Motor Spin Directions (Quad-X, nhìn từ trên):
 *   M1 FR: CCW (+Yaw)   M2 FL: CW  (-Yaw)
 *   M3 RL: CW  (-Yaw)   M4 RR: CCW (+Yaw)  ← chú ý: PX4 dùng +Yaw = CW từ trên nhìn xuống
 *
 * Airmode (MC_AIRMODE):
 *   0 = Disabled: Motor không bao giờ được tăng Thrust để bù (an toàn nhất)
 *   1 = Roll/Pitch: Cho phép tăng Thrust để bù Roll/Pitch khi bão hòa
 *   2 = Roll/Pitch/Yaw: Tất cả axes đều được phép tăng Thrust
 */
#pragma once

#include <stdint.h>
#include <math.h>
#include <float.h>

#define NUM_MOTORS 4
#define NUM_AXES   4  // Roll=0, Pitch=1, Yaw=2, Thrust=3

struct MixerSaturation {
    bool roll_pos, roll_neg;
    bool pitch_pos, pitch_neg;
    bool yaw_pos, yaw_neg;
};

class ControlAllocation {
public:
    ControlAllocation();
    ~ControlAllocation() = default;

    /**
     * Cập nhật geometry từ param (gọi khi param thay đổi)
     * @param arm_lx  Cánh tay đòn Pitch axis (mm) - CA_ARM_LX
     * @param arm_ly  Cánh tay đòn Roll  axis (mm) - CA_ARM_LY
     * @param airmode MC_AIRMODE: 0/1/2
     */
    void updateGeometry(float arm_lx, float arm_ly, int airmode); // Legacy (deprecated)

    /**
     * Cập nhật geometry từng Rotor riêng lẻ (đúng API PX4: CA_ROTOR${i}_PX/PY/KM)
     * @param px[4]   Vị trí X của từng rotor (mét, từ trọng tâm)
     * @param py[4]   Vị trí Y của từng rotor (mét, từ trọng tâm)
     * @param km[4]   Hướng quay: +1.0=CCW (Yaw+), -1.0=CW (Yaw-)
     * @param airmode MC_AIRMODE: 0/1/2
     */
    void updateGeometryPerRotor(float px[4], float py[4], float km[4], int airmode);

    /**
     * Phân bổ lực điều khiển vào 4 motor (Sequential Desaturation - đúng PX4)
     * @param roll    [-1,1]  Torque Roll
     * @param pitch   [-1,1]  Torque Pitch
     * @param yaw     [-1,1]  Torque Yaw
     * @param thrust  [0,1]   Lực nâng
     * @param motor_out[4]    [0,1] Công suất motor (OUTPUT)
     * @param sat      Phản hồi bão hòa (OUTPUT)
     */
    void allocate(float roll, float pitch, float yaw, float thrust,
                  float motor_out[NUM_MOTORS], MixerSaturation &sat);

private:
    void desaturateActuators(float sp[NUM_MOTORS], const float vec[NUM_MOTORS],
                             bool increase_only = false);
    float computeDesaturationGain(const float vec[NUM_MOTORS], const float sp[NUM_MOTORS]);
    void mixYaw(float sp[NUM_MOTORS], float yaw_delta);
    void detectSaturation(const float sp[NUM_MOTORS], MixerSaturation &sat);
    void mixAirmodeDisabled(float sp[NUM_MOTORS], float roll, float pitch, float yaw, float thrust);
    void mixAirmodeRP(float sp[NUM_MOTORS], float roll, float pitch, float yaw, float thrust);
    void mixAirmodeRPY(float sp[NUM_MOTORS], float roll, float pitch, float yaw, float thrust);

    // Normalized Geometry Matrix [Motor][Axis]: Roll, Pitch, Yaw, Thrust
    float _B[NUM_MOTORS][NUM_AXES];

    int _airmode = 0;

    static constexpr float ACTUATOR_MIN = 0.0f;
    static constexpr float ACTUATOR_MAX = 1.0f;
    static constexpr float MINIMUM_YAW_MARGIN = 0.05f;
};
