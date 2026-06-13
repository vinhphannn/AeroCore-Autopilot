#include "Mixer.h"
#include "../../Parameters/param.h"
#include "../../Main/fc_logging.h"

Mixer g_mixer;

Mixer::Mixer() :
    _torque_sub(orb_vehicle_torque_setpoint, 0),
    _thrust_sub(orb_vehicle_thrust_setpoint, 0),
    _armed_sub(orb_actuator_armed, 0)
{}

void Mixer::init() {
    // Load geometry từng Rotor từ Parameter System (đúng tên PX4: CA_ROTOR${i}_PX/PY/KM)
    float px[4], py[4], km[4];
    param_get(PARAM_CA_ROTOR0_PX, &px[0]); param_get(PARAM_CA_ROTOR0_PY, &py[0]);
    param_get(PARAM_CA_ROTOR1_PX, &px[1]); param_get(PARAM_CA_ROTOR1_PY, &py[1]);
    param_get(PARAM_CA_ROTOR2_PX, &px[2]); param_get(PARAM_CA_ROTOR2_PY, &py[2]);
    param_get(PARAM_CA_ROTOR3_PX, &px[3]); param_get(PARAM_CA_ROTOR3_PY, &py[3]);
    param_get(PARAM_CA_ROTOR0_KM, &km[0]);
    param_get(PARAM_CA_ROTOR1_KM, &km[1]);
    param_get(PARAM_CA_ROTOR2_KM, &km[2]);
    param_get(PARAM_CA_ROTOR3_KM, &km[3]);
    int32_t airmode = 0;
    param_get(PARAM_MC_AIRMODE, &airmode);

    _allocator.updateGeometryPerRotor(px, py, km, (int)airmode);

    // Đảm bảo motor tắt khi khởi động
    for (int i = 0; i < 4; i++) g_dshot.set_throttle(i, 0);
    g_dshot.update();

    FC_INFO("Mixer init: M1(%.2f,%.2f) M2(%.2f,%.2f) M3(%.2f,%.2f) M4(%.2f,%.2f) airmode=%d",
            (double)px[0],(double)py[0], (double)px[1],(double)py[1],
            (double)px[2],(double)py[2], (double)px[3],(double)py[3], (int)airmode);
}

void Mixer::update() {
    // 1. Kiểm tra Armed
    actuator_armed_s armed;
    if (_armed_sub.update(armed)) { _armed = armed.armed; }

    // 2. SAFETY GATE - Nếu DISARMED: tắt motor ngay lập tức
    if (!_armed) {
        for (int i = 0; i < 4; i++) g_dshot.set_throttle(i, 0);
        g_dshot.update();
        return;
    }

    // 3. Lấy Torque và Thrust setpoint từ AttitudeController
    vehicle_torque_setpoint_s torque;
    vehicle_thrust_setpoint_s thrust;
    _torque_sub.copy(torque);
    _thrust_sub.copy(thrust);

    // PX4 NED convention: thrust Z là âm (nâng lên = -Z)
    float thrust_z = -thrust.xyz[2];
    if (thrust_z < 0.0f) thrust_z = 0.0f;
    if (thrust_z > 1.0f) thrust_z = 1.0f;

    // 4. Control Allocation (Sequential Desaturation - PX4 algorithm)
    float motor_normalized[4];
    _allocator.allocate(torque.xyz[0], torque.xyz[1], torque.xyz[2],
                        thrust_z, motor_normalized, _last_sat);

    // 5. Map [0,1] -> DShot [DSHOT_MIN, DSHOT_MAX]
    for (int i = 0; i < 4; i++) {
        uint16_t dshot_val;
        if (motor_normalized[i] < 0.001f) {
            dshot_val = DSHOT_IDLE; // Idle khi armed nhưng không bay
        } else {
            float mapped = DSHOT_MIN + motor_normalized[i] * (float)(DSHOT_MAX - DSHOT_MIN);
            dshot_val = (uint16_t)mapped;
            if (dshot_val < DSHOT_MIN) dshot_val = DSHOT_MIN;
            if (dshot_val > DSHOT_MAX) dshot_val = DSHOT_MAX;
        }
        g_dshot.set_throttle(i, dshot_val);
    }

    // 6. Trigger DShot DMA
    g_dshot.update();
}
