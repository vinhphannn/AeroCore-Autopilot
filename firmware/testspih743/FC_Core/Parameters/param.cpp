#include "param.h"
#include "param_flash.h"
#include "main.h" // Dùng để gọi lệnh tắt ngắt lõi ARM
#include <string.h>

// MẢNG ROM CỐ ĐỊNH (TỐN 0 BYTE RAM)
// Thứ tự phải khớp tuyệt đối với enum param_id_t
static const struct param_info_s px4_parameters[PARAM_COUNT] = {
    {"SYS_AUTOSTART", PARAM_TYPE_INT32, {.i = 4001}}, 
    
    {"SENS_BOARD_ROT",  PARAM_TYPE_INT32, {.i = 0}},
    {"SENS_BOARD_X_OFF",PARAM_TYPE_FLOAT, {.f = 0.0f}},
    {"SENS_BOARD_Y_OFF",PARAM_TYPE_FLOAT, {.f = 0.0f}},
    {"SENS_BOARD_Z_OFF",PARAM_TYPE_FLOAT, {.f = 0.0f}},
    
    {"MC_ROLL_P",     PARAM_TYPE_FLOAT, {.f = 6.5f}},
    {"MC_ROLL_I",     PARAM_TYPE_FLOAT, {.f = 0.2f}},
    {"MC_ROLL_D",     PARAM_TYPE_FLOAT, {.f = 0.1f}},

    {"MC_PITCH_P",    PARAM_TYPE_FLOAT, {.f = 6.5f}},
    {"MC_PITCH_I",    PARAM_TYPE_FLOAT, {.f = 0.2f}},
    {"MC_PITCH_D",    PARAM_TYPE_FLOAT, {.f = 0.1f}},

    {"MC_YAW_P",      PARAM_TYPE_FLOAT, {.f = 2.8f}},
    {"MC_YAW_I",      PARAM_TYPE_FLOAT, {.f = 0.1f}},
    {"MC_YAW_D",      PARAM_TYPE_FLOAT, {.f = 0.0f}},

    // Control Allocation: Vị trí từng Rotor (đúng tên PX4: CA_ROTOR${i}_PX/PY)
    // Đơn vị: mét (m). Mặc định = Quad-X 330 size (arm = 0.15m = 150mm)
    // Quad-X: M1=FR, M2=FL, M3=RL, M4=RR
    {"CA_ROTOR0_PX", PARAM_TYPE_FLOAT, {.f =  0.15f}}, // M1 FR: X+
    {"CA_ROTOR0_PY", PARAM_TYPE_FLOAT, {.f = -0.15f}}, // M1 FR: Y-
    {"CA_ROTOR1_PX", PARAM_TYPE_FLOAT, {.f =  0.15f}}, // M2 FL: X+
    {"CA_ROTOR1_PY", PARAM_TYPE_FLOAT, {.f =  0.15f}}, // M2 FL: Y+
    {"CA_ROTOR2_PX", PARAM_TYPE_FLOAT, {.f = -0.15f}}, // M3 RL: X-
    {"CA_ROTOR2_PY", PARAM_TYPE_FLOAT, {.f =  0.15f}}, // M3 RL: Y+
    {"CA_ROTOR3_PX", PARAM_TYPE_FLOAT, {.f = -0.15f}}, // M4 RR: X-
    {"CA_ROTOR3_PY", PARAM_TYPE_FLOAT, {.f = -0.15f}}, // M4 RR: Y-

    // Hướng quay: CA_ROTOR${i}_KM (>0=CCW, <0=CW) - đúng tên PX4
    {"CA_ROTOR0_KM", PARAM_TYPE_FLOAT, {.f =  1.0f}},  // M1 FR CCW
    {"CA_ROTOR1_KM", PARAM_TYPE_FLOAT, {.f = -1.0f}},  // M2 FL CW
    {"CA_ROTOR2_KM", PARAM_TYPE_FLOAT, {.f =  1.0f}},  // M3 RL CCW
    {"CA_ROTOR3_KM", PARAM_TYPE_FLOAT, {.f = -1.0f}},  // M4 RR CW

    // MC_AIRMODE: 0=Disabled, 1=Roll/Pitch, 2=Roll/Pitch/Yaw
    {"MC_AIRMODE",   PARAM_TYPE_INT32, {.i = 0}},
};

// MẢNG RAM HOẠT ĐỘNG
static union param_value_u active_params[PARAM_COUNT];

// CỜ THEO DÕI SỰ THAY ĐỔI
static bool param_dirty[PARAM_COUNT];

void param_init(void) {
    // 1. Copy toàn bộ Default ROM -> RAM Active
    for (int i = 0; i < PARAM_COUNT; i++) {
        active_params[i] = px4_parameters[i].default_val;
        param_dirty[i] = false;
    }

    // 2. Yêu cầu module Flash đọc EEPROM đè lên RAM (nếu người dùng đã từng lưu PID mới)
    param_flash_load(active_params, param_dirty, PARAM_COUNT);
}

param_t param_find(const char *name) {
    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(px4_parameters[i].name, name) == 0) {
            return i;
        }
    }
    return PARAM_INVALID;
}

const char* param_name(param_t param) {
    if (param >= PARAM_COUNT) return "";
    return px4_parameters[param].name;
}

uint8_t param_type(param_t param) {
    if (param >= PARAM_COUNT) return PARAM_TYPE_UNKNOWN;
    return px4_parameters[param].type;
}

int param_get(param_t param, void *val) {
    if (param >= PARAM_COUNT || val == NULL) return -1;
    
    if (px4_parameters[param].type == PARAM_TYPE_INT32) {
        *(int32_t*)val = active_params[param].i;
    } else if (px4_parameters[param].type == PARAM_TYPE_FLOAT) {
        *(float*)val = active_params[param].f;
    }
    return 0;
}

int param_set(param_t param, const void *val) {
    if (param >= PARAM_COUNT || val == NULL) return -1;

    // KHÓA NGẮT - Chống Data Tearing nếu EKF đang chạy param_get() lúc MAVLink đang param_set()
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (px4_parameters[param].type == PARAM_TYPE_INT32) {
        active_params[param].i = *(const int32_t*)val;
    } else if (px4_parameters[param].type == PARAM_TYPE_FLOAT) {
        active_params[param].f = *(const float*)val;
    }
    
    // Đánh dấu tham số này đã bị QGroundControl sửa (Dirty)
    param_dirty[param] = true;

    __set_PRIMASK(primask);
    return 0;
}

int param_save_default(void) {
    // Chỉ lưu những thông số bị Dirty xuống Flash
    return param_flash_save(active_params, param_dirty, PARAM_COUNT);
}
