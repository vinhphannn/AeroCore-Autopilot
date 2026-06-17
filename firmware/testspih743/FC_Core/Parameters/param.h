#pragma once

#include <stdint.h>
#include <stdbool.h>

#define PARAM_TYPE_UNKNOWN 0
#define PARAM_TYPE_INT32   1
#define PARAM_TYPE_FLOAT   2

typedef uint16_t param_t;
#define PARAM_INVALID ((uint16_t)0xffff)

// 1. NƠI BẠN ĐỊNH NGHĨA TẤT CẢ CÁC THAM SỐ CỦA MÁY BAY
enum param_id_t {
    PARAM_SYS_AUTOSTART = 0, // ID Hệ thống (Ví dụ: 4001 = Quadrotor X)
    
    // Cấu hình mạch Cảm biến (SENS_)
    PARAM_SENS_BOARD_ROT,
    PARAM_SENS_BOARD_X_OFF,
    PARAM_SENS_BOARD_Y_OFF,
    PARAM_SENS_BOARD_Z_OFF,
    
    // PID Góc nghiêng Roll
    PARAM_MC_ROLL_P,
    PARAM_MC_ROLL_I,
    PARAM_MC_ROLL_D,

    // PID Góc nghiêng Pitch
    PARAM_MC_PITCH_P,
    PARAM_MC_PITCH_I,
    PARAM_MC_PITCH_D,

    // PID Hướng Yaw
    PARAM_MC_YAW_P,
    PARAM_MC_YAW_I,
    PARAM_MC_YAW_D,
    
    // Control Allocation - Vị trí từng Rotor (CA_)
    // Đúng tên PX4: CA_ROTOR${i}_PX / CA_ROTOR${i}_PY
    // Đơn vị: mét (m), tính từ trọng tâm đến trục motor
    // M1: Front-Right  M2: Front-Left  M3: Rear-Left  M4: Rear-Right
    PARAM_CA_ROTOR0_PX, PARAM_CA_ROTOR0_PY, // M1 Front-Right
    PARAM_CA_ROTOR1_PX, PARAM_CA_ROTOR1_PY, // M2 Front-Left
    PARAM_CA_ROTOR2_PX, PARAM_CA_ROTOR2_PY, // M3 Rear-Left
    PARAM_CA_ROTOR3_PX, PARAM_CA_ROTOR3_PY, // M4 Rear-Right

    // Hướng quay của từng Rotor (CA_ROTOR${i}_KM)
    // >0 = CCW (dương), <0 = CW (âm)
    PARAM_CA_ROTOR0_KM, // M1 FR CCW = +1
    PARAM_CA_ROTOR1_KM, // M2 FL CW  = -1
    PARAM_CA_ROTOR2_KM, // M3 RL CCW = +1
    PARAM_CA_ROTOR3_KM, // M4 RR CW  = -1
    
    // Airmode: 0=Disabled, 1=Roll/Pitch, 2=Roll/Pitch/Yaw
    PARAM_MC_AIRMODE,

    // Navigation & RTL Parameters
    PARAM_NAV_ACC_RAD,
    PARAM_RTL_ALT,
    
    // LUÔN ĐỂ PARAM_COUNT Ở CUỐI ĐỂ ĐẾM TỔNG SỐ PARAM CÓ TRONG HỆ THỐNG
    PARAM_COUNT
};

union param_value_u {
    int32_t i;
    float   f;
};

// Cấu trúc lưu trữ Mặc định (Sẽ nằm trên ROM)
struct param_info_s {
    const char* name;
    uint8_t type;
    union param_value_u default_val;
};

// --- BỘ API CHUẨN GIỐNG HỆT PX4 ---

// Khởi tạo: Copy cấu hình mặc định vào RAM, sau đó load Flash bù lên
void param_init(void);

// Tìm ID của tham số dựa trên tên chuỗi (Dành cho MAVLink)
param_t param_find(const char *name);

// Lấy thông tin
const char* param_name(param_t param);
uint8_t param_type(param_t param);

// Lấy và Ghi tham số (Từ RAM)
int param_get(param_t param, void *val);
int param_set(param_t param, const void *val);

// Lưu tham số xuống Flash (Chạy ở background thread)
int param_save_default(void);
