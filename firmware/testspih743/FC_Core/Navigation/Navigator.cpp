#include "Navigator.h"
#include "../Main/fc_logging.h"
#include "../Parameters/param.h"
#include "stm32h7xx_hal.h"
#include <math.h>

// Định nghĩa đối tượng toàn cục Navigator
Navigator g_navigator;

Navigator::Navigator() :
    _local_pos_sub(orb_vehicle_local_position, 0),
    _status_sub(orb_vehicle_status, 0),
    _armed_sub(orb_actuator_armed, 0),
    _att_sub(orb_vehicle_attitude, 0),
    _pos_sp_triplet_pub(orb_position_setpoint_triplet),
    _status_pub(orb_vehicle_status),
    _waypoint_count(0),
    _current_waypoint_index(0),
    _acceptance_radius(1.0f),
    _was_armed(false),
    _rtl_state(RTL_STATE_NONE),
    _rtl_altitude(-2.0f) // 2 mét (NED Z âm)
{
    _home_pos.x = 0.0f;
    _home_pos.y = 0.0f;
    _home_pos.z = 0.0f;
    _home_pos.valid = false;
}

static uint64_t get_time_us() {
    return (uint64_t)HAL_GetTick() * 1000;
}

#define DATAMAN_FLASH_ADDR 0x081C0000 // Sector 6 Bank 2
#define DATAMAN_MAGIC 0x55AA55AA

struct __attribute__((packed)) mission_flash_header_s {
    uint32_t magic;
    uint32_t waypoint_count;
};

void Navigator::init() {
    clear_waypoints();

    // Thử load sứ mệnh từ Flash. Nếu thất bại hoặc chưa có, nạp Demo.
    if (!load_mission_from_flash()) {
        load_demo_mission();
    }
}

void Navigator::load_demo_mission() {
    clear_waypoints();

    // Load một Sứ mệnh Demo gồm 4 Waypoints tạo thành hình vuông bay quanh Home
    position_setpoint_s wp1{};
    wp1.x = 3.0f;  // Đi về phía Bắc 3 mét
    wp1.y = 0.0f;
    wp1.z = -2.0f; // Độ cao 2 mét
    wp1.yaw = 0.0f;
    wp1.type = SETPOINT_TYPE_POSITION;
    wp1.valid = true;
    add_waypoint(wp1);

    position_setpoint_s wp2{};
    wp2.x = 3.0f;
    wp2.y = 3.0f;  // Sang Đông 3 mét
    wp2.z = -2.0f;
    wp2.yaw = 1.5708f; // Quay đầu góc 90 độ (Yaw = 90 deg)
    wp2.type = SETPOINT_TYPE_POSITION;
    wp2.valid = true;
    add_waypoint(wp2);

    position_setpoint_s wp3{};
    wp3.x = 0.0f;
    wp3.y = 3.0f;  // Quay về trục X=0
    wp3.z = -2.0f;
    wp3.yaw = 3.1415f; // Yaw = 180 deg
    wp3.type = SETPOINT_TYPE_POSITION;
    wp3.valid = true;
    add_waypoint(wp3);

    position_setpoint_s wp4{};
    wp4.x = 0.0f;
    wp4.y = 0.0f;  // Về lại điểm xuất phát X=0, Y=0
    wp4.z = -2.0f;
    wp4.yaw = 0.0f;
    wp4.type = SETPOINT_TYPE_POSITION;
    wp4.valid = true;
    add_waypoint(wp4);

    FC_INFO("Navigator initialized with Demo Mission.");
}

bool Navigator::save_mission_to_flash() {
    // 1. Chuẩn bị buffer được căn lề 32-byte (256-bit) để ghi Flash Word trên STM32H7
    alignas(32) uint8_t write_buf[608] = {0};

    mission_flash_header_s header;
    header.magic = DATAMAN_MAGIC;
    header.waypoint_count = _waypoint_count;

    memcpy(write_buf, &header, sizeof(header));
    memcpy(write_buf + sizeof(header), _waypoints, sizeof(_waypoints));

    // 2. Mở khóa Flash
    HAL_FLASH_Unlock();

    // 3. Xóa Sector 6 của Bank 2
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = FLASH_SECTOR_6;
    erase_init.Banks = FLASH_BANK_2;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_error = 0;

    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        FC_ERR("[Dataman] Flash sector erase failed!");
        return false;
    }

    // 4. Ghi dữ liệu theo từng Flash Word 256-bit (32 bytes)
    bool success = true;
    for (uint32_t offset = 0; offset < sizeof(write_buf); offset += 32) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, DATAMAN_FLASH_ADDR + offset, (uint32_t)(write_buf + offset)) != HAL_OK) {
            success = false;
            break;
        }
    }

    // 5. Khóa lại Flash
    HAL_FLASH_Lock();

    if (success) {
        FC_INFO("[Dataman] Mission saved to Flash at 0x%08X (size: %u waypoints)", DATAMAN_FLASH_ADDR, _waypoint_count);
    } else {
        FC_ERR("[Dataman] Mission flash programming failed!");
    }

    return success;
}

bool Navigator::load_mission_from_flash() {
    mission_flash_header_s* header = (mission_flash_header_s*)DATAMAN_FLASH_ADDR;

    if (header->magic != DATAMAN_MAGIC) {
        FC_INFO("[Dataman] No valid mission found in Flash (magic mismatch)");
        return false;
    }

    uint32_t count = header->waypoint_count;
    if (count > MAX_WAYPOINTS) {
        FC_ERR("[Dataman] Flash mission waypoint count invalid: %lu", count);
        return false;
    }

    _waypoint_count = count;
    _current_waypoint_index = 0;

    // Copy trực tiếp từ Flash memory map vào RAM
    memcpy(_waypoints, (const void*)(DATAMAN_FLASH_ADDR + sizeof(mission_flash_header_s)), _waypoint_count * sizeof(position_setpoint_s));

    FC_INFO("[Dataman] Loaded %d waypoints successfully from Flash", _waypoint_count);
    return true;
}

void Navigator::update() {
    vehicle_local_position_s local_pos{};
    _local_pos_sub.copy(local_pos);

    vehicle_status_s status{};
    _status_sub.copy(status);

    actuator_armed_s armed{};
    _armed_sub.copy(armed);

    if (status.timestamp == 0 || local_pos.timestamp_us == 0) {
        return; // Chưa có dữ liệu định vị hoặc trạng thái
    }

    // Load tham số cấu hình động từ hệ thống Parameter (chuẩn PX4)
    float nav_acc_rad = 1.0f;
    if (param_get(PARAM_NAV_ACC_RAD, &nav_acc_rad) == 0) {
        _acceptance_radius = nav_acc_rad;
    }
    float rtl_alt_m = 2.0f;
    if (param_get(PARAM_RTL_ALT, &rtl_alt_m) == 0) {
        _rtl_altitude = -rtl_alt_m; // Chuyển sang NED Z-down (Z âm)
    }

    // 1. Quản lý việc lưu vị trí Home khi Armed
    if (armed.armed && !_was_armed) {
        _home_pos.x = local_pos.x;
        _home_pos.y = local_pos.y;
        _home_pos.z = local_pos.z;
        _home_pos.valid = true;
        FC_INFO("Home Saved: X=%.2f Y=%.2f Z=%.2f", _home_pos.x, _home_pos.y, _home_pos.z);
    }
    _was_armed = armed.armed;

    // Reset chỉ mục khi Disarmed để lần bay sau bắt đầu lại từ đầu
    if (!armed.armed) {
        _current_waypoint_index = 0;
        _rtl_state = RTL_STATE_NONE;
    }

    // 2. Phân nhánh xử lý chế độ tự động
    if (status.nav_state == NAVIGATION_STATE_AUTO_MISSION) {
        handle_mission_mode(local_pos);
    } else if (status.nav_state == NAVIGATION_STATE_AUTO_RTL) {
        handle_rtl_mode(local_pos);
    } else {
        // Nếu chuyển sang chế độ thủ công khác, reset trạng thái RTL
        _rtl_state = RTL_STATE_NONE;
    }
}

void Navigator::handle_mission_mode(const vehicle_local_position_s& local_pos) {
    // Nếu hoàn thành sứ mệnh hoặc chưa có waypoint nào -> Tự động chuyển sang RTL
    if (_waypoint_count == 0 || _current_waypoint_index >= _waypoint_count) {
        vehicle_status_s status{};
        _status_sub.copy(status);
        status.nav_state = NAVIGATION_STATE_AUTO_RTL;
        status.timestamp = get_time_us();
        _status_pub.publish(status);
        
        FC_INFO("Mission Completed! Switching to AUTO_RTL...");
        return;
    }

    // Gửi Setpoint Waypoint hiện tại cho PositionController
    publish_triplet();

    // Tính khoảng cách 3D từ vị trí hiện tại đến Waypoint mục tiêu
    position_setpoint_s current_wp = _waypoints[_current_waypoint_index];
    float dx = current_wp.x - local_pos.x;
    float dy = current_wp.y - local_pos.y;
    float dz = current_wp.z - local_pos.z;
    float dist = sqrtf(dx * dx + dy * dy + dz * dz);

    // Nếu lọt vào bán kính chấp nhận -> Chuyển sang Waypoint tiếp theo
    if (dist < _acceptance_radius) {
        FC_INFO("Reached Waypoint %d/%d!", _current_waypoint_index + 1, _waypoint_count);
        _current_waypoint_index++;
    }
}

void Navigator::handle_rtl_mode(const vehicle_local_position_s& local_pos) {
    position_setpoint_triplet_s triplet{};
    triplet.timestamp = get_time_us();

    // Thiết lập vị trí đích mặc định là Home hoặc tọa độ (0,0)
    float home_x = _home_pos.valid ? _home_pos.x : 0.0f;
    float home_y = _home_pos.valid ? _home_pos.y : 0.0f;

    switch (_rtl_state) {
    case RTL_STATE_NONE:
        _rtl_state = RTL_STATE_CLIMB;
        FC_INFO("[RTL] Stage 1: Climbing to safe altitude (Z = %.2f)", _rtl_altitude);
        break;

    case RTL_STATE_CLIMB: {
        // Duy trì vị trí X-Y cũ, chỉ bay thẳng đứng lên cao
        // (Z trong hệ NED đi lên là giá trị âm lớn hơn, vd: từ -1.5m lên -2.0m là nhỏ hơn)
        float target_z = (local_pos.z < _rtl_altitude) ? local_pos.z : _rtl_altitude;

        vehicle_attitude_s att{};
        _att_sub.copy(att);

        triplet.current.x = local_pos.x;
        triplet.current.y = local_pos.y;
        triplet.current.z = target_z;
        triplet.current.yaw = att.yaw_rad;
        triplet.current.type = SETPOINT_TYPE_POSITION;
        triplet.current.valid = true;

        _pos_sp_triplet_pub.publish(triplet);

        // Kiểm tra xem đã đạt độ cao an toàn chưa (dung sai 0.2m)
        if (fabsf(local_pos.z - target_z) < 0.2f) {
            _rtl_state = RTL_STATE_RETURN;
            FC_INFO("[RTL] Stage 2: Safe altitude reached. Returning to Home...");
        }
        break;
    }

    case RTL_STATE_RETURN: {
        // Bay ngang trở về Home X-Y ở độ cao an toàn RTL
        triplet.current.x = home_x;
        triplet.current.y = home_y;
        triplet.current.z = _rtl_altitude;
        triplet.current.yaw = 0.0f; // Quay đầu về hướng Bắc (0 rad) khi quay về
        triplet.current.type = SETPOINT_TYPE_POSITION;
        triplet.current.valid = true;

        _pos_sp_triplet_pub.publish(triplet);

        // Tính khoảng cách ngang 2D tới Home
        float dx = home_x - local_pos.x;
        float dy = home_y - local_pos.y;
        float dist_2d = sqrtf(dx * dx + dy * dy);

        // Khi cách Home dưới 0.8 mét -> Bắt đầu tự động hạ cánh
        if (dist_2d < 0.8f) {
            _rtl_state = RTL_STATE_LAND;
            FC_INFO("[RTL] Stage 3: Arrived home. Initiating Auto Land...");
        }
        break;
    }

    case RTL_STATE_LAND: {
        // Chuyển trực tiếp trạng thái bay sang AUTO_LAND để bộ điều khiển hạ cánh và disarm xử lý
        vehicle_status_s status{};
        _status_sub.copy(status);
        status.nav_state = NAVIGATION_STATE_AUTO_LAND;
        status.timestamp = get_time_us();
        _status_pub.publish(status);
        break;
    }
    }
}

void Navigator::publish_triplet() {
    if (_current_waypoint_index >= _waypoint_count) return;

    position_setpoint_triplet_s triplet{};
    triplet.timestamp = get_time_us();

    // 1. Waypoint Hiện Tại
    triplet.current = _waypoints[_current_waypoint_index];

    // 2. Waypoint Trước Đó (Nếu có)
    if (_current_waypoint_index > 0) {
        triplet.previous = _waypoints[_current_waypoint_index - 1];
    }

    // 3. Waypoint Tiếp Theo (Nếu có)
    if (_current_waypoint_index + 1 < _waypoint_count) {
        triplet.next = _waypoints[_current_waypoint_index + 1];
    }

    _pos_sp_triplet_pub.publish(triplet);
}

bool Navigator::add_waypoint(const position_setpoint_s& wp) {
    if (_waypoint_count >= MAX_WAYPOINTS) return false;
    _waypoints[_waypoint_count] = wp;
    _waypoint_count++;
    return true;
}

void Navigator::clear_waypoints() {
    _waypoint_count = 0;
    _current_waypoint_index = 0;
    for (int i = 0; i < MAX_WAYPOINTS; i++) {
        _waypoints[i] = position_setpoint_s{};
    }
}
