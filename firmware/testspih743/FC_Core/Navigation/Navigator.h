#pragma once

#include <stdint.h>
#include "../MessageBus/topics/vehicle_local_position.h"
#include "../MessageBus/topics/vehicle_status.h"
#include "../MessageBus/topics/position_setpoint_triplet.h"
#include "../MessageBus/topics/actuator_armed.h"
#include "../MessageBus/topics/vehicle_attitude.h"
#include "../MessageBus/uORB_lite.h"

#define MAX_WAYPOINTS 16

class Navigator {
public:
    enum RTLState {
        RTL_STATE_NONE = 0,
        RTL_STATE_CLIMB,   // Bay lên độ cao RTL an toàn
        RTL_STATE_RETURN,  // Quay về vị trí Home (X-Y)
        RTL_STATE_LAND     // Chuyển sang AUTO_LAND để hạ cánh
    };

    Navigator();
    ~Navigator() = default;

    void init();
    void update();

    // MAVLink Mission Protocol APIs
    bool add_waypoint(const position_setpoint_s& wp);
    void clear_waypoints();
    uint8_t get_waypoint_count() const { return _waypoint_count; }
    uint8_t get_current_waypoint_index() const { return _current_waypoint_index; }
    position_setpoint_s get_waypoint(uint8_t index) const {
        if (index < _waypoint_count) return _waypoints[index];
        return position_setpoint_s{};
    }

    // Flash Storage (Dataman emulation)
    bool save_mission_to_flash();
    bool load_mission_from_flash();
    void load_demo_mission();

private:
    void handle_mission_mode(const vehicle_local_position_s& local_pos);
    void handle_rtl_mode(const vehicle_local_position_s& local_pos);
    void publish_triplet();

    // uORB Subscriptions
    uORB::SubscriptionMulti<vehicle_local_position_s> _local_pos_sub;
    uORB::SubscriptionMulti<vehicle_status_s>         _status_sub;
    uORB::SubscriptionMulti<actuator_armed_s>         _armed_sub;
    uORB::SubscriptionMulti<vehicle_attitude_s>       _att_sub;

    // uORB Publications
    uORB::PublicationMulti<position_setpoint_triplet_s> _pos_sp_triplet_pub;
    uORB::PublicationMulti<vehicle_status_s>             _status_pub;

    // Cấu hình sứ mệnh (Mission Waypoints)
    position_setpoint_s _waypoints[MAX_WAYPOINTS];
    uint8_t _waypoint_count;
    uint8_t _current_waypoint_index;
    float _acceptance_radius;

    // Quản lý Home Position
    struct {
        float x;
        float y;
        float z;
        bool valid;
    } _home_pos;
    bool _was_armed;

    // Trạng thái RTL
    RTLState _rtl_state;
    float _rtl_altitude; // Độ cao an toàn khi RTL (Z âm)
};

extern Navigator g_navigator;
