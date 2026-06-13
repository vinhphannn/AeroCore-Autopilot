#include "fc_logging.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

mavlink_log_s g_mavlink_log_data;

extern "C" void fc_log_print(uint8_t severity, const char *fmt, ...) {
    // 1. Chuẩn bị buffer an toàn
    char buffer[127];
    memset(buffer, 0, sizeof(buffer));

    // 2. Định dạng chuỗi (sprintf)
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    // 3. Đóng gói vào uORB struct
    g_mavlink_log_data.timestamp = xTaskGetTickCount();
    g_mavlink_log_data.severity = severity;
    strncpy(g_mavlink_log_data.text, buffer, sizeof(g_mavlink_log_data.text) - 1);

    // TODO: Khi class uORB hoàn thiện, ta sẽ gọi uORB::publish(ORB_ID_MAVLINK_LOG, &g_mavlink_log_data) tại đây.
    // Tạm thời, MavlinkManager sẽ poll biến g_mavlink_log_data để bắn ra QGroundControl.
}
