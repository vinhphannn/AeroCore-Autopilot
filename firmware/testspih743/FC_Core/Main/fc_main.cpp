#include "fc_main.h"
#include "stm32h7xx_hal.h"
#include "../Communication/mavlink_main.h"
#include "../Drivers/imu/icm42688p.h"
#include "../Drivers/barometer/ms5607.h"
#include "../Drivers/mag/qmc5883l.h"
#include "../Drivers/optical_flow/pmw3901.h"
#include "../Drivers/rangefinder/gy53l1x.h"
#include "../Drivers/gps/gps_m10.h"
#include "../Drivers/rc/crsf.h"
#include "../Estimator/ahrs/AttitudeEstimator.h"
#include "../Sensors/SensorHub.h"
#include "../Commander/Commander.h"
#include "../Controllers/mc_att_control/AttitudeController.h"
#include "../Controllers/control_allocator/Mixer.h"
#include "../Commander/flight_mode_manager/FlightModeManager.h"
#include "sys_monitor.h"

#include "FreeRTOS.h"
#include "task.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart7;

// Global Sensor Instances
GPS_M10 gps(&huart3);

extern "C" void Init_Sensor_Semaphores(void);

extern "C" void FC_Run(void) {
    // 1. Khởi tạo phần cứng (Chạy 1 lần duy nhất trước khi RTOS Start)
    // MAVLink (USB CDC) đã init ở MX_USB_DEVICE_Init
}

extern "C" void FC_MavlinkTask(void) {
    // Chờ hệ thống ổn định
    vTaskDelay(pdMS_TO_TICKS(1000));
    while(1) {
        g_mavlink.run();
        vTaskDelay(pdMS_TO_TICKS(5)); // Chạy khoảng 200Hz
    }
}

extern "C" void FC_SensorsTask(void) {
    // Kích hoạt Semaphores
    Init_Sensor_Semaphores();
    
    // Tạo System Monitor Task (Chạy ngầm, đo % CPU và RAM)
    xTaskCreate((TaskFunction_t)cpuload_monitor_task, "topTask", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
    
    // Khởi tạo các cảm biến
    g_icm42688p.init();
    g_ms5607_1.init();
    QMC5883L_Init();
    PMW3901_Init();
    GY53L1X_Init(&huart2); // ToF trên USART2
    g_crsf.init();         // Tay cầm ELRS trên USART6
    
    g_sensor_hub.init();
    g_estimator.init();
    g_commander.init();
    g_flight_mode_mgr.init();
    g_att_control.init();
    g_mixer.init();
    
    QMC5883L_Data_t mag_data;
    PMW3901_Data_t opt_data;
    GY53L1X_Data_t tof_data;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    
    while(1) {
        // Đọc IMU (Sẽ tự publish lên uORB)
        g_icm42688p.run();
        
        // Đọc Baro (Sẽ tự publish lên uORB)
        g_ms5607_1.run();
        
        // Đọc Mag (Sẽ tự publish)
        QMC5883L_Read(&mag_data);
        
        // Đọc Optical Flow (Sẽ tự publish)
        PMW3901_ReadMotion(&opt_data);
        
        // Đọc ToF (Sẽ tự publish)
        GY53L1X_Update(&huart2, &tof_data);
        
        // Cập nhật DMA Tay cầm (Siêu mượt)
        g_crsf.update();
        
        // SensorHub lấy data raw, xoay trục chuẩn và publish vehicle_imu
        g_sensor_hub.update();
        
        // Cập nhật bộ lọc Mahony để tính toán góc nghiêng 3D từ vehicle_imu
        g_estimator.update();
        
        // Cập nhật Trạng thái an toàn máy bay (Arm/Disarm)
        g_commander.update();
        
        // FlightModeManager: Quản lý Mode, sinh ra Setpoint Góc + Lực đẩy
        g_flight_mode_mgr.update();
        
        // Vòng lặp PID đa trục (Outer: AttitudeControl Quat -> Inner: RateControl PID)
        g_att_control.update();
        
        // Mixer: ControlAllocation (Sequential Desaturation) -> DShot
        g_mixer.update();
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

extern "C" void FC_GPSTask(void) {
    while(1) {
        // Tạm thời nếu dùng ngắt (RXNE) thì parser đã chạy trong callback.
        // Hàm này có thể dùng để check timeout hoặc update state.
        // Hoặc vòng lặp đọc buffer DMA.
        gps.update();
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}
