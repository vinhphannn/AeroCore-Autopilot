#include "sys_monitor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static uint8_t g_current_cpu_load = 0;

void px4_top(void) {
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime, ulStatsAsPercentage;

    uxArraySize = uxTaskGetNumberOfTasks();
    pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

    if (pxTaskStatusArray != NULL) {
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

        char buf[128];
        // Xóa màn hình và in Header y hệt PX4
        snprintf(buf, sizeof(buf), "\033[2J\033[H"); 
        CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
        vTaskDelay(pdMS_TO_TICKS(5));

        snprintf(buf, sizeof(buf), "PID COMMAND                   STATUS       PRIO   CPU   STACK\r\n");
        CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
        vTaskDelay(pdMS_TO_TICKS(5));

        uint32_t total_idle_pct = 0;

        for (x = 0; x < uxArraySize; x++) {
            ulStatsAsPercentage = 0;
            if (ulTotalRunTime > 0) {
                ulStatsAsPercentage = (pxTaskStatusArray[x].ulRunTimeCounter * 100) / ulTotalRunTime;
            }

            const char* state_str = "READY";
            switch(pxTaskStatusArray[x].eCurrentState) {
                case eRunning:   state_str = "RUN  "; break;
                case eReady:     state_str = "READY"; break;
                case eBlocked:   state_str = "BLOCK"; break;
                case eSuspended: state_str = "SUSP "; break;
                case eDeleted:   state_str = "DEL  "; break;
                default:         state_str = "UNKN "; break;
            }

            // Tính Idle để xuất ra MAVLink
            if (strstr(pxTaskStatusArray[x].pcTaskName, "IDLE") != NULL || strstr(pxTaskStatusArray[x].pcTaskName, "Idle") != NULL) {
                total_idle_pct += ulStatsAsPercentage;
            }

            snprintf(buf, sizeof(buf), "%3u %-25s %-12s %4u %5u%% %5u\r\n",
                     (unsigned int)pxTaskStatusArray[x].xTaskNumber,
                     pxTaskStatusArray[x].pcTaskName,
                     state_str,
                     (unsigned int)pxTaskStatusArray[x].uxCurrentPriority,
                     (unsigned int)ulStatsAsPercentage,
                     (unsigned int)pxTaskStatusArray[x].usStackHighWaterMark);
            CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        
        g_current_cpu_load = (total_idle_pct <= 100) ? (100 - total_idle_pct) : 0;
        
        size_t free_heap = xPortGetFreeHeapSize();
        snprintf(buf, sizeof(buf), "\r\nTotal CPU Load: %u%% | Free Heap: %u Bytes\r\n\r\n", g_current_cpu_load, free_heap);
        CDC_Transmit_FS((uint8_t*)buf, strlen(buf));

        vPortFree(pxTaskStatusArray);
    }
}

uint8_t cpuload_get_percentage(void) {
    return g_current_cpu_load;
}

void cpuload_monitor_task(void) {
    while (1) {
        // Cứ 5 giây báo cáo 1 lần
        vTaskDelay(pdMS_TO_TICKS(5000));
        px4_top();
    }
}
