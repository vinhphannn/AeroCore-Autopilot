#include "qmc5883l.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include "../../MessageBus/topics/sensor_mag.h"
#include "../../Main/fc_logging.h"

// Biến pub của uORB
static uORB::PublicationMulti<sensor_mag_s> mag_pub(orb_sensor_mag);

extern "C" {
    extern I2C_HandleTypeDef hi2c1;
}
#define QMC_ADDR (0x0D << 1)

static uint8_t mag_present = 0;
static int16_t x_min = 32767, x_max = -32768;
static int16_t y_min = 32767, y_max = -32768;

uint8_t QMC5883L_Init(void)
{
    uint8_t cfg[2];
    cfg[0] = 0x09; cfg[1] = 0x11; // 200Hz, 2G, 512OSR
    if(HAL_I2C_Master_Transmit(&hi2c1, QMC_ADDR, cfg, 2, 50) == HAL_OK) {
        cfg[0] = 0x0B; cfg[1] = 0x01;
        HAL_I2C_Master_Transmit(&hi2c1, QMC_ADDR, cfg, 2, 50);
        mag_present = 1;
        FC_INFO("QMC5883L Init OK on I2C1");
        return 1;
    }
    FC_ERR("QMC5883L Init FAIL on I2C1");
    return 0;
}

void QMC5883L_Read(QMC5883L_Data_t *data)
{
    if (!data) return;
    uint8_t raw[6];
    if (HAL_I2C_Mem_Read(&hi2c1, QMC_ADDR, 0x00, 1, raw, 6, 20) == HAL_OK) {
        int16_t mx = (int16_t)(raw[1] << 8 | raw[0]);
        int16_t my = (int16_t)(raw[3] << 8 | raw[2]);
        
        if (mx < x_min) x_min = mx; 
        if (mx > x_max) x_max = mx;
        if (my < y_min) y_min = my; 
        if (my > y_max) y_max = my;

        int16_t x_offset = (x_min + x_max) / 2;
        int16_t y_offset = (y_min + y_max) / 2;

        /* Công thức chuẩn để Bắc=0, Đông=90 */
        float cx = -(float)(mx - x_offset); // Đảo X để Bắc đúng
        float cy = (float)(my - y_offset);  // Giữ Y (hoặc đảo tùy board) để Đông đúng
        
        /* Nếu sau bước này vẫn ngược Đông/Tây, hãy đổi dấu cy */
        float heading = atan2f(cy, cx) * 57.29578f;
        if (heading < 0) heading += 360.0f;
        
        data->heading_deg = heading;
        data->mag_x_ut = (float)mx;
        data->mag_y_ut = (float)my;
        mag_present = 1;

        // Publish lên uORB
        sensor_mag_s msg;
        msg.timestamp_us = HAL_GetTick() * 1000ULL; // Nên dùng hrt_absolute_time() sau này
        msg.mag_x_ut = data->mag_x_ut;
        msg.mag_y_ut = data->mag_y_ut;
        msg.mag_z_ut = 0.0f; // QMC5883L có Z nhưng driver cũ chưa đọc Z, giả lập là 0
        msg.heading_deg = data->heading_deg;
        msg.temperature_c = 0.0f;
        msg.device_id = 0;
        mag_pub.publish(msg);
    } else {
        mag_present = 0;
    }
}

void QMC5883L_PrintLog(const QMC5883L_Data_t *data)
{
    char buf[128];
    if (!mag_present) {
        snprintf(buf, sizeof(buf), "[MAG] NOT PRESENT\r\n");
    } else {
        const char *dir;
        float h = data->heading_deg;
        if (h < 22.5f || h >= 337.5f) dir = "B";
        else if (h < 67.5f) dir = "BD";
        else if (h < 112.5f) dir = "D";
        else if (h < 157.5f) dir = "ND";
        else if (h < 202.5f) dir = "N";
        else if (h < 247.5f) dir = "NT";
        else if (h < 292.5f) dir = "T";
        else dir = "BT";

        snprintf(buf, sizeof(buf), "[MAG] Head:%d [%s] (X:%d Y:%d)\r\n", 
                 (int)h, dir, (int)data->mag_x_ut, (int)data->mag_y_ut);
    }
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
}
