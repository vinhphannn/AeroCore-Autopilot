#include "pmw3901.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include "../../MessageBus/topics/sensor_optical_flow.h"
#include "../../Main/fc_logging.h"

static uORB::PublicationMulti<sensor_optical_flow_s> opt_pub(orb_sensor_optical_flow);

extern "C" {
    extern SPI_HandleTypeDef hspi4;
}
#define PMW_CS_PORT GPIOD
#define PMW_CS_PIN  GPIO_PIN_11

static inline void pmw_cs_delay(void) {
    for(volatile uint32_t i = 0; i < 500; i++) __NOP();
}

static void PMW_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), val };
    HAL_GPIO_WritePin(PMW_CS_PORT, PMW_CS_PIN, GPIO_PIN_RESET);
    pmw_cs_delay();
    HAL_SPI_Transmit(&hspi4, tx, 2, 10);
    pmw_cs_delay();
    HAL_GPIO_WritePin(PMW_CS_PORT, PMW_CS_PIN, GPIO_PIN_SET);
}

static uint8_t PMW_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0xFF };
    uint8_t rx[2] = { 0, 0 };
    HAL_GPIO_WritePin(PMW_CS_PORT, PMW_CS_PIN, GPIO_PIN_RESET);
    pmw_cs_delay();
    HAL_SPI_TransmitReceive(&hspi4, tx, rx, 2, 10);
    pmw_cs_delay();
    HAL_GPIO_WritePin(PMW_CS_PORT, PMW_CS_PIN, GPIO_PIN_SET);
    return rx[1];
}

static void PMW_InitRegisters(void)
{
    PMW_WriteReg(0x7F, 0x00); PMW_WriteReg(0x61, 0xAD); PMW_WriteReg(0x7F, 0x03);
    PMW_WriteReg(0x40, 0x00); PMW_WriteReg(0x7F, 0x05); PMW_WriteReg(0x41, 0xB3);
    PMW_WriteReg(0x43, 0xF1); PMW_WriteReg(0x45, 0x14); PMW_WriteReg(0x5B, 0x32);
    PMW_WriteReg(0x5F, 0x34); PMW_WriteReg(0x7B, 0x08); PMW_WriteReg(0x7F, 0x06);
    PMW_WriteReg(0x44, 0x1B); PMW_WriteReg(0x40, 0xBF); PMW_WriteReg(0x4E, 0x3F);
    PMW_WriteReg(0x7F, 0x08); PMW_WriteReg(0x65, 0x20); PMW_WriteReg(0x6A, 0x18);
    PMW_WriteReg(0x7F, 0x09); PMW_WriteReg(0x4F, 0xAF); PMW_WriteReg(0x5F, 0x40);
    PMW_WriteReg(0x48, 0x80); PMW_WriteReg(0x49, 0x80); PMW_WriteReg(0x57, 0x77);
    PMW_WriteReg(0x60, 0x78); PMW_WriteReg(0x61, 0x78); PMW_WriteReg(0x62, 0x08);
    PMW_WriteReg(0x63, 0x50); PMW_WriteReg(0x7F, 0x0A); PMW_WriteReg(0x45, 0x60);
    PMW_WriteReg(0x7F, 0x00); PMW_WriteReg(0x4D, 0x11); PMW_WriteReg(0x55, 0x80);
    PMW_WriteReg(0x74, 0x1F); PMW_WriteReg(0x75, 0x1F); PMW_WriteReg(0x4A, 0x78);
    PMW_WriteReg(0x4B, 0x78); PMW_WriteReg(0x44, 0x08); PMW_WriteReg(0x45, 0x50);
    PMW_WriteReg(0x64, 0xFF); PMW_WriteReg(0x65, 0x1F); PMW_WriteReg(0x7F, 0x14);
    PMW_WriteReg(0x65, 0x67); PMW_WriteReg(0x66, 0x08); PMW_WriteReg(0x63, 0x70);
    PMW_WriteReg(0x7F, 0x15); PMW_WriteReg(0x48, 0x48); PMW_WriteReg(0x7F, 0x07);
    PMW_WriteReg(0x41, 0x0D); PMW_WriteReg(0x43, 0x14); PMW_WriteReg(0x4B, 0x0E);
    PMW_WriteReg(0x45, 0x0F); PMW_WriteReg(0x44, 0x42); PMW_WriteReg(0x4C, 0x80);
    PMW_WriteReg(0x7F, 0x10); PMW_WriteReg(0x5B, 0x02); PMW_WriteReg(0x7F, 0x07);
    PMW_WriteReg(0x40, 0x41); PMW_WriteReg(0x70, 0x00);
    HAL_Delay(10);
    PMW_WriteReg(0x32, 0x44); PMW_WriteReg(0x7F, 0x07); PMW_WriteReg(0x40, 0x40);
    PMW_WriteReg(0x7F, 0x06); PMW_WriteReg(0x62, 0xF0); PMW_WriteReg(0x63, 0x00);
    PMW_WriteReg(0x7F, 0x0D); PMW_WriteReg(0x48, 0xC0); PMW_WriteReg(0x6F, 0xD5);
    PMW_WriteReg(0x7F, 0x00); PMW_WriteReg(0x5B, 0xA0); PMW_WriteReg(0x4E, 0xA8);
    PMW_WriteReg(0x5A, 0x50); PMW_WriteReg(0x40, 0x80);
    PMW_WriteReg(0x7F, 0x00); PMW_WriteReg(0x5A, 0x10); PMW_WriteReg(0x54, 0x00);
}

uint8_t PMW3901_Init(void)
{
    HAL_GPIO_WritePin(PMW_CS_PORT, PMW_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(100);
    PMW_WriteReg(0x3A, 0x5A);
    HAL_Delay(10);
    uint8_t id = PMW_ReadReg(0x00);
    
    if (id != 0x49) {
        FC_ERR("PMW3901 Init FAIL on SPI4 (ID = 0x%02X)", id);
        return 0;
    }
    
    PMW_InitRegisters();
    FC_INFO("PMW3901 Init OK on SPI4 (ID = 0x%02X)", id);
    return 1;
}

void PMW3901_ReadMotion(PMW3901_Data_t *data)
{
    if (!data) return;
    uint8_t tx[13] = {0x16, 0};
    uint8_t rx[13] = {0};
    HAL_GPIO_WritePin(PMW_CS_PORT, PMW_CS_PIN, GPIO_PIN_RESET);
    pmw_cs_delay();
    HAL_SPI_TransmitReceive(&hspi4, tx, rx, 13, 10);
    pmw_cs_delay();
    HAL_GPIO_WritePin(PMW_CS_PORT, PMW_CS_PIN, GPIO_PIN_SET);
    data->deltaX = (int16_t)((rx[4] << 8) | rx[3]);
    data->deltaY = (int16_t)((rx[6] << 8) | rx[5]);
    data->squal = rx[7];

    // Publish uORB
    sensor_optical_flow_s msg;
    msg.timestamp_us = HAL_GetTick() * 1000ULL;
    msg.pixel_flow_x_integral = data->deltaX;
    msg.pixel_flow_y_integral = data->deltaY;
    msg.integration_timespan_us = 10000; // Giả sử loop 100Hz = 10ms
    msg.quality = data->squal;
    msg.device_id = 0;
    opt_pub.publish(msg);
}

void PMW3901_PrintLog(const PMW3901_Data_t *data)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "[OP] SQ:%d X:%d Y:%d\r\n", data->squal, data->deltaX, data->deltaY);
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
}
