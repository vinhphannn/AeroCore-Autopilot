#ifndef __QMC5883L_H
#define __QMC5883L_H

#include "main.h"
#include "i2c.h"
#include <stdint.h>

/* I2C address (7-bit → shift 1 for HAL) */
#define QMC_I2C_ADDR     (0x0D << 1)  /* 0x1A */

/* Registers */
#define QMC_REG_DATA_XL  0x00   /* X Low,  X High, Y Low, Y High, Z Low, Z High */
#define QMC_REG_STATUS   0x06
#define QMC_REG_TPR      0x07
#define QMC_REG_CTRL1    0x09   /* Mode, ODR, RNG, OSR */
#define QMC_REG_CTRL2    0x0A
#define QMC_REG_SETRESET 0x0B   /* Must write 0x01 */
#define QMC_REG_CHIP_ID  0x0D   /* Expected: 0xFF */

/* Sensitivity for ±8G range: 3000 LSB/Gauss */
#define QMC_SENS_8G  3000.0f

typedef struct {
    uint8_t chip_id;            /* reg 0x0D, expect 0xFF */
    int16_t raw_x, raw_y, raw_z;
    float   mag_x_ut, mag_y_ut, mag_z_ut;  /* micro-Tesla (µT) */
    float   heading_deg;        /* 0 = North, 90 = East */
    uint8_t status;             /* STATUS register */
} QMC5883L_Data_t;

#ifdef __cplusplus
extern "C" {
#endif

/* ---- API ---- */
/**
 * @brief Init: configure continuous mode 200Hz 8G OSR512, read + log chip ID.
 * @return 1 = OK (chip_id == 0xFF), 0 = FAIL
 */
uint8_t QMC5883L_Init(void);

/**
 * @brief Read X/Y/Z raw + compute µT and heading.
 */
void    QMC5883L_Read(QMC5883L_Data_t *d);

/**
 * @brief Print compass data over USB-CDC.
 */
void    QMC5883L_PrintLog(const QMC5883L_Data_t *d);

#ifdef __cplusplus
}
#endif

#endif /* __QMC5883L_H */
