#ifndef __PMW3901_H
#define __PMW3901_H

#include "main.h"
#include "spi.h"
#include <stdint.h>

/* Registers */
#define PMW3901_REG_ID          0x00   /* Product ID: expect 0x49 */
#define PMW3901_REG_INV_ID      0x5F   /* Inverse Product ID: expect 0xB6 */
#define PMW3901_REG_MOTION      0x02
#define PMW3901_REG_BURST       0x16

typedef struct {
    uint8_t  chip_id;       /* Product ID (0x49 = OK)  */
    uint8_t  inv_id;        /* Inverse ID (0xB6 = OK)  */
    int16_t  deltaX;        /* Motion delta X (pixels) */
    int16_t  deltaY;        /* Motion delta Y (pixels) */
    uint8_t  squal;         /* Surface quality (0–255, >25 = good flow) */
    uint8_t  motion;        /* Motion register byte    */
    uint8_t  raw_sum;       /* Raw pixel average       */
    uint8_t  raw_max;       /* Raw pixel max           */
    uint8_t  raw_min;       /* Raw pixel min           */
    uint16_t shutter;       /* Shutter (exposure) time */
    char     raw_str[48];   /* Debug HEX of burst buffer */
} PMW3901_Data_t;

/* ---- API ---- */
#ifdef __cplusplus
extern "C" {
#endif

uint8_t PMW3901_Init(void);
void    PMW3901_ReadMotion(PMW3901_Data_t *data);
void    PMW3901_PrintLog(const PMW3901_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __PMW3901_H */
