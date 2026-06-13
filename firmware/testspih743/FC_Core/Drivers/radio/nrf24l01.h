#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "main.h"
#include "spi.h"
#include <stdint.h>

/* Hardware mapping */
#define NRF_SPI         hspi4
#define NRF_CS_PORT     GPIOE
#define NRF_CS_PIN      GPIO_PIN_11
#define NRF_CE_PORT     GPIOE
#define NRF_CE_PIN      GPIO_PIN_15

/* SPI Commands */
#define NRF_CMD_R_REGISTER   0x00   /* | reg addr */
#define NRF_CMD_W_REGISTER   0x20   /* | reg addr */
#define NRF_CMD_NOP          0xFF

/* Registers */
#define NRF_REG_CONFIG       0x00   /* CONFIG: PRIM_RX, PWR_UP, CRC... POR=0x08 */
#define NRF_REG_EN_AA        0x01   /* Auto-Ack POR=0x3F */
#define NRF_REG_EN_RXADDR    0x02   /* POR=0x03 */
#define NRF_REG_SETUP_AW     0x03   /* Address width: 0x03=5byte */
#define NRF_REG_SETUP_RETR   0x04   /* Retry: POR=0x03 */
#define NRF_REG_RF_CH        0x05   /* RF channel: POR=0x02 */
#define NRF_REG_RF_SETUP     0x06   /* RF setup: POR=0x0E */
#define NRF_REG_STATUS       0x07   /* STATUS: RX_DR, TX_DS, MAX_RT */
#define NRF_REG_FIFO_STATUS  0x17   /* FIFO status */

typedef struct {
    uint8_t status;          /* STATUS register (byte returned on every SPI cmd) */
    uint8_t config;          /* CONFIG register  (POR: 0x08) */
    uint8_t en_aa;           /* EN_AA POR: 0x3F */
    uint8_t setup_aw;        /* SETUP_AW POR: 0x03 (5-byte addr) */
    uint8_t rf_ch;           /* RF_CH POR: 0x02 */
    uint8_t rf_setup;        /* RF_SETUP POR: 0x0E */
    uint8_t fifo_status;     /* FIFO_STATUS */
    uint8_t alive;           /* 1 = SPI communication OK */
} NRF24L01_Data_t;

/* ---- API ---- */
/**
 * @brief Read key registers to verify SPI communication and print startup log.
 * @return 1 = alive (STATUS != 0x00 and != 0xFF), 0 = not detected
 */
uint8_t NRF24L01_Init(NRF24L01_Data_t *d);

/**
 * @brief Re-read STATUS and FIFO_STATUS registers.
 */
void    NRF24L01_Read(NRF24L01_Data_t *d);

/**
 * @brief Print all register values over USB-CDC.
 */
void    NRF24L01_PrintLog(const NRF24L01_Data_t *d);

#endif /* __NRF24L01_H */
