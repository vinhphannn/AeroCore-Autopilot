#include "nrf24l01.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef NRF_SPI;

/* ------------------------------------------------------------------ */
static inline void nrf_cs_delay(void)
{
    /* ~1-2µs on H743 480MHz */
    for (volatile uint32_t i = 0; i < 500; i++) __NOP();
}

static uint8_t NRF_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(NRF_CMD_R_REGISTER | (reg & 0x1F)), NRF_CMD_NOP };
    uint8_t rx[2] = { 0, 0 };
    HAL_GPIO_WritePin(NRF_CS_PORT, NRF_CS_PIN, GPIO_PIN_RESET);
    nrf_cs_delay();
    HAL_SPI_TransmitReceive(&NRF_SPI, tx, rx, 2, 10);
    nrf_cs_delay();
    HAL_GPIO_WritePin(NRF_CS_PORT, NRF_CS_PIN, GPIO_PIN_SET);
    return rx[1]; /* rx[0] = STATUS (auto-returned), rx[1] = register value */
}

static uint8_t NRF_GetStatus(void)
{
    uint8_t tx = NRF_CMD_NOP;
    uint8_t rx = 0;
    HAL_GPIO_WritePin(NRF_CS_PORT, NRF_CS_PIN, GPIO_PIN_RESET);
    nrf_cs_delay();
    HAL_SPI_TransmitReceive(&NRF_SPI, &tx, &rx, 1, 5);
    nrf_cs_delay();
    HAL_GPIO_WritePin(NRF_CS_PORT, NRF_CS_PIN, GPIO_PIN_SET);
    return rx;
}

/* ------------------------------------------------------------------ */
uint8_t NRF24L01_Init(NRF24L01_Data_t *d)
{
    if (!d) return 0;

    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET); /* CE low = Standby */
    HAL_GPIO_WritePin(NRF_CS_PORT, NRF_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(5);

    /* Read key registers */
    d->status      = NRF_GetStatus();
    d->config      = NRF_ReadReg(NRF_REG_CONFIG);
    d->en_aa       = NRF_ReadReg(NRF_REG_EN_AA);
    d->setup_aw    = NRF_ReadReg(NRF_REG_SETUP_AW);
    d->rf_ch       = NRF_ReadReg(NRF_REG_RF_CH);
    d->rf_setup    = NRF_ReadReg(NRF_REG_RF_SETUP);
    d->fifo_status = NRF_ReadReg(NRF_REG_FIFO_STATUS);

    /* nRF alive: STATUS != 0x00 and != 0xFF */
    d->alive = (d->status != 0x00 && d->status != 0xFF) ? 1 : 0;

    char buf[320];
    snprintf(buf, sizeof(buf),
        "\r\n=== nRF24L01 (SPI4, CS=PE11, CE=PE15) ===\r\n"
        "  STATUS     : 0x%02X  [%s]\r\n"
        "  CONFIG     : 0x%02X  (POR=0x08: PWR_UP=0, PRIM_RX=0)\r\n"
        "  EN_AA      : 0x%02X  (POR=0x3F: auto-ack all pipes)\r\n"
        "  SETUP_AW   : 0x%02X  (POR=0x03 = 5-byte address)\r\n"
        "  RF_CH      : 0x%02X  (channel: POR=2 → 2.402GHz)\r\n"
        "  RF_SETUP   : 0x%02X  (POR=0x0E: 2Mbps, 0dBm)\r\n"
        "  FIFO_STATUS: 0x%02X\r\n",
        d->status, d->alive ? "ALIVE" : "NO RESPONSE",
        d->config, d->en_aa, d->setup_aw,
        d->rf_ch, d->rf_setup, d->fifo_status);
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
    HAL_Delay(10);

    return d->alive;
}

/* ------------------------------------------------------------------ */
void NRF24L01_Read(NRF24L01_Data_t *d)
{
    if (!d) return;
    d->status      = NRF_GetStatus();
    d->fifo_status = NRF_ReadReg(NRF_REG_FIFO_STATUS);
    d->alive       = (d->status != 0x00 && d->status != 0xFF) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
void NRF24L01_PrintLog(const NRF24L01_Data_t *d)
{
    if (!d) return;
    char buf[128];
    snprintf(buf, sizeof(buf),
        "[nRF24L01 ] STATUS:0x%02X(%s)  FIFO:0x%02X\r\n"
        "  RX_DR:%d TX_DS:%d MAX_RT:%d  TX_FULL:%d\r\n",
        d->status, d->alive ? "OK" : "FAIL", d->fifo_status,
        (d->status >> 6) & 1,   /* RX_DR  */
        (d->status >> 5) & 1,   /* TX_DS  */
        (d->status >> 4) & 1,   /* MAX_RT */
        d->status & 1);         /* TX_FULL */
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
}
