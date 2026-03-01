#ifndef BLUENRG_M0_H
#define BLUENRG_M0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * BlueNRG-M0 host interface lines (per datasheet):
 * - SPI: CLK, CS, MOSI, MISO
 * - IRQ: indicates data available from slave
 * - RESET: active-low reset input
 * Datasheet also notes SPI clock max 8 MHz and IRQ requires external pull-down. :contentReference[oaicite:2]{index=2}
 */

typedef struct
{
    SPI_HandleTypeDef *hspi;

    /* Chip select */
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    /* IRQ (data ready) */
    GPIO_TypeDef *irq_port;
    uint16_t irq_pin;

    /* Reset (active low) */
    GPIO_TypeDef *rst_port;
    uint16_t rst_pin;

    uint32_t spi_timeout_ms;
} BlueNRG_M0_Handle_t;

typedef enum
{
    BLUENRG_OK = 0,
    BLUENRG_ERR_NULL = -1,
    BLUENRG_ERR_SPI  = -2
} BlueNRG_Status_t;

/* ---------- Basic control ---------- */
BlueNRG_Status_t BlueNRG_M0_Init(BlueNRG_M0_Handle_t *dev,
                                 SPI_HandleTypeDef *hspi,
                                 GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                 GPIO_TypeDef *irq_port, uint16_t irq_pin,
                                 GPIO_TypeDef *rst_port, uint16_t rst_pin);

void BlueNRG_M0_ResetPulse(BlueNRG_M0_Handle_t *dev,
                           uint32_t low_time_ms,
                           uint32_t post_delay_ms);

/* ---------- IRQ helpers ---------- */
static inline bool BlueNRG_M0_IrqIsAsserted(BlueNRG_M0_Handle_t *dev)
{
    if (!dev || !dev->irq_port) return false;
    return (HAL_GPIO_ReadPin(dev->irq_port, dev->irq_pin) == GPIO_PIN_SET);
}

/* Optional: call this from EXTI ISR when IRQ rises */
void BlueNRG_M0_OnIrqRise(BlueNRG_M0_Handle_t *dev);

/* ---------- SPI helpers (8-bit) ---------- */
BlueNRG_Status_t BlueNRG_M0_SpiWriteRead(BlueNRG_M0_Handle_t *dev,
                                        const uint8_t *tx,
                                        uint8_t *rx,
                                        uint16_t len);

/* Convenience: write-only / read-only */
BlueNRG_Status_t BlueNRG_M0_SpiWrite(BlueNRG_M0_Handle_t *dev,
                                    const uint8_t *tx,
                                    uint16_t len);

BlueNRG_Status_t BlueNRG_M0_SpiRead(BlueNRG_M0_Handle_t *dev,
                                   uint8_t *rx,
                                   uint16_t len,
                                   uint8_t fill_byte);

/*
 * ---------- ACI transport (generic) ----------
 * This is a "raw" exchange:
 * - If you use ST's host ACI library, it typically needs a function that:
 *   1) checks IRQ
 *   2) does an SPI transaction
 *   3) returns received bytes
 *
 * You can use this as your low-level hook.
 */
BlueNRG_Status_t BlueNRG_M0_AciRawExchange(BlueNRG_M0_Handle_t *dev,
                                          const uint8_t *tx,
                                          uint8_t *rx,
                                          uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* BLUENRG_M0_H */
