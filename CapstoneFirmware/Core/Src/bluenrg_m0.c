#include "bluenrg_m0.h"

/* ---- CS helpers ---- */
static inline void cs_low(BlueNRG_M0_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(BlueNRG_M0_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

BlueNRG_Status_t BlueNRG_M0_Init(BlueNRG_M0_Handle_t *dev,
                                 SPI_HandleTypeDef *hspi,
                                 GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                 GPIO_TypeDef *irq_port, uint16_t irq_pin,
                                 GPIO_TypeDef *rst_port, uint16_t rst_pin)
{
    if (!dev || !hspi || !cs_port || !irq_port || !rst_port)
        return BLUENRG_ERR_NULL;

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;

    dev->irq_port = irq_port;
    dev->irq_pin  = irq_pin;

    dev->rst_port = rst_port;
    dev->rst_pin  = rst_pin;

    dev->spi_timeout_ms = 20;

    /* Default idle states: CS high, RESET high (inactive) */
    cs_high(dev);
    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);

    return BLUENRG_OK;
}

void BlueNRG_M0_ResetPulse(BlueNRG_M0_Handle_t *dev,
                           uint32_t low_time_ms,
                           uint32_t post_delay_ms)
{
    if (!dev) return;

    /* RESET is active-low per datasheet. :contentReference[oaicite:3]{index=3} */
    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(low_time_ms);

    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);
    HAL_Delay(post_delay_ms);
}

/* If you use EXTI on IRQ, call this from HAL_GPIO_EXTI_Callback */
void BlueNRG_M0_OnIrqRise(BlueNRG_M0_Handle_t *dev)
{
    (void)dev;
    /* Hook point: set a flag, release a semaphore, etc. */
}

BlueNRG_Status_t BlueNRG_M0_SpiWriteRead(BlueNRG_M0_Handle_t *dev,
                                        const uint8_t *tx,
                                        uint8_t *rx,
                                        uint16_t len)
{
    if (!dev || !dev->hspi || !tx || !rx || (len == 0))
        return BLUENRG_ERR_NULL;

    cs_low(dev);

    HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive(dev->hspi,
                                (uint8_t*)tx,
                                rx,
                                len,
                                dev->spi_timeout_ms);

    cs_high(dev);

    return (st == HAL_OK) ? BLUENRG_OK : BLUENRG_ERR_SPI;
}

BlueNRG_Status_t BlueNRG_M0_SpiWrite(BlueNRG_M0_Handle_t *dev,
                                    const uint8_t *tx,
                                    uint16_t len)
{
    if (!dev || !dev->hspi || !tx || (len == 0))
        return BLUENRG_ERR_NULL;

    cs_low(dev);

    HAL_StatusTypeDef st =
        HAL_SPI_Transmit(dev->hspi,
                         (uint8_t*)tx,
                         len,
                         dev->spi_timeout_ms);

    cs_high(dev);

    return (st == HAL_OK) ? BLUENRG_OK : BLUENRG_ERR_SPI;
}

BlueNRG_Status_t BlueNRG_M0_SpiRead(BlueNRG_M0_Handle_t *dev,
                                   uint8_t *rx,
                                   uint16_t len,
                                   uint8_t fill_byte)
{
    if (!dev || !dev->hspi || !rx || (len == 0))
        return BLUENRG_ERR_NULL;

    cs_low(dev);

    HAL_StatusTypeDef st =
        HAL_SPI_Receive(dev->hspi,
                        rx,
                        len,
                        dev->spi_timeout_ms);

    /* If your hardware needs clocking for reads, use TransmitReceive with fill_byte:
       - Create a temp buffer filled with fill_byte and do TransmitReceive instead.
       Some SPI slaves require clocks for data out; ACI typically does.
    */

    cs_high(dev);

    return (st == HAL_OK) ? BLUENRG_OK : BLUENRG_ERR_SPI;
}

BlueNRG_Status_t BlueNRG_M0_AciRawExchange(BlueNRG_M0_Handle_t *dev,
                                          const uint8_t *tx,
                                          uint8_t *rx,
                                          uint16_t len)
{
    /* This is intentionally identical to SpiWriteRead. Keep it separate as the
       “ACI transport hook” so you can swap implementation later (DMA/IRQ, etc.). */
    return BlueNRG_M0_SpiWriteRead(dev, tx, rx, len);
}

