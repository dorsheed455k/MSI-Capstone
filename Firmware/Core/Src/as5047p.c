#include "as5047p.h"

/* ======== low-level helpers ======== */

static inline void cs_low(AS5047P_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(AS5047P_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/* Even parity of a 16-bit word: returns 1 if number of 1s is odd, else 0.
   For "even parity bit" you typically set parity_bit = odd_parity(lower_bits). */
static uint8_t parity_odd_u16(uint16_t x)
{
    /* XOR fold */
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return (uint8_t)(x & 0x1u);
}

/* Build command frame:
   bit15 PARC = even parity over lower 15 bits (bits14..0)
   bit14 R/W
   bits13..0 ADDR
*/
static uint16_t build_cmd(uint16_t addr14, bool is_read)
{
    uint16_t cmd = 0;
    cmd |= (uint16_t)(addr14 & 0x3FFFu);
    if (is_read) cmd |= (1u << 14);

    /* parity over lower 15 bits */
    uint16_t lower15 = (uint16_t)(cmd & 0x7FFFu);
    uint16_t parc = (uint16_t)parity_odd_u16(lower15); /* set to make total even */
    cmd |= (uint16_t)(parc << 15);

    return cmd;
}

/* Check data-frame parity:
   bit15 PARD is even parity over lower 15 bits (bits14..0)
*/
static bool check_data_parity(uint16_t frame)
{
    uint16_t lower15 = (uint16_t)(frame & 0x7FFFu);
    uint8_t expected_pard = parity_odd_u16(lower15);
    uint8_t pard = (uint8_t)((frame >> 15) & 0x1u);
    return (pard == expected_pard);
}

/* SPI 16-bit transfer with manual CS.
   Many AS5x47 parts return the requested data on the *next* frame,
   so a read is typically:
     1) send READ(addr)  -> receives "previous" data
     2) send NOP         -> receives DATA(addr)
*/
static AS5047P_Status_t spi_xfer16(AS5047P_Handle_t *dev, uint16_t tx, uint16_t *rx)
{
    if (!dev || !dev->hspi) return AS5047P_ERR_NULL;

    HAL_StatusTypeDef st;
    uint16_t r = 0;

    cs_low(dev);
    st = HAL_SPI_TransmitReceive(dev->hspi,
                                 (uint8_t *)&tx,
                                 (uint8_t *)&r,
                                 1,  /* 1 x 16-bit word when DataSize=16bit */
                                 dev->timeout_ms);
    cs_high(dev);

    if (st != HAL_OK) return AS5047P_ERR_SPI;

    if (rx) *rx = r;
    return AS5047P_OK;
}

/* ======== public API ======== */

AS5047P_Status_t AS5047P_Init(AS5047P_Handle_t *dev,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port,
                              uint16_t cs_pin)
{
    if (!dev || !hspi || !cs_port) return AS5047P_ERR_NULL;

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->timeout_ms = 10;

    /* Ensure CS is high (inactive) */
    cs_high(dev);

    return AS5047P_OK;
}

AS5047P_Status_t AS5047P_ReadRegister(AS5047P_Handle_t *dev,
                                      uint16_t addr,
                                      uint16_t *data14,
                                      bool *ef)
{
    if (!dev || !data14) return AS5047P_ERR_NULL;

    uint16_t rx1 = 0, rx2 = 0;

    /* 1) Issue read command (data returned in NEXT frame) */
    uint16_t cmd = build_cmd((uint16_t)(addr & 0x3FFFu), true);
    AS5047P_Status_t st = spi_xfer16(dev, cmd, &rx1);
    if (st != AS5047P_OK) return st;

    /* 2) Send NOP to clock out response to read command */
    uint16_t nop = build_cmd(AS5047P_REG_NOP, true);
    st = spi_xfer16(dev, nop, &rx2);
    if (st != AS5047P_OK) return st;

    /* Validate parity on returned frame */
    if (!check_data_parity(rx2))
    {
        return AS5047P_ERR_PARITY;
    }

    bool ef_bit = ((rx2 >> 14) & 0x1u) ? true : false;
    if (ef) *ef = ef_bit;

    /* Extract 14-bit data */
    *data14 = (uint16_t)(rx2 & 0x3FFFu);

    /* If EF is set, caller may want to read ERRFL (0x0001) */
    if (ef_bit)
    {
        return AS5047P_ERR_EF;
    }

    return AS5047P_OK;
}

/* Convenience */
AS5047P_Status_t AS5047P_ReadAngleCom(AS5047P_Handle_t *dev, uint16_t *angle14)
{
    bool ef = false;
    return AS5047P_ReadRegister(dev, AS5047P_REG_ANGLECOM, angle14, &ef);
}

AS5047P_Status_t AS5047P_ReadAngleUnc(AS5047P_Handle_t *dev, uint16_t *angle14)
{
    bool ef = false;
    return AS5047P_ReadRegister(dev, AS5047P_REG_ANGLEUNC, angle14, &ef);
}

AS5047P_Status_t AS5047P_ReadMag(AS5047P_Handle_t *dev, uint16_t *mag14)
{
    bool ef = false;
    return AS5047P_ReadRegister(dev, AS5047P_REG_MAG, mag14, &ef);
}

AS5047P_Status_t AS5047P_ReadDiaAgc(AS5047P_Handle_t *dev, uint16_t *diaagc)
{
    bool ef = false;
    return AS5047P_ReadRegister(dev, AS5047P_REG_DIAAGC, diaagc, &ef);
}

AS5047P_Status_t AS5047P_ReadErrfl(AS5047P_Handle_t *dev, uint16_t *errfl)
{
    bool ef = false;
    /* Note: reading ERRFL clears it in the device */
    return AS5047P_ReadRegister(dev, AS5047P_REG_ERRFL, errfl, &ef);
}

/* RPM estimation with wrap handling:
   delta_counts mapped to [-8192, +8191] around wrap.
*/
float AS5047P_EstimateRPM(uint16_t prev_angle14,
                          uint16_t curr_angle14,
                          float dt_seconds)
{
    if (dt_seconds <= 0.0f) return 0.0f;

    int32_t delta = (int32_t)curr_angle14 - (int32_t)prev_angle14;

    /* Wrap to shortest path */
    if (delta > (int32_t)(AS5047P_ANGLE_MAX_COUNTS / 2u))
        delta -= (int32_t)AS5047P_ANGLE_MAX_COUNTS;
    else if (delta < -(int32_t)(AS5047P_ANGLE_MAX_COUNTS / 2u))
        delta += (int32_t)AS5047P_ANGLE_MAX_COUNTS;

    /* revolutions per second */
    float rps = ((float)delta / (float)AS5047P_ANGLE_MAX_COUNTS) / dt_seconds;

    return rps * 60.0f;
}
