#ifndef AS5047P_H
#define AS5047P_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* =========================
 *  AS5047P register map
 * ========================= */
#define AS5047P_REG_NOP        0x0000u
#define AS5047P_REG_ERRFL      0x0001u
#define AS5047P_REG_PROG       0x0003u

#define AS5047P_REG_DIAAGC     0x3FFCu
#define AS5047P_REG_MAG        0x3FFDu
#define AS5047P_REG_ANGLEUNC   0x3FFEu
#define AS5047P_REG_ANGLECOM   0x3FFFu

/* ERRFL bits (volatile) */
#define AS5047P_ERRFL_FRERR_Pos    0u
#define AS5047P_ERRFL_INVCOMM_Pos  1u
#define AS5047P_ERRFL_PARERR_Pos   2u

/* DIAAGC bits */
#define AS5047P_DIAAGC_AGC_Msk     0x00FFu
#define AS5047P_DIAAGC_LF_Pos      8u
#define AS5047P_DIAAGC_COF_Pos     9u
#define AS5047P_DIAAGC_MAGH_Pos    10u
#define AS5047P_DIAAGC_MAGL_Pos    11u

/* Angle is 14-bit (0..16383) */
#define AS5047P_ANGLE_MAX_COUNTS   16384u

typedef struct
{
    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    /* Optional: SPI timeout (ms) */
    uint32_t timeout_ms;
} AS5047P_Handle_t;

typedef enum
{
    AS5047P_OK = 0,
    AS5047P_ERR_NULL = -1,
    AS5047P_ERR_SPI = -2,
    AS5047P_ERR_PARITY = -3,
    AS5047P_ERR_EF = -4
} AS5047P_Status_t;

/* =========================
 *  Public API
 * ========================= */
AS5047P_Status_t AS5047P_Init(AS5047P_Handle_t *dev,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port,
                              uint16_t cs_pin);

AS5047P_Status_t AS5047P_ReadRegister(AS5047P_Handle_t *dev,
                                      uint16_t addr,
                                      uint16_t *data14,
                                      bool *ef);

/* Convenience reads */
AS5047P_Status_t AS5047P_ReadAngleCom(AS5047P_Handle_t *dev, uint16_t *angle14);
AS5047P_Status_t AS5047P_ReadAngleUnc(AS5047P_Handle_t *dev, uint16_t *angle14);
AS5047P_Status_t AS5047P_ReadMag(AS5047P_Handle_t *dev, uint16_t *mag14);
AS5047P_Status_t AS5047P_ReadDiaAgc(AS5047P_Handle_t *dev, uint16_t *diaagc);
AS5047P_Status_t AS5047P_ReadErrfl(AS5047P_Handle_t *dev, uint16_t *errfl);

/* Helpers */
static inline float AS5047P_AngleToDegrees(uint16_t angle14)
{
    return (360.0f * (float)angle14) / (float)AS5047P_ANGLE_MAX_COUNTS;
}

static inline float AS5047P_AngleToRadians(uint16_t angle14)
{
    return (6.28318530718f * (float)angle14) / (float)AS5047P_ANGLE_MAX_COUNTS;
}

/* Speed estimate from delta-angle + dt (seconds). Returns RPM. */
float AS5047P_EstimateRPM(uint16_t prev_angle14,
                          uint16_t curr_angle14,
                          float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif /* AS5047P_H */
