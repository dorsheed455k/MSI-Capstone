#include "encoder_as5047p.h"

void Encoder_Init(Encoder_t *e, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  AS5047P_Init(&e->dev, hspi, cs_port, cs_pin);
  e->angle14 = 0;
  e->mech_rad = 0.0f;
  e->mech_rpm = 0.0f;
  e->last_angle14 = 0;
  e->last_dt = 0.0f;
}

void Encoder_Update(Encoder_t *e, float dt_s)
{
  uint16_t ang = 0;
  if (AS5047P_ReadAngleCom(&e->dev, &ang) == AS5047P_OK)
  {
    e->angle14 = ang;
    e->mech_rad = AS5047P_AngleToRadians(ang);
    if (dt_s > 0.0f)
      e->mech_rpm = AS5047P_EstimateRPM(e->last_angle14, ang, dt_s);
    e->last_angle14 = ang;
    e->last_dt = dt_s;
  }
}
