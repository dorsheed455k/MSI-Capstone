#ifndef ENCODER_AS5047P_H
#define ENCODER_AS5047P_H

#include "as5047p.h"

typedef struct {
  AS5047P_Handle_t dev;
  uint16_t angle14;
  float mech_rad;
  float mech_rpm;
  uint16_t last_angle14;
  float last_dt;
} Encoder_t;

void Encoder_Init(Encoder_t *e, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin);

void Encoder_Update(Encoder_t *e, float dt_s);

#endif
