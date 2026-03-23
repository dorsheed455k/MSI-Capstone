// git test
#ifndef CONTROLLER_VERIFICATION_H
#define CONTROLLER_VERIFICATION_H

#include "as5047p.h"
#include "foc.h"
#include "ms_switch.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>

typedef struct {
  uint8_t completed;
  uint8_t current_test;
  uint32_t state_tick;
} ControllerVerification_t;

void ControllerVerification_Init(ControllerVerification_t *ctx);
void ControllerVerification_RunStep(ControllerVerification_t *ctx,
                                    TIM_HandleTypeDef *tim_pwm,
                                    MS_Switch_t *ms,
                                    FOC_t *foc,
                                    AS5047P_Handle_t *as5047,
                                    int32_t throttle_count,
                                    float max_rpm);

#endif /* CONTROLLER_VERIFICATION_H */
