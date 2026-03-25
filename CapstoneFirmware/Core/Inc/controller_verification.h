#ifndef CONTROLLER_VERIFICATION_H
#define CONTROLLER_VERIFICATION_H

#include "as5047p.h"
#include "foc.h"
#include "ms_switch.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  VERIFY_TEST_PWM = 1u,
  VERIFY_TEST_MS_SWITCH = 2u,
  VERIFY_TEST_CURRENT_SENSE = 3u,
  VERIFY_TEST_FOC_MATH = 4u,
  VERIFY_TEST_AS5047P = 5u,
  VERIFY_TEST_THROTTLE = 6u,
  VERIFY_TEST_MOTOR_FUNCTION = 7u
} VerifyTestId_t;

/*
 * Enable exactly ONE test with 1u, keep all others 0u.
 * This is the primary selector used by verification init.
 */
#define VERIFY_ENABLE_TEST_PWM 0u
#define VERIFY_ENABLE_TEST_MS_SWITCH 0u
#define VERIFY_ENABLE_TEST_CURRENT_SENSE 0u
#define VERIFY_ENABLE_TEST_FOC_MATH 0u
#define VERIFY_ENABLE_TEST_AS5047P 0u
#define VERIFY_ENABLE_TEST_THROTTLE 0u
#define VERIFY_ENABLE_TEST_MOTOR_FUNCTION 1u

typedef struct {
  uint8_t completed;
  uint8_t started;
  VerifyTestId_t current_test;
  uint8_t test_phase;
  uint32_t state_tick;
  uint32_t test_start_tick;
  float motor_iq_cmd;
} ControllerVerification_t;

void ControllerVerification_Init(ControllerVerification_t *ctx);
void ControllerVerification_RunStep(ControllerVerification_t *ctx,
                                    TIM_HandleTypeDef *tim_pwm,
                                    MS_Switch_t *ms,
                                    AS5047P_Handle_t *as5047,
                                    int32_t throttle_count,
                                    float max_rpm,
                                    float actual_rpm);

#endif /* CONTROLLER_VERIFICATION_H */
