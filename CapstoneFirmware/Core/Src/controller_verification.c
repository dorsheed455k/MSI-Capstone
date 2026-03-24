#include "controller_verification.h"

#include "amc1302.h"
#include <math.h>
#include <stdio.h>

#define TEST_PWM 1u
#define TEST_MS_SWITCH 2u
#define TEST_CURRENT_SENSE 3u
#define TEST_FOC_MATH 4u
#define TEST_AS5047P 5u
#define TEST_THROTTLE 6u

#define THROTTLE_MIN_COUNT 0
#define THROTTLE_MAX_COUNT 200

static float absf(float x) { return (x >= 0.0f) ? x : -x; }

static float throttle_to_rpm_limited(int32_t count, float max_rpm) {
  if (count < THROTTLE_MIN_COUNT) {
    count = THROTTLE_MIN_COUNT;
  }
  if (count > THROTTLE_MAX_COUNT) {
    count = THROTTLE_MAX_COUNT;
  }

  return ((float)(count - THROTTLE_MIN_COUNT) /
          (float)(THROTTLE_MAX_COUNT - THROTTLE_MIN_COUNT)) *
         max_rpm;
}

static void next_test(ControllerVerification_t *ctx) {
  ctx->current_test++;
  ctx->state_tick = HAL_GetTick();
  if (ctx->current_test > TEST_THROTTLE) {
    ctx->completed = 1u;
    printf("[VERIFY] All controller verification tests completed.\r\n");
  }
}

void ControllerVerification_Init(ControllerVerification_t *ctx) {
  if (!ctx) {
    return;
  }
  ctx->completed = 0u;
  ctx->current_test = TEST_PWM;
  ctx->state_tick = HAL_GetTick();
  
  HAL_Delay(1000); // Wait for USB enumeration/terminal connection
  printf("[VERIFY] Controller verification mode started.\r\n");
}

void ControllerVerification_RunStep(ControllerVerification_t *ctx,
                                    TIM_HandleTypeDef *tim_pwm,
                                    MS_Switch_t *ms,
                                    FOC_t *foc,
                                    AS5047P_Handle_t *as5047,
                                    int32_t throttle_count,
                                    float max_rpm) {
  uint32_t now;
  uint32_t arr;
  float alpha;
  float beta;
  float d;
  float q;
  float ia;
  float ib;
  float ic;
  float du;
  float dv;
  float dw;
  float amps;
  uint16_t angle;
  bool ef;
  AS5047P_Status_t st;
  float rpm_wrap_pos;
  float rpm_wrap_neg;
  float rpm_map;

  if (!ctx || ctx->completed) {
    return;
  }

  now = HAL_GetTick();

  switch (ctx->current_test) {
  case TEST_PWM:
    if (!tim_pwm) {
      printf("[VERIFY][PWM] FAIL: TIM handle is null.\r\n");
      next_test(ctx);
      break;
    }

    arr = __HAL_TIM_GET_AUTORELOAD(tim_pwm);
    __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, (arr + 1u) / 4u);
    __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, (arr + 1u) / 2u);
    __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_3, (3u * (arr + 1u)) / 4u);

    if ((tim_pwm->Instance->CCR1 > 0u) && (tim_pwm->Instance->CCR2 > 0u) &&
        (tim_pwm->Instance->CCR3 > 0u)) {
      printf("[VERIFY][PWM] PASS: TIM1 CCR1=%lu CCR2=%lu CCR3=%lu\r\n",
             (unsigned long)tim_pwm->Instance->CCR1,
             (unsigned long)tim_pwm->Instance->CCR2,
             (unsigned long)tim_pwm->Instance->CCR3);
    } else {
      printf("[VERIFY][PWM] FAIL: One or more CCR registers stayed zero.\r\n");
    }
    next_test(ctx);
    break;

  case TEST_MS_SWITCH:
    if (!ms) {
      printf("[VERIFY][MS] FAIL: MS switch handle is null.\r\n");
      next_test(ctx);
      break;
    }

    if ((now - ctx->state_tick) < 300u) {
      break;
    }

    if (ms->active_mode == 0 || ms->active_mode == MS_MODE_3_36V) {
      MS_SetMode(ms, MS_MODE_1_12V, MS_MODE2_VAR_A);
      printf("[VERIFY][MS] Set 12V mode.\r\n");
    } else if (ms->active_mode == MS_MODE_1_12V) {
      MS_SetMode(ms, MS_MODE_2_24V, MS_MODE2_VAR_A);
      printf("[VERIFY][MS] Set 24V mode.\r\n");
    } else {
      MS_SetMode(ms, MS_MODE_3_36V, MS_MODE2_VAR_A);
      printf("[VERIFY][MS] Set 36V mode.\r\n");
      printf("[VERIFY][MS] PASS: Mode cycle with deadtime executed.\r\n");
      next_test(ctx);
      break;
    }
    ctx->state_tick = now;
    break;

  case TEST_CURRENT_SENSE:
    amps = CurrentSense_ConvertCountsToAmps(256, 0.0f);
    if (isfinite(amps) && (absf(amps) < 1000.0f)) {
      printf("[VERIFY][ADC->A] PASS: counts=256 => %.5f A\r\n", amps);
    } else {
      printf("[VERIFY][ADC->A] FAIL: conversion result invalid.\r\n");
    }
    next_test(ctx);
    break;

  case TEST_FOC_MATH:
    ia = 1.0f;
    ib = -0.5f;
    ic = -0.5f;

    Clarke(ia, ib, ic, &alpha, &beta);
    Park(alpha, beta, 0.0f, &d, &q);
    SVM(alpha, beta, 24.0f, &du, &dv, &dw);

    if (absf(d - alpha) < 0.001f && absf(q - beta) < 0.001f &&
        du >= 0.0f && du <= 1.0f && dv >= 0.0f && dv <= 1.0f && dw >= 0.0f &&
        dw <= 1.0f) {
      printf("[VERIFY][FOC] PASS: Clarke/Park/SVM numerics valid.\r\n");
    } else {
      printf("[VERIFY][FOC] FAIL: transform or SVM output out of range.\r\n");
    }

    if (foc) {
      FOC_SetVdc(foc, 24.0f);
      FOC_Run(foc, ia, ib, ic, 0.1f, 1.0f / 25000.0f);
      printf("[VERIFY][FOC] Duty output du=%.3f dv=%.3f dw=%.3f\r\n", foc->d_u,
             foc->d_v, foc->d_w);
    }
    next_test(ctx);
    break;

  case TEST_AS5047P:
    rpm_wrap_pos = AS5047P_EstimateRPM(16380u, 5u, 0.001f);
    rpm_wrap_neg = AS5047P_EstimateRPM(5u, 16380u, 0.001f);
    printf("[VERIFY][AS5047P] Wrap check RPM +=%.2f RPM -=%.2f\r\n", rpm_wrap_pos,
           rpm_wrap_neg);

    if (as5047) {
      st = AS5047P_ReadAngleCom(as5047, &angle);
      ef = false;
      (void)AS5047P_ReadRegister(as5047, AS5047P_REG_ANGLECOM, &angle, &ef);
      if (st == AS5047P_OK) {
        printf("[VERIFY][AS5047P] PASS: live angle=%u\r\n", (unsigned)angle);
      } else {
        printf("[VERIFY][AS5047P] WARN: live read status=%d\r\n", (int)st);
      }
    }
    next_test(ctx);
    break;

  case TEST_THROTTLE:
    rpm_map = throttle_to_rpm_limited(throttle_count, max_rpm);
    if (rpm_map >= 0.0f && rpm_map <= max_rpm) {
      printf("[VERIFY][THROTTLE] PASS: count=%ld -> RPM=%.1f (range 0..%.1f)\r\n",
             (long)throttle_count, rpm_map, max_rpm);
    } else {
      printf("[VERIFY][THROTTLE] FAIL: mapped RPM out of range.\r\n");
    }
    next_test(ctx);
    break;

  default:
    ctx->completed = 1u;
    break;
  }
}
