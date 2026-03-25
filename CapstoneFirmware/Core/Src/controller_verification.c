#include "controller_verification.h"

#include "amc1302.h"
#include <math.h>
#include <stdio.h>

#define THROTTLE_MIN_COUNT 0
#define THROTTLE_MAX_COUNT 200
#define VERIFY_MS_DWELL_MS 300u

#define VERIFY_MOTOR_RPM_MIN_REACHED 150.0f
#define VERIFY_MOTOR_RAMP_UP_MS 2500u
#define VERIFY_MOTOR_HOLD_MS 1200u
#define VERIFY_MOTOR_RAMP_DOWN_MS 1200u
#define VERIFY_MOTOR_FREQ_START_HZ 2.0f
#define VERIFY_MOTOR_FREQ_TARGET_HZ 18.0f
#define VERIFY_MOTOR_SVM_VMAG 2.0f
#define VERIFY_TWO_PI 6.283185307f

static float absf(float x) { return (x >= 0.0f) ? x : -x; }

static void pwm_set_zero(TIM_HandleTypeDef *tim_pwm) {
  if (!tim_pwm) {
    return;
  }
  __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, 0u);
  __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, 0u);
  __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_3, 0u);
}

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

static void finish_test(ControllerVerification_t *ctx) {
  if (!ctx) {
    return;
  }
  ctx->completed = 1u;
  printf("[VERIFY] Test complete\r\n");
}

static void safe_shutdown(const char *reason, TIM_HandleTypeDef *tim_pwm,
                          MS_Switch_t *ms) {
  pwm_set_zero(tim_pwm);
  if (ms) {
    MS_AllOff(ms);
    MS_SetMode(ms, MS_MODE_1_12V, MS_MODE2_VAR_A);
  }
  printf("[VERIFY][MOTOR_FUNCTION] FAIL: %s\r\n", reason);
}

static VerifyTestId_t select_enabled_test(void) {
  uint8_t enabled_count = 0u;
  VerifyTestId_t selected = VERIFY_TEST_PWM;

  if (VERIFY_ENABLE_TEST_PWM) {
    enabled_count++;
    selected = VERIFY_TEST_PWM;
  }
  if (VERIFY_ENABLE_TEST_MS_SWITCH) {
    enabled_count++;
    selected = VERIFY_TEST_MS_SWITCH;
  }
  if (VERIFY_ENABLE_TEST_CURRENT_SENSE) {
    enabled_count++;
    selected = VERIFY_TEST_CURRENT_SENSE;
  }
  if (VERIFY_ENABLE_TEST_FOC_MATH) {
    enabled_count++;
    selected = VERIFY_TEST_FOC_MATH;
  }
  if (VERIFY_ENABLE_TEST_AS5047P) {
    enabled_count++;
    selected = VERIFY_TEST_AS5047P;
  }
  if (VERIFY_ENABLE_TEST_THROTTLE) {
    enabled_count++;
    selected = VERIFY_TEST_THROTTLE;
  }
  if (VERIFY_ENABLE_TEST_MOTOR_FUNCTION) {
    enabled_count++;
    selected = VERIFY_TEST_MOTOR_FUNCTION;
  }

  if (enabled_count != 1u) {
    return 0u;
  }
  return selected;
}

void ControllerVerification_Init(ControllerVerification_t *ctx) {
  VerifyTestId_t selected_test;
  if (!ctx) {
    return;
  }
  selected_test = select_enabled_test();
  ctx->completed = 0u;
  ctx->started = 0u;
  ctx->current_test = (selected_test == 0u) ? VERIFY_TEST_PWM : selected_test;
  ctx->test_phase = 0u;
  ctx->state_tick = HAL_GetTick();
  ctx->test_start_tick = HAL_GetTick();
  ctx->motor_iq_cmd = 0.0f;
  printf("[VERIFY] Controller verification mode started.\r\n");
  if (selected_test == 0u) {
    printf("[VERIFY] FAIL: Enable exactly one test flag in controller_verification.h\r\n");
    ctx->completed = 1u;
    return;
  }
  printf("[VERIFY] Selected test id=%u\r\n", (unsigned)selected_test);
}

void ControllerVerification_RunStep(ControllerVerification_t *ctx,
                                    TIM_HandleTypeDef *tim_pwm,
                                    MS_Switch_t *ms,
                                    AS5047P_Handle_t *as5047,
                                    int32_t throttle_count,
                                    float max_rpm,
                                    float actual_rpm) {
  uint32_t now = HAL_GetTick();
  uint32_t arr;
  float alpha;
  float beta;
  float d;
  float q;
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
  float theta;
  float freq_hz;
  float elapsed_s;
  float dt_s;
  uint32_t dt_ms;

  if (!ctx || ctx->completed) {
    return;
  }

  if (!ctx->started) {
    ctx->started = 1u;
    printf("[VERIFY] Starting test id=%u\r\n", (unsigned)ctx->current_test);
  }

  switch (ctx->current_test) {
  case VERIFY_TEST_PWM:
    if (!tim_pwm) {
      printf("[VERIFY][PWM] FAIL: TIM handle is null.\r\n");
      finish_test(ctx);
      break;
    }
    arr = __HAL_TIM_GET_AUTORELOAD(tim_pwm);
    __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, (arr + 1u) / 4u);
    __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, (arr + 1u) / 2u);
    __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_3, (3u * (arr + 1u)) / 4u);
    if ((tim_pwm->Instance->CCR1 > 0u) && (tim_pwm->Instance->CCR2 > 0u) &&
        (tim_pwm->Instance->CCR3 > 0u)) {
      printf("[VERIFY][PWM] PASS: CCR1=%lu CCR2=%lu CCR3=%lu\r\n",
             (unsigned long)tim_pwm->Instance->CCR1,
             (unsigned long)tim_pwm->Instance->CCR2,
             (unsigned long)tim_pwm->Instance->CCR3);
    } else {
      printf("[VERIFY][PWM] FAIL: One or more CCR registers stayed zero.\r\n");
    }
    finish_test(ctx);
    break;

  case VERIFY_TEST_MS_SWITCH:
    if (!ms) {
      printf("[VERIFY][MS_SWITCH] FAIL: handle is null.\r\n");
      finish_test(ctx);
      break;
    }
    if (ctx->test_phase == 0u) {
      MS_SetMode(ms, MS_MODE_1_12V, MS_MODE2_VAR_A);
      printf("[VERIFY][MS_SWITCH] Set 12V\r\n");
      ctx->test_phase = 1u;
      ctx->test_start_tick = now;
      break;
    }
    if (ctx->test_phase == 1u && (now - ctx->test_start_tick) >= VERIFY_MS_DWELL_MS) {
      MS_SetMode(ms, MS_MODE_2_24V, MS_MODE2_VAR_A);
      printf("[VERIFY][MS_SWITCH] Set 24V\r\n");
      ctx->test_phase = 2u;
      ctx->test_start_tick = now;
      break;
    }
    if (ctx->test_phase == 2u && (now - ctx->test_start_tick) >= VERIFY_MS_DWELL_MS) {
      MS_SetMode(ms, MS_MODE_3_36V, MS_MODE2_VAR_A);
      printf("[VERIFY][MS_SWITCH] Set 36V\r\n");
      printf("[VERIFY][MS_SWITCH] PASS: mode transitions executed.\r\n");
      finish_test(ctx);
      break;
    }
    break;

  case VERIFY_TEST_CURRENT_SENSE:
    amps = CurrentSense_ConvertCountsToAmps(256, 0.0f);
    if (isfinite(amps) && (absf(amps) < 1000.0f)) {
      printf("[VERIFY][CURRENT_SENSE] PASS: counts=256 => %.5f A\r\n", amps);
    } else {
      printf("[VERIFY][CURRENT_SENSE] FAIL: conversion result invalid.\r\n");
    }
    finish_test(ctx);
    break;

  case VERIFY_TEST_FOC_MATH:
    Clarke(1.0f, -0.5f, -0.5f, &alpha, &beta);
    Park(alpha, beta, 0.0f, &d, &q);
    SVM(alpha, beta, 24.0f, &du, &dv, &dw);
    if (absf(d - alpha) < 0.001f && absf(q - beta) < 0.001f &&
        du >= 0.0f && du <= 1.0f && dv >= 0.0f && dv <= 1.0f && dw >= 0.0f &&
        dw <= 1.0f) {
      printf("[VERIFY][FOC_MATH] PASS: Clarke/Park/SVM numerics valid.\r\n");
    } else {
      printf("[VERIFY][FOC_MATH] FAIL: transform or SVM output out of range.\r\n");
    }
    finish_test(ctx);
    break;

  case VERIFY_TEST_AS5047P:
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
    } else {
      printf("[VERIFY][AS5047P] WARN: encoder handle is null.\r\n");
    }
    finish_test(ctx);
    break;

  case VERIFY_TEST_THROTTLE:
    rpm_map = throttle_to_rpm_limited(throttle_count, max_rpm);
    if (rpm_map >= 0.0f && rpm_map <= max_rpm) {
      printf("[VERIFY][THROTTLE] PASS: count=%ld -> RPM=%.1f (range 0..%.1f)\r\n",
             (long)throttle_count, rpm_map, max_rpm);
    } else {
      printf("[VERIFY][THROTTLE] FAIL: mapped RPM out of range.\r\n");
    }
    finish_test(ctx);
    break;

  case VERIFY_TEST_MOTOR_FUNCTION:
    if (!tim_pwm || !ms) {
      safe_shutdown("null handle", tim_pwm, ms);
      finish_test(ctx);
      break;
    }
    if (ctx->test_phase == 0u) {
      MS_SetMode(ms, MS_MODE_1_12V, MS_MODE2_VAR_A);
      pwm_set_zero(tim_pwm);
      ctx->motor_iq_cmd = 0.0f; /* reused as electrical angle accumulator */
      ctx->test_phase = 1u;
      ctx->test_start_tick = now;
      printf("[VERIFY][MOTOR_FUNCTION] Arm inverter outputs.\r\n");
      break;
    }

    if (ctx->test_phase == 1u) {
      elapsed_s = (float)(now - ctx->test_start_tick) / 1000.0f;
      if ((now - ctx->test_start_tick) > VERIFY_MOTOR_RAMP_UP_MS) {
        ctx->test_phase = 2u;
        ctx->test_start_tick = now;
        printf("[VERIFY][MOTOR_FUNCTION] Ramp-up done.\r\n");
        break;
      }

      freq_hz = VERIFY_MOTOR_FREQ_START_HZ +
                (VERIFY_MOTOR_FREQ_TARGET_HZ - VERIFY_MOTOR_FREQ_START_HZ) *
                    (elapsed_s / ((float)VERIFY_MOTOR_RAMP_UP_MS / 1000.0f));
      if (freq_hz > VERIFY_MOTOR_FREQ_TARGET_HZ) {
        freq_hz = VERIFY_MOTOR_FREQ_TARGET_HZ;
      }

      dt_ms = now - ctx->state_tick;
      dt_s = (dt_ms > 0u) ? ((float)dt_ms / 1000.0f) : 0.001f;
      theta = ctx->motor_iq_cmd + (VERIFY_TWO_PI * freq_hz * dt_s);
      while (theta >= VERIFY_TWO_PI) {
        theta -= VERIFY_TWO_PI;
      }
      ctx->motor_iq_cmd = theta;

      alpha = VERIFY_MOTOR_SVM_VMAG * cosf(theta);
      beta = VERIFY_MOTOR_SVM_VMAG * sinf(theta);
      SVM(alpha, beta, MS_ModeToVbus(ms->active_mode), &du, &dv, &dw);
      arr = __HAL_TIM_GET_AUTORELOAD(tim_pwm);
      __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, (uint32_t)(du * (arr + 1u)));
      __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, (uint32_t)(dv * (arr + 1u)));
      __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_3, (uint32_t)(dw * (arr + 1u)));

      if (actual_rpm < -20.0f) {
        safe_shutdown("direction mismatch", tim_pwm, ms);
        finish_test(ctx);
        break;
      }
      break;
    }

    if (ctx->test_phase == 2u) {
      freq_hz = VERIFY_MOTOR_FREQ_TARGET_HZ;
      dt_ms = now - ctx->state_tick;
      dt_s = (dt_ms > 0u) ? ((float)dt_ms / 1000.0f) : 0.001f;
      theta = ctx->motor_iq_cmd + (VERIFY_TWO_PI * freq_hz * dt_s);
      while (theta >= VERIFY_TWO_PI) {
        theta -= VERIFY_TWO_PI;
      }
      ctx->motor_iq_cmd = theta;
      alpha = VERIFY_MOTOR_SVM_VMAG * cosf(theta);
      beta = VERIFY_MOTOR_SVM_VMAG * sinf(theta);
      SVM(alpha, beta, MS_ModeToVbus(ms->active_mode), &du, &dv, &dw);
      arr = __HAL_TIM_GET_AUTORELOAD(tim_pwm);
      __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, (uint32_t)(du * (arr + 1u)));
      __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, (uint32_t)(dv * (arr + 1u)));
      __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_3, (uint32_t)(dw * (arr + 1u)));

      if (actual_rpm >= VERIFY_MOTOR_RPM_MIN_REACHED) {
        printf("[VERIFY][MOTOR_FUNCTION] PASS: RPM threshold reached.\r\n");
      } else {
        printf("[VERIFY][MOTOR_FUNCTION] WARN: RPM threshold not reached.\r\n");
      }
      if ((now - ctx->test_start_tick) >= VERIFY_MOTOR_HOLD_MS) {
        ctx->test_phase = 3u;
        ctx->test_start_tick = now;
      }
      break;
    }
    if (ctx->test_phase == 3u) {
      elapsed_s = (float)(now - ctx->test_start_tick) / 1000.0f;
      freq_hz = VERIFY_MOTOR_FREQ_TARGET_HZ *
                (1.0f - (elapsed_s / ((float)VERIFY_MOTOR_RAMP_DOWN_MS / 1000.0f)));
      if (freq_hz < 0.0f) {
        freq_hz = 0.0f;
      }

      dt_ms = now - ctx->state_tick;
      dt_s = (dt_ms > 0u) ? ((float)dt_ms / 1000.0f) : 0.001f;
      theta = ctx->motor_iq_cmd + (VERIFY_TWO_PI * freq_hz * dt_s);
      while (theta >= VERIFY_TWO_PI) {
        theta -= VERIFY_TWO_PI;
      }
      ctx->motor_iq_cmd = theta;

      if (freq_hz > 0.1f) {
        alpha = VERIFY_MOTOR_SVM_VMAG * cosf(theta);
        beta = VERIFY_MOTOR_SVM_VMAG * sinf(theta);
        SVM(alpha, beta, MS_ModeToVbus(ms->active_mode), &du, &dv, &dw);
        arr = __HAL_TIM_GET_AUTORELOAD(tim_pwm);
        __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1,
                              (uint32_t)(du * (arr + 1u)));
        __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2,
                              (uint32_t)(dv * (arr + 1u)));
        __HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_3,
                              (uint32_t)(dw * (arr + 1u)));
      }

      if ((now - ctx->test_start_tick) >= VERIFY_MOTOR_RAMP_DOWN_MS) {
        pwm_set_zero(tim_pwm);
        MS_AllOff(ms);
        printf("[VERIFY][MOTOR_FUNCTION] PASS: hold and ramp-down complete.\r\n");
        finish_test(ctx);
      }
      break;
    }
    break;

  default:
    printf("[VERIFY] FAIL: invalid test id=%u\r\n", (unsigned)ctx->current_test);
    finish_test(ctx);
    break;
  }
  ctx->state_tick = now;
}
