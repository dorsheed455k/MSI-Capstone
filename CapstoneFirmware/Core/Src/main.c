/**
 * main.c — STM32 Multi-Source Inverter (MSI) controller
 *
 * This MCU coordinates the whole MSI system:
 * - Reads throttle (rotary encoder), current (AMC1302/ADC), and rotor angle (AS5047P).
 * - Runs FOC (Field-Oriented Control) at 25 kHz to drive the inverter MOSFETs (TIM1 PWM).
 * - Selects one of 3 MS modes (12V/24V/36V) via ms_switch (Q1/Q2/Q3) and updates FOC bus voltage.
 * - Sends telemetry over BLE and prints status over USB CDC.
 *
 * High-level flow: main loop does slow tasks (LED, encoder read, prints, MS mode logic);
 * ADC DMA completion runs the fast FOC loop (current → Park/Clarke → PI → SVM → PWM).
 */

#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "gpio.h"
#include "hci.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include <math.h>

#include "amc1302.h"
#include "as5047p.h"
#include "ble_telemetry.h"
#include "bluenrg_m0.h"
#include "controller_verification.h"
#include "foc.h"
#include "ms_switch.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

AS5047P_Handle_t encoder;
BlueNRG_M0_Handle_t ble;
static CurrentSense_t cs;

void SystemClock_Config(void);

// Function initialization
static void blink_Testing(void);
static void speed_Sense(uint16_t angle, uint16_t mag, uint16_t err, bool ef);
static void display_Current(uint32_t *last);
static void throttle_Display(void);
static void status_Display(void);
static void MS_UpdateSlow(void);
static float throttle_to_rpm(int32_t c);
static void ble_Telemetry(uint32_t *last_ble);
void hci_user_evt_proc(void); // only if needed
static void Control_Fast_25kHz(void);
static void CRS_USB_ClockSync_Init(void);
static void speed_Control_Update(void);

static inline uint8_t read_clk(void) {
  return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET);
}
static inline uint8_t read_dt(void) {
  return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET);
}

/* Rotary Encoder */
volatile int32_t enc_count = 0;
volatile uint8_t last_clk = 0;
volatile uint8_t button_pressed = 0;

static MS_Switch_t ms;

/* constants */
#define PWM_FREQ_HZ 25000.0f
#define TS (1.0f / PWM_FREQ_HZ)
#define TWO_PI 6.283185307f
#define DT_FAST (1.0f / PWM_FREQ_HZ)
#define THROTTLE_MIN_COUNT 0
#define THROTTLE_MAX_COUNT 200
#define THROTTLE_MAX_RPM 3000.0f

/* Set to 1 to run verification tests separately from runtime control loop. */
#define MSI_RUN_CONTROLLER_VERIFICATION 1

/* your motor */
#define POLE_PAIRS 4 // <-- set correctly!

/* global state */
static volatile float theta_e = 0.0f;  // electrical angle (rad)
static volatile float rpm_cmd = 0.0f;  // from throttle (RPM)
static volatile float vbus_est = 0.0f; // if measured; else set to mode value
static volatile float v_req_mag =
    0.0f; // “required voltage magnitude” for MS logic

static volatile uint8_t ms_desired_mode = MS_MODE_1_12V;
static volatile uint8_t ms_mode2_variant = MS_MODE2_VAR_A;

static volatile float actual_rpm = 0.0f;
static uint16_t last_angle14 = 0;
static uint8_t encoder_initialized = 0;
static float rpm_error = 0.0f;

static inline float wrap_2pi(float a) {
  while (a >= TWO_PI)
    a -= TWO_PI;
  while (a < 0.0f)
    a += TWO_PI;
  return a;
}

static FOC_t foc; // whatever your foc.h defines
static float v_util = 0.0f;
static MS_Mode_t desired_mode = MS_MODE_1_12V;
static MS_Mode2Variant_t mode2var = MS_MODE2_VAR_A;
static ControllerVerification_t verification;

volatile float g_iu = 0.0f;
volatile float g_iv = 0.0f;
volatile float g_iw = 0.0f;

uint16_t telemetry_service_handle;
uint16_t telemetry_char_handle;

typedef struct {
  float Kp;
  float Ki;
  float integral;
  float limit;
} SpeedPI_t;

static SpeedPI_t speed_pi = {
    .Kp = 0.268466f,
    .Ki = 33.7365f,
    .integral = 0.0f,
    .limit = 8.0f // max iq_ref in amps (adjust to your motor/current limits)
};

int main(void) {
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_DAC1_Init();
  MX_SPI3_Init();
  MX_USB_Device_Init();
  CRS_USB_ClockSync_Init();

  // Inverter MOSFETS Initiation
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // MS Switching MOSFETS Initiation
  MS_Init(&ms);
  MS_SetMode(&ms, MS_MODE_1_12V, MS_MODE2_VAR_A);

  // FOC Initiation
  FOC_Init(&foc);
  FOC_SetVdc(&foc, MS_ModeToVbus(MS_MODE_1_12V));

  /* Initialize BLUENRG_M0  */
  BlueNRG_M0_Init(&ble, &hspi1, GPIOA, GPIO_PIN_3, // CS
                  GPIOC, GPIO_PIN_7,               // IRQ
                  GPIOC, GPIO_PIN_6);              // RESET

  BlueNRG_M0_ResetPulse(&ble, 5, 20);

  BLE_Stack_Init();
  BLE_Telemetry_Init();
  BLE_StartAdvertising();

  /* Current Sense Initialization */
  CurrentSense_SetParams(3.3f, 12, 41.0f,
                         0.002f); // Vref, ADC bits, AMC gain, shunt
  CurrentSense_Init(&cs, &hadc1);
  CurrentSense_Start(&cs);

  /* IMPORTANT: run this with motor OFF / no current */
  CurrentSense_CalibrateOffsets(&cs, 200); // ~200 ms in your current code

  /* Initializing AS5047P */
  AS5047P_Init(&encoder, &hspi3, GPIOA, GPIO_PIN_15); // CS = PA15
  encoder_initialized = 1;

#if MSI_RUN_CONTROLLER_VERIFICATION
  ControllerVerification_Init(&verification);
#endif

  uint16_t angle = 0;
  uint16_t mag = 0;
  uint16_t err = 0;
  bool ef = true;

  static uint32_t last = 0;
  last_clk = read_clk();

  static uint32_t last_ms = 0;
  static uint32_t last_ble = 0;

  while (1) {
#if MSI_RUN_CONTROLLER_VERIFICATION
    ControllerVerification_RunStep(&verification, &htim1, &ms, &foc, &encoder,
                                   enc_count, THROTTLE_MAX_RPM);
    blink_Testing();
    hci_user_evt_proc();
    ble_Telemetry(&last_ble);
    HAL_Delay(100); // Prevent USB saturation
    continue;
#endif

    blink_Testing();
    speed_Sense(angle, mag, err, ef);
    display_Current(&last);
    throttle_Display();
    status_Display();

    if (HAL_GetTick() - last_ms >= 20) {
      last_ms = HAL_GetTick();
      MS_UpdateSlow();
    }

    hci_user_evt_proc();
    ble_Telemetry(&last_ble);

    rpm_cmd = throttle_to_rpm(enc_count);

    speed_Control_Update();
  }
}

void blink_Testing(void) {
  static uint32_t last_blink = 0;
  if (HAL_GetTick() - last_blink >= 500) {
    last_blink = HAL_GetTick();
    HAL_GPIO_TogglePin(MCU_LED_GPIO_Port, MCU_LED_Pin);
  }
}

void speed_Sense(uint16_t angle, uint16_t mag, uint16_t err, bool ef) {
  if (!encoder_initialized)
    return;

  static uint32_t last_tick = 0;
  uint32_t now = HAL_GetTick();

  if (now - last_tick < 10)
    return; // 100 Hz sensing

  float dt = (now - last_tick) / 1000.0f;
  last_tick = now;

  AS5047P_Status_t status =
      AS5047P_ReadRegister(&encoder, AS5047P_REG_ANGLECOM, &angle, &ef);

  if (status == AS5047P_OK) {
    actual_rpm = AS5047P_EstimateRPM(last_angle14, angle, dt);
    last_angle14 = angle;
  }

  if (ef) {
    AS5047P_ReadErrfl(&encoder, &err);
  }
}

void display_Current(uint32_t *last) {

  if (HAL_GetTick() - *last >= 100) // 100 ms → 10 Hz
  {
    *last = HAL_GetTick();
    printf("IU=%.3f A  IV=%.3f A  IW=%.3f A\r\n", g_iu, g_iv, g_iw);
  }
}

void ble_Telemetry(uint32_t *last_ble) {
  if (HAL_GetTick() - *last_ble >= 100) {
    *last_ble = HAL_GetTick();
    TelemetryPacket_t pkt;
    pkt.iu = g_iu;
    pkt.iv = g_iv;
    pkt.iw = g_iw;
    pkt.speed_rpm = rpm_cmd;
    pkt.mode = (uint8_t)ms.active_mode;

    BLE_SendTelemetry(&pkt);
  }
}

void throttle_Display(void) {
  static uint32_t last_print = 0;
  static int32_t min_cnt = THROTTLE_MIN_COUNT, max_cnt = THROTTLE_MAX_COUNT;

  if (button_pressed) {
    button_pressed = 0;
    enc_count = 0; // example: reset throttle on press
  }

  if (HAL_GetTick() - last_print >= 100) {
    last_print = HAL_GetTick();

    int32_t c = enc_count;

    if (c < min_cnt)
      c = min_cnt;
    if (c > max_cnt)
      c = max_cnt;

    float throttle = (float)(c - min_cnt) / (float)(max_cnt - min_cnt); // 0..1
    printf("ENC=%ld  Throttle=%.3f %%\r\n", (long)enc_count, throttle * 100.0f);
  }
}

static float throttle_to_rpm(int32_t c) {
  const int32_t MIN_CNT = THROTTLE_MIN_COUNT;
  const int32_t MAX_CNT = THROTTLE_MAX_COUNT;
  const float MAX_RPM = THROTTLE_MAX_RPM;

  if (c < MIN_CNT)
    c = MIN_CNT;
  if (c > MAX_CNT)
    c = MAX_CNT;

  float t = (float)(c - MIN_CNT) / (float)(MAX_CNT - MIN_CNT);
  return t * MAX_RPM;
}

void status_Display(void) {
  static uint32_t last_print = 0;

  if (HAL_GetTick() - last_print >= 100) {
    last_print = HAL_GetTick();

    rpm_error = rpm_cmd - actual_rpm;

    printf("RPM_cmd=%.1f  RPM_actual=%.1f  RPM_err=%.1f  IU=%.2f IV=%.2f "
           "IW=%.2f  Mode=%d\r\n",
           rpm_cmd, actual_rpm, rpm_error, g_iu, g_iv, g_iw, ms.active_mode);
  }
}

static float clampf(float x, float lo, float hi) {
  if (x < lo)
    return lo;
  if (x > hi)
    return hi;
  return x;
}

static float Speed_PI_Update(SpeedPI_t *pi, float error, float dt) {
  pi->integral += error * pi->Ki * dt;
  pi->integral = clampf(pi->integral, -pi->limit, pi->limit);

  float out = (pi->Kp * error) + pi->integral;
  out = clampf(out, -pi->limit, pi->limit);

  return out;
}

static void speed_Control_Update(void) {
  static uint32_t last_speed_tick = 0;
  uint32_t now = HAL_GetTick();

  if ((now - last_speed_tick) < 10)
    return; // 100 Hz speed loop

  float dt = (now - last_speed_tick) / 1000.0f;
  last_speed_tick = now;

  float rpm_error = rpm_cmd - actual_rpm;

  foc.id_ref = 0.0f;
  foc.iq_ref = Speed_PI_Update(&speed_pi, rpm_error, dt);
}

static inline void InverterPWM_SetDuty(float du, float dv, float dw) {
  if (du < 0.0f)
    du = 0.0f;
  if (du > 1.0f)
    du = 1.0f;

  if (dv < 0.0f)
    dv = 0.0f;
  if (dv > 1.0f)
    dv = 1.0f;

  if (dw < 0.0f)
    dw = 0.0f;
  if (dw > 1.0f)
    dw = 1.0f;

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(du * (arr + 1u)));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dv * (arr + 1u)));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dw * (arr + 1u)));
}

static void MS_UpdateSlow(void) {
  static uint32_t last_change = 0;
  const uint32_t dwell_ms = 200; // don’t change too often
  const float up_th = 0.92f;     // tune
  const float down_th = 0.65f;   // tune

  uint32_t now = HAL_GetTick();
  if (now - last_change < dwell_ms)
    return;

  /* choose desired_mode based on v_util (3 modes only) */
  if (v_util > up_th) {
    if (desired_mode < MS_MODE_3_36V)
      desired_mode++;
  } else if (v_util < down_th) {
    if (desired_mode > MS_MODE_1_12V)
      desired_mode--;
  }

  /* Apply only if safe: low current & not in ISR */
  float iabs = fabsf(g_iu) + fabsf(g_iv) + fabsf(g_iw);
  if (iabs < 2.0f) // tune threshold
  {
    if (ms.active_mode != desired_mode) {
      MS_SetMode(&ms, desired_mode, mode2var);

      FOC_SetVdc(&foc, MS_ModeToVbus(desired_mode));
      FOC_ApplyMSModePI(&foc, (uint8_t)desired_mode);

      last_change = now;
    }
  }
}

void Control_Fast_25kHz(void) {
  /* 1) currents from your ADC processing (amps) */
  float iu = g_iu;
  float iv = g_iv;
  float iw = g_iw;

  /* 2) speed control loops maintain desired RPM -> electrical frequency */
  float fe = (actual_rpm / 60.0f) * (float)POLE_PAIRS; // Hz
  theta_e = wrap_2pi(theta_e + TWO_PI * fe * DT_FAST);

  /* 3) ensure torque-producing current reference comes from the PI loop
     The 100Hz speed_Control_Update() handles setting foc.iq_ref. We simply zero
     id_ref */
  foc.id_ref = 0.0f;

  /* 4) run FOC (updates foc.d_u/d_v/d_w) once, properly updating PI states */
  FOC_Run(&foc, iu, iv, iw, theta_e, DT_FAST);

  /* 5) estimate modulation utilization for MS decision (slow loop uses this) */
  float dmax = fmaxf(foc.d_u, fmaxf(foc.d_v, foc.d_w));
  float dmin = fminf(foc.d_u, fminf(foc.d_v, foc.d_w));
  v_util = dmax - dmin; // higher means closer to bus limit

  /* 6) apply duties to TIM1 */
  InverterPWM_SetDuty(foc.d_u, foc.d_v, foc.d_w);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Voltage scaling: keep SCALE1 for normal high-speed operation */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* HSE (8 MHz crystal) + HSI48 (USB clock) */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON; // crystal on PF0/PF1
  RCC_OscInitStruct.HSI48State =
      RCC_HSI48_ON; // required for USB if using HSI48

  /* PLL from HSE -> SYSCLK 84 MHz */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;

  /* PLLP/PLLQ not needed unless you route them to peripherals; keep safe
   * defaults */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* Bus clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // HCLK = 84 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  /* Flash latency: 84 MHz typically needs 2 wait states on G4 (68< HCLK <=102)
   */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void CRS_USB_ClockSync_Init(void) {
  RCC_CRSInitTypeDef CRSInit = {0};

  __HAL_RCC_CRS_CLK_ENABLE();

  CRSInit.Prescaler = RCC_CRS_SYNC_DIV1;
  CRSInit.Source = RCC_CRS_SYNC_SOURCE_USB; // USB SOF
  CRSInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  CRSInit.ReloadValue =
      __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000); // 1kHz SOF
  CRSInit.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  CRSInit.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;

  HAL_RCCEx_CRSConfig(&CRSInit);
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    CurrentSense_OnDmaComplete(&cs);
    Control_Fast_25kHz();

    g_iu = cs.iu;
    g_iv = cs.iv;
    g_iw = cs.iw;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_3) // CLK on PA3
  {
    uint8_t clk = read_clk();
    uint8_t dt = read_dt();

    /* only act on one edge to reduce bounce (rising edge example) */
    if (clk == 1 && last_clk == 0) {
      /* Quadrature direction:
         if DT != CLK at rising edge => one direction, else other */
      if (dt != clk)
        enc_count++;
      else
        enc_count--;
    }
    last_clk = clk;
  } else if (GPIO_Pin == GPIO_PIN_13) // SW on PC13
  {
    button_pressed = 1; // handle in main loop
  }
}
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
