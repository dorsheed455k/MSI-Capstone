
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "usb_device.h"
#include <math.h>


#include "as5047p.h"
#include "bluenrg_m0.h"
#include "amc1302.h"

#include "ms_switch.h"
#include "foc.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

AS5047P_Handle_t encoder;
BlueNRG_M0_Handle_t ble;

void SystemClock_Config(void);

static void blink_Testing(void);
static void bluetooth_Test(void);
static void speed_Sense(uint16_t angle, uint16_t mag, uint16_t err, bool ef);
static void display_Current(uint32_t *last);
static void throttle_Display(void);

static void status_Display(void);
static void MS_UpdateSlow(void);
static float throttle_to_rpm(int32_t c);
static void Control_Fast_25kHz(void);

static inline uint8_t read_clk(void) { return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET); }
static inline uint8_t read_dt(void)  { return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET); }

volatile int32_t enc_count = 0;
volatile uint8_t last_clk = 0;
volatile uint8_t button_pressed = 0;


static MS_Switch_t ms;

/* constants */
#define PWM_FREQ_HZ     25000.0f
#define TS              (1.0f / PWM_FREQ_HZ)
#define TWO_PI          6.283185307f

#define PWM_FREQ_HZ 25000.0f
#define DT_FAST (1.0f / PWM_FREQ_HZ)
#define TWO_PI 6.283185307f

/* your motor */
#define POLE_PAIRS      7   // <-- set correctly!

/* global state */
static volatile float theta_e = 0.0f;       // electrical angle (rad)
static volatile float rpm_cmd = 0.0f;       // from throttle (RPM)
static volatile float vbus_est = 0.0f;      // if measured; else set to mode value
static volatile float v_req_mag = 0.0f;     // “required voltage magnitude” for MS logic

static volatile uint8_t ms_desired_mode = MS_MODE_1_9V;
static volatile uint8_t ms_mode3_variant = MS_MODE3_VAR_A;


static inline float wrap_2pi(float a)
{
    while (a >= TWO_PI) a -= TWO_PI;
    while (a <  0.0f)   a += TWO_PI;
    return a;
}

static FOC_t foc;     // whatever your foc.h defines
static MS_Switch_t ms;
static float v_util = 0.0f;
static MS_Mode_t desired_mode = MS_MODE_1_9V;
static MS_Mode3Variant_t mode3var = MS_MODE3_VAR_A;

volatile float g_iu = 0.0f;
volatile float g_iv = 0.0f;
volatile float g_iw = 0.0f;

int main(void)
{
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

	// Inverter MOSFETS Initiation
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	// MS Switching MOSFETS Initiation
	MS_Init(&ms);
	MS_SetMode(&ms, MS_MODE_1_9V, MS_MODE3_VAR_A);

	// FOC Initiation
	FOC_Init(&foc);
	FOC_SetVdc(&foc, MS_ModeToVbus(MS_MODE_1_9V));

	 /* Initialize BLUENRG_M0  */
	BlueNRG_M0_Init(&ble, &hspi1,
	                GPIOA, GPIO_PIN_3,   // CS
	                GPIOC, GPIO_PIN_7,   // IRQ
	                GPIOC, GPIO_PIN_6);  // RESET

	BlueNRG_M0_ResetPulse(&ble, 5, 20);

    uint16_t angle = 0;
    uint16_t mag = 0;
    uint16_t err = 0;
    bool ef = true;

    static uint32_t last = 0;
    last_clk = read_clk();
	extern volatile int32_t enc_count;  // if declared elsewhere

    while (1)
    {
	  blink_Testing();
	  bluetooth_Test();
	  speed_Sense(angle, mag, err, ef);
	  display_Current(&last);
	  throttle_Display();
	  status_Display();

	  static uint32_t last_ms = 0;
	  if (HAL_GetTick() - last_ms >= 20) {
	      last_ms = HAL_GetTick();
	      MS_UpdateSlow();
	  }
	  rpm_cmd = throttle_to_rpm(enc_count);
    }
}

void blink_Testing(void) {
	HAL_GPIO_TogglePin(MCU_LED_GPIO_Port, MCU_LED_Pin);
	HAL_Delay(500);  // 500 ms
}

void bluetooth_Test(void) {
	if (BlueNRG_M0_IrqIsAsserted(&ble)) {
	  /* Example raw SPI exchange (placeholder). Real ACI packets depend on ST ACI SPI framing. */
	  uint8_t tx[4] = {0x00,0x00,0x00,0x00};
	  uint8_t rx[4] = {0};
	  BlueNRG_M0_AciRawExchange(&ble, tx, rx, sizeof(tx));
	 }
}

void speed_Sense(uint16_t angle, uint16_t mag, uint16_t err, bool ef) {
	double degrees = 0.0;
	AS5047P_Status_t status =
	          AS5047P_ReadRegister(&encoder, AS5047P_REG_ANGLECOM, &angle, &ef);

	      if (status == AS5047P_OK)
	      {
	          degrees = AS5047P_AngleToDegrees(angle);
	          printf("Angle=%.2f deg\r\n", degrees);
	      }

	      if (ef)
	      {
	          AS5047P_ReadErrfl(&encoder, &err);
	      }

	      HAL_Delay(1);
}

 void display_Current(uint32_t *last) {

	 if (HAL_GetTick() - *last >= 100)   // 100 ms → 10 Hz
	    {
	        *last = HAL_GetTick();
	        printf("IU=%.3f IV=%.3f IW=%.3f\r\n", g_iu, g_iv, g_iw);
	    }
}

 void throttle_Display(void) {
	 static uint32_t last_print = 0;
	 static int32_t min_cnt = 0, max_cnt = 200;   // pick your range

	 if (button_pressed)
	 {
	     button_pressed = 0;
	     enc_count = 0;           // example: reset throttle on press
	 }

	 if (HAL_GetTick() - last_print >= 100)
	 {
	     last_print = HAL_GetTick();

	     int32_t c = enc_count;

	     if (c < min_cnt) c = min_cnt;
	     if (c > max_cnt) c = max_cnt;

	     float throttle = (float)(c - min_cnt) / (float)(max_cnt - min_cnt); // 0..1
	     printf("ENC=%ld  Throttle=%.3f %%\r\n", (long)enc_count, throttle * 100.0f);
	 }
 }

 static float throttle_to_rpm(int32_t c)
 {
     const int32_t MIN_CNT = 0;
     const int32_t MAX_CNT = 200;
     const float MAX_RPM = 3000.0f; // set safe max

     if (c < MIN_CNT) c = MIN_CNT;
     if (c > MAX_CNT) c = MAX_CNT;

     float t = (float)(c - MIN_CNT) / (float)(MAX_CNT - MIN_CNT);
     return t * MAX_RPM;
 }

 void status_Display(void) {
	 static uint32_t last_print = 0;
	 if (HAL_GetTick() - last_print >= 100) {
	     last_print = HAL_GetTick();
	     printf("RPMcmd=%.0f IU=%.2f IV=%.2f IW=%.2f Mode=%d Vbus=%.0f\r\n",
	            rpm_cmd, g_iu, g_iv, g_iw, ms.active_mode, vbus_est);
	 }
 }

 static inline void InverterPWM_SetDuty(float du, float dv, float dw)
 {
	 if (du < 0.0f) du = 0.0f;
	 if (du > 1.0f) du = 1.0f;

	 if (dv < 0.0f) dv = 0.0f;
	 if (dv > 1.0f) dv = 1.0f;

	 if (dw < 0.0f) dw = 0.0f;
	 if (dw > 1.0f) dw = 1.0f;

     uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(du * (arr + 1u)));
     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dv * (arr + 1u)));
     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dw * (arr + 1u)));
 }

 static void MS_UpdateSlow(void)
 {
	 static uint32_t last_change = 0;
	 const uint32_t dwell_ms = 200;     // don’t change too often
	 const float up_th   = 0.92f;       // tune
	 const float down_th = 0.65f;       // tune

	    uint32_t now = HAL_GetTick();
	    if (now - last_change < dwell_ms) return;

	    /* choose desired_mode based on v_util */
	    if (v_util > up_th)
	    {
	        if (desired_mode < MS_MODE_4_33V) desired_mode++;
	    }
	    else if (v_util < down_th)
	    {
	        if (desired_mode > MS_MODE_1_9V) desired_mode--;
	    }

	    /* Apply only if safe: low current & not in ISR */
	    float iabs = fabsf(g_iu) + fabsf(g_iv) + fabsf(g_iw);
	    if (iabs < 2.0f)  // tune threshold
	    {
	        if (ms.active_mode != desired_mode)
	        {
	            MS_SetMode(&ms, desired_mode, mode3var);

	            /* update FOC bus voltage so SVM scaling is correct */
	            FOC_SetVdc(&foc, MS_ModeToVbus(desired_mode));

	            last_change = now;
	        }
	    }
 }

 void Control_Fast_25kHz(void)
 {
     /* 1) currents from your ADC processing (amps) */
     float iu = g_iu;
     float iv = g_iv;
     float iw = g_iw;

     /* 2) throttle sets desired RPM -> electrical frequency */
     float fe = (rpm_cmd / 60.0f) * (float)POLE_PAIRS;   // Hz
     theta_e = wrap_2pi(theta_e + TWO_PI * fe * DT_FAST);

     /* 3) set torque-producing current reference from throttle (example)
        you can do speed PI here later; for now simple mapping */
     foc.id_ref = 0.0f;
     foc.iq_ref = rpm_cmd * 0.001f;   // simple scaling for now

     /* 4) run FOC (updates foc.d_u/d_v/d_w) */
     FOC_Run(&foc, iu, iv, iw, theta_e, DT_FAST);

     /* 5) estimate modulation utilization for MS decision (slow loop uses this) */
     float dmax = fmaxf(foc.d_u, fmaxf(foc.d_v, foc.d_w));
     float dmin = fminf(foc.d_u, fminf(foc.d_v, foc.d_w));
     v_util = dmax - dmin;   // higher means closer to bus limit

     /* 6) apply duties to TIM1 */
     InverterPWM_SetDuty(foc.d_u, foc.d_v, foc.d_w);
 }


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 21;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
    	Control_Fast_25kHz();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_3)  // CLK on PA3
    {
        uint8_t clk = read_clk();
        uint8_t dt  = read_dt();

        /* only act on one edge to reduce bounce (rising edge example) */
        if (clk == 1 && last_clk == 0)
        {
            /* Quadrature direction:
               if DT != CLK at rising edge => one direction, else other */
            if (dt != clk) enc_count++;
            else           enc_count--;
        }
        last_clk = clk;
    }
    else if (GPIO_Pin == GPIO_PIN_13) // SW on PC13
    {
        button_pressed = 1;  // handle in main loop
    }
}
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
