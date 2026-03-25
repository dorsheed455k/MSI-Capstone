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
#include "hci_tl.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include <math.h>
#include <stdio.h>

#include "amc1302.h"
#include "as5047p.h"
#include "ble_telemetry.h"
#include "bluenrg_m0.h"
#include "controller_verification.h"
#include "foc.h"
#include "ms_switch.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void CRS_USB_ClockSync_Init(void);

/* USER CODE BEGIN PFP */
extern void BLE_Transport_Register(void);
extern void hci_user_evt_proc(void);
extern void hci_init(void(* UserEvtRx)(void* pData), void* pConf);
extern int32_t hci_notify_asynch_evt(void* pdata);
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* APP VARIABLES */
BlueNRG_M0_Handle_t ble;
static TelemetryPacket_t telemetry = {0};
static uint32_t last_telemetry_tick = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  
  /* HEARTBEAT TEST: Blink 3 times quickly to show the brain is awake */
  for (int i = 0; i < 6; i++) {
    HAL_GPIO_TogglePin(MCU_LED_GPIO_Port, MCU_LED_Pin);
    HAL_Delay(100);
  }

  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  CRS_USB_ClockSync_Init();
  
  /* Give the system a moment to stabilize clock and power before USB pull-up */
  HAL_Delay(100);
  
  MX_USB_Device_Init();

  /* USER CODE BEGIN 2 */
  /* STARTUP DELAY */
  HAL_Delay(1000);

  /* BLE INITIALIZATION (Corrected to SPI1 per Report 3 / Schematic) */
  BlueNRG_M0_Init(&ble, &hspi1,
                  GPIOA, GPIO_PIN_4, // CS (NSS) is PA4
                  BT_IRQ_GPIO_Port, BT_IRQ_Pin,
                  BT_RESET_GPIO_Port, BT_RESET_Pin);

  /* Reset the module */
  BlueNRG_M0_ResetPulse(&ble, 10, 100);

  /* BLE INITIALIZATION */
  BLE_Transport_Register();
  hci_init(NULL, NULL);
  
  BLE_Stack_Init();
  BLE_Telemetry_Init();
  BLE_StartAdvertising();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t last_blink_tick = 0;
    uint8_t usb_state = hUsbDeviceFS.dev_state;
    uint32_t blink_delay = 500;

    /*
       USBD_STATE_DEFAULT = 1
       USBD_STATE_ADDRESSED = 2
       USBD_STATE_CONFIGURED = 3
       USBD_STATE_SUSPENDED = 4
    */
    if (usb_state == 3) { // CONFIGURED
      blink_delay = 1000;
    } else if (usb_state == 4) { // SUSPENDED
      blink_delay = 100;
    } else { // DEFAULT / NOT DETECTED
      blink_delay = 250;
    }

    if (HAL_GetTick() - last_blink_tick > blink_delay) {
      last_blink_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(MCU_LED_GPIO_Port, MCU_LED_Pin);
    }

    /* BLE PROCESS (Call as often as possible) */
    hci_notify_asynch_evt(NULL);
    hci_user_evt_proc();

    /* PERIODIC TELEMETRY (10Hz) */
    if (HAL_GetTick() - last_telemetry_tick > 100) {
      last_telemetry_tick = HAL_GetTick();

      /* Create dummy wave data for testing */
      static float phase = 0;
      phase += 0.2f;
      telemetry.iu = sinf(phase) * 2.0f;
      telemetry.iv = sinf(phase + 2.094f) * 2.0f; // 120 deg
      telemetry.iw = sinf(phase + 4.188f) * 2.0f; // 240 deg
      telemetry.speed_rpm = 1200.0f + (sinf(phase * 0.1f) * 200.0f);
      telemetry.mode = 1; // 12V Mode

      BLE_SendTelemetry(&telemetry);

      /* Serial (USB) Telemetry */
      printf("IU:%.2f, IV:%.2f, IW:%.2f, RPM:%.1f, MODE:%d\r\n", 
             telemetry.iu, telemetry.iv, telemetry.iw, telemetry.speed_rpm, telemetry.mode);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
#if 0
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
#endif

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif /* USE_FULL_ASSERT */

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void CRS_USB_ClockSync_Init(void) {
  RCC_CRSInitTypeDef CRSInit = {0};
  __HAL_RCC_CRS_CLK_ENABLE();
  CRSInit.Prescaler = RCC_CRS_SYNC_DIV1;
  CRSInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  CRSInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  CRSInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  CRSInit.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  CRSInit.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
  HAL_RCCEx_CRSConfig(&CRSInit);
}
