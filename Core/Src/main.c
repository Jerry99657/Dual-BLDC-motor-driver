/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "JerryFOC.h"
#include "MT6701.h"
#include "angle_src_encoder.h"
#include "motor_params.h"
#include "motor_sampling.h"
#include "motor_control.h"
#include "st_hso.h"
#include "ws2812.h"
#include "protocol.h"
#include "app_storage.h"
#include "kk_oled.h"
#include "kk_oled_driver.h"
#include "kk_ui.h"
#include "ui_app.h"
#include "ui_input.h"
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t rx_byte;
static volatile uint8_t g_need_align = 0U;
static uint8_t ui_ok = 0U;
static uint8_t g_aligning = 0U;
static uint32_t g_align_start = 0U;
volatile uint32_t g_boot_stage;
volatile uint32_t g_error_return_address;

#define ALIGN_TIME_MS 3000U
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Protocol_Feed(PROTO_SRC_USART, &rx_byte, 1U);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    }
}

/* 单颗 WS2812 只显示状态：
 *   红   = 编码器或 ADC/母线采样异常
 *   黄   = 编码器正常但尚未完成零点标定
 *   绿   = 已标定、待机
 *   蓝   = 正在运行（有速度）
 */
static void WS2812_UpdateStatus(void)
{
    static uint32_t last_color = 0xFFFFFFFFU;
    uint32_t color;

    MotorAxisSnapshot m1,m2;
    MotorControl_GetAxis(MOTOR_AXIS_1,&m1);
    MotorControl_GetAxis(MOTOR_AXIS_2,&m2);
    bool a1=MotorControl_AxisEnabled(MOTOR_AXIS_1), a2=MotorControl_AxisEnabled(MOTOR_AXIS_2);
    if ((a1 && (!m1.ready || !m1.healthy)) || (a2 && (!m2.ready || !m2.healthy))) {
        color = (30U << 16);                                  /* 红 */
    } else if ((a1 && !MotorControl_IsPassive(MOTOR_AXIS_1) && !m1.aligned) ||
               (a2 && !MotorControl_IsPassive(MOTOR_AXIS_2) && !m2.aligned)) {
        color = (30U << 16) | (15U << 8);                     /* 黄 */
    } else if ((a1 && fabsf(m1.velocity)>1.0f) || (a2 && fabsf(m2.velocity)>1.0f)) {
        color = 30U;                                          /* 蓝 */
    } else {
        color = (30U << 8);                                   /* 绿 */
    }

    if (!AppStorage_LedEnabled()) {
        color = 0U;
    } else {
        uint8_t bright = AppStorage_LedBrightness();
        if (bright < 100U) {
            uint32_t r = (((color >> 16) & 0xFFU) * bright) / 100U;
            uint32_t g = (((color >> 8) & 0xFFU) * bright) / 100U;
            uint32_t b = ((color & 0xFFU) * bright) / 100U;
            color = (r << 16) | (g << 8) | b;
        }
    }

    if (color != last_color) {
        last_color = color;
        WS2812_SetRgb(0U,
                      (uint8_t)((color >> 16) & 0xFFU),
                      (uint8_t)((color >> 8) & 0xFFU),
                      (uint8_t)(color & 0xFFU));
        WS2812_Show();
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
   g_boot_stage = 1U;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset peripherals, initialize the Flash interface and SysTick. */
  if (HAL_Init() != HAL_OK) Error_Handler();

  /* USER CODE BEGIN Init */
  g_boot_stage = 2U;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* HSI48 用 USB SOF 做 CRS 校准，保证 USB CDC 的 48 MHz 精度 */
  {
    RCC_CRSInitTypeDef pInit = {0};

    __HAL_RCC_CRS_CLK_ENABLE();
    pInit.Prescaler = RCC_CRS_SYNC_DIV1;
    pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
    pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
    pInit.ErrorLimitValue = 34;
    pInit.HSI48CalibrationValue = 32;
    HAL_RCCEx_CRSConfig(&pInit);
  }
  g_boot_stage = 3U;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_USB_Device_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  g_boot_stage = 4U;
  /* 0. SPI2/SPI3 使用软件 NSS，清除 CubeMX 默认生成的 NSSP 位 */
  CLEAR_BIT(SPI2->CR2, SPI_CR2_NSSP);
  CLEAR_BIT(SPI3->CR2, SPI_CR2_NSSP);

  /* 1. 开启 DWT 计数器，提供微秒级延时 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* 2. MT6701 定频采样与串口接收 */
  MT6701_StartContinuous();
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);

  /* Load the selected motor before ADC protection, alignment or FOC starts.
   * This also works when LCD initialization fails. */
  (void)AppStorage_Init();
  MotorControl_Init();

  /* 4. WS2812 状态灯 */
  WS2812_Init();

  /* 5. W25Q64 设置/日志与 KK_UI */
  ui_input_init();
  if (AppStorage_IsReady()) {
      AppStorage_Log(APP_LOG_BOOT, AppStorage_ReadID());
  }

  if (OLED_Init() == OLED_OK) {
      /* 面板原生 80x160，旋转为 160x80 横屏 */
      OLED_SetRotation(OLED_ROTATION_270);
      ui_app_reset(HAL_GetTick());
      if (KK_UI_Init(ui_app_get()) == KK_UI_OK) {
          ui_ok = 1U;
      }
  }

  /* 8. 启动音效（必须在 ADC 注入组启动前播放） */
//   JerryFOC_playStartupSound();

  /* 8. 上电请求一次编码器零点标定 */
  g_need_align = 1U;

  /* Acquisition runs even with no motor supply. Service enables M1 only
   * after zero calibration and stable VBUS. M2 power remains disabled. */
  g_boot_stage = 5U;
  if (!MotorSampling_Start()) {
      /* A hardware/HAL startup failure keeps power disabled; 0 V is NOT a
       * startup failure and continues to be measured until VBUS is connected. */
      g_need_align = 0U;
      if (ui_ok) (void)KK_UI_ShowToast(MotorSampling_GetStatusText(), 1800U);
  }

  /* 10. 1 ms 速度/位置外环 */
  g_boot_stage = 6U;
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
      MotorSampling_Stop();
      Error_Handler();
  }
  g_boot_stage = 7U;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    MotorSampling_Service();
    Protocol_Service();
    OLED_DriverService();
    AppStorage_Service(now);
    MT6701_2_Service(HAL_GetTick());
    MotorControl_Service(HAL_GetTick());
    now = HAL_GetTick();
    uint8_t ui_keys = ui_input_poll();

    if (ui_ok) {
        ui_app_update(now);
        /* 存储服务可能耗时，动画与 DMA 超时判定必须使用新时间。 */
        now = HAL_GetTick();
        KK_UI_Status ui_status = KK_UI_Update(
            now, (KK_UI_Input){.keys = ui_keys});
        if (ui_status == KK_UI_DISPLAY_ERROR ||
            ui_status == KK_UI_DISPLAY_TIMEOUT) {
            ui_app_note_display_error();
            (void)KK_UI_RecoverDisplay();
        }
        ui_app_process_events();
    }

    if (MotorControl_MotionAllowed(MOTOR_AXIS_1) &&
        (g_need_align || JerryFOC_needsAlignment()) && !g_aligning &&
        MotorSampling_IsReady() && MotorSampling_IsCalibrated() && MT6701_IsHealthy()) {
        g_need_align = 0U;
        JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
        JerryFOC_setCurrent(0.0f);

        JerryFOC_alignStart();
        g_aligning = 1U;
        g_align_start = now;

        if (ui_ok) {
            (void)KK_UI_ShowToast("Aligning...", ALIGN_TIME_MS);
            (void)KK_UI_Update(HAL_GetTick(), (KK_UI_Input){0U});
        }
    }

    /* 标定不阻塞主循环，期间按键和界面仍然可用 */
    if (g_aligning && (!JerryFOC_isAligning() || !MotorSampling_IsReady() ||
        (now - g_align_start) >= ALIGN_TIME_MS)) {
        g_aligning = 0U;
        JerryFOC_alignFinish();
        AppStorage_Log(APP_LOG_ALIGN_DONE,
                       AngleSrc_Encoder_IsAligned() ? 1U : 0U);

        if (ui_ok) {
            KK_UI_Invalidate();
        }
    }

    Telemetry_Service();
    WS2812_UpdateStatus();
    WS2812_Service();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  g_error_return_address = (uint32_t)(uintptr_t)__builtin_return_address(0);
  /* Also safe when initialization failed before timer clocks were enabled. */
  if (htim1.Instance != NULL) __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  if (htim8.Instance != NULL) __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
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
