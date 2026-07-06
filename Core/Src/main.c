/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @developer      : Amiya Krishna Gupta
  * @start_date     : 11 August 2025
  * @updated        : Consolidated fix — relay + LCD both operational,
  *                   2-state reverse toggle + DWIN RTC integration.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_hmi.h"
#include "button_matrix.h"
#include "action_comm.h"
#include "sensor_manager.h"
#include "rtc_manager.h"
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
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* ------------------------------------------------------------------
 * Debug snapshot for STM32CubeIDE Live Expressions.
 * Add "g_ctest" to the Live Expressions window to watch the RTC
 * advance second-by-second and see the last reverse-icon value
 * written to the DWIN.
 * ------------------------------------------------------------------ */
typedef struct
{
    RTC_Time_t rtc;          /* live software clock snapshot          */
    uint8_t    reverse_icon; /* last IA 0x6A value pushed (0/1)       */
} CTest_t;

volatile CTest_t g_ctest = {0};

/* Uncomment (or add REVERSE_RTC_SELFTEST to project symbols) to run
 * the reverse + RTC self-test once at boot. Remove for production.   */
//#define REVERSE_RTC_SELFTEST

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* ----------------------------------------------------------------
   * FIX: Explicitly set relay OFF at boot before any module init.
   * This guarantees the relay pin is in a known state even if
   * MX_GPIO_Init() default-LOW was not sufficient (e.g. glitch).
   * ---------------------------------------------------------------- */
  RELAY_SET(0);

  /* ----------------------------------------------------------------
   * Relay self-test (optional — remove after hardware verification)
   * ON 2s → OFF 2s confirms GPIO + relay driver circuit are working.
   * ---------------------------------------------------------------- */
#ifdef RELAY_SELF_TEST
  RELAY_SET(1);  HAL_Delay(2000);
  RELAY_SET(0);  HAL_Delay(2000);
#endif

  /* ----------------------------------------------------------------
   * Application init — order matters:
   *   1. LCD first  (no interrupts, just UART TX to DWIN)
   *   2. Sensor init (no interrupts)
   *   3. ACTION_COMM last — starts HAL_UART_Receive_IT
   *
   * FIX: UART handle assignments —
   *   huart2 → DWIN LCD   (USART2 = PA2 TX / PA3 RX)
   *   huart3 → BLE module (USART3 = PB10 TX / PB11 RX)
   *   huart1 → Debug/aux  (USART1 = PA9 TX / PA10 RX)
   *
   * If your hardware has different wiring, swap handles here —
   * DO NOT change the UART init functions.
   * ---------------------------------------------------------------- */
  LCD_HMI_Init(&huart2);
  Sensor_Init();
  ACTION_COMM_Init(&huart3, &huart1);   /* starts HAL_UART_Receive_IT */

  /* Drive all column outputs LOW before matrix scan starts */
  HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(C5_GPIO_Port, C5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(C6_GPIO_Port, C6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, GPIO_PIN_RESET);

  /* ----------------------------------------------------------------
   * FIX: Turn relay ON before startup sequence so the DWIN display
   * has power during the logo animation. Previous code only called
   * RELAY_SET inside LCD_TogglePower() which is a toggle — at boot
   * lcdPowerOn is already 1 so the relay was never explicitly set.
   * ---------------------------------------------------------------- */
  RELAY_SET(1);

  /* Small delay to let DWIN boot after power rail stabilises */
  HAL_Delay(100);

  LCD_ShowStartupSequence();

  /* ----------------------------------------------------------------
   * RTC bring-up. DWIN system VP 0x0010 is the RTC register, driven
   * over the same DWIN UART (huart2). No hardware RTC chip is fitted,
   * so the time is held in software and re-written every second by
   * RTC_Tick() in the main loop.
   *
   * RTC_SetRandom() seeds a random valid start time so the clock is
   * visibly advancing on the display right away. Swap for
   * RTC_SetDateTime(yy,mm,dd,hh,mm,ss) once a real time source exists.
   *
   * NOTE (DWIN side): the .HMI project must have an RTC display control
   * bound to VP 0x0010 on the page(s) that should show the clock.
   * ---------------------------------------------------------------- */
  RTC_Init(&huart2);
  RTC_SetRandom();

#ifdef REVERSE_RTC_SELFTEST
  /* ================================================================
   * REVERSE + RTC SELF-TEST  (line-by-line / Live Expressions)
   * ----------------------------------------------------------------
   * Set a breakpoint on the first line, step with F5/F10, watch the
   * DWIN screen change and add "g_ctest" to Live Expressions.
   * Delete or comment out REVERSE_RTC_SELFTEST for production builds.
   * ================================================================ */

  /* ---- Reverse indicator icon : IA 0x6A ----
   * Exercise the REAL button path: LCD_ToggleReverse() flips the state
   * and writes {5A A5 05 82 00 6A 00 IS} internally.                   */

  LCD_ToggleReverse();                 /* reverse ENGAGED : icon 6A = 1 */
  g_ctest.reverse_icon = LCD_IsReverse();
  HAL_Delay(1500);

  LCD_ToggleReverse();                 /* reverse NORMAL  : icon 6A = 0 */
  g_ctest.reverse_icon = LCD_IsReverse();
  HAL_Delay(1500);

  /* ---- RTC on the LCD ----
   * Visual walk-through: sets several times on VP 0x0010 and advances a
   * live 10-second rollover. Watch the clock change on the display.     */
  RTC_RunDisplayTest();
  RTC_GetTime((RTC_Time_t *)&g_ctest.rtc);
#endif /* REVERSE_RTC_SELFTEST */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    BUTTON_MATRIX_Scan();
    Sensor_UpdateDisplay();
    LCD_Task();
    RTC_Tick();                               /* advance + refresh clock 1x/sec */
    RTC_GetTime((RTC_Time_t *)&g_ctest.rtc);  /* live snapshot for debugging    */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY_Pin|C1_Pin|C2_Pin|C3_Pin
                          |C4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C5_Pin|C6_Pin|C7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RELAY_Pin C1_Pin C2_Pin C3_Pin
                           C4_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin|C1_Pin|C2_Pin|C3_Pin
                          |C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C5_Pin C6_Pin C7_Pin */
  GPIO_InitStruct.Pin = C5_Pin|C6_Pin|C7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : R5_Pin */
  GPIO_InitStruct.Pin = R5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(R5_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
