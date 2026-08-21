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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
    float vx;      /* byte0-3: m/s (そのまま) */
    float vy;      /* byte4-7: m/s (そのまま) */
    float omega;   /* byte8-11: rad/s (そのまま) */
} can_velocity_cmd_t;

typedef struct __attribute__((packed)) {
    float   speed_target;         /* byte0-3 */
    uint8_t pid_mode;             /* byte4: 0=locate_pid, 1=speed_pid */
    uint8_t control_motor_mode;   /* byte5: 0=電圧制御, 1=電流制御 */
    uint8_t reserved[2];          /* byte6-7 */
} can_motor_cmd_t;

#define CAN_VELOCITY_CMD_ID  0x100u

#define CAN_MOTOR_CMD_BASE_ID 0x301u

volatile float goal_speed_target = 300;
volatile uint8_t goal_pid_mode = 0;
volatile uint8_t goal_control_motor_mode = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WHEEL_RADIUS_M      0.037f  // ホイール半径[m] ← 実機に合わせて変更
#define ROBOT_RADIUS_M      0.31f   // 中心からホイールまでの距離[m] ← 実機に合わせて変更

#define WHEEL0_ANGLE_RAD    (-(float)M_PI / 3.0f)          //  60°  
#define WHEEL1_ANGLE_RAD    ((float)M_PI / 3.0f)          // -60°  
#define WHEEL2_ANGLE_RAD    ((float)M_PI)  

#define MAX_WHEEL_RPM 5000.0f  // doc1/2側のクリップ値と合わせる

volatile float rx_vx    = 0.0f;   /* m/s */
volatile float rx_vy    = 0.0f;   /* m/s */
volatile float rx_omega = 0.0f;   /* rad/s */
volatile uint32_t velocity_rx_count = 0;

volatile float wheel_speed_target[3] = {0.0f, 0.0f, 0.0f};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t tx_success_count = 0;
volatile uint32_t tx_fail_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN1_ConfigFilterAndStart(void);
static void compute_wheel_targets(float vx, float vy, float omega);
static inline float be_bytes_to_float(const uint8_t *p);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline float be_bytes_to_float(const uint8_t *p)
{
    union { uint32_t u; float f; } conv;
    conv.u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
           | ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
    return conv.f;
}

static void FDCAN1_ConfigFilterAndStart(void)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = CAN_VELOCITY_CMD_ID;
    sFilterConfig.FilterID2    = CAN_VELOCITY_CMD_ID;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                      FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* 3輪オムニの逆運動学: vx,vy[m/s], omega[rad/s] → 各ホイールrpm */
static void compute_wheel_targets(float vx, float vy, float omega)
{
    const float wheel_angle[3] = { WHEEL0_ANGLE_RAD, WHEEL1_ANGLE_RAD, WHEEL2_ANGLE_RAD };

    for (int i = 0; i < 3; i++)
    {
        // 接線方向速度成分 + 回転による寄与
        float v_wheel_mps = -vx * sinf(wheel_angle[i]) + vy * cosf(wheel_angle[i])
                             + ROBOT_RADIUS_M * omega;

        // m/s → rpm変換: rpm = v / (2*pi*r) * 60
        float rpm = v_wheel_mps / (2.0f * (float)M_PI * WHEEL_RADIUS_M) * 60.0f;

        if (rpm > MAX_WHEEL_RPM)  rpm = MAX_WHEEL_RPM;
        if (rpm < -MAX_WHEEL_RPM) rpm = -MAX_WHEEL_RPM;

        wheel_speed_target[i] = rpm;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1) return;
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];   // CAN FDは最大64byteなのでバッファを拡張

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        return;
    }

    if (RxHeader.IdType != FDCAN_STANDARD_ID) return;
    if (RxHeader.Identifier != CAN_VELOCITY_CMD_ID) return;
    if (RxHeader.FDFormat != FDCAN_FD_CAN ||
      RxHeader.BitRateSwitch != FDCAN_BRS_ON ||
      RxHeader.DataLength != FDCAN_DLC_BYTES_12)
    {
      return;
    }
    can_velocity_cmd_t cmd;
    
    cmd.vx    = be_bytes_to_float(&RxData[0]);
    cmd.vy    = be_bytes_to_float(&RxData[4]);
    cmd.omega = be_bytes_to_float(&RxData[8]);

    rx_vx    = cmd.vx;      
    rx_vy    = cmd.vy;
    rx_omega = cmd.omega;

    compute_wheel_targets(rx_vx, rx_vy, rx_omega);

    velocity_rx_count++;
}

static void FDCAN3_ConfigAndStart(void)  
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    /* 基本的に送信用のリポジトリなのでフィルタは全拒否のグローバル設定 */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
                                  FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)   //  hfdcan3
    {
        Error_Handler();
    }
}

HAL_StatusTypeDef send_motor_cmd(uint8_t motor_id, float speed_target,
                                  uint8_t pid_mode, uint8_t control_mode)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    can_motor_cmd_t cmd;

    cmd.speed_target = speed_target;
    cmd.pid_mode = pid_mode;
    cmd.control_motor_mode = control_mode;
    cmd.reserved[0] = 0;
    cmd.reserved[1] = 0;

    TxHeader.Identifier = CAN_MOTOR_CMD_BASE_ID + motor_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, (uint8_t*)&cmd);  // hfdcan3
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            HAL_StatusTypeDef st = send_motor_cmd(i, wheel_speed_target[i],
                                                   goal_pid_mode, goal_control_motor_mode);
            if (st == HAL_OK) { tx_success_count++; } else { tx_fail_count++; }
        }
    }
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}
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
  MX_FDCAN1_Init();
  MX_FDCAN3_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  FDCAN3_ConfigAndStart();   
  FDCAN1_ConfigFilterAndStart();
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0); 
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    FDCAN_ProtocolStatusTypeDef pstatus;
    FDCAN_ErrorCountersTypeDef  ecounters;
    HAL_FDCAN_GetProtocolStatus(&hfdcan3, &pstatus);
    HAL_FDCAN_GetErrorCounters(&hfdcan3, &ecounters);

  //   printf("tx_ok=%lu tx_fail=%lu BusOff=%d ErrPassive=%d TEC=%lu REC=%lu\r\n",
  //          tx_success_count, tx_fail_count,
  //          pstatus.BusOff, pstatus.ErrorPassive,
  //          (unsigned long)ecounters.TxErrorCnt, (unsigned long)ecounters.RxErrorCnt);

  //   printf("tx_ok=%lu tx_fail=%lu vel_rx=%lu vx=%.3f vy=%.3f omega=%.3f w0=%d w1=%d w2=%d\r\n",
  //  tx_success_count, tx_fail_count, velocity_rx_count,
  //  rx_vx, rx_vy, rx_omega,
  //  (int)wheel_speed_target[0], (int)wheel_speed_target[1], (int)wheel_speed_target[2]);
    // static uint32_t last_rx_count = 0;
    // static uint32_t last_print_tick = 0;

    // if (HAL_GetTick() - last_print_tick >= 300)
    // {
    //   FDCAN_ProtocolStatusTypeDef pstatus1;
    //   FDCAN_ErrorCountersTypeDef  ecounters1;
    //   HAL_FDCAN_GetProtocolStatus(&hfdcan1, &pstatus1);
    //   HAL_FDCAN_GetErrorCounters(&hfdcan1, &ecounters1);

    //   printf("FDCAN1: BusOff=%d ErrPassive=%d Warning=%d TEC=%lu REC=%lu LastErrCode=%lu Activity=%lu\r\n",
    //        pstatus1.BusOff, pstatus1.ErrorPassive, pstatus1.Warning,
    //        (unsigned long)ecounters1.TxErrorCnt,
    //        (unsigned long)ecounters1.RxErrorCnt,
    //        (unsigned long)pstatus1.LastErrorCode,
    //        (unsigned long)pstatus1.Activity);

    //   last_print_tick = HAL_GetTick();
    // }

    // if (velocity_rx_count != last_rx_count)
    // {
    //   printf("CAN0x100 RX#%d vx=%d vy=%d omega=%d\r\n",
    //   (int64_t)velocity_rx_count, (int64_t)rx_vx, (int64_t)rx_vy, (int64_t)rx_omega);
    //   last_rx_count = velocity_rx_count;
    // }
    printf("CAN0x100 RX#%lu vx=%ld vy=%ld omega=%ld (x1000)\r\n",   // ← ループの外
       (unsigned long)velocity_rx_count,
       (long)(rx_vx    * 1000.0f),
       (long)(rx_vy    * 1000.0f),
       (long)(rx_omega * 1000.0f));
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 31;
  hfdcan1.Init.DataTimeSeg2 = 8;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = ENABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 15;
  hfdcan3.Init.DataTimeSeg2 = 4;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 39999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
