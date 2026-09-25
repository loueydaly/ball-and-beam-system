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
#include "../Inc/main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l0x_api.h"
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define VL53L0X_ADDR (0x29 << 1) // default I2C address left-shifted for HAL

static VL53L0X_Dev_t                    vl53_dev;
static VL53L0X_RangingMeasurementData_t range_data;

// SMC Variables
double setpoint = 150.0; // Setpoint in mm
double smc_K = -7.0071;
double smc_c = 3.0;
double smc_M = 5.0;
double smc_Phi = 0.05;

// Adaptive SMC Parameters
double a_hat = 0;        // Estimate of 1/K (initialized to approx 1 / -7.0071)
double Gamma = 0.05;          // Reduced significantly to prevent explosion
double sigma_mod = 0.5;       // Increased to strongly drag it towards 0 when not adapting
double A_MIN = -3.0;          // Lower bound for a_hat
double A_MAX = +1.01;         // Upper bound for a_hat (since K is negative, 1/K must strictly be negative)

const double errorDeadband = 10.0; // mm
const int stepDeadband = 0;

double previousError = 0.0;
double currentDistance = 0.0;

// Motor state variables
volatile int currentStep = 61 * 16;
volatile int targetStep = 61 * 16;
volatile int stepsRemaining = 0;
volatile int motorDirection = 0; // 0 = LOW, 1 = HIGH
volatile uint8_t stepPinState = 0;

// Timing variables
volatile uint8_t pid_flag_20ms = 0;
volatile uint32_t ms_tick_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Blocking function to move the stepper for initialization
void moveStepper_Blocking(int numOfSteps, uint8_t direction) {
    // Set Direction
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (direction) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    for (int i = 0; i < numOfSteps; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        
        // Blocking delay (approximate delay using a loop since HAL_Delay is ms only)
        // 1250/16 approx 78 microseconds. Since clock is 180MHz, a simple wait loop is needed
        for (volatile int j = 0; j < 3000; j++); 
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        
        for (volatile int j = 0; j < 3000; j++);
    }
}

VL53L0X_Error VL53L0X_i2c_init(void)
{
    return VL53L0X_ERROR_NONE; /* I2C already init by CubeMX */
}
 
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (HAL_I2C_Mem_Write(&hi2c1, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 100) == HAL_OK)
        return VL53L0X_ERROR_NONE;
    return VL53L0X_ERROR_CONTROL_INTERFACE;
}
 
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (HAL_I2C_Mem_Read(&hi2c1, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 100) == HAL_OK)
        return VL53L0X_ERROR_NONE;
    return VL53L0X_ERROR_CONTROL_INTERFACE;
}
 
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}
 
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}
 
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buf[2] = {(uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};
    return VL53L0X_WriteMulti(Dev, index, buf, 2);
}
 
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    uint8_t buf[2];
    VL53L0X_Error err = VL53L0X_ReadMulti(Dev, index, buf, 2);
    *data = ((uint16_t)buf[0] << 8) | buf[1];
    return err;
}
 
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buf[4] = {
        (uint8_t)(data >> 24), (uint8_t)(data >> 16),
        (uint8_t)(data >>  8), (uint8_t)(data)
    };
    return VL53L0X_WriteMulti(Dev, index, buf, 4);
}
 
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    uint8_t buf[4];
    VL53L0X_Error err = VL53L0X_ReadMulti(Dev, index, buf, 4);
    *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
            ((uint32_t)buf[2] <<  8) |  (uint32_t)buf[3];
    return err;
}
 
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

    Status = VL53L0X_RdByte(Dev, index, &data);
    if (Status != VL53L0X_ERROR_NONE) {
        return Status;
    }
    data = (data & AndData) | OrData;
    Status = VL53L0X_WrByte(Dev, index, data);
    return Status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    HAL_Delay(5);
    return VL53L0X_ERROR_NONE;
}
 
static void VL53L0X_Init(void)
{
    uint32_t refSpadCount;
    uint8_t  isApertureSpads;
    uint8_t  VhvSettings, PhaseCal;
 
    vl53_dev.I2cDevAddr      = 0x52;  /* 8-bit address (7-bit 0x29 << 1) */
    vl53_dev.comms_type      = 1;     /* I2C */
    vl53_dev.comms_speed_khz = 400;
 
    VL53L0X_DataInit(&vl53_dev);
    VL53L0X_StaticInit(&vl53_dev);
    VL53L0X_PerformRefCalibration(&vl53_dev, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(&vl53_dev, &refSpadCount, &isApertureSpads);
    VL53L0X_SetDeviceMode(&vl53_dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
 
    /* Measurement quality limits */
    VL53L0X_SetLimitCheckEnable(&vl53_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(&vl53_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(&vl53_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
    VL53L0X_SetLimitCheckValue(&vl53_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&vl53_dev, 15000);
}

// Intercept SysTick for 20ms PID loop timing
void HAL_SYSTICK_Callback(void) {
    ms_tick_count++;
    if (ms_tick_count >= 20) {
        ms_tick_count = 0;
        pid_flag_20ms = 1;
    }
}

// Timer interrupt for non-blocking motor step generation
void Motor_Step_Handler(void) {
    if (stepsRemaining > 0) {
        if (stepPinState == 0) {
            // Set DIR pin and HIGH on STEP pin
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (motorDirection) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
            stepPinState = 1;
        } else {
            // LOW on STEP pin
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
            stepPinState = 0;
            stepsRemaining--;
            if (motorDirection == 0) currentStep++; else currentStep--;
        }
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
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Move stepper to initial horizontal position (61 * 16 steps)
  // GPIO_PIN_SET (1) represents HIGH in Arduino code
  moveStepper_Blocking(61 * 16, 1);
  HAL_Delay(1000); // 3 second wait before starting regulation

  // Start the motor timer in interrupt mode
  HAL_TIM_Base_Start_IT(&htim6);

  // Initialize VL53L0X
  VL53L0X_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Execute PID loop every 20ms
    if (pid_flag_20ms == 1) {
        pid_flag_20ms = 0; // Clear flag
        
        double dt = 0.020; // 20ms time step

        // 1. Read Sensor
        VL53L0X_PerformSingleRangingMeasurement(&vl53_dev, &range_data);
        
        // RangeStatus = 0 means valid measurement
        if (range_data.RangeStatus == 0 && range_data.RangeMilliMeter > 0 && range_data.RangeMilliMeter < 2000) {
            currentDistance = (double)range_data.RangeMilliMeter;

            // ── 2. Compute Adaptive SMC ────────────────────────────────────────────

            // Position error (metres)
            double x  = currentDistance / 1000.0;
            double xd = setpoint        / 1000.0;
            double error = xd - x;

            // Deadband (metres)
            if (error > -errorDeadband / 1000.0 && error < errorDeadband / 1000.0)
                error = 0.0;

            // Error derivative
            double de = (error - previousError) / dt;

            // ── Sliding surface  s = λ·e + ė ──────────────────────────────────────
            double s = smc_c * error + de;

            // ── Saturation (anti-chattering) ───────────────────────────────────────
            double s_phi = s / smc_Phi;
            double sat   = s_phi >  1.0 ?  1.0 :
                           s_phi < -1.0 ? -1.0 : s_phi;

            // ── Regressor v = −λ·ė − M·sat(s/Φ) ──────────────────────────────────
            double v = -smc_c * de - smc_M * sat;

            // ── Adaptive control output (radians) ─────────────────────────────────
            // With a negative gain true system (K < 0), we want u = a_hat * v.
            // Notice how a_hat starts negative, and v opposes de & sat.
            double u = a_hat * v;

            // ── Parameter update law (Euler integration) ──────────────────────────
            // Since the system has a negative sign implicitly inside its dynamic:
            // a_hat_dot = -(Gamma) * s * v - sigma * a_hat   <-- That drives it to right direction
            double a_hat_dot = -Gamma * s * v   -   sigma_mod * a_hat;
            a_hat += a_hat_dot * dt;

            // ── Projection: keep â in physically meaningful range ──────────────────
            // Keep a_hat strictly negative to represent 1/K
            if (a_hat < A_MIN) a_hat = A_MIN;
            // Limit closer to 0 but strictly negative.
            if (a_hat > A_MAX) a_hat = A_MAX;

            // ── Convert α (radians) → motor steps ─────────────────────────────────
            // Adding a sign flip to output conversion step since the hardware moves 
            // backwards compared to the mathematical model of error  xd - x
            double output = -u * (1600.0 / 3.1415926535);

            // Constrain
            if (output >  39.0 * 16.0) output =  39.0 * 16.0;
            if (output < -61.0 * 16.0) output = -61.0 * 16.0;

            targetStep = (61 * 16) + (int)output;
            __disable_irq(); // Protect shared variables
            int cs = currentStep;
            __enable_irq();
            int stepsDiff = targetStep - cs;

            // ── Command motor ──────────────────────────────────────────────────────
            if (stepsDiff > stepDeadband || stepsDiff < -stepDeadband) {
                __disable_irq(); // Protect shared variables
                stepsRemaining = 0;
                motorDirection = (stepsDiff > 0) ? 0 : 1;
                stepsRemaining = (stepsDiff > 0) ? stepsDiff : -stepsDiff;
                __enable_irq();
            }

            previousError = error;

            // ── Print variables for adaptation monitoring ───────────────────────
            char debug_buffer[128];
            // Multiply by 1000 since printf float support may be disabled
            snprintf(debug_buffer, sizeof(debug_buffer), "e:%d s:%d v:%d a^:%d u:%d steps:%d de:%d\r\n",
                     (int)(error * 1000), (int)(s * 1000), (int)(v * 1000), 
                     (int)(a_hat * 100000), (int)(u * 1000), stepsDiff, (int)(de * 1000));
            HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), 100);
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim6.Init.Prescaler = 99;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 299;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Check if the interrupt came from TIM6 (our motor timer)
    if (htim->Instance == TIM6) {
        Motor_Step_Handler(); // Send the pulse to the tb6560
    }
}
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
