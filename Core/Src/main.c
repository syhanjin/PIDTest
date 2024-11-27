/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

#include "motor.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define SPEED_LOOP
#define POSITION_LOOP
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t delay_counter = 10;

uint8_t rb[10];

Motor_t motors[2] = {
  {
    {&htim4, TIM_CHANNEL_1}, ///< PWM信号 {定时器, 通道}
    &htim2, ///< 编码器使用的定时器
    {MOTORA_IN1_GPIO_Port, MOTORA_IN1_Pin}, ///< 电机输入IN1 {GPIOx, Pin}
    {MOTORA_IN2_GPIO_Port, MOTORA_IN2_Pin}, ///< 电机输入IN2 {GPIOx, Pin}
    0, 0, 0, 0 ///< 方向，速度（0~1)，编码器测得的速度，编码器测得的圈数
  }
};
PID_t PIDs[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* printf retarget */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
  return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim == &htim1)
  {
    /* 编码器采样 */
    Encoder_Progress(motors + 0);
    /* PID计算 速度环 */
    if (PIDs[0].enable)
    {
#ifdef SPEED_LOOP
      float pid_output = PID_Calculate(PIDs + 0, motors[0].real_speed);
#endif
#ifdef POSITION_LOOP
      float pid_output = PID_Calculate(PIDs + 0, motors[0].real_round);
#endif
      Motor_SetSpeed(motors + 0, pid_output);
    }
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart == &huart1)
  {
    if (rb[0] == 0xAA && rb[6] == 0xBB)
    {
      switch (rb[1])
      {
      case 0x11:
        PIDs[0].target = *(float*)(rb + 2);
        break;
      case 0x22:
        Motor_Start(motors);
        Encoder_Start(motors);
        PIDs[0].enable = 1;
        break;
      case 0x33:
        Motor_Stop(motors);
        PIDs[0].enable = 0;
        PIDs[0].output = 0;
        delay_counter = 10;
        break;
      case 0x44: // 修改比例系数
        PIDs->KpE = *(float*)(rb + 2)
#ifdef SPEED_LOOP
          / MAX_SPEED
#endif
          ;
        break;
      case 0x55: // 修改积分系数
        PIDs->KiE = *(float*)(rb + 2)
#ifdef SPEED_LOOP
          / MAX_SPEED
#endif
          ;
        break;
      case 0x66: // 修改微分系数
        PIDs->KdE = *(float*)(rb + 2)
#ifdef SPEED_LOOP
          / MAX_SPEED
#endif
          ;
        break;
      case 0x77: // 修改微分低通滤波系数
        PIDs->dfilterE = *(float*)(rb + 2);
        break;
      case 0x99: //
        PIDs->ufilterE = *(float*)(rb + 2);
        break;
      default: ;
      }
      printf("ACK: %f,%f,%f,%f,%f,%f\n",
             PIDs->target, PIDs->KpE, PIDs->KiE, PIDs->KdE, PIDs->dfilterE, PIDs->ufilterE);
    }
    UART_Start_Receive_IT(&huart1, rb, 7);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  PID_Init(&PIDs[0],
           1.72742, 0.14087, 0.35868, 0.57931, 0.51708,
           2.47420, 0.02883, 0.22799, 0.60006, 0.60421, 0.02,
           1, 2, 0
  );
  printf("PID: KpS: %f, KiS: %f, KdS: %f, ufilterS: %f, dfilterS: %f\n", PIDs[0].KpS, PIDs[0].KiS, PIDs[0].KdS, PIDs[0].ufilterS, PIDs[0].dfilterS);
  printf("PID: KpE: %f, KiE: %f, KdE: %f, ufilterE: %f, dfilterE: %f\n", PIDs[0].KpE, PIDs[0].KiE, PIDs[0].KdE, PIDs[0].ufilterE, PIDs[0].dfilterE);
  printf("PID: output_abs_max: %f, cutoff %f, target: %f\n", PIDs[0].output_abs_max, PIDs[0].cutoff, PIDs[0].target);

  UART_Start_Receive_IT(&huart1, rb, 7);
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (PIDs[0].enable || delay_counter)
    {
      printf("%f,%f,%f,%f\n", motors[0].real_round, motors[0].real_speed, PIDs[0].output, PIDs[0].target);
      delay_counter--;
    }
    HAL_Delay(20);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
