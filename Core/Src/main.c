/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "math.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MOTOR_STEP_ANGLE     1.8f  //步距角
#define MOTOR_STEPS_PER_REV  200   //一圈200步
#define DRIVER_MICROSTEPS    16    
#define ANGLE_PER_STEP       (360.0f / (MOTOR_STEPS_PER_REV * DRIVER_MICROSTEPS)) 
/* 限位 */
#define UP_MAX 90
#define UP_MIN -90
#define DOWN_MAX 90
#define DOWN_MIN -90
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//全局变量，用于计算步进电机转向角度所需的脉冲
volatile uint32_t pulses_remaining1 = 0; 
volatile uint32_t pulses_remaining2 = 0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f)
{
	uint8_t temp[1] = {ch};
	HAL_UART_Transmit(
        &huart1 // 自定义串口号
    , temp, 1, 2); 
	return ch;
}

// 全局变量定义
#define RX_BUFFER_SIZE 128  //缓冲区数组长度
uint8_t USART_Rx_BUFFER[RX_BUFFER_SIZE + 1]; // 接收缓冲区（+1为字符串终止符）
volatile uint16_t USART_LEN = 0;            // 有效数据长度
volatile uint8_t UartRxComplete = 0;        // 接收完成标志

/***********************************************************************
控制步进电机转动角度的函数
相关参数：
				angle浮点型角度值
				方向         GPIO_PIN_RESET逆时针，GPIO_PIN_SET顺时针。
**************************************************************************/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2 && pulses_remaining1 > 0) {
        pulses_remaining1--;
        if (pulses_remaining1 == 0) {
            __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        }
    } else if (htim->Instance == TIM3 && pulses_remaining2 > 0) {
        pulses_remaining2--;
        if (pulses_remaining2 == 0) {
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        }
    }
}

float absoluteAnglesUp = 0, // 上电机绝对角度
	absoluteAnglesDown = 0; // 下电机绝对角度

void Stepper_Rotate_up(float delta_target_angle, GPIO_PinState direction) {
		// 限位
		if(direction && (absoluteAnglesUp + delta_target_angle > UP_MAX))
		{ // 正转
				return;
		} else if (!direction && (absoluteAnglesUp - delta_target_angle < UP_MIN)) { // 反转
			return;
		}
		
		absoluteAnglesUp += delta_target_angle * (direction ? 1:-1);

		
    if (pulses_remaining1 > 0) {
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        pulses_remaining1 = 0;
    }
    uint32_t total_pulses1 = (uint32_t)(fabs(delta_target_angle) / ANGLE_PER_STEP + 0.5f);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, direction);
    pulses_remaining1 = total_pulses1;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
}


void Stepper_Rotate_down(float delta_target_angle, GPIO_PinState direction) {
		
		// 限位
		if(direction && (absoluteAnglesDown + delta_target_angle > DOWN_MAX))
		{ // 正转
				return;
		} else if (!direction && (absoluteAnglesDown - delta_target_angle < DOWN_MIN)) { // 反转
			return;
		}
		
		absoluteAnglesDown += delta_target_angle * (direction ? 1:-1);
			
    if (pulses_remaining2 > 0) {
        __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        pulses_remaining2 = 0;
    }
    uint32_t total_pulses2 = (uint32_t)(fabs(delta_target_angle) / ANGLE_PER_STEP + 0.5f);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, direction);
    pulses_remaining2 = total_pulses2;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}


// 数据处理函数。用于摄像头一识别靶环，并传输靶环坐标数据
#if 0
void MAIXCAM_GetData(void)
{
    if (UartRxComplete) {
        float delta_x = 0;
        float delta_y = 0;
        
        /* 添加字符串终止符 */
        USART_Rx_BUFFER[USART_LEN] = '\0';
        /* 解析指令（示例）*/
        if (USART_Rx_BUFFER[0] == '@' && USART_Rx_BUFFER[1] == 'x') {
            delta_x = strtod((char*)&USART_Rx_BUFFER[2], NULL);
					  Stepper_Rotate_up(delta_x, GPIO_PIN_RESET); 
        }
        else if (USART_Rx_BUFFER[0] == '@' && USART_Rx_BUFFER[1] == 'y') {
            delta_y = strtod((char*)&USART_Rx_BUFFER[2], NULL);
					  Stepper_Rotate_up(delta_y, GPIO_PIN_RESET); 
        }

        // Motor_Control(delta_x, delta_y);
        /* 重置接收参数 */
        memset(USART_Rx_BUFFER, 0, RX_BUFFER_SIZE+1);
        UartRxComplete = 0;
        
        /* 重启接收 */
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, USART_Rx_BUFFER, RX_BUFFER_SIZE);
    }
	}
#endif

uint8_t rx[3];


//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		
	}
	HAL_UART_Receive_IT(huart,rx,3);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	
		
//电机初始化，不进行初始化会导致电机一直旋转
	
	HAL_UART_Receive_IT(&huart1,rx,3);
	
	// 初始化防止乱动
	Stepper_Rotate_up(1, GPIO_PIN_RESET); 
	Stepper_Rotate_up(1, GPIO_PIN_SET); 
	Stepper_Rotate_down(1, GPIO_PIN_RESET); 
	Stepper_Rotate_down(1, GPIO_PIN_SET); 
	
	// 实验
   Stepper_Rotate_up(45, GPIO_PIN_RESET); 
	 HAL_Delay(500);
	 Stepper_Rotate_up(45, GPIO_PIN_RESET); 
	 HAL_Delay(500);
	 Stepper_Rotate_up(45, GPIO_PIN_RESET); 
	 
	 Stepper_Rotate_down(45, GPIO_PIN_RESET); 
	 HAL_Delay(500);
	 Stepper_Rotate_down(45, GPIO_PIN_RESET); 
	 HAL_Delay(500);
	 Stepper_Rotate_down(45, GPIO_PIN_RESET); 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  while (1)
  {
  }
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
