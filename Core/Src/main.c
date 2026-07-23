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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "mpu9250.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KP 3.185f			// las mas chingonas 3.185 k y 0.012 de TD y 1000 TI
#define TI 900.0f			//5.0f
#define TD 0.012
#define T0 0.005f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//	VARIABLES PARA MPU9250
MPU9250 imu;
volatile uint8_t imu_sample_flag = 0;

// VARIABLES PID POSICION
volatile float e0 = 0.0f;
volatile float e1 = 0.0f;
volatile float e2 = 0.0f;
volatile float u = 0.0f;
volatile float delta_u = 0;
volatile float PWM = 0;
float u_temp = 0;


const float q0 = KP*(1.0f + (T0/(2.0f*TI)) + (TD/T0));
const float q1 = -KP*(1.0f - (T0/(2.0f*TI)) + (2.0f*TD/T0));
const float q2 = (KP*TD)/T0;

int L_patita_1 = 0;
int L_patita_2 = 0;

int R_patita_1 = 0;
int R_patita_2 = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
int uart2_write(int ch);
int __io_putchar(int ch);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  printf("hola");

  //I2C_Scan(&hi2c2);

  //	Iniciamos el sensr
  //MPU9250_Init(&imu, &hi2c2);

  //MPU9250_Calibrate(&imu, 1000);

  if(MPU9250_Init(&imu, &hspi2, MPU_CS_GPIO_Port, MPU_CS_Pin) != HAL_OK)
  {
      while(1) { HAL_GPIO_TogglePin(GPIOA, LD2_Pin); HAL_Delay(100); }
  }

  MPU9250_Calibrate(&imu, 1000);

  HAL_TIM_Base_Start_IT(&htim2);
  int print_count = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(imu_sample_flag >= 5)
	  {
		print_count++;

		if(MPU9250_GetData(&imu) == HAL_OK)
		{
			MPU9250_Update(&imu, 0.005f);
		}
		  /*
		  * CONTROL PID POSICION
		  */

		 e0 = (-7.10) - imu.roll;
		 delta_u = (q0*e0)+(q1*e1)+(q2*e2);
		 u += delta_u;




		 if(u >= 10.4)
		 {
		  u = 10.4;
		 }
		 if(u <= -10.4)
		 {
		  u = -10.4;
		 }


		 PWM = (fabs(u)/10.4)*1000.0;

		 /*
		 *  ASIGNACION DE PWM Y SENTIDO DE PINES
		 */

		 if(u >= 0.0)
		 {
			 L_patita_1 = 1;
			 L_patita_2 = 0;

			 R_patita_1 = 1;
			 R_patita_2 = 0;

		    HAL_GPIO_WritePin(GPIOA, R_Motor_Direction_2_Pin, R_patita_2);
		    HAL_GPIO_WritePin(GPIOA, R_Motor_Direction_1_Pin, R_patita_1);

		 	HAL_GPIO_WritePin(GPIOA, L_Motor_Direction_2_Pin, L_patita_2);
		    HAL_GPIO_WritePin(GPIOA, L_Motor_Direction_1_Pin, L_patita_1);

		 }
		 if(u < 0.0){

			 L_patita_1 = 0;
			 L_patita_2 = 1;

			 R_patita_1 = 0;
			 R_patita_2 = 1;

			 HAL_GPIO_WritePin(GPIOA, R_Motor_Direction_2_Pin, R_patita_2);
			 HAL_GPIO_WritePin(GPIOA, R_Motor_Direction_1_Pin, R_patita_1);

			 HAL_GPIO_WritePin(GPIOA, L_Motor_Direction_2_Pin, L_patita_2);
			 HAL_GPIO_WritePin(GPIOA, L_Motor_Direction_1_Pin, L_patita_1);

		 }

		 e2 = e1;
		 e1 = e0;

		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM);
		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM);

		 if(print_count == 10)
		 {
		   print_count = 0;
		   printf("Pitch: %.2f\r\n U: %.2f\r\n",(-7.10)- imu.roll, u);
		  // printf("R1=%d R2=%d L1=%d L2=%d\r\n",R_patita_1, R_patita_2, L_patita_1, L_patita_2);

		 }
		 imu_sample_flag = 0;

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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */
int uart2_write(int ch)
{
	while(!(USART2 -> SR & USART_SR_TXE)) {}
	USART2 -> DR = (ch & 0xFF);
	return ch;
}

int __io_putchar(int ch)
{
	uart2_write(ch);
	return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim -> Instance == TIM2)
	{
		imu_sample_flag += 1; //Activamos la bandera para toma de datos del mpu

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
