/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include "stdio.h"
	#include <string.h>
	#include "lcd1602_fc113_sm.h"
	#include "tm1637_sm.h"
	#include "bh1750_sm.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	char 		uart_buff_char[0xFF];
	#define 	DATE_as_int_str 	(__DATE__)
	#define 	TIME_as_int_str 	(__TIME__)

	sprintf(uart_buff_char,"Build: %s. Time: %s.\r\n" , DATE_as_int_str , TIME_as_int_str ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)uart_buff_char , strlen(uart_buff_char) , 100 ) ;

	lcd1602_fc113_struct h1_lcd1602_fc113 = {
		.i2c = &hi2c1,
		.device_i2c_address = LCD1602_I2C_ADDR
	};

	LCD1602_Init(&h1_lcd1602_fc113);
	LCD1602_scan_I2C_bus( &h1_lcd1602_fc113 ) ;
	LCD1602_Scan_I2C_to_UART( &h1_lcd1602_fc113, &huart1 ) ;
	//	HAL_Delay(1000);
	LCD1602_Clear(&h1_lcd1602_fc113);

	bh1750_struct h1_bh1750 = {
		.i2c = &hi2c1,
		.device_i2c_address = BH1750_I2C_ADDR
	};

	BH1750_init( &h1_bh1750, bh1750_one_time_h_resolutione );

	uint16_t lux_u16 = 0 ;
	BH1750_get_lux( &h1_bh1750, bh1750_one_time_h_resolutione, &lux_u16);
	sprintf(uart_buff_char,"LUS=%d\r\n" , (int)lux_u16 ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)uart_buff_char , strlen(uart_buff_char) , 100 ) ;
	uint32_t Power_PWM_u32 = 0;

	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin ) ;

	BH1750_get_lux( &h1_bh1750, bh1750_one_time_h_resolutione, &lux_u16);
	sprintf(uart_buff_char,"Lux: %05d;" , (int)lux_u16 ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)uart_buff_char , strlen(uart_buff_char) , 100 ) ;

	sprintf( uart_buff_char, "lux: %05d     *",  (int)lux_u16 ) ;
	LCD1602_Print_Line( &h1_lcd1602_fc113 , uart_buff_char , strlen(uart_buff_char) ) ;

	Power_PWM_u32 = 100 + lux_u16 / 65 ;
	TIM4->CCR1 = Power_PWM_u32;

	sprintf(uart_buff_char, "\t Power_PWM: %05d \r",  (int)Power_PWM_u32 ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)uart_buff_char , strlen(uart_buff_char) , 100 ) ;

	sprintf( uart_buff_char, "pwm: %05d     *",  (int)Power_PWM_u32 ) ;
	LCD1602_Print_Line( &h1_lcd1602_fc113 , uart_buff_char , strlen(uart_buff_char) ) ;

	LCD1602_Cursor_Return( &h1_lcd1602_fc113 ) ;

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
