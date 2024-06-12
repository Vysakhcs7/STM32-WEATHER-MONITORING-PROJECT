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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t rtc_slave_address = 0x68;
uint8_t rtc_reg_addr = 0x00;
uint8_t second;
uint8_t second_res;
#define DS1307_ADDRESS (0x68 << 1) // DS1307 I2C address shifted left
uint8_t get_time[3];
uint8_t tt_get_time[3];
uint8_t hours, minutes, seconds;
uint8_t t[3];
uint8_t seconds_hex;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void rtc_i2c_write(void);
void test(void);
void DS1307_GetTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds);
void DS1307_SetTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
uint8_t read_DS1307_seconds(void);
uint8_t secondsj;
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DS1307_SetTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    uint8_t set_time[3];
    set_time[0] = seconds;
    set_time[1] = minutes;
    set_time[2] = hours;
    HAL_I2C_Mem_Write(&hi2c1, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, set_time, 3, HAL_MAX_DELAY);
}

void DS1307_GetTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds) {

    HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, get_time, 3, HAL_MAX_DELAY);
    *seconds = get_time[0];
    *minutes = get_time[1];
    *hours = get_time[2];
}


void rtc_i2c_write(void)
{
	second = 0x10;

    HAL_I2C_Mem_Write(&hi2c1, rtc_slave_address, 0x00, 1,&second, 1, HAL_MAX_DELAY);

//	t[0] = 0x00;
//	t[1] = 0x20;
//	t[2] = 0x10;
	//t[3] = 0x09;
   //HAL_I2C_Mem_Write(&hi2c1, rtc_slave_address << 1, rtc_reg_addr, 1, t, 3, HAL_MAX_DELAY);

	//HAL_I2C_Master_Transmit(&hi2c1, rtc_slave_address << 1, t, 4, 10);
	//HAL_I2C_Master_Transmit(&hi2c1, rtc_slave_address << 1, t, 3, 10);
//			HAL_I2C_Master_Transmit(&hi2c1, rtc_slave_address << 1, &second_t, 1, 10);
//			HAL_I2C_Master_Transmit(&hi2c1, rtc_slave_address << 1, &minute_t, 1, 10);
//			HAL_I2C_Master_Transmit(&hi2c1, rtc_slave_address << 1, &hour_t, 1, 10);
//			HAL_I2C_Master_Transmit(&hi2c1, rtc_slave_address << 1, &dow, 1, 10);
//			HAL_I2C_Master_Transmit(&hi2c1, rtc_slave_address << 1, &date, 1, 10);
//			HAL_I2C_Master_Transmit(&hi2c1, rtc_slave_address << 1, &month, 1, 10);
//			HAL_I2C_Master_Transmit(&hi2c1, rtc_slave_address << 1, &year, 1, 10);
		//}

//		else
//		{
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//		}
}

void rtc_i2c_read(void)
{
		//if(HAL_I2C_IsDeviceReady(&hi2c1, rtc_slave_address << 1, 3, 5) == HAL_OK)
		//{
  //  HAL_I2C_Mem_Read(&hi2c1, rtc_slave_address, 0x00, 1, &r_second, 1, HAL_MAX_DELAY);
 // HAL_I2C_Mem_Read(&hi2c1,rtc_slave_address << 1 , rtc_reg_addr, 1,&second_res, 1, HAL_MAX_DELAY);
  //HAL_I2C_Mem_Read(&hi2c1,rtc_slave_address << 1 , 0x01, 1, &r_minute, 1, HAL_MAX_DELAY);
//	HAL_I2C_Master_Transmit(&hi2c1, rtc_slave_address << 1, &rtc_reg_addr, 1, 10);
//
//	HAL_I2C_Master_Receive(&hi2c1, rtc_slave_address_r, get_time, 3, 10);
//	r_second = get_time[0];
//	r_second = get_time[1];
//	r_second = get_time[2];

  uint8_t seconds_reg = 0x00; // Register address for seconds


    // Read seconds register
    HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDRESS, seconds_reg, I2C_MEMADD_SIZE_8BIT, &seconds_hex, 1, HAL_MAX_DELAY);

}

uint8_t read_DS1307_seconds(void)
{
    uint8_t seconds_reg = 0x00; // Register address for seconds
    uint8_t seconds_bcd = 0;    // Ensure the variable is initialized

    // Read seconds register
    if (HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDRESS, seconds_reg, I2C_MEMADD_SIZE_8BIT, &seconds_bcd, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        // Handle the error if the read operation fails
        Error_Handler();
    }

    // Return the seconds in hexadecimal (BCD) format
    return seconds_bcd;
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
  /* USER CODE BEGIN 2 */
//rtc_i2c_write();
 //Set initial time using hex values (e.g., 12:30:45)
DS1307_SetTime(0x12, 0x30, 0x45);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
     // secondsj = read_DS1307_seconds();

	//  HAL_Delay();

//       Get current time from DS1307
      DS1307_GetTime(&hours, &minutes, &seconds);

      // For debugging: you can place a breakpoint here or use a printf statement
      // to check the values of hours, minutes, and seconds.

      // Add a delay to avoid flooding the I2C bus
     HAL_Delay(1000);

  /* USER CODE END 3 */
}
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
