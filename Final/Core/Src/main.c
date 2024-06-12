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
/* LCD */
#include <stdio.h>
#include<string.h>
#define delay_t				   1
#define LCD_FUNCTION_SET1      0x33
#define LCD_FUNCTION_SET2      0x32
#define LCD_4BIT_2LINE_MODE    0x28
#define LCD_DISP_CURS_ON       0x0E
#define LCD_DISP_ON_CURS_OFF   0x0C  //Display on, cursor off
#define LCD_DISPLAY_OFF        0x08
#define LCD_DISPLAY_ON         0x04
#define LCD_CLEAR_DISPLAY      0x01
#define LCD_ENTRY_MODE_SET     0x04
#define LCD_INCREMENT_CURSER   0x06
#define LCD_SET_ROW1_COL1      0x80  //Force cursor to beginning ( 1st line)
#define LCD_SET_ROW2_COL1      0xC0  //Force cursor to beginning ( 2nd line)
#define LCD_MOVE_DISPLAY_LEFT  0x18
#define LCD_MOVE_DISPLAY_RIGHT 0x1C

#define slave_address      0x3F
#define DS1307_ADDRESS (0x68 << 1)

uint8_t data[5];
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

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
uint8_t a = 0;
char sec[2];
char min[2];
char hr[2];
uint8_t hour, minute, second;

char tCelsius_str[5];
		  char RH_str[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_UART5_Init(void);
uint8_t DHT11_Read (void);
uint8_t DHT11_Start (void);
void microDelay (uint16_t delay);
void i2c_transmit_command(uint8_t data);
void lcd_init(void);
void i2c_transmit_data(uint8_t data);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart5, (uint8_t*)ptr, len, 100);
	return len;
}


uint8_t DEC2BCD(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

uint8_t BCD2DEC(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

void DS1307_SetTime(uint8_t hour, uint8_t minute, uint8_t second)
{
    uint8_t data[3];
    data[0] = DEC2BCD(second);
    data[1] = DEC2BCD(minute);
    data[2] = DEC2BCD(hour);

    HAL_I2C_Mem_Write(&hi2c1, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, data, 3, HAL_MAX_DELAY);
}

void DS1307_GetTime(uint8_t *hour, uint8_t *minute, uint8_t *second) {
    uint8_t data[3];

    HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, data, 3, HAL_MAX_DELAY);

    *second = BCD2DEC(data[0]);
    *minute = BCD2DEC(data[1]);
    *hour = BCD2DEC(data[2]);
}

void lcd_init(void) {

	/* Wait for 15ms */
	HAL_Delay(15);

	/*Function Set - As per HD44780U*/
	i2c_transmit_command(LCD_FUNCTION_SET1);

	/*Function Set -As per HD44780U*/
	i2c_transmit_command(LCD_FUNCTION_SET2);

	/*Set 4bit mode and 2 lines */
	i2c_transmit_command(LCD_4BIT_2LINE_MODE);

	/*Display on, cursor off*/
	i2c_transmit_command(0x0C);

	/* SET Row1 and Col1 (1st Line) */
	i2c_transmit_command(0x80);

	/*Clear Display*/
	i2c_transmit_command(LCD_CLEAR_DISPLAY);
}



void i2c_transmit_command(uint8_t data)
{

	uint8_t command_en_on 	= ( (data & 0xF0) | 0x0C); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t command_en_off 	= ( command_en_on & (~(1 << 2)) );  //LED ON, RW = 0, RS = 0, EN =0;


	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_en_on, 1, 100);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_en_off, 1, 100);

	uint8_t command_lsb_en_on 	= (( (data << 4 ) & 0xF0 ) | 0X0C); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t command_lsb_en_off 	= ( command_lsb_en_on & (~(1 << 2)) );  //LED ON, RW = 0, RS = 0, EN =0;

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_lsb_en_on, 1, 100);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_lsb_en_off, 1, 100);

}


void i2c_transmit_data(uint8_t data)
{


	uint8_t data_en_on 	= ( (data & 0xF0) | 0x0D); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t data_en_off 	= ( data_en_on & (~(1 << 2)) );  //LED ON, RW = 0, RS = 0, EN =0;

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_en_on, 1, 100);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_en_off, 1, 100);

	uint8_t data_lsb_en_on 	= (( (data << 4 ) & 0xF0 ) | 0X0D); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t data_lsb_en_off 	= ( data_lsb_en_on & (~(1 << 2)) );  //LED ON, RW = 0, RS = 0, EN =0;

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_lsb_en_on, 1, 100);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_lsb_en_off, 1, 100);

}

void LCD_Send_String(char *str)
{
	while (*str)
	{
		i2c_transmit_data(*str++);
	}
}




#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1

uint8_t RHI, RHD, TCI, TCD, SUM;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;
uint8_t TFI = 0;
uint8_t TFD = 0;
char strCopy[15];

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  while (__HAL_TIM_GET_COUNTER(&htim6) < delay);
}

uint8_t DHT11_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
  HAL_Delay(20);   // wait for 20ms
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT11_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
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
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  DS1307_SetTime(18, 37, 4);
  HAL_TIM_Base_Start(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  if(DHT11_Start())

	      {
		  //i2c_transmit_command(LCD_CLEAR_DISPLAY);
	        RHI = DHT11_Read(); // Relative humidity integral
	        RHD = DHT11_Read(); // Relative humidity decimal
	        TCI = DHT11_Read(); // Celsius integral
	        TCD = DHT11_Read(); // Celsius decimal
	        SUM = DHT11_Read(); // Check sum
	        if (RHI + RHD + TCI + TCD == SUM)
	        {
	          // Can use RHI and TCI for any purposes if whole number only needed
	          tCelsius = (float)TCI + (float)(TCD/10.0);
	          tFahrenheit = tCelsius * 9/5 + 32;
	          RH = (float)RHI + (float)(RHD/10.0);
	          // Can use tCelsius, tFahrenheit and RH for any purposes
	          TFI = tFahrenheit;  // Fahrenheit integral
	          TFD = tFahrenheit*10-TFI*10; // Fahrenheit decimal
			  printf("Temperature 	: %.2f\n\r",tCelsius);
			  printf("Humidity	: %.2f\n\r",RH);

			  sprintf(tCelsius_str,"%.2f",tCelsius);
			  sprintf(RH_str,"%.2f",RH);

			//  i2c_transmit_command(LCD_CLEAR_DISPLAY);
			  i2c_transmit_command(LCD_SET_ROW1_COL1); // Force cursor to begin on 1st row
				  LCD_Send_String("T:");
				  LCD_Send_String(tCelsius_str);
				  LCD_Send_String(" RH:");
				  LCD_Send_String(RH_str);
	      }
	      }
	 // i2c_transmit_command(LCD_CLEAR_DISPLAY);
      DS1307_GetTime(&hour, &minute, &second);


	  sprintf(sec,"%02u",second);
	  sprintf(min,"%02u",minute);
	  sprintf(hr,"%02u",hour);
	 // i2c_transmit_command(LCD_CLEAR_DISPLAY);

	  i2c_transmit_command(LCD_SET_ROW2_COL1); // Force cursor to begin on 1st row

	  LCD_Send_String("Time: ");
	  LCD_Send_String(hr);
	  LCD_Send_String(":");
	  LCD_Send_String(min);
		  LCD_Send_String(":");
		  LCD_Send_String(sec);


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
  htim6.Init.Prescaler = 55-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
