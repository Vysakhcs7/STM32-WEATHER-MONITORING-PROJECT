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
#define	CLEAR_SCREEN			0X01	//Clear Display Screen
#define DISPLAY_CURSOR_ON		0x0E 	//Display ON, Cursor ON
#define CURSON_ROW_1			0x80	//Force cursor to begin on 1st row
#define CURSOR_ROW_2			0xC0	//Change position to 1st coloumn of 2nd row
#define delay_t					1

//#define LCD_FUNCTION_SET1      0x33
//#define LCD_FUNCTION_SET2      0x32
//#define LCD_4BIT_2LINE_MODE    0x28
//#define LCD_DISP_CURS_ON       0x0E
//#define LCD_DISP_ON_CURS_OFF   0x0C  //Display on, cursor off
//#define LCD_DISPLAY_OFF        0x08
//#define LCD_DISPLAY_ON         0x04
//#define LCD_CLEAR_DISPLAY      0x01
//#define LCD_ENTRY_MODE_SET     0x04
//#define LCD_INCREMENT_CURSER   0x06
//#define LCD_SET_ROW1_COL1      0x80  //Force cursor to beginning ( 1st line)
//#define LCD_SET_ROW2_COL1      0xC0  //Force cursor to beginning ( 2nd line)
//#define LCD_MOVE_DISPLAY_LEFT  0x18
//#define LCD_MOVE_DISPLAY_RIGHT 0x1C
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
const uint8_t slave_address = 0x3F;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void lcd_init(void);
void i2c_transmit_command(uint8_t data);
void i2c_transmit_data(uint8_t data);
void t_i2c_transmit_command(uint8_t data);
void t_i2c_transmit_data(uint8_t data);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lcd_init(void) {
	//i2c_transmit_command(0x02); // Initialize LCD in 4-bit mode
	//i2c_transmit_command(0x28); // 2 line, 5x7 matrix in 4-bit mode
	i2c_transmit_command(CLEAR_SCREEN);
	HAL_Delay(delay_t); // Wait for clear to complete
	i2c_transmit_command(DISPLAY_CURSOR_ON);
	i2c_transmit_command(CURSON_ROW_1);



}




void i2c_transmit_command(uint8_t data)
{

	uint8_t command_en_on 	= ( (data & 0xF0) | 0x0C); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t command_en_off 	= ( command_en_on & (~(1 << 2)) );  //LED ON, RW = 0, RS = 0, EN =0;

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_en_on, 1, HAL_MAX_DELAY);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_en_off, 1, HAL_MAX_DELAY);

	uint8_t command_lsb_en_on 	= (( (data << 4 ) & 0xF0 ) | 0X0C); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t command_lsb_en_off 	= ( command_lsb_en_on & (~(1 << 2)) );  //LED ON, RW = 0, RS = 0, EN =0;

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_lsb_en_on, 1, HAL_MAX_DELAY);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &command_lsb_en_off, 1, HAL_MAX_DELAY);

}

void t_i2c_transmit_command(uint8_t data)
{
	uint8_t k = '\0', cmd = '\0',  cmd2 = '\0',  cmd3 = '\0';

	k = data & 0xF0;
	cmd = (k | 0x08) & (~0x01);
	cmd2 = (cmd | 0x04);
	cmd3 = (cmd & (~0x04));

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &cmd2, 1, HAL_MAX_DELAY);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &cmd3, 1, HAL_MAX_DELAY);

	k = data << 4;
	cmd = (k | 0x08) & (~0x01);
	cmd2 = (cmd | 0x04);
	cmd3 = (cmd & (~0x04));

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &cmd2, 1, HAL_MAX_DELAY);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &cmd3, 1, HAL_MAX_DELAY);

}

void t_i2c_transmit_data(uint8_t data)
{

	uint8_t k = '\0', cmd = '\0',  cmd2 = '\0',  cmd3 = '\0';

	k = data & 0xF0;
	cmd =  k | 0x08 | 0x01;
	cmd2 = (cmd | 0x04);
	cmd3 = (cmd & (~0x04));

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &cmd2, 1, HAL_MAX_DELAY);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &cmd3, 1, HAL_MAX_DELAY);

	k = data << 4;
	cmd =  k | 0x08 | 0x01;
	cmd2 = (cmd | 0x04);
	cmd3 = (cmd & (~0x04));

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &cmd2, 1, HAL_MAX_DELAY);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &cmd3, 1, HAL_MAX_DELAY);

}

void i2c_transmit_data(uint8_t data)
{


	uint8_t data_en_on 	= ( (data & 0xF0) | 0x0D); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t data_en_off 	= ( data_en_on & (~(1 << 2)) );  //LED ON, RW = 0, RS = 0, EN =0;

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_en_on, 1, HAL_MAX_DELAY);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_en_off, 1, HAL_MAX_DELAY);

	uint8_t data_lsb_en_on 	= (( (data << 4 ) & 0xF0 ) | 0X0D); //LED ON, RW = 0, RS = 0, EN =1;
	uint8_t data_lsb_en_off 	= ( data_lsb_en_on & (~(1 << 2)) );  //LED ON, RW = 0, RS = 0, EN =0;

	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_lsb_en_on, 1, HAL_MAX_DELAY);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, slave_address << 1, &data_lsb_en_off, 1, HAL_MAX_DELAY);

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
  lcd_init();
  i2c_transmit_data('E');
  i2c_transmit_data('M');
  i2c_transmit_data('B');
  i2c_transmit_data('E');
  i2c_transmit_data('D');
  i2c_transmit_data('D');
  i2c_transmit_data('E');
  i2c_transmit_data('D');
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
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
