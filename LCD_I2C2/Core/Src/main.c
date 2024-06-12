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
#define I2C_SLAVE_ADDRESS      0x3F << 1
#define PIN_RS                 (1 << 0)
#define PIN_EN                 (1 << 2)
#define BACKLIGHT              (1 << 3)
/*LCD Commands*/
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void LCD_Init(void);
void LCD_Send_Cmd(uint8_t cmd);
void LCD_Send_String_On_Line2(char *str);
void LCD_Display_Long_Message(char *string);
void LCD_Send_String_On_Line1(char *str);
void LCD_Clear_Then_Display(char *str);
void LCD_Send_String(char *str);
void LCD_Send_Data(uint8_t data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LCD_Send_Cmd(uint8_t cmd)
{
	uint8_t data_u, data_l;
	uint8_t data_u_tx[2], data_l_tx[2];

	/*Store upper nibble*/
	data_u = (cmd & 0xF0);

	/*Store lower nibble*/
	data_l = ((cmd << 4) & 0xF0);

	/* Construct upper byte-> compatible for LCD*/
	data_u_tx[0] = data_u|BACKLIGHT|PIN_EN;
	data_u_tx[1] = data_u|BACKLIGHT;

	/* Construct lower byte-> compatible for LCD*/
	data_l_tx[0] = data_l|BACKLIGHT|PIN_EN;
	data_l_tx[1] = data_l|BACKLIGHT;

	/* Transmit upper nibble using I2C APIs*/
	if(HAL_I2C_IsDeviceReady(&hi2c1, I2C_SLAVE_ADDRESS, 1, 10) == HAL_OK)
		HAL_I2C_Master_Transmit(&hi2c1,I2C_SLAVE_ADDRESS, data_u_tx, 2, 100);

	/*Provide a delay */
	HAL_Delay(5);

	/* Transmit lower nibble using I2C APIs*/
	if(HAL_I2C_IsDeviceReady(&hi2c1, I2C_SLAVE_ADDRESS, 1, 10) == HAL_OK)
		HAL_I2C_Master_Transmit(&hi2c1,I2C_SLAVE_ADDRESS, data_l_tx, 2, 100);

	/*Provide a delay */
	HAL_Delay(5);
}

void LCD_Send_Data(uint8_t data)
{
	uint8_t data_u, data_l;
	uint8_t data_u_tx[2], data_l_tx[2];

	/*Store upper nibble*/
	data_u = (data & 0xF0);

	/*Store lower nibble*/
	data_l = ((data << 4) & 0xF0);

	/* Construct upper byte-> compatible for LCD*/
	data_u_tx[0] = data_u|BACKLIGHT|PIN_EN|PIN_RS;
	data_u_tx[1] = data_u|BACKLIGHT|PIN_RS;

	/* Construct lower byte-> compatible for LCD*/
	data_l_tx[0] = data_l|BACKLIGHT|PIN_EN|PIN_RS;
	data_l_tx[1] = data_l|BACKLIGHT|PIN_RS;

	/* Transmit upper nibble using I2C APIs*/
	if(HAL_I2C_IsDeviceReady(&hi2c1, I2C_SLAVE_ADDRESS, 1, 10) == HAL_OK)
		HAL_I2C_Master_Transmit(&hi2c1,I2C_SLAVE_ADDRESS, data_u_tx, 2, 100);

	/*Provide a delay */
	//HAL_Delay(1);

	/* Transmit lower nibble using I2C APIs*/
	if(HAL_I2C_IsDeviceReady(&hi2c1, I2C_SLAVE_ADDRESS, 1, 10) == HAL_OK)
		HAL_I2C_Master_Transmit(&hi2c1,I2C_SLAVE_ADDRESS, data_l_tx, 2, 100);

   /*Provide a delay */
   HAL_Delay(5);
}

void LCD_Init(void)
{
	/* Wait for 15ms */
	HAL_Delay(15);

	/*Function Set - As per HD44780U*/
	LCD_Send_Cmd(LCD_FUNCTION_SET1);

	/*Function Set -As per HD44780U*/
	LCD_Send_Cmd(LCD_FUNCTION_SET2);

	/*Set 4bit mode and 2 lines */
	LCD_Send_Cmd(LCD_4BIT_2LINE_MODE);

	/*Display on, cursor off*/
	LCD_Send_Cmd(0x0C);

	/* SET Row1 and Col1 (1st Line) */
	LCD_Send_Cmd(0x80);

	/*Clear Display*/
	LCD_Send_Cmd(LCD_CLEAR_DISPLAY);

}


void LCD_Send_String(char *str)
{
	while (*str)
	{
		LCD_Send_Data(*str++);
	}
}


/**
 * @brief Clears screen first, then displays the given string
 * @param str: pointer to strings
 * @retval None
 */
void LCD_Clear_Then_Display(char *str)
{
	LCD_Send_Cmd(LCD_CLEAR_DISPLAY);
	LCD_Send_String(str);
}



/**
 * @brief Display the strings on Line1
 * @param str: pointer to strings
 * @retval None
 */
void LCD_Send_String_On_Line1(char *str)
{
	LCD_Send_Cmd(LCD_SET_ROW1_COL1);
	LCD_Send_String(str);
}


/**
 * @brief Display the strings on Line2
 * @param str: pointer to strings
 * @retval None
 */
void LCD_Send_String_On_Line2(char *str)
{
	LCD_Send_Cmd(LCD_SET_ROW2_COL1);
	LCD_Send_String(str);
}


/**
 * @brief Display long messages of any size on LCD
 * @param str: pointer to strings
 * @retval None
 */
void LCD_Display_Long_Message(char *string)
{
	int i =0, count =1, j=0;
	/*Clear display and Set position to Line1 start*/
	LCD_Send_Cmd(LCD_CLEAR_DISPLAY);
	LCD_Send_Cmd(LCD_SET_ROW1_COL1);

	while(string[i] != '\0')
	{
		LCD_Send_Data(string[i]);

		/*If we reach 1st Line end, then goto 2nd line start*/
		if(j>=15 && (count%2 == 1))
		{
			count++;
			LCD_Send_Cmd(LCD_SET_ROW2_COL1);
		}

		/*If we reach second line end, clear display start from line1 again*/
		if(j>=31 && (count%2 == 0))
		{
			count++;
			j=0;
			LCD_Send_Cmd(LCD_CLEAR_DISPLAY);
			LCD_Send_Cmd(LCD_SET_ROW1_COL1);
		}

		HAL_Delay(100);
		i++;
		j++;
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
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
	LCD_Init();

	LCD_Send_String("LCD on STM32F407");
	LCD_Send_Cmd(LCD_SET_ROW2_COL1);
	LCD_Send_String("Using I2C Module");
HAL_Delay(100);
	LCD_Display_Long_Message("This is a function to display long messages on 16x02 LCD :-)");
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
