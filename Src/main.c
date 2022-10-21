/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define D10_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0)
#define D10_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1)

#define D11_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0)
#define D11_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1)

#define D12_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0)
#define D12_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1)

#define D13_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0)
#define D13_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern int data_display;
extern int i;
extern uint8_t A0_flag;
uint8_t time_data[32];
uint8_t data_send[16];
char str[7];
unsigned char dot;

void PrintSegments(unsigned int data);
void WriteByte(char _byte);

void VirtualPort(unsigned int data) {
	unsigned int temp_data, i;

	for (i = 0; i < 4; i++) {
		temp_data = ~(data >> i) & 0x01;
		switch (i) {
		case 0:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, temp_data);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, temp_data);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, temp_data);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, temp_data);
			break;
		}
	}
}

extern void VirtualPortClear() {
	D10_OFF;
	D11_OFF;
	D12_OFF;
	D13_OFF;
}

/* Virtual SPI 595
 * STM   -->         <-- Board
 * PA9  - D8  Data   D8 14 - Ds serial data input
 * PA8  - D7  CLK	 D7 11 - SHcp shift register clock input
 * PB5  - D4  STcp   D4 12 - STcp storage register input
 */

uint8_t RTC_ConvertFromDec(uint8_t c)

{
	uint8_t ch = ((c >> 4) * 10 + (0x0F & c));
	return ch;
}

uint8_t RTC_ConvertFromBinDec(uint8_t c) {
	uint8_t ch = ((c / 10) << 4) | (c % 10);
	return ch;
}

void SetTime(uint8_t hour, uint8_t min, uint8_t sec) {
	uint8_t tx[1];
	tx[0] = RTC_ConvertFromBinDec(sec);
	HAL_I2C_Mem_Write(&hi2c2, 0xD0, 0x00, 1, tx, 1, 1000);
	tx[0] = RTC_ConvertFromBinDec(min);
	HAL_I2C_Mem_Write(&hi2c2, 0xD0, 0x01, 1, tx, 1, 1000);
	tx[0] = RTC_ConvertFromBinDec(hour);
	HAL_I2C_Mem_Write(&hi2c2, 0xD0, 0x02, 1, tx, 1, 1000);
}

void Command(char c) {
	switch (c) {
	case '4':
		VirtualPort(1 << 0);
		break;
	case '3':
		VirtualPort(1 << 1);
		break;
	case '2':
		VirtualPort(1 << 2);
		break;
	case '1':
		VirtualPort(1 << 3);
		break;
	case '0':
		VirtualPortClear();
		break;
	}
}

int Get_int(char t) {
	return t - 48;
}

void Command_Handler(char *s) {
	switch (s[0]) {
	case 'l':
		Command(s[1]);
		break;
	case 't':
		SetTime(Get_int(s[1]) * 10 + Get_int(s[2]),
				Get_int(s[3]) * 10 + Get_int(s[4]),
				Get_int(s[5]) * 10 + Get_int(s[6]));
		break;
	}
}

void PrintTime(void) {
	char rx[5];

	for (int i = 0; i < 3; i++) {
		HAL_I2C_Mem_Read(&hi2c2, 0xD0, i, 1, time_data, 1, 1000);
		rx[i] = time_data[0];
	}

	data_display = RTC_ConvertFromDec(rx[2]) * 100 + RTC_ConvertFromDec(rx[1]);

}

void SetA1_Button(uint8_t set) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	if (set == 1) {
		GPIO_InitStruct.Pin = A1_Button_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
	} else if (set == 0) {
		GPIO_InitStruct.Pin = A1_Button_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
	}
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SPI2_Init();
	MX_TIM6_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim6);

	VirtualPortClear();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	dot = 0;

	while (1) {
		if (A0_flag == 1) {
			SetA1_Button(A0_flag);
			dot = 0;
			data_display = 0;

			if (HAL_GPIO_ReadPin(A1_Button_GPIO_Port, A1_Button_Pin) == 0) {
				VirtualPort(1<<0);
			}
			if (HAL_GPIO_ReadPin(A2_Button_GPIO_Port, A2_Button_Pin) == 0) {
				VirtualPort(1<<1);
			}
			if (HAL_GPIO_ReadPin(A3_Button_GPIO_Port, A3_Button_Pin) == 0) {
				A0_flag = 0;
				VirtualPort(1<<2);
				SetA1_Button(A0_flag);
				HAL_Delay(500);
				VirtualPortClear();
			}

		} else {
			PrintTime();
			dot ^= 1;
			HAL_Delay(250);
			if (huart2.RxXferCount == 0) {
				str[7] = 0;
				HAL_UART_Receive_IT(&huart2, (uint8_t*) str, 7);
				Command_Handler(str);
			}
		}

		//			sprintf((char*) data_send, "%s", "A0_flag");
		//			HAL_UART_Transmit(&huart2, data_send, 7, 0xFFFF);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x20303E5D;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 10500;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			LD2_Pin | GPIO_PIN_6 | GPIO_PIN_7 | CLK_Pin | Data_Pin | GPIO_PIN_10,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, STcp_Pin | GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : A1_Button_Pin */
	GPIO_InitStruct.Pin = A1_Button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(A1_Button_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : A2_Button_Pin */
	GPIO_InitStruct.Pin = A2_Button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(A2_Button_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA6 PA7 CLK_Pin
	 Data_Pin PA10 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_6 | GPIO_PIN_7 | CLK_Pin | Data_Pin
			| GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : A3_Button_Pin */
	GPIO_InitStruct.Pin = A3_Button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(A3_Button_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : STcp_Pin PB6 */
	GPIO_InitStruct.Pin = STcp_Pin | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
