/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
#include <stdio.h>

//User code begin - To give delay in microsends
#include "dwt_stm32_delay.h"

//Note: Output of DHT11 is connected to PA7 i.e pin 7 of port A
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_0

//USART Initialization - for printing DHT11 data on terminal.
UART_HandleTypeDef huart2;

/* Private function prototypes - This is done for printf() function on serial terminal------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


void SystemClock_Config(void);  		//Function to set system clock configurations.
static void MX_GPIO_Init(void); 		//Function to set GPIO Pin configurations.
static void MX_USART2_UART_Init(void);	//Function to set USART2 configurations.

/* USER CODE BEGIN 0 - Functions related to DHT11 operations*/

//variables declared for Relative Humidity(RH), Temperature data storage
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t sum;

GPIO_InitTypeDef GPIO_InitStruct;

//To Configure PA5 as output
void set_gpio_output (void)
{
	/*Configure GPIO pin output: PA7 - to this DHT11 output is connected */
  GPIO_InitStruct.Pin = DHT11_PIN;                   //Pin5 is selected
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;         //Mode is output push-pull
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;        //GPIO clock frequency selected as low
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);             //Initialize Pin5 of GPIO Port A
}

//To Configure PA5 as input
void set_gpio_input (void)
{
  /*Configure GPIO pin input: PA2 */
  GPIO_InitStruct.Pin = DHT11_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

//Initialize DHT11
void DHT11_start (void)
{
	set_gpio_output ();  						// set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	DWT_Delay_us (18000);   					// wait for 18ms
	set_gpio_input ();   						// set as input for receiving data
}

//Check response from DHT11
void check_response (void)
{
	DWT_Delay_us (40);  							//wait for 40us
	if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))==0)	// read PA5 pin & check if the pin PA5 is low
	{
		DWT_Delay_us (80);							//wait for 80us
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))==1)
			{
				//printf("Check response ok");        //Check if the pin is high. If it is, than the response is ok.
			}
	}
	//This will totally be a delay of 120us and the pin should be high now.
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))==1);   // wait for the pin to go low
}

//Read Data from DHT11
uint8_t read_data (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high

		/* wait for 40 us. This is because the length of �0� bit is 26-28us
		 * and if the pin is high after 40us, it indicates that the bit is �1�.
		 * */
		DWT_Delay_us (40);   //This function is included from library dwt_stm32_delay.h to give delay in microseconds
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) == 0)   // if the pin is low
		{
			i&= ~(1<<(7-j));    // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}
/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
   /* Output a message on terminal using printf function */

   /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 - Initialize function to give delay in microseconds*/
  DWT_Delay_Init ();
  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("\n\r Entering in the while loop of code\n\r");
  while (1)
  {

	  DHT11_start ();            //Initialize DHT11
	  check_response ();         //Check response from DHT11
	  Rh_byte1 = read_data ();   //Read date i.e. Relative Humidity - first byte
	  Rh_byte2 = read_data ();   //Read date i.e. Relative Humidity - second byte
	  Temp_byte1 = read_data (); //Read date i.e. Temperature - first byte
	  Temp_byte2 = read_data (); //Read date i.e. Temperature - second byte
	  sum = read_data();         //Read checksum data

	  if (sum == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))    // if the data is correct
	  {
          printf("\n\rRelative Humidity value is: %c%c percentage",((Rh_byte1/10)+48),((Rh_byte1%10)+48));
          printf("\n\rTemperature value is: %c%c Celcius",((Temp_byte1/10)+48),((Temp_byte1%10)+48));
	  }
	  else
	  {
		  printf("\n\r Checksum not matched..Incorrect data from DHT11\n\r");
	  }
      HAL_Delay(1000); // This is delay function in HAL library . Take another reading after 1 second
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA5 - To connect DHT11 output */
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
     printf("\nInside Error_Hadler");
     while(1){}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
