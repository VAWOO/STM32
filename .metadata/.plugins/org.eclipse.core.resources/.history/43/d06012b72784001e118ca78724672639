/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "eth.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define LCD_ADDR (0x27 << 1)
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
extern void I2C_Scan(void);
void init();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 500);

	return len;
}
int _io_putchar(int ch)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1000);
	return ch;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char showTime[30] = {0};
char ampm[2][3] = {"AM", "PM"};

void get_time(void)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	HAL_RTC_GetTime(&hrtc,&sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&sDate, RTC_FORMAT_BIN);

	sprintf((char*)showTime, "%02d:%02d:%02d         ", sTime.Hours, sTime.Minutes, sTime.Seconds);
}


uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;

void delay_us(uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);              // ???ï¿½ï¿½ë¨¸ï¿½?? 0?ï¿½ï¿½ï¿?????????? ì´ˆê¸°?ï¿½ï¿½
	while((__HAL_TIM_GET_COUNTER(&htim1))<time);   // ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½ê°„ê¹Œï¿?????????? ??ï¿??????????
}
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void DHT11_Start (void)
{
	Set_Pin_Output (DHT11_GPIO_Port, DHT11_Pin);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_GPIO_Port, DHT11_Pin, 0);   // pull the pin low
	delay_us(18000);   // wait for 18ms
	Set_Pin_Input(DHT11_GPIO_Port, DHT11_Pin);    // set as input
}
uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	delay_us (40);
	if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)))
	{
		delay_us (80);
		if ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin))) Response = 1;
		else Response = -1;
	}
	while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)));   // wait for the pin to go low

	return Response;
}
uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)));   // wait for the pin to go high
		delay_us (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)));  // wait for the pin to go low
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);

  init();
  int count = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  int return_val = HAL_UART_Receive(&huart3, (uint8_t *)&data, sizeof(data), 500);
//
//	  printf("Receive : %d", return_val);
//
////	  if(0)
////	  {
////		  HAL_UART_Transmit(&huart3, (uint8_t *)&data, sizeof(data), 500);
////		  HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
////	  }
//
////	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
////	  {
////		  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, 1);
////		  HAL_Delay(500);
////		  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, 0);
////		  HAL_Delay(500);
////		  printf("USART print test count = %d\r\n", count++);
////	  }
//
//
//
//  }

  double temperature = 0;

  //HAL_ADC_Start(&hadc1);

  printf("Hello World !!\r\n");



  while(1)
  {

//    HAL_ADC_Start(&hadc1);
//    RTC_TimeTypeDef sTime;
//
//    if(HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
//    {
//      uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
//      temperature = adc_value * 3.3/ 0xfff;
//      temperature = (temperature-0.76)/0.0025 + 25.0;
//      //printf("Temperature : %f\r\n", temperature);
//    }

    	DHT11_Start();
	  Presence = DHT11_Check_Response();
	  Rh_byte1 = DHT11_Read ();
	  Rh_byte2 = DHT11_Read ();
	  Temp_byte1 = DHT11_Read ();
	  Temp_byte2 = DHT11_Read ();
	  SUM = DHT11_Read();

	  TEMP = Temp_byte1;
	  RH = Rh_byte1;

	  Temperature = (float) TEMP;
	  Humidity = (float) RH;

	  char buf[30];
	  sprintf(buf, "%3.1f, %2.1f        ", Temperature, Humidity);

	  printf(buf);
	  printf("\r\n");
	  HAL_Delay(500);

//	  delay_us(100000);
//	  printf("%d",count++);
//	  printf(" 100ms\r\n");
	  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, 1);
	  delay_us(10);
	  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, 0);
	  delay_us(10);

	  get_time();

	    LCD_SendCommand(LCD_ADDR, 0b10000000);
	    LCD_SendString(LCD_ADDR, buf);

//	    // set address to 0x40
	    LCD_SendCommand(LCD_ADDR, 0b11000000);
	    LCD_SendString(LCD_ADDR, showTime);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
