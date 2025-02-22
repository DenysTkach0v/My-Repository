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
uint8_t ds_buff[9];                  //сюда принимаем данные
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define PORT  GPIOB   //указать порт, к которому подключены датчики
#define PIN 9
#define TIMER TIM3    //задаем таймер, используемый для формирования задержек
#define SKIP_ROM_ADR 0xCC // пропуск адресации
#define CONVERT_TEMP 0x44 // запускает преобразование температуры
#define READ_DATA_COMAND 0xBE //команда чтения содержимого памяти

 uint16_t	temp;
// uint32_t temp;
 char znak =  '-' ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//*********************************************************************************************
//function  импульс сброса                                                                   //
//argument  маска порта                                                                      //
//return    0 - устройство обнаружен, 1 - не обнаружено, 2 - к.з. на линии                   //
//*********************************************************************************************
uint8_t ds_reset_pulse(uint16_t PinMask)
{
   uint16_t result;

   if((PORT->IDR & PinMask)==0)  return 2;         //проверить линию на отсутствие замыкания
   PORT->ODR &= ~PinMask;                          //потянуть шину к земле
   TIMER->CNT=0;
   while(TIMER->CNT<480){};                        //ждать 480 микросекунд
   PORT->ODR |=  PinMask;                          //отпустить шину
   while(TIMER->CNT<550){};                        //ждать 70 микросекунд
   result =  PORT->IDR & PinMask;                 //прочитать шину
   while(TIMER->CNT<960){};                        //дождаться окончания инициализации
   if(result) return 1;                            //датчик не обнаружен
   return 0;                                       //датчик обнаружен
}


//*********************************************************************************************
//function  передача бита                                                                    //
//argument  значение передаваемого бита,маска порта                                          //
//return    none                                                                             //
//*********************************************************************************************
void ds_write_bit(uint8_t bit,uint16_t PinMask)
{
   TIMER->CNT=0;
   PORT->ODR &= ~PinMask;                          //потянуть шину к земле
   while(TIMER->CNT<2){};                          //ждать 1 микросекунду
   if(bit) PORT->ODR |=  PinMask;                  //если передаем 1, то отпускаем шину
   while(TIMER->CNT<60){};                         //задержка 60 микросекунд
   PORT->ODR |=  PinMask;                          //отпускаем шину
}

//*********************************************************************************************
//function  чтение бита                                                                      //
//argument  маска порта                                                                      //
//return    прочитанный бит                                                                  //
//*********************************************************************************************
uint16_t ds_read_bit(uint16_t PinMask)
{
   uint16_t result;

  TIMER->CNT=0;
   PORT->ODR &= ~PinMask;                          //потянуть шину к земле
   while(TIMER->CNT<2){};
   PORT->ODR |=  PinMask;                          //отпускаем шину
   while(TIMER->CNT<15){};                         //задержка 15 микросекунд
   result     =  PORT->IDR & PinMask;              //прочитать шину
   while(TIMER->CNT<60){};                         //оставшееся время
   return result;                                  //возвратить результат
}
//*********************************************************************************************
//function  запись байта                                                                     //
//argument  передаваемый байт,маска порта                                                    //
//return    none                                                                             //
//*********************************************************************************************
void ds_write_byte(uint8_t byte, uint16_t PinMask)
{
   uint8_t i;
   for(i=0;i<8;i++) ds_write_bit(byte&(1<<i), PinMask);
}
//*********************************************************************************************
//function  чтение байта                                                                     //
//argument  маска порта                                                                      //
//return    прочитанный байт                                                                 //
//*********************************************************************************************
uint8_t ds_read_byte(uint16_t PinMask)
{
   uint8_t i,result = 0;
   for(i=0;i<8;i++)
   if(ds_read_bit(PinMask)) result |= 1<<i;
   return result;
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

ds_reset_pulse(1<<PIN);          //послать импульс сброса                                       /
ds_write_byte(SKIP_ROM_ADR,1<<PIN);//разрешить доступ к датчику не используя адрес
ds_write_byte(CONVERT_TEMP,1<<PIN);      //запустить преобразование
HAL_Delay (1000);               //дать время для измерения
ds_reset_pulse(1<<PIN);          //послать импульс сброса
ds_write_byte(SKIP_ROM_ADR,1<<PIN);//разрешить доступ к датчику не используя адрес
ds_write_byte(READ_DATA_COMAND,1<<PIN);      //команда, заставляющая датчик выдать 9 байт своей памяти
for(int i=0; i<9; i++ )           //прочитать 9 байт в массив
ds_buff[i] = ds_read_byte(1<<PIN);
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//  ds_buff[1]=255;//255 проверка отрицательных температур
//	ds_buff[0]=94;//94//248

//		temp = ds_buff[1];
//		temp = temp<<8;
//		temp |= ds_buff[0];
//		//переводим в градусы
//		temp *= 0.0625;//0.0625 градуса на единицу данных
//		if(temp > 1000)//если температура <0    1111 1111 1111 1000 = 65528
//	{
//		temp = 4096 - temp;
//	  znak =  '-';
//	}else znak =  '+';


		temp = ds_buff[1];
		temp = temp<<8;
		temp |= ds_buff[0];
		temp =temp>>4;//отбрасываем дробную чать
		if(temp > 1000)//если температура <0
	{
		temp = 4096 - temp;
	  znak =  '-';
	}else znak =  '+';
	HAL_UART_Transmit(&huart2, temp, 27, 10);
	HAL_Delay(1000);


/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
