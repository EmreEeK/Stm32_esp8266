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
#include "string.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const char API_KEY[] = "RZCYFBZ8JC6HWCMU";

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


uint8_t dhtval[2];
uint8_t rxTemBuf[1]; 		/// receiver byte buffer
uint8_t Buf[200];			/// temporary buffer

uint8_t Buf_pc[200];
uint8_t	rxTemBuf_pc[1];

uint8_t idx = 0;				/// buffer index
uint8_t idx_pc =0;

uint32_t Holder=0;
uint32_t uart_interrupt_counter =0;

uint8_t sending=0;
uint8_t Esp_will_answer = 0;
uint8_t Esp_answered=0;

char pc_receive_bool= 0;


void stm_to_esp(uint8_t* command){
    HAL_UART_Transmit(&huart1, command, strlen((char*)command), 100);
    while(1){
    	  if (Esp_answered==1){
    		HAL_UART_Transmit(&huart2, Buf, sizeof(Buf), 100);	//SEND RECEIVED DATA TO PC. YOU SHOULD SEE ANS=OK
    						  memset(Buf, 0, sizeof(Buf)); // Buffer reset
    						  idx = 0;
    						  Esp_answered=0;
    						  Esp_will_answer=0;
    		break;
    	  }
      }
}


void pc_to_web (void){
			dhtval[0] = atoi(Buf_pc);
			// sending =Buf[0];
			memset(Buf_pc, 0, sizeof(Buf_pc)); // Buffer reset
				  idx_pc = 0;


}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) /// UART interrupt callback function
{
	if(huart == &huart1)         // UART 1 interrupt receive
	{
			Buf[idx]=rxTemBuf[0];
			idx++;
			Holder=uwTick;
			Esp_will_answer = 1;
			HAL_UART_Receive_IT(&huart1, rxTemBuf, 1);   /// INTERRUPT CREATION
	}

	if(huart == &huart2)    /// UART 2 interrupt receive

	{
		Buf_pc[idx_pc]=rxTemBuf_pc[0];
		idx_pc++;
		Holder=uwTick;
		pc_receive_bool=1;
		HAL_UART_Receive_IT(&huart2, rxTemBuf_pc, 1);   /// INTERRUPT CREATION
	}
}


int main(void)
{
  /* USER CODE BEGIN 1 */
uint8_t buffLen, buffLen2, buffCnt = 0;

dhtval[0] = (uint8_t)0;
dhtval[1] = (uint8_t)0;
char sendBuff[64], sendBuff2[64];
uint8_t AT[] = "AT\r\n"; //=\"RedmiNN\",\"12312312338\"\r\n";  // büyük AT  RN OLMADAN CEVAP VERMıyo
uint8_t AT_RST[] = "AT+RST\r\n"; // AT+RST
uint8_t AT_GMR[] = "AT+GMR\r\n"; // AT+GMR
uint8_t AT_cwmode_sorgula[] = "AT+CWMODE?\r\n"; // AT+CWMODE
uint8_t AT_cwmode_station[] = "AT+CWMODE=1\r\n"; // STATION mode
uint8_t AT_CWLAP_wifitara[] = "AT+CWLAP\r\n"; // wifi tara
uint8_t AT_CWJAP[] = "AT+CWJAP=\"RedmiNN\",\"12312312338\"\r\n"; // telefon ağına baglan
uint8_t AT_CIFSR[] = "AT+CIFSR\r\n"; // ip sorgulama (galiba station modda çalışmıyo AP modda çalışıyor olabilir)
//uint8_t AT_flash_wifi[] = "AT+CWJAP_DEF=\"\""\r\n";  // hafızadaki wifi imiş
uint8_t AT_CWQAP[] = "AT+CWQAP\r\n"; // wifi bağlantısını kes
uint8_t teto[]="teto\r\n";

uint8_t AT_Chip_start[] = "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n";

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
  /* Enable the GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure the PA5 pin as output */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);




  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_SYSTICK_Config(SystemCoreClock / 1000); //HAL_SYSTICK_Config(SystemCoreClock /  1000);

  HAL_UART_Receive_IT(&huart1,rxTemBuf,1);
  HAL_UART_Receive_IT(&huart2, rxTemBuf_pc, 1);
///////////////////////////////////////////////////////////////////////////// ESP - CONFIG /////////////////////////
  //HAL_UART_Transmit(&huart1, AT_RST, sizeof(AT_RST), 100);  // ESP RESET
  //HAL_Delay(2000);
  //PC_transmit_answer();

  stm_to_esp(AT_CWQAP);
  //HAL_UART_Transmit(&huart1, AT_CWQAP, sizeof(AT_CWQAP), 100); /// to ESP
  stm_to_esp(AT_RST);
  //HAL_UART_Transmit(&huart1, AT_CWJAP, sizeof(AT_CWJAP), 100); /// to ESP
  //HAL_Delay(3000);
  stm_to_esp(AT);
  HAL_Delay(5000);

  //PC_transmit_answer();

  //HAL_UART_Transmit(&huart2, AT_Chip_start, sizeof(AT_Chip_start), 100);

  //PC_transmit_answer();
  char zartzurt =5; //
  char zartzaaaurt =53; //
  char first_time =1; /// first time char
  char first_block_waiting=0;
  char skip_initial = 0;
  uint32_t hold_tick =0;
  uint8_t aaaa=15;

  while (1)
  {
    /* USER CODE END WHILE */


	  if (abs((HAL_GetTick()-hold_tick >=20000)) || (first_time == 1) || (first_block_waiting ==1))
	  {
		  first_time=0;



		  if (skip_initial ==0)  ///////////////////////////////////////////////// INITIAL BLOCK ////////////////////////////
		  {
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);


	  	  stm_to_esp(AT_Chip_start);

	  	  first_block_waiting =1;
	  	  skip_initial =1;
	  	  hold_tick = HAL_GetTick();

		  }
	  	  ///HAL_Delay(1000);
		  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////




		  if (abs(HAL_GetTick()-hold_tick >=1000)) //////////////////// FIRST BLOCK /////////////////////////////////////
		  {

			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);



	  	  buffLen = sprintf(sendBuff, "GET /update?api_key=%s&field1=%d&field2=%d\r\n", API_KEY, dhtval[0], dhtval[1]);
	  	  buffLen2 = sprintf(sendBuff2, "AT+CIPSEND=%d\r\n", buffLen);
	  	  stm_to_esp((uint8_t *)sendBuff2);
	  	  stm_to_esp((uint8_t *)sendBuff);


	  	  dhtval[1] += 1;
	  	  // HAL_Delay(20000);




	  	  hold_tick = HAL_GetTick();
	  	  first_block_waiting = 0;
	  	  skip_initial =0;
		  }
		  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	 }




    /* USER CODE BEGIN 3 */
/*
//bağlantı kontrol /*
	  HAL_UART_Transmit(&huart1, AT_Chip_start, sizeof(AT_Chip_start), 3000);
	  HAL_UART_Receive(&huart1, rx_data, sizeof(rx_data), 3000);
	  memcpy(rx_copy, rx_data, sizeof rx_copy);
	  HAL_UART_Transmit(&huart2, rx_data, sizeof(rx_data), 3000);
	  HAL_Delay(3000);


	//gonderim
	  buffLen = sprintf(sendBuff, "GET /update?api_key=%s&field1=%d&field2=%d\r\n", API_KEY, dhtval[0], dhtval[1]);
	  buffLen2 = sprintf(sendBuff2, "AT+CIPSEND=%d\r\n", buffLen);


	  HAL_UART_Transmit(&huart1, (uint8_t *)sendBuff2, buffLen2, 3000);
	  HAL_UART_Receive(&huart1, rx_data, sizeof(rx_data), 3000);
	  memcpy(rx_copy, rx_data, sizeof rx_copy);
	  HAL_UART_Transmit(&huart2, rx_data, sizeof(rx_data), 3000);
	  HAL_Delay(3000);



	  HAL_UART_Transmit(&huart1, (uint8_t *)sendBuff, buffLen, 3000);
	  HAL_UART_Receive(&huart1, rx_data, sizeof(rx_data), 3000);
	  memcpy(rx_copy, rx_data, sizeof rx_copy);
	  HAL_UART_Transmit(&huart2, rx_data, sizeof(rx_data), 3000);



	  HAL_Delay(15000); // site bekletiyor thigspeaksin bekletmesi min 15 saniye
	  dhtval[0] += 1;
	  dhtval[1] += 1;



	  */
	  /*

	  HAL_UART_Transmit(&huart1, tx_data, sizeof(tx_data), 3000); // bekleme süresini düzgün yap
	  	     // HAL_UART_Receive(&huart1, rx_data, sizeof(rx_data), 500);
	  	     // HAL_UART_Transmit(&huart2, tx_data, sizeof(tx_data), 500); // Send data to ESP-01 board
	  	          HAL_UART_Receive(&huart1, rx_data, sizeof(rx_data), 3000); // Receive data from ESP-01 board
	  	          memcpy(yazici, rx_data, sizeof yazici);

	  	          HAL_UART_Transmit(&huart2, rx_copy, sizeof(rx_copy), 3000); // bilgisayara gönder
	  	          //HAL_UART_Receive(&huart2, yazici, sizeof(yazici), 1000);
	  	      // Do something with received data here

	  	      HAL_Delay(4000);

*/
	      // uarttaki delaya göre biraz daha düzgün AT+CWLAP sonuçları geliyor
	      //
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
