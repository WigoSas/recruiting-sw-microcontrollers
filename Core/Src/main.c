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
#include<stdbool.h>
#include<stdio.h>
#include<string.h>
#include <time.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
  APP_INIT, APP_WAIT_REQUEST, APP_LISTENING, APP_PAUSE, APP_WARNING, APP_ERROR
} App_State;

typedef enum{
  FILTER_NONE, FILTER_RAW, FILTER_AVG, FILTER_RND
} Filter_Type;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 30
#define RECORDABLE_VALUES 150

#define CHECK(res) if(res!=HAL_OK) State = APP_ERROR;
#define BREAK_IF_ERROR() if(State==APP_ERROR) break;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint32_t Analog_Hall = 0;
uint32_t Analog_Result = 0;
uint32_t Analog_Array[RECORDABLE_VALUES] = {0};
uint8_t Analog_Array_Length = 0;
uint8_t Analog_Array_Index = 0;
uint32_t Analog_Array_Sum = 0;

uint32_t Digital_Hall = 0;
uint8_t Digital_Result = 0;
uint32_t Digital_Array[RECORDABLE_VALUES] = {0};
uint8_t Digital_Array_Length = 0;
uint8_t Digital_Array_Index = 0;
uint32_t Digital_Array_Sum = 0;

uint8_t input_buffer[BUFFER_SIZE] = {0};
uint8_t command[BUFFER_SIZE] = {0};
uint8_t command_idx = 0;

App_State State = APP_INIT;

bool is_adc_ready = false;
bool request_state_change = false;

Filter_Type filter = FILTER_NONE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void resetSensorValues(void);
void randomFilter(void);
void movingAverage(void);
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
  State = APP_INIT;
  srand(time(NULL));
  

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    switch (State)
    {
    case APP_INIT:
      for(int i=0; i<BUFFER_SIZE; i++) input_buffer[i]=0;
      for(int i=0; i<BUFFER_SIZE; i++) command[i]=0;
      command_idx = 0;

      is_adc_ready = false;
      request_state_change = false;
      filter = FILTER_NONE;

      State = APP_WAIT_REQUEST;
      CHECK(HAL_UARTEx_ReceiveToIdle_IT(&huart2,input_buffer,sizeof(input_buffer)));
      break;

    case APP_WAIT_REQUEST:
      if(request_state_change){ 
        State = APP_LISTENING;
        request_state_change = false;
        resetSensorValues();
        CHECK(HAL_UART_Abort(&huart2) );
      }
      break;

    case APP_LISTENING:
      //DMA conversion is not continuous, must be called every cycle
      CHECK(HAL_ADC_Start_DMA(&hadc1, &Analog_Hall, 1));
      BREAK_IF_ERROR();
      while(!is_adc_ready);

      switch (filter)
      {
      case FILTER_AVG:
        movingAverage();
        break;

      case FILTER_RND:
        randomFilter();
        break;

      case FILTER_RAW:
        Analog_Result = Analog_Hall;
        Digital_Result = Digital_Hall;
        break;
    
      default:
        State = APP_ERROR;
        break;
      }
      BREAK_IF_ERROR();

      char buf[50] = {0};
      snprintf(buf,50,"%lu\t%d\n",Analog_Result,Digital_Result);
      CHECK(HAL_UART_Transmit(&huart2,(unsigned char *)buf,strlen(buf),20));
      is_adc_ready = false;
      HAL_Delay(5);

      if(request_state_change){
        request_state_change = false;
        State = APP_PAUSE;
        CHECK(HAL_UARTEx_ReceiveToIdle_IT(&huart2,input_buffer,sizeof(input_buffer)));
      }
      break;
    
    case APP_PAUSE:
      if(request_state_change){
        request_state_change = false;
        State = APP_LISTENING;
        resetSensorValues();
        CHECK(HAL_UART_Abort(&huart2) );
      }
      break;

    case APP_WARNING:
      HAL_UART_Transmit(&huart2,(unsigned char *)"WARNING\r\n",10,30);
      HAL_Delay(250);
      if(request_state_change){
        request_state_change = false;
        State = APP_WAIT_REQUEST;
        CHECK(HAL_UARTEx_ReceiveToIdle_IT(&huart2,input_buffer,sizeof(input_buffer)));
        resetSensorValues();
      }
      break;

    case APP_ERROR:
      HAL_UART_Transmit(&huart2,(unsigned char *)"ERROR\r\n",8,30);
      HAL_Delay(250);
      if(request_state_change){
        request_state_change = false;
        State = APP_INIT;
      }
      break;
    
    default:
      break;
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//to know that the ADC conversion is over
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  is_adc_ready = true;
}

// used to set the digital hall value
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
  if(GPIO_Pin == GPIO_PIN_7){
    Digital_Hall = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) == GPIO_PIN_RESET ? 0 : 100;
  }
  else if(GPIO_Pin == B1_Pin){
    /*if(State == APP_ERROR) HAL_NVIC_SystemReset();
    else request_state_change = true; */ // idk if the request was this
    request_state_change = true;
  }
  
}

//
// used for serial input from terminal, it works whether input is sent all at once
// or char by char like on a terminal
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

  for(int i=0; i<Size; i++){
    uint8_t ch = input_buffer[i];

    if(ch == '\0' || ch == '\n' || ch == '\r' || command_idx == BUFFER_SIZE-1){ // accept command

      command[command_idx] = '\0';
      if(strncmp("raw",(char *)command,4) == 0) filter = FILTER_RAW;
      else if(strncmp("moving average",(char *)command,15) == 0) filter = FILTER_AVG;
      else if(strncmp("random noise",(char *)command,13) == 0)filter = FILTER_RND;
      else filter = FILTER_NONE;
      //to let people know what they typed
      char buf[50] = {0};
      snprintf(buf,50,"Received!: %s\n",command);
      HAL_UART_Transmit(&huart2,(unsigned char*)buf,strlen(buf),50);
      
      command_idx = 0;
      for(int i=0; i<BUFFER_SIZE; i++) command[i]=0;
      break;

    } else if(ch == 127){ // see if input is DELETE
      command_idx = command_idx>1 ? command_idx-1 : 0;

    } else if(ch > 31 && ch < 127){ // printable char, so copy it 
      command[command_idx] = ch;
      command_idx++;
    } // else ignore
  }
  HAL_UARTEx_ReceiveToIdle_IT(huart, input_buffer,BUFFER_SIZE);
}


void resetSensorValues(void){
  Analog_Hall = 0;
  Analog_Result = 0;
  for(int i=0; i<RECORDABLE_VALUES; i++) Analog_Array[i] = 0;
  Analog_Array_Length = 0;
  Analog_Array_Index = 0;
  Analog_Array_Sum = 0;

  Digital_Hall = 0;
  Digital_Result = 0;
  for(int i=0; i<RECORDABLE_VALUES; i++) Digital_Array[i] = 0;
  Digital_Array_Length = 0;
  Digital_Array_Index = 0;
  Digital_Array_Sum = 0;
}

void randomFilter(void){
  //10% variation of analog results
  uint16_t val = rand()%400;
  if(val >= 200) {
    uint16_t new_value = Analog_Hall+val-200;
    Analog_Result = 4095 >= new_value ? new_value : 4095;
  }
  else {
    val = 200 - val;
    Analog_Result = Analog_Hall >= val ? Analog_Hall - val : 0;
  }

  if(val == 399) Digital_Result = 100;
  else if(val == 0) Digital_Result = 0;
  else Digital_Result = Digital_Hall;
}


void movingAverage(void){
  //Eliminate the 151th last value, since ArrayIdx points to the next 
  //value to be removed. Even if a 151th value doesn't exist, the subtraction
  // doesn't harm because for it to not have such value, it must be reset
  // If it was reset then all values forward are zero
  Analog_Array_Sum -= Analog_Array[Analog_Array_Index]; 
  Analog_Array_Sum += Analog_Hall;
  Analog_Array[Analog_Array_Index] = Analog_Hall;
  Analog_Array_Index = (Analog_Array_Index+1)%RECORDABLE_VALUES;
  if(Analog_Array_Length < RECORDABLE_VALUES) Analog_Array_Length++;
  Analog_Result = Analog_Array_Sum / Analog_Array_Length;

  Digital_Array_Sum -= Digital_Array[Digital_Array_Index];
  Digital_Array_Sum += Digital_Hall;
  Digital_Array[Digital_Array_Index] = Digital_Hall;
  Digital_Array_Index = (Digital_Array_Index+1)%RECORDABLE_VALUES;
  if(Digital_Array_Length < RECORDABLE_VALUES) Digital_Array_Length++;
  Digital_Result = Digital_Array_Sum / Digital_Array_Length;
}
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
