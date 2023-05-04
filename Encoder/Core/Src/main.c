/* USER CODE BEGIN Header */
/**
***************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define END_INICIAL 0x0800F000 // Pag. 60 | Max (pag 63): 0x0800FC00
#define AMOSTRA 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t buffer[70];
char EndReception='\0';
char RXBuffer[1];
char RXBufferArray[10];
char readStatus;
uint8_t i=0;
int16_t dados[4];
char errorFlag = 'H';

uint16_t timeTotal=0;

int32_t bufferPulso[AMOSTRA];

////

int32_t currentPulse[AMOSTRA];
volatile int32_t pulseCounter = 0;
int16_t pulseBuffer = 0;

int16_t previous = 0;
int16_t actual = 0;
volatile char USB_FLAG = 0;

uint16_t edgeTime = 0;
unsigned char readState = INIT;

///

struct Envio{
int inicio;
int endOrigem;
int endDestino;
int funcao;
int dadosCount;
int checksum;
int fim;
}Send_message;

struct Recepcao{
int inicio;
int endOrigem;
int endDestino;
int funcao;
int dadosCount;
int checksum;
int fim;
}receive_message;

struct structDevice{
int Id;
int Code;

}infoDevice;

union Pulso
{
int32_t all;
char pt[4];

};

union Pulso encoderPulso = {.all=0};


union Time
{
uint16_t all;
char pt[2];

};
union Time timeEncoder = {.all=0};



uint16_t c_sum=0;
uint8_t checksum=0, Total=0;
char StateMachine;
char uart_state;
int timerEnable = 0;
uint16_t currentTime[AMOSTRA];
uint16_t bufferTime = 0;;
uint8_t samples = 0;
volatile uint8_t direction=0;
static int16_t count=0;
volatile int16_t aux_count=0;
uint8_t deviceFlag = 0;
//int countEncoder=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int selectCommand(struct Recepcao receive_message);
char StartDevice(char deviceFlag);
char StopDevice(char deviceFlag);
void getID();
void procMeasurement();
//int getID(struct Recepcao receive_message,struct structDevice Device);
void TransmitData(struct Recepcao receive_message,int32_t *dataToSend,uint8_t dataLenght);
int Checksum(char *c);
void command(char*);
void DeviceParamenter(struct Recepcao message);
int  VerificaErro();
void structDados();
char deviceReset();
void delay(uint16_t delay);
void acquisition(void);

volatile uint32_t interruptCount = 0;

#define HZ_1000 10 /* 1x80*/
#define HZ_500  20 /* 2x80*/
#define HZ_200  50 /* 5x80*/
#define HZ_100  100 /* 10x80*/
#define HZ_50 	200 /* 20x80*/



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay(uint16_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim3,0);
	while(__HAL_TIM_GET_COUNTER(&htim3) < delay);
}

void acquisition(void)
{
	readState = INIT;
	edgeTime = 0;
	while(1)
	{
		if(readState == INIT)
		{
			pulseBuffer = 0;
			USB_FLAG = 0;
			__HAL_TIM_SET_COUNTER(&htim3,0);
			HAL_TIM_Base_Start(&htim3);
//			HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_1);
			HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
//			HAL_Delay(1);
			readState = COLETA_ALL;


		}

		if(readState == COLETA_ALL)
		{
			/*
			 * Cada tick do countador TIM3 equivale a 100us;
			 * Na funcao DeviceParamenter(), o valor recebido em dados[1] equivale a taxa de aquisição,
			 * sendo o valor mínimo de aquisição igual 1ms (Fmax = 1kHz, dado[1] = 1), realizo multiplicação
			 * dado[1] * 10 para que 1ms seja equivalente a 10 ticks do Timer3 (10 ticks x 100u = 1ms)
			 * e então o polling abaixo funcione;
			 */
			__HAL_TIM_SET_COUNTER(&htim3,0);
			while(__HAL_TIM_GET_COUNTER(&htim3) < timeTotal && samples<AMOSTRA);
			memcpy(&bufferTime,&(__HAL_TIM_GET_COUNTER(&htim3)),sizeof(uint16_t));
			memcpy(&pulseBuffer,&count,sizeof(int16_t));
//			pulseBuffer = pulseBuffer>>1; // Teste de apenas 1 canal
			pulseBuffer = pulseBuffer>>2; // aplicação
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//			actual = pulseBuffer - previous;
//			previous = pulseBuffer;




			bufferTime = bufferTime/10;
			edgeTime = bufferTime + edgeTime;

			currentPulse[samples] = pulseBuffer;
			currentTime[samples] = edgeTime;
			samples++;

			if(samples == AMOSTRA)
			{
				samples = 0;
				readState = TRANSMISSAO;
			}

		}


		if(readState == TRANSMISSAO)
		{
			TransmitData(receive_message,currentPulse,67);
			HAL_GPIO_TogglePin(GPIOA, ACQUISITION_Pin);
			EndReception = '\0';
			if(USB_FLAG == 1)
			{
				readState = COMUNICACAO;
			}
			else readState = COLETA_ALL;
		}
		if(readState == COMUNICACAO) break;

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
/*	CDC_Transmit_FS((uint8_t*)data, strlen(data));*/
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
HAL_Delay(500);
HAL_UART_Receive_IT(&huart2, (uint8_t *)RXBuffer, 1);
StateMachine = iddle;
char reset_status;
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB,STATUS_Pin, GPIO_PIN_SET);
char startFlag = 0;
char stopFlag = 1;
char i=0;

//TEMPO TESTE
//StateMachine = Start;
//readStatus = AUTO;
//timeTotal = HZ_50;
//timeTotal = HZ_1000;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

while (1)
{
  switch(StateMachine)
  {
	  case iddle:
		if(USB_FLAG == 1)
		{
			while(EndReception!=Fim) command(buffer);
			USB_FLAG = 0;
		}
		if(EndReception == Fim)
		{
			structDados();
			errorFlag = VerificaErro();
			if(errorFlag != OK)
			 {
				StateMachine = error;
			 }
			else
			{
				StateMachine = Inicio;
			}
		}
		else if(EndReception == Reset)
		{
			reset_status = deviceReset();
			if(reset_status == HAL_OK)
			{
				StateMachine = iddle;
				HAL_GPIO_WritePin(GPIOB,ERRO_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,STOP_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,CONFIG_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,ACQUISITION_Pin, GPIO_PIN_RESET);
				EndReception = '\0';
				USB_FLAG = 0;
			}
			else
			{
				StateMachine = iddle;
				EndReception = Reset;
			}

		}
		else if(USB_FLAG == 0 && EndReception == '\0') StateMachine = iddle;
	  break;

	  case Inicio:
				StateMachine = selectCommand(receive_message);
	  break;

	  case Identification:
			getID();
			TransmitData(receive_message,infoDevice.Id,8);
			StateMachine = iddle;
			EndReception = '\0';
			USB_FLAG = 0;
	  break;

	  case Codification:
			setID();
			TransmitData(receive_message,infoDevice.Id,8);
			StateMachine = iddle;
			EndReception = '\0';
			USB_FLAG = 0;
	  break;

	  case Config:
		DeviceParamenter(receive_message);
		HAL_GPIO_WritePin(GPIOB,CONFIG_Pin, GPIO_PIN_SET);
		TransmitData(receive_message,2,7);
		StateMachine = iddle;
		EndReception = '\0';
		USB_FLAG = 0;
	   break;

	  case Start:
		startFlag = StartDevice(deviceFlag);
		deviceFlag = startFlag;
		if(deviceFlag == 1)
		{
			HAL_GPIO_WritePin(GPIOB,STOP_Pin, GPIO_PIN_RESET);
			TransmitData(receive_message,0,7);
			StateMachine = Read;
			EndReception = '\0';
			USB_FLAG = 0;
		}
		else
		{
//	  			StateMachine = error;
			StateMachine = iddle;
			EndReception = '\0';
			USB_FLAG = 0;
		}

	  break;

	  case Read:
		  //TEMPO TESTE
//		 HAL_TIM_Base_Start(&htim3);
//		 while(1)
//		 {
//			 HAL_GPIO_TogglePin(GPIOB, CONFIG_Pin);
//			 delay(1000); //1tick = 1us 60000 ticks ->60ms
//		 }
		acquisition();
		HAL_GPIO_WritePin(GPIOB,CONFIG_Pin, GPIO_PIN_RESET);
		StateMachine = iddle;
	  break;

	  case ReadError:
		  memcpy(bufferPulso,currentPulse,sizeof(int32_t));
		HAL_GPIO_WritePin(GPIOA,ACQUISITION_Pin, GPIO_PIN_SET);
		TransmitData(receive_message,bufferPulso,67);
		HAL_GPIO_WritePin(GPIOA,ACQUISITION_Pin, GPIO_PIN_RESET);
		StateMachine = iddle;
		EndReception = '\0';
		USB_FLAG = 0;
	  break;

	  case Stop:
		stopFlag = StopDevice(deviceFlag);
		deviceFlag = stopFlag;
		if(stopFlag == 0)
		{
			HAL_GPIO_WritePin(GPIOB,STOP_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA,ACQUISITION_Pin, GPIO_PIN_RESET);
			TransmitData(receive_message,0,7);
			StateMachine = iddle;
			EndReception = '\0';
			USB_FLAG = 0;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB,STOP_Pin, GPIO_PIN_RESET);
			StateMachine = iddle;
			EndReception = '\0';
			USB_FLAG = 0;
		}
	   break;

	  case error:
		HAL_GPIO_WritePin(GPIOB,ERRO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,CONFIG_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,STOP_Pin, GPIO_PIN_RESET);
		TransmitData(receive_message,0,8);
		StateMachine = iddle;
		EndReception = '\0';
		USB_FLAG = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 4800-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65356-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STOP_Pin|ERRO_Pin|STATUS_Pin|CONFIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACQUISITION_GPIO_Port, ACQUISITION_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STOP_Pin ERRO_Pin STATUS_Pin CONFIG_Pin */
  GPIO_InitStruct.Pin = STOP_Pin|ERRO_Pin|STATUS_Pin|CONFIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ACQUISITION_Pin */
  GPIO_InitStruct.Pin = ACQUISITION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACQUISITION_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_Pin */
  GPIO_InitStruct.Pin = RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
char deviceReset()
{
char i=0;
char reset;
infoDevice.Id =0;
for(i=0;i<10;i++)
{
  bufferPulso[i] = 0;
  currentTime[i] = 0;
  currentPulse[i] = 0;
}
reset = StopDevice(HAL_OK);

return reset;
}


/*
 * 	000007 - 0A C0 D0 02 02 01 14 57 0F  - Configuração
	000009 - 0A C0 D0 02 02 01 0A 61 0F  - Configuração
	000011 - 0A C0 D0 02 02 01 05 66 0F  - Configuração
	000013 - 0A C0 D0 02 02 01 02 69 0F  - Configuração
	000015 - 0A C0 D0 02 02 01 01 6A 0F  - Configuração
 */
void structDados()
{
uint8_t counter=0;
receive_message.inicio = RXBufferArray[0];
receive_message.endOrigem = RXBufferArray[1];
receive_message.endDestino = RXBufferArray[2];
receive_message.funcao = RXBufferArray[3];
receive_message.dadosCount = RXBufferArray[4];
while(counter<receive_message.dadosCount)
{
	dados[counter] = RXBufferArray[5+counter];
	counter++;
}
receive_message.checksum = RXBufferArray[5+counter];
receive_message.fim=RXBufferArray[6+counter];
counter=0;
}

int VerificaErro()
{
if(receive_message.checksum != Checksum(RXBufferArray))
{
	return errorChecksum;
}
if(receive_message.endDestino>255 || receive_message.endDestino<0)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	return errorAddress;
}
/*	if(infoDevice.Id != 0x00 && infoDevice.Id != receive_message.endDestino)
{
	return errorAddress;
}*/
if(receive_message.funcao == 2 && receive_message.dadosCount < 0)
{
	return errorConfig;
}
if(receive_message.funcao > 0x08)
{
	return errorFunction;
}
if(receive_message.funcao == 2 && dados[1]<1)
{
	return errorData;
}
if(receive_message.inicio != 0x0A)
{
	return errorStartHeader;
}
if(receive_message.fim != 0x0f)
{
	return errorEndHeader;
}
return OK;

}

void command(char *ReceivedChar)
{

static int FirstData = 0;
/*	int ReceivedInt = *ReceivedChar - '\0';*/
if (FirstData == 0)
{
RXBufferArray[0] = ReceivedChar[0];
FirstData = 1;
uart_state = Origem;
}
else
{
if (uart_state != iddle)
{
  switch (uart_state) {
  case Origem:
		RXBufferArray[1] = ReceivedChar[1];
		uart_state = Destino;
	break;

  case Destino:
		RXBufferArray[2] = ReceivedChar[2];
		uart_state = Funcao;
	break;

  case Funcao:
		RXBufferArray[3] = ReceivedChar[3];
		uart_state = DadosCount;
	break;

  case DadosCount:
	if(ReceivedChar[4]>0)
	{
		RXBufferArray[4] = ReceivedChar[4];
		uart_state = Dados;
	}
	else if (ReceivedChar[4]==0)
	{
		RXBufferArray[4] = ReceivedChar[4];
		uart_state = CheckSum;
	}
	break;

  case Dados:
	  RXBufferArray[5+i] = ReceivedChar[5+i];
	  i++;
	  if(i<RXBufferArray[4])
	  {
		  uart_state = Dados;
	  }
	  else
	  {
		  uart_state = CheckSum;
	  }

	  break;
  case CheckSum:
	  RXBufferArray[5+i] = ReceivedChar[5+i];
	  checksum = Checksum(RXBufferArray);
	  uart_state = Fim;
  break;

  case Fim:
	  RXBufferArray[6+i] = ReceivedChar[6+i];
	  EndReception = Fim;
	  i=0;
	  FirstData = 0;
	  uart_state = iddle;
	break;
  default:
	uart_state = iddle;
  }
}
}
}

int selectCommand(struct Recepcao receive_message)

{
if (receive_message.funcao == 1)
 {
	  StateMachine = Identification;
 }
  else if (receive_message.funcao == 2)
  {
	  StateMachine = Config;
  }
  else if (receive_message.funcao == 3)
  {
	  StateMachine = Start;
  }
else if (receive_message.funcao == 4)
{
	  StateMachine = Read;
}
else if (receive_message.funcao == 5)
{
	  StateMachine = ReadError;
}
else if (receive_message.funcao == 6)
{
	  StateMachine = Stop;
}
else if (receive_message.funcao == 7)
{
	  StateMachine = Codification;
}
return StateMachine;
}

void getID()
{
	uint16_t codCefise=0;
	receive_message.dadosCount = 1;
	FLASH_le_16bits(END_INICIAL, &codCefise);
	if(codCefise != 0xFFFF)
	{
		infoDevice.Id = codCefise;
	}
}

void setID()
{
	uint16_t codCefise=0;
	receive_message.dadosCount = 1;
	FLASH_apaga(END_INICIAL, 1);
	codCefise = dados[0];
	infoDevice.Id = codCefise;
	FLASH_escreve_16bits(END_INICIAL, &codCefise);
}

void DeviceParamenter(struct Recepcao message)
{
timeTotal = dados[1] * 10;
switch(dados[0])
{
	case 0x01:
		readStatus = AUTO;
	break;
	case 0x02:
		readStatus = MAN;
	break;

}
}

char StartDevice(char deviceFlag)
{
char startEncoder;
if(deviceFlag == 0)
{
	startEncoder = HAL_OK;
	HAL_Delay(10);
	//
}
if(startEncoder == HAL_OK)
	return 1;
else return 0;
}

void TransmitData(struct Recepcao Send_message,int32_t *dataToSend, uint8_t dataLenght)
{
uint8_t txBuffer[dataLenght];
static uint8_t txBufferRead[67];
uint8_t errorBuffer[dataLenght];
uint8_t counter = 2;
uint8_t countArray = 0;
uint8_t counterAux = 0;
switch(StateMachine)
{
	case Identification:
		txBuffer[0] = Send_message.inicio;
		txBuffer[1] = Send_message.endDestino;
		txBuffer[2] = Send_message.endOrigem;
		txBuffer[3] = Send_message.funcao;
		txBuffer[4] = Send_message.dadosCount;
		if(Send_message.dadosCount>0)
		{
			txBuffer[5] = infoDevice.Id;
		}
		txBuffer[6] = Checksum(txBuffer);
		txBuffer[7] = Send_message.fim;
		CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));

	break;

	case Codification:
		txBuffer[0] = Send_message.inicio;
		txBuffer[1] = Send_message.endDestino;
		txBuffer[2] = Send_message.endOrigem;
		txBuffer[3] = 0x07;
		txBuffer[4] = Send_message.dadosCount;
		if(Send_message.dadosCount>0)
		{
			txBuffer[5] = infoDevice.Id;
		}
		txBuffer[6] = Checksum(txBuffer);
		txBuffer[7] = Send_message.fim;
		CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));

	break;
	case Config:
		txBuffer[0] = Send_message.inicio;
		txBuffer[1] = infoDevice.Id;
		txBuffer[2] = Send_message.endOrigem;
		txBuffer[3] = Send_message.funcao;
		txBuffer[4] = 0x00;
		txBuffer[5] = Checksum(txBuffer);
		txBuffer[6] = Send_message.fim;
		CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));

	break;
	case Start: // Msg de inicio de dispositivo
		txBuffer[0] = Send_message.inicio;
		txBuffer[1] = infoDevice.Id;
		txBuffer[2] = Send_message.endOrigem;
		txBuffer[3] = 0x03;
		txBuffer[4] = Send_message.dadosCount;
		txBuffer[5] = Checksum(txBuffer);
		txBuffer[6] = Send_message.fim;
		CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
	break;

	case Read:
		txBufferRead[0] = Send_message.inicio;
		txBufferRead[1] = infoDevice.Id;
		txBufferRead[2] = Send_message.endOrigem;
		txBufferRead[3] = 0x04;
		txBufferRead[4] = 0x3C;
		for(countArray = 0; countArray<10; countArray++)
		{
			timeEncoder.all = currentTime[countArray];
//			timeEncoder.all = bufferTime[countArray];
			encoderPulso.all = dataToSend[countArray];
			for(counter = 2; counter>0; counter--)
			{
				txBufferRead[5+counterAux] = timeEncoder.pt[counter-1];
				counterAux++;
			}
			for(counter = 4; counter>0; counter--)
			{
				txBufferRead[5+counterAux] = encoderPulso.pt[counter-1];
				counterAux++;
			}
		}
		txBufferRead[65] = Checksum(txBufferRead);
		txBufferRead[66] = Send_message.fim;

		CDC_Transmit_FS((uint8_t*)txBufferRead, sizeof(txBufferRead));

	break;

	case ReadError:
		txBufferRead[0] = 0x0A;
		txBufferRead[3] = 0x05;
		txBufferRead[65] = Checksum(txBufferRead);
		txBufferRead[66] = 0x0F;
		CDC_Transmit_FS((uint8_t*)txBufferRead, sizeof(txBufferRead));
	break;

	case Stop:
	txBuffer[0] = Send_message.inicio;
	txBuffer[1] = infoDevice.Id;
	txBuffer[2] = Send_message.endOrigem;
	txBuffer[3] = Send_message.funcao;
	txBuffer[4] = Send_message.dadosCount;
	txBuffer[5] = Checksum(txBuffer);
	txBuffer[6] = Send_message.fim;
	CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
	break;

	case error:
		if(errorFlag == errorChecksum)
		{
			txBuffer[0] = Send_message.inicio;
			txBuffer[1] = infoDevice.Id;
			txBuffer[2] = Send_message.endOrigem;
			txBuffer[3] = (Send_message.funcao | 0xF0);
			txBuffer[4] = 0x01;
			txBuffer[5] = 0x00; //codigo de erro
			txBuffer[6] = Checksum(txBuffer);
			txBuffer[7] = Send_message.fim;
			CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
		}
		else if (errorFlag == timeOut)
		{
			txBuffer[0] = Send_message.inicio;
			txBuffer[1] = infoDevice.Id;
			txBuffer[2] = Send_message.endOrigem;
			txBuffer[3] = (Send_message.funcao | 0xF0);
			txBuffer[4] = 0x01;
			txBuffer[5] = 0x01;
			txBuffer[6] = Checksum(txBuffer);
			txBuffer[7] = Send_message.fim;
			CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
		}
		else if (errorFlag == noAddress)
		{
			txBuffer[0] = Send_message.inicio;
			txBuffer[1] = infoDevice.Id;
			txBuffer[2] = Send_message.endOrigem;
			txBuffer[3] = (0x01 | 0xF0);
			txBuffer[4] = 0x01;
			txBuffer[5] = 0x02;
			txBuffer[6] = Checksum(txBuffer);
			txBuffer[7] = Send_message.fim;
			CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
		}
		else if (errorFlag == errorAddress)
		{
			txBuffer[0] = Send_message.inicio;
			txBuffer[1] = infoDevice.Id;
			txBuffer[2] = Send_message.endOrigem;
			txBuffer[3] = (0x01 | 0xF0);
			txBuffer[4] = 0x01;
			txBuffer[5] = 0x03;
			txBuffer[6] = Checksum(txBuffer);
			txBuffer[7] = 0x0F;
			CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
		}
		else if (errorFlag == errorDevice)
		{
			txBuffer[0] = Send_message.inicio;
			txBuffer[1] = infoDevice.Id;
			txBuffer[2] = Send_message.endOrigem;
			txBuffer[3] = (Send_message.funcao | 0xF0);
			txBuffer[4] = 0x01;
			txBuffer[5] = 0x04;
			txBuffer[6] = Checksum(txBuffer);
			txBuffer[7] = Send_message.fim;
			CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
		}
		else if (errorFlag == errorConfig)
		{
			txBuffer[0] = Send_message.inicio;
			txBuffer[1] = infoDevice.Id;
			txBuffer[2] = Send_message.endOrigem;
			txBuffer[3] = (0x02 | 0xF0);
			txBuffer[4] = 0x01;
			txBuffer[5] = 0x05;
			txBuffer[6] = Checksum(txBuffer);
			txBuffer[7] = Send_message.fim;
			CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
		}
		else if (errorFlag == errorFunction)
		{
			txBuffer[0] = Send_message.inicio;
			txBuffer[1] = infoDevice.Id;
			txBuffer[2] = Send_message.endOrigem;
			txBuffer[3] = (Send_message.funcao | 0xF0);
			txBuffer[4] = 0x01;
			txBuffer[5] = 0x06;
			txBuffer[6] = Checksum(txBuffer);
			txBuffer[7] = Send_message.fim;
			CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
		}
		else if (errorFlag == errorData)
		{
			txBuffer[0] = Send_message.inicio;
			txBuffer[1] = infoDevice.Id;
			txBuffer[2] = Send_message.endOrigem;
			txBuffer[3] = (Send_message.funcao | 0xF0);
			txBuffer[4] = 0x01;
			txBuffer[5] = 0x07;
			txBuffer[6] = Checksum(txBuffer);
			txBuffer[7] = Send_message.fim;
			CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
		}
		else if (errorFlag == errorStartHeader)
		{
			txBuffer[0] = 0x0A;
			txBuffer[1] = infoDevice.Id;
			txBuffer[2] = Send_message.endOrigem;
			txBuffer[3] = (Send_message.funcao | 0xF0);
			txBuffer[4] = 0x01;
			txBuffer[5] = 0x08;
			txBuffer[6] = Checksum(txBuffer);
			txBuffer[7] = Send_message.fim;
			CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
		}
		else if (errorFlag == errorEndHeader)
		{
			txBuffer[0] = Send_message.inicio;
			txBuffer[1] = infoDevice.Id;
			txBuffer[2] = Send_message.endOrigem;
			txBuffer[3] = (Send_message.funcao | 0xF0);
			txBuffer[4] = 0x01;
			txBuffer[5] = 0x09;
			txBuffer[6] = Checksum(txBuffer);
			txBuffer[7] = Send_message.fim;
			CDC_Transmit_FS((uint8_t*)txBuffer, sizeof(txBuffer));
		}
		break;
}
}

char StopDevice(char deviceFlag)
{
static char stopEncoder;
char contador = 0;
if(deviceFlag == 1)
{
//	stopEncoder = HAL_TIM_Encoder_Stop_IT(&htim2, TIM_CHANNEL_1);
	stopEncoder = HAL_TIM_Encoder_Stop_IT(&htim2, TIM_CHANNEL_ALL);
	timerEnable = HAL_TIM_Base_Stop_IT(&htim3);
	deviceFlag = 0;
}


HAL_Delay(10);
if(stopEncoder == HAL_OK && timerEnable == HAL_OK)
{
	for(contador=0;contador<10;contador++)
	{
		bufferPulso[contador] = 0;
		currentPulse[contador] = 0;
		currentTime[contador] = 0;
		USB_FLAG = 0;
	}
	pulseBuffer = 0;
	samples = 0;
	return 0;
}
else return 1;
}


int Checksum(char *c)
{
//if(c[0] == 0x0A)//Verifica Byte Inicio
//{
	if(c[4] == 0)Total=7;//Define tamanho do pacote
	if(c[4] == 1)Total=8;
	if(c[4] == 2)Total=9;
	if(c[4] == 4)Total=11;
	if(c[4] == 5) Total = 12;
	if(c[4] == 6) Total = 13;
	if(c[4] == 7) Total = 14;
	if(c[4] == 8) Total = 15;
	if(c[4] == 9) Total = 16;
	if(c[4] == 10) Total = 17;
	if(c[4] == 12) Total = 19;
	if(c[4] == 14) Total = 21;
	if(c[4] == 23 ) Total = 29;
	if(c[4] == 32 ) Total = 38;
	if(c[4] == 40 ) Total = 47;
	if(c[4] == 60 ) Total = 67;

	c_sum=0;
	for(int i=1; i<(Total-2);i++)//Soma os bytes
	{
		c_sum += c[i];
	}

	checksum = 0xFF-c_sum;//Faz os calculos seguintes
	checksum += 0x01;
//}
return checksum;
}




//Interrupções
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		aux_count = __HAL_TIM_GET_COUNTER(&htim2);
//		count = aux_count>>1;
		count = aux_count;
	}


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
if(EndReception == '\0')
{
	command(RXBuffer[0]);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)RXBuffer, 1);
}

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
if(GPIO_Pin == RESET_Pin)
{
	StateMachine = iddle;
	EndReception = Reset;
}
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

