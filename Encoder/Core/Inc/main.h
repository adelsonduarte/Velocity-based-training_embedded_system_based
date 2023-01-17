/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STOP_Pin GPIO_PIN_12
#define STOP_GPIO_Port GPIOB
#define ERRO_Pin GPIO_PIN_13
#define ERRO_GPIO_Port GPIOB
#define STATUS_Pin GPIO_PIN_14
#define STATUS_GPIO_Port GPIOB
#define CONFIG_Pin GPIO_PIN_15
#define CONFIG_GPIO_Port GPIOB
#define ACQUISITION_Pin GPIO_PIN_8
#define ACQUISITION_GPIO_Port GPIOA
#define RESET_Pin GPIO_PIN_9
#define RESET_GPIO_Port GPIOA
#define RESET_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */
#define iddle 			'I'
#define Inicio 			'0'
#define Identification	'1'
#define Config 			'2'
#define Start  			'3'
#define Read 			'4'
#define ReadError		'5'
#define Stop			'6'
#define Codification	'7'
#define Origem			'7'
#define Destino			'8'
#define Funcao			'9'
#define DadosCount		'A'
#define Dados			'B'
#define CheckSum		'C'
#define Fim				'D'
#define AUTO			'F'
#define MAN				'M'
#define error			'E'
#define errorChecksum	'0'
#define timeOut			'1'
#define noAddress		'2'
#define errorAddress	'3'
#define errorDevice		'4'
#define errorConfig		'5'
#define errorFunction	'6'
#define errorData		'7'
#define errorStartHeader '8'
#define errorEndHeader	'9'
#define OK				'A'
#define horario			'1'
#define antihorario		'2'
#define centro			'K'
#define parado			'P'
#define Reset			'R'






/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
