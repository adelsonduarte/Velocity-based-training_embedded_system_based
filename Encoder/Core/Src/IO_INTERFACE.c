#include "Data.h"
#include "IO_interface.h"
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static unsigned char encoderStatus;
static unsigned char timerStatus;

unsigned char* start_encoder(void)
{
//        printf("Liga Encoder \n");
        encoderStatus = HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
        HAL_Delay(10);
        return !encoderStatus;
}

unsigned char* stop_encoder(void)
{
//    printf("Desliga Encoder \n");
	encoderStatus = HAL_TIM_Encoder_Stop_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_Delay(10);
        return encoderStatus;
}

unsigned char* start_timer(void)
{
//    printf("Liga Timer \n");
     timerStatus =HAL_TIM_Base_Start_IT(&htim3);
     HAL_Delay(10);
    return !timerStatus;
}

unsigned char* stop_timer(void)
{
//    printf("Desliga Timer \n");
     timerStatus =HAL_TIM_Base_Stop_IT(&htim3);
    HAL_Delay(10);
    return timerStatus;
}

unsigned char reset_hardware(void)
{
//    for(unsigned char i=0;i<SAMPLES;i++)
//	{
//	  counterPulso[i] = 0;
//	  currentTime[i] = 0;
//	  fimPulso[contador] = 0;
//	}
}

