/*
 * MachineState.c
 *
 *  Created on: Jun 22, 2022
 *      Author: Adelson
 */
#include<stdint-gcc.h>
#include "MachineState.h"
#include "Data.h"
#include "IO_interface.h"
#include "Configuracao.h"
#include "comunicacao.h"

unsigned char rxFlag = FALSE;
unsigned char RXbuffer[70];
unsigned char state = IDDLE;

extern unsigned char timerFlag;
extern unsigned char readFlag;
extern unsigned char encoderDirection;
extern int32_t pulseCounter;
extern uint16_t newTime;
extern unsigned char samples;
extern int32_t bufferPulso[SAMPLES];
extern uint16_t acquiredTime;
extern uint16_t currentTime[SAMPLES];
extern unsigned char transmitFlag;

unsigned char machine_state(void)
{

	static unsigned char payloadSize;
	static unsigned char TXsize;
    static unsigned char rxGet;
    static struct communicationStruct *ptr_dataReceiver;
    static struct communicationStruct *ptr_dataTransmitter;
    static struct deviceStruct dataDevice;
    static struct deviceStruct *ptr_dataDevice = &(dataDevice);
    static char* getData;
    static unsigned char encoderStatus = 0;
    static unsigned char timerStatus = 0;
    static int32_t counterPulso[SAMPLES];
    static char readEnable = FALSE;
    static char dataToStruct[2];

    switch(state)
    {
        case IDDLE:
            if(rxFlag == TRUE) state = RXDATA;
            else if (rxFlag == FALSE) state = IDDLE;
            break;

        case RXDATA:

            getData = get_data(RXbuffer);
            ptr_dataReceiver = set_receive_data_struct(getData);
            rxGet = TRUE;
            rxFlag = FALSE;
            state = INICIO;
            break;

        case INICIO:
            switch(ptr_dataReceiver->function)
            {
                case IDENTIFICATION:
                    state = IDENTIFICATION;
                break;

                case CONFIG:
                    state = CONFIG;
                    break;

                case START:
                    state = START;
                    break;

                case READ:
                    state = READ;
                    break;

                case STOP:
                    state = STOP;
                    break;

                case READERROR:
                    state = READERROR;
                    break;
            }
            break;

        case IDENTIFICATION:
            if(ptr_dataReceiver->payloadSize==1) //codificacao
            {
            	ptr_dataDevice = set_ID(ptr_dataReceiver);
            	payloadSize = 1;
            	TXsize = 8;
            	dataToStruct[0] = IDENTIFICATION;
            	dataToStruct[1] = payloadSize;
            	ptr_dataTransmitter = set_transmission_data_struct(dataToStruct);
                transmit_data(TXsize,ptr_dataTransmitter);
                erasePayload(&(ptr_dataTransmitter->payload));
            }
            else if(ptr_dataReceiver->payloadSize==0) //identificacao
            {
            	ptr_dataDevice = get_ID();
            	payloadSize = 1;
				TXsize = 8;
                dataToStruct[0] = IDENTIFICATION;
            	dataToStruct[1] = payloadSize;
                ptr_dataTransmitter = set_transmission_data_struct(dataToStruct);
				transmit_data(TXsize,ptr_dataTransmitter);
				erasePayload(&(ptr_dataTransmitter->payload));
            }
            rxGet = FALSE;
            state = IDDLE;
            break;

        case CONFIG:
        	ptr_dataDevice->timeSampling = set_frequency_samples(ptr_dataReceiver);
			ptr_dataDevice->readStatus = set_operation_mode(ptr_dataReceiver);
		    payloadSize = 0;
		    TXsize = 7;
		    dataToStruct[0] = CONFIG;
            dataToStruct[1] = payloadSize;
		    ptr_dataTransmitter = set_transmission_data_struct(dataToStruct);
		    transmit_data(TXsize,ptr_dataTransmitter);
            rxGet = FALSE;
            state = IDDLE;
            break;

        case START:
            if(timerStatus == 0) timerStatus = start_timer();
            if(encoderStatus == 0) encoderStatus = start_encoder();
            payloadSize = 0;
            TXsize = 7;
            dataToStruct[0] = START;
            dataToStruct[1] = payloadSize;
            ptr_dataTransmitter = set_transmission_data_struct(dataToStruct);
            transmit_data(TXsize,ptr_dataTransmitter);
            rxGet = FALSE;
            state = IDDLE;
            readEnable = TRUE;
            break;

        case READ:
        	if(readEnable == TRUE)
        	{
				pulseCounter = 0;
				acquiredTime = 0;
				newTime = 0;
				samples = 0;
				readFlag = TRUE;
				timerFlag = TRUE;
				readEnable = FALSE;
        	}


            if(ptr_dataDevice->readStatus == AUTO)
            {
                if(transmitFlag == TRUE && rxFlag == FALSE)
                {
            		payloadSize = 60;
    				TXsize = 67;
                    dataToStruct[0] = READ;
                    dataToStruct[1] = payloadSize;
					ptr_dataTransmitter = set_transmission_data_struct(dataToStruct);
					transmit_data(TXsize,ptr_dataTransmitter);
					transmitFlag = FALSE;
//					HAL_GPIO_TogglePin(GPIOA, ACQUISITION_Pin);
                    rxGet = FALSE;
					state = IDDLE;
                }
                else if(transmitFlag == TRUE && rxFlag == TRUE)
                {
                    state = STOP;
                }
//                else state = INICIO;
            }
            else if(ptr_dataDevice->readStatus == MAN)
            {
                while(transmitFlag==0);
//                HAL_GPIO_TogglePin(GPIOA, ACQUISITION_Pin);
                transmitFlag = 0;
                for(unsigned char i=0;i<SAMPLES;i++) counterPulso[i] = bufferPulso[i];
                //transmit
                for(unsigned char i=0;i<SAMPLES;i++) counterPulso[i] = 0;
                rxGet = FALSE;
                state = IDDLE;
            }
            break;

        case STOP:
            if(timerStatus == 1) timerStatus = stop_timer();
            if(encoderStatus == 1) encoderStatus = stop_encoder();
            erasePayload(&(ptr_dataTransmitter->payload));
            payloadSize = 0;
            TXsize = 7;
            dataToStruct[0] = STOP;
            dataToStruct[1] = payloadSize;
            ptr_dataTransmitter = set_transmission_data_struct(dataToStruct);
            transmit_data(TXsize,ptr_dataTransmitter);
			readFlag = FALSE;
			timerFlag = FALSE;
            rxGet = FALSE;
            state = IDDLE;
            readEnable = TRUE;
            reset_buffer();
            break;



        case READERROR:
        	payloadSize = 60;
			TXsize = 67;
			dataToStruct[0] = READ;
			dataToStruct[1] = payloadSize;
			transmit_data(TXsize,ptr_dataTransmitter);
			transmitFlag = FALSE;
//					HAL_GPIO_TogglePin(GPIOA, ACQUISITION_Pin);
			rxGet = FALSE;
			state = IDDLE;
            break;

        case ERROR:
            break;
    }
}

void erasePayload(char* TXpayload)
{
	for(unsigned char i=0;i<SIZE_PAYLOAD;i++) TXpayload[i]=0;

}

