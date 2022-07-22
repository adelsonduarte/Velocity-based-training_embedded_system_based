/*
 * MachineState.h
 *
 *  Created on: Jun 22, 2022
 *      Author: Adelson
 */

#ifndef INC_MACHINESTATE_H_
#define INC_MACHINESTATE_H_

#define IDDLE 0
#define IDENTIFICATION 1
#define CONFIG 2
#define START 3
#define READ 4
#define READERROR 5
#define STOP 6
#define RXDATA 7
#define INICIO 8
#define ERROR 9

#define TRUE 1
#define FALSE 0

#define AUTO 1
#define MAN 2
#define SAMPLES 10

unsigned char machine_state(void);
extern unsigned char readFlag;
extern unsigned char encoderDirection;
extern int32_t pulseCounter;
extern uint16_t newTime;
extern uint16_t timeTotal;
extern unsigned char samples;
extern int32_t bufferPulso[SAMPLES];
extern uint16_t acquiredTime;
extern uint16_t currentTime[SAMPLES];
extern unsigned char transmitFlag;
//extern unsigned char rxFlag;
//extern unsigned char RXbuffer[70];

#endif /* INC_MACHINESTATE_H_ */
