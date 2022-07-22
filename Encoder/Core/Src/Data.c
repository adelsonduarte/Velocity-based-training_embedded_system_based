#include <stdio.h>
#include <stdlib.h>
#include "comunicacao.h"
#include "Data.h"
#include "MachineState.h"

struct deviceStruct dataDevice;
struct deviceStruct* ptr_dataDevice = &dataDevice;


char* get_data(char *data)
{
  static char receive_Data[SIZE_DATA];

  receive_Data[0] = get_header_start(data);

  receive_Data[1] = get_source_address(data);

  receive_Data[2] = get_destination_address(data);

  receive_Data[3] = get_function(data);

  receive_Data[4] = get_payload_size(data);

  for(unsigned char idx=0;idx<get_payload_size(data);idx++)
  {
    receive_Data[5+idx] = get_payload(data,idx);
  }
    receive_Data[5+get_payload_size(data)] = get_checksum(data);

    receive_Data[6+get_payload_size(data)] = get_header_end(data);

    receive_Data[7+get_payload_size(data)] = get_array_end();


    printf("endereço de receive_Data = %d\n",receive_Data);
    printf("conteudo de receive_Data = %p\n",*receive_Data);
   return receive_Data;
}

struct communicationStruct* set_receive_data_struct(char* rawData){

    static struct communicationStruct dataReceive = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    struct communicationStruct* ptr_dataReceive = &dataReceive;
    ptr_dataReceive->startHeader = rawData[0];
    ptr_dataReceive->sourceAdress = rawData[1];
    ptr_dataReceive->destinationAdress = rawData[2];
    ptr_dataReceive->function = rawData[3];
    ptr_dataReceive->payloadSize = rawData[4];
    for(unsigned char idx=0;idx<ptr_dataReceive->payloadSize;idx++){
        ptr_dataReceive->payload[idx] = rawData[5+idx];
    }
    ptr_dataReceive->checksum = rawData[5+ptr_dataReceive->payloadSize];
    ptr_dataReceive->endHeader = rawData[6+ptr_dataReceive->payloadSize];
    return ptr_dataReceive;
}

struct communicationStruct* set_transmission_data_struct(unsigned char* rawData){

    static struct communicationStruct dataTransmission;
    struct communicationStruct* ptr_dataTransmission = &dataTransmission;
    unsigned char* payload;
    ptr_dataTransmission->startHeader = HEADER;
    ptr_dataTransmission->sourceAdress = ptr_dataDevice->id;
    ptr_dataTransmission->destinationAdress = DESTINATION;
    ptr_dataTransmission->function = rawData[0];
    ptr_dataTransmission->payloadSize = rawData[1];
    if(ptr_dataTransmission->payloadSize>0) payload = get_transmitt_payload(&(ptr_dataTransmission->function));
    for(unsigned char idx=0;idx<ptr_dataTransmission->payloadSize;idx++){
        ptr_dataTransmission->payload[idx] = payload[idx];
    }
    ptr_dataTransmission->checksum = set_checksum(ptr_dataTransmission, sizeof(*ptr_dataTransmission));
    ptr_dataTransmission->endHeader = END;
    printf("Endereco ptr_dataTransmission = %d\n",ptr_dataTransmission);
    return ptr_dataTransmission;
}

struct deviceStruct* set_device_ID_struct(char *data)
{
    ptr_dataDevice->id = *data;
//    printf("endereço ptr_dataDevice = %d\n",ptr_dataDevice);
//    printf("conteudo ptr_dataDevice = %p\n",*ptr_dataDevice);
    return ptr_dataDevice;
};

struct deviceStruct* get_device_ID_struct()
{
     char* data = &(ptr_dataDevice->id);
    return data;
};

unsigned char* get_transmitt_payload(unsigned char* ID)
{
    extern int32_t bufferPulso[SAMPLES];
    extern uint16_t currentTime[SAMPLES];
    unsigned char* data;
    if(*ID == IDENTIFICATION) data = get_device_ID_struct();
    else if(*ID == READ) data = set_device_measurement_struct(currentTime,bufferPulso);
    return data;
}

struct deviceStruct* set_device_frequency_struct(char* data){
    ptr_dataDevice->timeSampling = data[1];
    return ptr_dataDevice->timeSampling;
};

struct deviceStruct* set_device_mode_struct(char* data){
    ptr_dataDevice->readStatus = data[0];
    return ptr_dataDevice->readStatus;
};

struct deviceStruct* set_device_function_struct(char* data){
    ptr_dataDevice->function = *data;
    return ptr_dataDevice->readStatus;
};

struct deviceStruct* set_device_error_struct(char* data){
    ptr_dataDevice->error = *data;
    return ptr_dataDevice->error;
};

struct deviceStruct* set_device_measurement_struct(uint16_t* data1, int32_t* data2){
    unsigned char position = 0;

    for(unsigned char i=0;i<SAMPLES;i++)
    {
      ptr_dataDevice->timeMeasurement.timeAll = data1[i];
      for(unsigned char y = 2;y>0;y--)
      {
          ptr_dataDevice->measurement[position] = ptr_dataDevice->timeMeasurement.timePT[y-1];
          position++;
      }
    }

    for(unsigned char i=0;i<SAMPLES;i++)
    {
      ptr_dataDevice->pulseMeasurement.pulseAll = data2[i];
      for(unsigned char y = 4;y>0;y--)
      {
          ptr_dataDevice->measurement[position] = ptr_dataDevice->pulseMeasurement.pulsePT[y-1];
          position++;
      }
    }
    return ptr_dataDevice->measurement;
}

char set_checksum(char* data, char dataSize)
{
    unsigned char checksum_value = 0;
    unsigned char checksum_sum=0;

    for(unsigned char i=1; i<(dataSize-2);i++)
    {
        checksum_sum += data[i];
    }

    checksum_value = 0xFF-checksum_sum;
    checksum_value += 0x01;
    return checksum_value;

}

void reset_buffer(void)
{
	extern int32_t bufferPulso[SAMPLES];
	extern uint16_t currentTime[SAMPLES];
	for(unsigned x=0;x<SAMPLES;x++)
	{
		bufferPulso[x] = 0;
		currentTime[x] = 0;
	}

}


