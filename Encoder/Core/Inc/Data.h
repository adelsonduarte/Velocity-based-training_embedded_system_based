#ifndef DATA_H_INCLUDED
#define DATA_H_INCLUDED
#include <string.h>
#include<stdint-gcc.h>
#define SIZE_DATA 10
#define HEADER 0x0A
#define SIZE_PAYLOAD 60
#define DESTINATION 0xC0
#define END 0x0F
#define SAMPLES 10

struct communicationStruct{
        unsigned char startHeader;
        unsigned char sourceAdress;
        unsigned char destinationAdress;
        unsigned char function;
        unsigned char payloadSize;
        unsigned char payload[SIZE_PAYLOAD];
        unsigned  char checksum;
        unsigned char endHeader;
    };

struct deviceStruct{
    unsigned char id;
    unsigned char readStatus;
    unsigned char timeSampling;
    unsigned char function;
    unsigned char error;
    unsigned char deviceStatus; // VERIFICAR A NECESSIDADE
    unsigned char measurement[SIZE_PAYLOAD];
    union time
    {
        uint16_t timeAll;
        char timePT[2];

    }timeMeasurement;
    union pulse
    {
        int32_t pulseAll;
        char pulsePT[4];

    }pulseMeasurement;
    };

struct communicationStruct* set_transmission_data_struct(unsigned char*rawData);
struct communicationStruct* set_receive_data_struct(char* rawData);
struct deviceStruct* set_device_ID_struct(char* data);
struct deviceStruct* set_device_readStatus_struct(char* rawData);
struct deviceStruct* set_device_timeSampling_struct(char* rawData);
struct deviceStruct* set_device_function_struct(char* rawData);
struct deviceStruct* set_device_error_struct(char* rawData);

struct deviceStruct* set_device_measurement_struct(uint16_t* data1, int32_t* data2);
struct deviceStruct* get_device_ID_struct();
char* get_data(char *data);
char set_checksum(char* data,char dataSize);
unsigned char* get_transmitt_payload(unsigned char* ID);

extern struct deviceStruct* ptr_dataDevice;


#endif // DATA_H_INCLUDED
