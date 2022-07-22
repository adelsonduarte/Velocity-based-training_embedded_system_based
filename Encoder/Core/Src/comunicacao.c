#include <stdio.h>
#include <stdlib.h>
#include "comunicacao.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

unsigned char get_header_start(char *buffer){
    return buffer[0];
}
unsigned char get_source_address(char *buffer){
    return buffer[1];
}

unsigned char get_destination_address(char *buffer){
    return buffer[2];
}

unsigned char get_function(char *buffer){
    return buffer[3];
}

unsigned char get_payload_size(char *buffer){
    return buffer[4];
}

char get_payload(char *buffer, char idx){
	return buffer[5+idx];}

unsigned char get_checksum(char *buffer){
    return buffer[5+get_payload_size(buffer)];
}

unsigned char get_header_end(char *buffer){
    return buffer[6+get_payload_size(buffer)];
}

unsigned char get_array_end(){
    return END_ARRAY;
}


void transmit_data(unsigned char dataSize, unsigned char* buffer){
    unsigned char TXbuffer[dataSize];

    if(buffer[4] == 0)
    {
        TXbuffer[0] = buffer[0];
        TXbuffer[1] = buffer[1];
        TXbuffer[2] = buffer[2];
        TXbuffer[3] = buffer[3];
        TXbuffer[4] = buffer[4];
        TXbuffer[5] = buffer[65];
        TXbuffer[6] = buffer[66];
        CDC_Transmit_FS((unsigned*)TXbuffer, sizeof(TXbuffer));
    }
    else if(buffer[4] == 1)
    {
        TXbuffer[0] = buffer[0];
        TXbuffer[1] = buffer[1];
        TXbuffer[2] = buffer[2];
        TXbuffer[3] = buffer[3];
        TXbuffer[4] = buffer[4];
        TXbuffer[5] = buffer[5];
        TXbuffer[6] = buffer[65];
        TXbuffer[7] = buffer[66];
        CDC_Transmit_FS((unsigned*)TXbuffer, sizeof(TXbuffer));
    }

    else
    {
        for(unsigned char x=0;x<dataSize;x++) TXbuffer[x] = buffer[x];
        CDC_Transmit_FS((unsigned*)TXbuffer, sizeof(TXbuffer));
    }


//    for(unsigned char y=0;y<dataSize;y++){
//        printf("Msg de retorno ID Txbuffer[%d] = %d\n",y,TXbuffer[y]);
//    }
}



//




