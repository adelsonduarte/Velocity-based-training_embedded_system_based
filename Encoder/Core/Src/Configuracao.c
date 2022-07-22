#include <stdio.h>
#include <stdlib.h>
#include "Configuracao.h"
#include "Data.h"

unsigned char* set_ID(struct communicationStruct *dataReceiver){
    unsigned char* codCefise;
//  FLASH_apaga(END_INICIAL, 1);
    codCefise = set_device_ID_struct(&(dataReceiver->destinationAdress));
//    printf("endere�o codCefise = %d\n",codCefise);
//    printf("conteudo codCefise = %p\n",*codCefise);
//    FLASH_escreve_16bits(END_INICIAL, codCefise);
    return codCefise;
}

unsigned char* get_ID(){
    unsigned char* codCefise;
    codCefise = get_device_ID_struct();
    //FLASH_le_16bits(END_INICIAL, (codCefise);
    return codCefise;
}

unsigned char* set_frequency_samples(struct communicationStruct *dataReceiver){
    unsigned char* period;
    period = set_device_frequency_struct(&(dataReceiver->payload));
    return period;
}

unsigned char* set_operation_mode(struct communicationStruct *dataReceiver){
    unsigned char* mode;
    mode = set_device_mode_struct(&(dataReceiver->payload));
    return mode;
}


