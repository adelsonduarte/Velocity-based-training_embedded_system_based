#ifndef CONFIGURACAO_H_INCLUDED
#define CONFIGURACAO_H_INCLUDED

struct communicationStruct;
struct deviceStruct;
unsigned char* set_ID(struct communicationStruct *dataReceiver);

unsigned char* get_ID();

unsigned char* set_frequency_samples(struct communicationStruct *dataReceiver);

unsigned char* set_operation_mode(struct communicationStruct *dataReceiver);


#endif // CONFIGURACAO_H_INCLUDED
