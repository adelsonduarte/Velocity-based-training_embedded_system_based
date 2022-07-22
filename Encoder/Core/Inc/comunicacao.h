#ifndef COMUNICAO_H_INCLUDED
#define COMUNICAO_H_INCLUDED
#define END_ARRAY '@'
unsigned char get_header_start(char *buffer);

unsigned char get_source_address(char *buffer);

unsigned char get_destination_address(char *buffer);

unsigned char get_function(char *buffer);

unsigned char get_payload_size(char *buffer);

char get_payload(char *buffer, char idx);

unsigned char get_checksum(char *buffer);

unsigned char get_header_end(char *buffer);

unsigned char get_array_end(void);

void transmit_data(unsigned char dataSize, unsigned char* buffer);

#endif // COMUNICAO_H_INCLUDED
