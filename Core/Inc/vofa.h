#ifndef __VOFA_H
#define __VOFA_H

#include "usart.h"
#include "HC_05.h"

#define VOFA_FRAM_HEADER 0xA5
#define VOFA_FRAM_TAIL_0 0x5A
#define VOFA_FRAM_TAIL_1 0xA5

enum rxState
{
    WAITING_FOR_HEADER_0,
    WAITING_FOR_ID,
    WAITING_FOR_DATA,
    WAITING_FOR_END_0,
    WAITING_FOR_END_1
};

union Vofa_Pid{
	uint8_t origin[4];
	float Pid_Data;
};

void Vofa_Tail_Send();
void Vofa_SendFloat(float param);
void StartUart2ReceiveIT();
void Vofa_Handle_Receive(uint8_t buffer);


#endif
