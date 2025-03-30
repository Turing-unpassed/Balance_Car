#ifndef __VOFA_H
#define __VOFA_H

#include "usart.h"

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

void Vofa_Tail();
void Vofa_SendFloat(float param);
void StartUartReceiveIT();
void Vofa_Handle_Receive(uint8_t buffer);


#endif
