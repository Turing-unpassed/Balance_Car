#ifndef __HC_05_H
#define __HC_05_H

#define HC_05_FRAM_HEADER 0xA5
#define HC_05_FRAM_ID_0 0x0 //蓝牙遥控ID
#define HC_05_FRAM_ID_1 0x1 //蓝牙调参ID
#define HC_05_FRAM_TAIL 0x5A

#include "usart.h"
#include "string.h"

typedef enum {
    WAITING_HEADER = 0,
    WAITING_ID,
    RECEIVING_REMOTE_DATA,
    RECEIVING_PID_DATA,
    WATING_CHECKSUM,
    WATING_TAIL,
} HC_05_State_t;

typedef struct {
    float Velocity_x;
    float Velocity_y;
    float Omega;
	
}Velocity_t;

union HC_05_PID_t{
	uint8_t origin[16];
	float Data[4];
};

union Remote_Data_t{
	uint8_t origin[16];
	float Data[4];
};

typedef enum{
	AUTO=0,
	REMOTE,
}Control_Model;


void HC_05_Data_Handle(uint8_t RxBuffer);
void StartUart1ReceiveIT(void);
void HC_05_Send_Header(void);
uint32_t HC_05_Send_Float(float Data);
void HC_05_Send_Checksum(uint8_t checksum);
void HC_05_Send_Tail(void);
void HC_05_Send_Package(uint8_t *package,uint8_t length);

#endif
