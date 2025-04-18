#include "HC_05.h"

HC_05_State_t HC_05_State = WAITING_HEADER;
Velocity_t Velocity_Data;
uint8_t Uart1_RxBuffer;

void HC_05_Data_Handle(uint8_t RxBuffer){
    Velocity_t Velocity_data;
    switch(HC_05_State){
        case WAITING_HEADER:
            if(RxBuffer == 0xA5){
                HC_05_State = RECEIVING_DATA;
            }
            break;
        case RECEIVING_DATA:
            if(RxBuffer == 0xf){
                Velocity_data.Velocity_y = 3.0f;
                HC_05_State = WATING_TAIL;
            }else if(RxBuffer == 0xb){
                Velocity_data.Velocity_y = -3.0f;
                HC_05_State = WATING_TAIL;
            }else if(RxBuffer == 0xa){ 
                Velocity_data.Omega = -1.5f;
                HC_05_State = WATING_TAIL;
            }else if(RxBuffer == 0xd){
                Velocity_data.Omega = 1.5f;
                HC_05_State = WATING_TAIL;
            }else{
                HC_05_State = WAITING_HEADER;
            }
            break;
        case WATING_TAIL:
            if(RxBuffer == 0x5A){
                Velocity_Data = Velocity_data;
                HC_05_State = WAITING_HEADER;
            }else{
                HC_05_State = WAITING_HEADER;
            }
            break;
    }
}

void StartUart1ReceiveIT(){
	HAL_UART_Receive_IT(&huart1,&Uart1_RxBuffer,1);
}