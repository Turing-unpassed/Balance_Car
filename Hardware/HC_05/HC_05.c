#include "HC_05.h"

HC_05_State_t HC_05_State = WAITING_HEADER;
Velocity_t Velocity_Data;
union HC_05_PID_t HC_05_PID;
uint8_t Uart1_RxBuffer;
uint32_t checksum;
union Remote_Data_t Remote_Data;
union HC_05_PID_t HC_05_PID_Temp;
uint8_t Index,ID;
uint8_t checksum_temp;

void HC_05_Data_Handle(uint8_t RxBuffer){
    
    switch(HC_05_State){
        case WAITING_HEADER:
            if(RxBuffer == HC_05_FRAM_HEADER){
                HC_05_State = WAITING_ID;
            }
            break;
        case WAITING_ID:
            if(RxBuffer == HC_05_FRAM_ID_0){
                ID = 0;
                Index = 0;
                checksum = 0;
				checksum += ID;
                HC_05_State = RECEIVING_REMOTE_DATA;
            }else if(RxBuffer == HC_05_FRAM_ID_1){
                ID = 1;
                Index = 0;
                checksum = 0;
				checksum += ID;
                HC_05_State = RECEIVING_PID_DATA;
            }else{
                HC_05_State = WAITING_HEADER;
            }
            break;
        case RECEIVING_REMOTE_DATA:
            Remote_Data.origin[Index] = RxBuffer;
            checksum += RxBuffer;
            Index++;
            if(Index == 16){
                HC_05_State = WATING_CHECKSUM;
            }
            break;
        case RECEIVING_PID_DATA:      
			HC_05_PID_Temp.origin[Index] = RxBuffer;
			checksum += RxBuffer;
			Index++;           
            if(Index == 16){              
                HC_05_State = WATING_CHECKSUM;
            }
            break;
        case WATING_CHECKSUM:
            checksum_temp = (uint8_t)checksum;
            if(checksum_temp == RxBuffer){
                HC_05_State = WATING_TAIL;
            }else{
                HC_05_State = WAITING_HEADER;
            }
            break;
        case WATING_TAIL:
            if(RxBuffer == HC_05_FRAM_TAIL){
                if(ID == 0){
                    Velocity_Data.Velocity_x = Remote_Data.Data[0];
                    Velocity_Data.Velocity_y = Remote_Data.Data[1];
                    Velocity_Data.Omega = Remote_Data.Data[2];
					
                }else if(ID == 1){
                    HC_05_PID.Data[0] = HC_05_PID_Temp.Data[0];
					HC_05_PID.Data[1] = HC_05_PID_Temp.Data[1];
					HC_05_PID.Data[2] = HC_05_PID_Temp.Data[2];
					HC_05_PID.Data[3] = HC_05_PID_Temp.Data[3];
                }
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

void HC_05_Send_Header(void){
    uint8_t data = HC_05_FRAM_HEADER;
    HAL_UART_Transmit(&huart1,&data,1,HAL_MAX_DELAY);
}

uint32_t HC_05_Send_Float(float Data){
    uint32_t sum = 0;
    uint8_t buffer[4];
    memcpy(buffer, &Data, sizeof(float));
    sum += buffer[0];
    sum += buffer[1];
    sum += buffer[2];
    sum += buffer[3];
	HAL_UART_Transmit(&huart1,buffer,sizeof(float),HAL_MAX_DELAY);
    return sum;
}

void HC_05_Send_Checksum(uint8_t Checksum){
    HAL_UART_Transmit(&huart1,&Checksum,1,HAL_MAX_DELAY);
}

void HC_05_Send_Tail(void){
    uint8_t data = HC_05_FRAM_TAIL;
    HAL_UART_Transmit(&huart1,&data,1,HAL_MAX_DELAY);
}

void HC_05_Send_Package(uint8_t *package,uint8_t length){
	HAL_UART_Transmit(&huart1,package,length,HAL_MAX_DELAY);
}