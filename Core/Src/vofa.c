#include "vofa.h"
#include "pid.h"


uint8_t RxBuffer;
uint8_t i=0;
union Vofa_Pid kp,ki,kd,Target;
uint8_t id;

enum rxState state = WAITING_FOR_HEADER_0;

void Vofa_Tail(){
	uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
	HAL_UART_Transmit(&huart2,tail,sizeof(tail),HAL_MAX_DELAY);
}

void Vofa_SendFloat(float param){
	uint8_t buffer[4];
    memcpy(buffer, &param, sizeof(float));
	HAL_UART_Transmit(&huart2,buffer,sizeof(float),HAL_MAX_DELAY);
}

void StartUartReceiveIT(){
	HAL_UART_Receive_IT(&huart2,&RxBuffer,1);
}

void Vofa_Handle_Receive(uint8_t buffer){
	switch(state){
		case WAITING_FOR_HEADER_0:
			if(buffer == 0xA5)
            {
                state = WAITING_FOR_ID;
            }
		break;
		case WAITING_FOR_ID:
			if(buffer >= 0x00 && buffer <= 0x03)
            {
				i=0;
				id = buffer;
                state = WAITING_FOR_DATA;
				
            }else{
				state = WAITING_FOR_HEADER_0;
			}
		break;
		case WAITING_FOR_DATA:
			if(id==0){
				kp.origin[i] = buffer;
			}else if(id==1){
				ki.origin[i] = buffer;
			}else if(id==2){
				kd.origin[i] = buffer;
			}else if(id==3){
				Target.origin[i] = buffer;
			}
			i++;
			if(i>= 4){
				state = WAITING_FOR_END_0;
			}
		break;
		case WAITING_FOR_END_0:
			if(buffer == 0x5A)
            {
                state = WAITING_FOR_END_0;
            }else{
				state = WAITING_FOR_HEADER_0;
			}
		break;
		case WAITING_FOR_END_1:
			if(buffer == 0xA5)
            {
                state = WAITING_FOR_HEADER_0;
				
            }else{
				state = WAITING_FOR_HEADER_0;
			}
		break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
		Vofa_Handle_Receive(RxBuffer);
	}
	
	StartUartReceiveIT();
}