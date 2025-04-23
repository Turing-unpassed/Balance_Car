#include "vofa.h"
#include "pid.h"



extern uint8_t Uart1_RxBuffer;
uint8_t Uart2_RxBuffer;
uint8_t i=0;
union Vofa_Pid vofa_kp,vofa_ki,vofa_kd,vofa_Target;
uint8_t id;

enum rxState state = WAITING_FOR_HEADER_0;


#ifdef VOFA_DEBUG

void Vofa_Tail_Send(){
	uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
	HAL_UART_Transmit(&huart2,tail,sizeof(tail),HAL_MAX_DELAY);
}

void Vofa_SendFloat(float param){
	uint8_t buffer[4];
    memcpy(buffer, &param, sizeof(float));
	HAL_UART_Transmit(&huart2,buffer,sizeof(float),HAL_MAX_DELAY);
}

void StartUart2ReceiveIT(){
	HAL_UART_Receive_IT(&huart2,&Uart2_RxBuffer,1);
}

void Vofa_Handle_Receive(uint8_t buffer){
	switch(state){
		case WAITING_FOR_HEADER_0:
			if(buffer == VOFA_FRAM_HEADER)
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
				vofa_kp.origin[i] = buffer;
			}else if(id==1){
				vofa_ki.origin[i] = buffer;
			}else if(id==2){
				vofa_kd.origin[i] = buffer;
			}else if(id==3){
				vofa_Target.origin[i] = buffer;
			}
			i++;
			if(i>= 4){
				state = WAITING_FOR_END_0;
			}
		break;
		case WAITING_FOR_END_0:
			if(buffer == VOFA_FRAM_TAIL_0)
            {
                state = WAITING_FOR_END_0;
            }else{
				state = WAITING_FOR_HEADER_0;
			}
		break;
		case WAITING_FOR_END_1:
			if(buffer == VOFA_FRAM_TAIL_1)
            {
                state = WAITING_FOR_HEADER_0;
				
            }else{
				state = WAITING_FOR_HEADER_0;
			}
		break;
	}
}

#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
#ifdef VOFA_DEBUG
	if(huart == &huart2){
		Vofa_Handle_Receive(Uart2_RxBuffer);
	}
	StartUart2ReceiveIT();

#endif
	
	if(huart == &huart1){
		HC_05_Data_Handle(Uart1_RxBuffer);
	}
	StartUart1ReceiveIT();
}