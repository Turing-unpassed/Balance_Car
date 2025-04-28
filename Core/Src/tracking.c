#include "tracking.h"


bool if_left_black(void){

    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET){
        return true;
    }else{
        return false;
    }
}

bool if_right_black(void){
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET){
        return true;
    }else{
        return false;
    }
}

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              sensor_status get_sensor_status(void){
    if(if_left_black() && if_right_black()){
        return TWO_BLACK;
    }
    else if(!if_left_black() && !if_right_black()){
        return TWO_WHITE;
    }
    else if(if_left_black() && !if_right_black()){
        return L_BLACK_R_WHITE;
    }
    else{
        return L_WHITE_R_BLACK;
    }
}
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																							  
TASK get_task(){
	uint8_t a = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
	uint8_t b = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
	uint8_t c = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
	uint8_t status = 0;
	status = a | (b<<1) | (c<<2);
	if(status == 0){
		return task1;
	}else if(status == 1){
		return task2;
	}else if(status == 2){
		return task3;
	}else if(status == 3){
		return task4;
	}else if(status == 4){
		return task5;
	}else{
		return -1;
	}
	
	
	
	
}