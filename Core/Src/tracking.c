#include "tracking.h"


bool if_left_black(void){
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
        return true;
    }else{
        return false;
    }
}

bool if_right_black(void){
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET){
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