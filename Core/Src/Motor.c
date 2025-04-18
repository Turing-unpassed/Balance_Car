#include "Motor.h"

void Motor_Ctrol(float r_out,float l_out){
	
	float r_output,l_output;
	
	if(r_out>0){
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
        r_output = r_out;
    }
    else if(r_out<0){
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
        r_output = -r_out;
    }
    if(l_out>0){
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
        l_output = l_out;
    }
    else if(l_out<0){
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
        l_output = -l_out;
    }
	
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,l_output);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,r_output);
}