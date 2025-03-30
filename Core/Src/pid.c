#include "pid.h"
#include "math.h"

void PID_parameter_init(PID *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone)  
{  
	  pp->Integralmax = Integralmax;
	  pp->outputmax = outputmax;
	  pp->Proportion = Kp;
	  pp->Integral   = Ki;
	  pp->Derivative = Kd;
      pp->DError = pp->Error  = pp->output = pp->LastError = pp->PrevError  = 0.0f;
	  pp->deadzone = deadzone;
}  
// 重置PID系数
void PID_reset_PID(PID *pp, float Kp, float Ki, float Kd)
{
    pp->Proportion = Kp;
	pp->Integral   = Ki;
	pp->Derivative = Kd;
}

// 增量式PID
void PID_incremental_PID_calculation(PID *pp, float CurrentPoint, float NextPoint)  
{    

	
	pp->Error =  NextPoint - CurrentPoint; 
	if(fabs(pp->Error) < pp->deadzone)
		{
			pp->Error = 0;
		}	
	pp->DError = pp->Error - pp->LastError;
	
	pp->pout = pp->Proportion * pp->DError;
	pp->iout = pp->Integral * pp->Error;
	pp->dout = pp->Derivative * ( pp->Error +  pp->PrevError - 2*pp->LastError);
	
	if(pp->iout>pp->Integralmax){
		pp->iout=pp->Integralmax ;
	}else if(pp->iout< -(pp->Integralmax)){
		pp->iout = -(pp->Integralmax);
	}
	
	pp->output +=  (pp->iout + pp->pout + pp->dout);
	
	if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	if(pp->output < - pp->outputmax )  
		pp->output = -pp->outputmax;
	
	pp->PrevError = pp->LastError;  
	pp->LastError = pp->Error;
	
	
}

//位置式pid
void PID_position_PID_calculation_by_error(PID *pp, float error)  
{   

	
	if(fabs(error) < pp->deadzone)
	{
		error = 0;
	}
	
	pp->Error =  error;          
	pp->SumError += pp->Error;                      
	pp->DError = pp->Error - pp->LastError;
	
	pp->pout = pp->Proportion * pp->Error;
	pp->iout = pp->Integral * pp->SumError;
	pp->dout = pp->Derivative * pp->DError ;  
	
	if(pp->iout>pp->Integralmax){
		pp->iout=pp->Integralmax ;
	}else if(pp->iout< -(pp->Integralmax)){
		pp->iout = -(pp->Integralmax);
	}
	
	pp->output =  pp->iout + pp->pout + pp->dout;
								

	if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	if(pp->output < - pp->outputmax )  pp->output = -pp->outputmax; 
	pp->LastError = pp->Error;
	
	
}