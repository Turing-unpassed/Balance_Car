#ifndef __PID_H
#define __PID_H

typedef struct PID 
{
  float  Proportion;           
  float  Integral;          
  float  Derivative;         
  float  PrevError;          //  Error[-2]
  float  LastError;          //  Error[-1]  
  float  Error;
  float  DError;
  float  Integralmax;
  float  output;
  float  outputmax;
  float  deadzone;
  float  SumError;           //  Sums of Errors  
  float  dout;
  float  iout;
  float  pout;
	  
}PID;

void PID_parameter_init(PID *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone); 
void PID_reset_PID(PID *pp, float Kp, float Ki, float Kd);
void PID_incremental_PID_calculation(PID *pp, float CurrentPoint, float NextPoint);
void PID_position_PID_calculation_by_error(PID *pp, float error);
void Pitch_pid_calculation(PID *pp, float Target_yaw, float Current_yaw);
void Turn_pid_calculation(PID *pp, float Target_Yaw, float Current_Yaw);

#endif
