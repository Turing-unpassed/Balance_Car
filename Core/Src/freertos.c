/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "vofa.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "tim.h"
#include "oled.h"
#include "math.h"
#include "Motor.h"
#include "HC_05.h"
#include "tracking.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define aabs(x) ((x)>0?(x):-(x))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


extern Control_Model control_model;
extern Velocity_t Velocity_Data;
extern union HC_05_PID_t HC_05_PID;
extern union Vofa_Pid vofa_kp,vofa_ki,vofa_kd,vofa_Target;
int16_t TIM3_CNT,TIM4_CNT;
float L_RPS,R_RPS;
PID L_pid_Speed,R_pid_Speed,Angle_pid,Pitch_pid,Turn_pid;
float R_Target_RPS,L_Target_RPS,Target_RPS,Target_Speed,Turn_Out,Target_Yaw;
float Roll,Pitch,Yaw;
short Gyro[3];
short Gyro_y,Gyro_x,Gyro_z;
/* USER CODE END Variables */
osThreadId DebugTaskHandle;
osThreadId MotorCtrlTaskHandle;
osThreadId RPMGetTaskHandle;
osThreadId Task_StatusHandle;
osMutexId Encoder_MutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebug(void const * argument);
void StartMotorCtrl(void const * argument);
void StartRPMGet(void const * argument);
void Start_Task_Handle(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of Encoder_Mutex */
  osMutexDef(Encoder_Mutex);
  Encoder_MutexHandle = osMutexCreate(osMutex(Encoder_Mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DebugTask */
  osThreadDef(DebugTask, StartDebug, osPriorityNormal, 0, 128);
  DebugTaskHandle = osThreadCreate(osThread(DebugTask), NULL);

  /* definition and creation of MotorCtrlTask */
  osThreadDef(MotorCtrlTask, StartMotorCtrl, osPriorityNormal, 0, 128);
  MotorCtrlTaskHandle = osThreadCreate(osThread(MotorCtrlTask), NULL);

  /* definition and creation of RPMGetTask */
  osThreadDef(RPMGetTask, StartRPMGet, osPriorityNormal, 0, 128);
  RPMGetTaskHandle = osThreadCreate(osThread(RPMGetTask), NULL);

  /* definition and creation of Task_Status */
  osThreadDef(Task_Status, Start_Task_Handle, osPriorityNormal, 0, 128);
  Task_StatusHandle = osThreadCreate(osThread(Task_Status), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDebug */
/**
  * @brief  Function implementing the DebugTask thread.
  * @param  argument: Not used
  * @retval None
  */
TickType_t time1,time2;
/* USER CODE END Header_StartDebug */
void StartDebug(void const * argument)
{
  /* USER CODE BEGIN StartDebug */
  float pitch, roll, yaw;
  /* Infinite loop */
  for(;;)
  {
	
    //xSemaphoreTake(Ora_MutexHandle,portMAX_DELAY);
    if(MPU6050_DMP_Get_Date(&Pitch, &Roll, &Yaw)==0){
		pitch = -Pitch;
		roll = Roll;  
		yaw = Yaw;
    }

    osDelay(1);
  }
  /* USER CODE END StartDebug */
}

/* USER CODE BEGIN Header_StartMotorCtrl */
/**
* @brief Function implementing the MotorCtrlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorCtrl */
void StartMotorCtrl(void const * argument)
{
  /* USER CODE BEGIN StartMotorCtrl */
#ifdef VOFA_DEBUG
    StartUart2ReceiveIT();
#endif
	
    StartUart1ReceiveIT();
    float r_rps,l_rps,r_output,l_output,angle_error,target_speed,Target_angle;
    float pitch,roll,yaw;

    PID_parameter_init(&Pitch_pid,1,0.007,200,20,10,0);
    PID_parameter_init(&L_pid_Speed,1420,100,0,10000,5000,0);
    PID_parameter_init(&R_pid_Speed,1420,100,0,10000,5000,0);
    PID_parameter_init(&Angle_pid,0.25,0.003,1,15,15,0);
    PID_parameter_init(&Turn_pid,0.1,0,0,1,0.5,0);
    Target_RPS = 0;
    TickType_t preTime = xTaskGetTickCount();
    uint32_t sum_temp =0;
    uint8_t sum=0;
	  float turn_out1=0,turn_out2=0,target_yaw=0;
    sensor_status status;
	float a = 0;
  /* Infinite loop */
  for(;;)
  {		
    //time1 = xTaskGetTickCount(); 
    //xSemaphoreTake(Encoder_MutexHandle,portMAX_DELAY);
    r_rps = R_RPS;
    l_rps = L_RPS;
    //xSemaphoreGive(Encoder_MutexHandle);

    //xSemaphoreTake(Ora_MutexHandle,portMAX_DELAY);
    pitch = -Pitch;
	  roll = Roll;
    yaw = Yaw;
    //xSemaphoreGive(Ora_MutexHandle);  
	if(control_model == AUTO){
		target_speed = Target_Speed;
		target_yaw = Target_Yaw;
		turn_out2 = Turn_Out;
		if(target_speed==0){
			 PID_reset_PID(&Pitch_pid,1.2,0.001,200);
		}else{
			PID_reset_PID(&Pitch_pid,0.9,0.002,400);
		}
	}else{
		target_yaw =-1;
		target_speed = Velocity_Data.Velocity_y/128*3.5;
		if(target_speed==0){
			 PID_reset_PID(&Pitch_pid,1.2,0.001,200);
		}else{
			PID_reset_PID(&Pitch_pid,0.9,0.002,400);
		}
		turn_out2 = -Velocity_Data.Omega/128*1;
	}
  
//	  if(HC_05_PID.Data[0]!=0||HC_05_PID.Data[1]!=0||HC_05_PID.Data[2]!=0){
//		  PID_reset_PID(&Pitch_pid,HC_05_PID.Data[0],HC_05_PID.Data[1],HC_05_PID.Data[2]);
//	  }
    if(target_yaw != -1){
		Turn_pid_calculation(&Turn_pid,target_yaw,yaw);
	    turn_out1 = Turn_pid.output;
    }else{
		turn_out1 = 0;
    }
    
    Pitch_pid_calculation(&Pitch_pid,target_speed,r_rps);
    Target_angle = Pitch_pid.output;
    angle_error = pitch-Target_angle;
    if(fabsf(pitch)>55){
	    Angle_pid.output = 0;
    }else{
      PID_position_PID_calculation_by_error(&Angle_pid,angle_error);
    }
    Target_RPS = vofa_Target.Pid_Data;
    PID_incremental_PID_calculation(&R_pid_Speed,r_rps,Angle_pid.output+turn_out1+turn_out2);
    PID_incremental_PID_calculation(&L_pid_Speed,l_rps,Angle_pid.output-turn_out1-turn_out2);
	
     Motor_Ctrol(R_pid_Speed.output,L_pid_Speed.output);
 
//    HC_05_Send_Header();
//    sum_temp+= HC_05_Send_Float(Target_Speed);
//    sum_temp+= HC_05_Send_Float(0);
//    sum = (uint8_t)sum_temp;
//    HC_05_Send_Checksum(sum);
//    HC_05_Send_Tail();
	
	
//   Vofa_SendFloat(Velocity_Data.Velocity_y);
//   Vofa_SendFloat(Velocity_Data.Omega);
//   Vofa_SendFloat(a);
//   Vofa_Tail_Send();
	
	//time2 = xTaskGetTickCount()-time1;
	
	osDelayUntil(&preTime,pdMS_TO_TICKS(3));
	
  }
  
  /* USER CODE END StartMotorCtrl */
}

/* USER CODE BEGIN Header_StartRPMGet */
/**
* @brief Function implementing the RPMGetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRPMGet */
void StartRPMGet(void const * argument)
{
  /* USER CODE BEGIN StartRPMGet */
	float r_rps,l_rps,a=0.7;
	TickType_t preTime=xTaskGetTickCount();
	int Tim3_cnt_last,Tim4_cnt_last;
  /* Infinite loop */
  for(;;)
  {
	  
	  
	  TIM3_CNT = __HAL_TIM_GetCounter(&htim3); 
	  TIM4_CNT = __HAL_TIM_GetCounter(&htim4);
	  __HAL_TIM_SetCounter(&htim3,0);
	  __HAL_TIM_SetCounter(&htim4,0);
	  //xSemaphoreTake(Encoder_MutexHandle,portMAX_DELAY);
	  L_RPS = -(float)TIM3_CNT/52/20*1000/3;
    R_RPS = (float)TIM4_CNT/52/20*1000/3;
	  l_rps = L_RPS;
	  r_rps = R_RPS;	 
	  //xSemaphoreGive(Encoder_MutexHandle);
    
//	   Vofa_SendFloat(Target.Pid_Data);
//	   Vofa_SendFloat(r_rps);
//	  Vofa_SendFloat(l_rps);
//	   Vofa_Tail_Send();
	  
 	 

	  osDelayUntil(&preTime,pdMS_TO_TICKS(3));
	 
  }
  
  /* USER CODE END StartRPMGet */
}

/* USER CODE BEGIN Header_Start_Task_Handle */
/**
* @brief Function implementing the Task_Status thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_Handle */
void Start_Task_Handle(void const * argument)
{
  /* USER CODE BEGIN Start_Task_Handle */
  /* Infinite loop */
  sensor_status status = TWO_WHITE;
  sensor_status last_status;
  TASK task;
  uint8_t flag1 = 0,flag2 = 0;
	uint8_t finish_flag=0;
  for(;;)
  {
	if(control_model == AUTO){
		if(finish_flag == 0){
			task = get_task();
			last_status = status;
			status = get_sensor_status();
			switch(task){
			  case task1:
				switch (status)
				{
				case TWO_WHITE:
				  Target_Speed = 1.2;
				  Target_Yaw = 0;
				  Turn_Out = 0;
				  break;
				default:
				  Target_Speed = 0;
				  finish_flag = 1;
				  break;
				}
				break;
			  case task2:
				switch (status)
				{
				case TWO_WHITE:
				  Target_Speed = 0.6;
					if(Yaw<-178&&Yaw<178){
						Target_Speed = 0;
						finish_flag = 1;
					}
				  
				  break;
				case TWO_BLACK:
				  Target_Speed = 0.8;
				  Target_Yaw = -1;
				  Turn_Out = 0;
				  break;
				case L_BLACK_R_WHITE:
				  Target_Speed = 0.5;
				  Target_Yaw = -1;
				  Turn_Out = 0.6;
				  break;
				case L_WHITE_R_BLACK:
				  Target_Speed = 0.5;
				  Target_Yaw = -1;
				  Turn_Out = -0.6;
				  break;
				default:
				  break;
				}
				break;      
			  case task3:
				switch (status)
				{
				  case TWO_WHITE:
					if(last_status != TWO_WHITE){
						Turn_Out = 0;
					
						flag1++;
					 
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					  osDelay(100);
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					  
					}
					

					if(flag1 == 0){
					  Turn_Out = 0;
					  Target_Speed = 0.9;
					  Target_Yaw = 0;
					}else if(flag1 == 1){
					  Turn_Out = 0;
					  Target_Speed = 1;
					  Target_Yaw = 178;
					}else{
					  Turn_Out = 0;
					  flag1 = 0;
					  Target_Speed = 0;
						
						finish_flag = 1;
					
					  
					}
					
					break;
				  case TWO_BLACK:
					if(last_status == TWO_WHITE){
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					  osDelay(100);
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					}
					Target_Speed = 0.8;
					Target_Yaw = -1;
					Turn_Out = -0.1;
					break;
				  case L_BLACK_R_WHITE:
					if(last_status == TWO_WHITE){
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					  osDelay(100);
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					}
					Target_Speed = 0.6;
					Target_Yaw = -1;
					Turn_Out = 0.6;
					break;
				  case L_WHITE_R_BLACK:
					if(last_status == TWO_WHITE){
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					  osDelay(100);
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					}
					Target_Speed = 0.6;
					Target_Yaw = -1;
					Turn_Out = -0.6;
					
					break;
				  default:
					break;
				}
				break;
			  case task4:
				switch (status)
				{
				case TWO_WHITE:
				  if(last_status != TWO_WHITE){
					  Target_Speed = 0.6;
					  Turn_Out = 0;
					 
						Target_Yaw=-35;
					 
					if(Yaw>175&&Yaw<-175){
						Target_Yaw = -140;
						flag1++;
					}
					if(Yaw<5&&Yaw>-5){
						if(flag1>0){
							Target_Speed = 0;
							finish_flag =1;
						}
					}
					
				  }
//				  if(flag1 <2){
//					if(flag1%2 == 0){
//					  Target_Speed = 0.95;
//					  Target_Yaw = -37;
//					}else{
//					  Target_Speed = 0.95;
//					  Target_Yaw = -140;
//					}
//				  }else if (flag1 ==2)
//				  {
//					Target_Speed = 0;
//					flag1 = 0;
//					finish_flag = 1;
//				  }
				  Turn_Out = 0;
				  break;
				case TWO_BLACK:
				  if(last_status == TWO_WHITE){
					if((Yaw>-45&&Yaw<-30)&&(Yaw>170&&Yaw<-170)){
						flag2++;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					  osDelay(100);
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					}
					
				  }
				  if(flag2%2 == 0){
					Target_Speed = 0.6;
					Target_Yaw = -1;
					Turn_Out = -0.2;
				  }else{
					Target_Speed = 0.8;
					Target_Yaw = -1;
					Turn_Out = 0.2;
				  }
				  if(flag2 == 2){
					flag2 = 0;
				  }
				  break;
				  case L_BLACK_R_WHITE:
					if((Yaw>-45&&Yaw<-30)&&(Yaw>170&&Yaw<-170)){
						flag2++;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					  osDelay(100);
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					}
					Target_Speed = 0.5;
					Target_Yaw = -1;
					Turn_Out = 0.7;
					break;
				  case L_WHITE_R_BLACK:
					if((Yaw>-45&&Yaw<-30)&&(Yaw>175&&Yaw<-175)){
						flag2++;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					  osDelay(100);
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					}
					Target_Speed = 0.5;
					Target_Yaw = -1;
					Turn_Out = -0.7;
					break;
				  default:
					break;
				}
				break;
			  case task5:
				switch (status)
				{
				case TWO_WHITE:
				  if(last_status != TWO_WHITE){
					flag1++;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					  osDelay(100);
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
				  }
				  if(flag1 <8){
					if(flag1%2 == 0){
					  Target_Speed = 1.5;
					  Target_Yaw = -38;
					}else{
					  Target_Speed = 1.5;
					  Target_Yaw = 218;
					}
				  }else if (flag1 ==8)
				  {
					Target_Speed = 0;
					flag1 = 0;
					finish_flag = 1;
				  }
				  Turn_Out = 0;
				  break;
				case TWO_BLACK:
				  if(last_status == TWO_WHITE){
					flag2++;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					  osDelay(100);
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
				  }
				  if(flag2%2 == 0){
					Target_Speed = 0.8;
					Target_Yaw = -1;
					Turn_Out = -0.2;
				  }else{
					Target_Speed = 0.8;
					Target_Yaw = -1;
					Turn_Out = 0.2;
				  }
				  break;
				  case L_BLACK_R_WHITE:
					if(last_status == TWO_WHITE){
					  flag2++;
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					  osDelay(100);
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					}
					Target_Speed = 0.6;
					Target_Yaw = -1;
					Turn_Out = 0.6;
					break;
				  case L_WHITE_R_BLACK:
					if(last_status == TWO_WHITE){
					  flag2++;
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					  osDelay(100);
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					}
					Target_Speed = 0.6;
					Target_Yaw = -1;
					Turn_Out = -0.6;
					break;
				  default:
					break;
				}
				break;
			  default:
				Target_Speed = 0;
				Turn_Out = 0;
				Target_Yaw = 0;
				break;
			}
		}else{
			Target_Speed = 0;
			Turn_Out = 0;
		}
		
	}

    osDelay(1);
  }
  /* USER CODE END Start_Task_Handle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

