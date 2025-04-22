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

typedef enum{
  task1 = 0,
  task2,
  task3,
  task4,
}TASK;

extern Velocity_t Velocity_Data;
extern union HC_05_PID_t HC_05_PID;
extern union Vofa_Pid vofa_kp,vofa_ki,vofa_kd,vofa_Target;
int16_t TIM3_CNT,TIM4_CNT;
float L_RPS,R_RPS;
PID L_pid_Speed,R_pid_Speed,Angle_pid,Pitch_pid,Turn_pid;
float R_Target_RPS,L_Target_RPS,Target_RPS;
float Roll,Pitch,Yaw;
short Gyro[3];
short Gyro_y,Gyro_x,Gyro_z;
/* USER CODE END Variables */
osThreadId DebugTaskHandle;
osThreadId MotorCtrlTaskHandle;
osThreadId RPMGetTaskHandle;
osMutexId Encoder_MutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebug(void const * argument);
void StartMotorCtrl(void const * argument);
void StartRPMGet(void const * argument);

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
    StartUart2ReceiveIT();
    StartUart1ReceiveIT();
    float r_rps,l_rps,r_output,l_output,angle_error,Target_Speed,Target_angle;
    float pitch,roll,yaw;

    PID_parameter_init(&Pitch_pid,0.8,0.002,11,20,10,0);
    PID_parameter_init(&L_pid_Speed,1420,100,0,10000,5000,0);
    PID_parameter_init(&R_pid_Speed,1420,100,0,10000,5000,0);
    PID_parameter_init(&Angle_pid,0.25,0.003,1,15,15,0);
    PID_parameter_init(&Turn_pid,0,0,0,180,90,0);
    Target_RPS = 0;
    TickType_t preTime = xTaskGetTickCount();
    uint32_t sum_temp =0;
    uint8_t sum=0;
	float turn_out=0;
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

    Target_Speed = 5;
	  if(HC_05_PID.Data[0]!=0||HC_05_PID.Data[1]!=0||HC_05_PID.Data[2]!=0){
		  PID_reset_PID(&Pitch_pid,HC_05_PID.Data[0],HC_05_PID.Data[1],HC_05_PID.Data[2]);
	  }
    Pitch_pid_calculation(&Pitch_pid,Target_Speed,r_rps);
    Target_angle = Pitch_pid.output;
    angle_error = pitch-Target_angle;
    if(fabsf(pitch)>55){
	    Angle_pid.output = 0;
    }else{
      PID_position_PID_calculation_by_error(&Angle_pid,angle_error);
    }
    Target_RPS = vofa_Target.Pid_Data;
    PID_incremental_PID_calculation(&R_pid_Speed,r_rps,Angle_pid.output+turn_out);
    PID_incremental_PID_calculation(&L_pid_Speed,l_rps,Angle_pid.output-turn_out);
	
    Motor_Ctrol(R_pid_Speed.output,L_pid_Speed.output);
 
//    HC_05_Send_Header();
//    sum_temp+= HC_05_Send_Float(Target_Speed);
//    sum_temp+= HC_05_Send_Float(0);
//    sum = (uint8_t)sum_temp;
//    HC_05_Send_Checksum(sum);
//    HC_05_Send_Tail();
	
	
//    Vofa_SendFloat(r_rps);
//    Vofa_SendFloat(l_rps);
//    Vofa_Tail_Send();
	
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

