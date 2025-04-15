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
extern union Vofa_Pid kp,ki,kd,Target;
int16_t TIM3_CNT,TIM4_CNT;
float L_RPM,R_RPM;
PID L_pid_Speed,R_pid_Speed,L_pid_Angle,R_pid_Angle,Pid_Angle;
float R_Target_RPM,L_Target_RPM,Target_RPM;
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
  
//	while(mpu_dmp_init()){
//		HAL_Delay(200);
//	}
  /* Infinite loop */
  for(;;)
  {
	 //time1 = xTaskGetTickCount(); 
    //xSemaphoreTake(Ora_MutexHandle,portMAX_DELAY);
    if(MPU6050_DMP_Get_Date(&Pitch, &Roll, &Yaw)==0){
		pitch = -Pitch;
		roll = Roll;  
		yaw = Yaw;
	}
	if(mpu_get_gyro_reg(Gyro,NULL)==0){
		Gyro_y = Gyro[1];
		Gyro_x = Gyro[0];
		Gyro_z = Gyro[2];
	}  
	//time2 = xTaskGetTickCount()-time1;
//	if(mpu_dmp_get_data(&Pitch,&Roll,&Yaw)==0){
//		pitch = Pitch;
//		roll = Roll;  
//		yaw = Yaw;
//	}
//    
	  
    //xSemaphoreGive(Ora_MutexHandle);
//    OLED_ShowSignedNum(1,1,pitch,5); 
//    OLED_ShowSignedNum(2,1,roll,5);
//    OLED_ShowSignedNum(3,1,yaw,5);
//    Vofa_SendFloat(roll);
//    Vofa_SendFloat(pitch);
//    Vofa_SendFloat(yaw);
//    Vofa_Tail();
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
	StartUartReceiveIT();
  float r_rpm,l_rpm,r_output,l_output,angle_error;
	float pitch,roll,yaw;
	
  PID_parameter_init(&L_pid_Speed,30,4,0.5,10000,5000,0);
	PID_parameter_init(&R_pid_Speed,30,4,0.5,10000,5000,0);
  PID_parameter_init(&Pid_Angle,15,1,60,1000,1000,0);
  Target_RPM = 0;
	TickType_t preTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {		
     
    //xSemaphoreTake(Encoder_MutexHandle,portMAX_DELAY);
    r_rpm = R_RPM;
    l_rpm = L_RPM;
    //xSemaphoreGive(Encoder_MutexHandle);

    //xSemaphoreTake(Ora_MutexHandle,portMAX_DELAY);
    pitch = -Pitch;
	roll = Roll;  
    yaw = Yaw;
    //xSemaphoreGive(Ora_MutexHandle);  
	if(kp.Pid_Data!=0||ki.Pid_Data!=0||kd.Pid_Data!=0){
		  PID_reset_PID(&R_pid_Speed,kp.Pid_Data,ki.Pid_Data,kd.Pid_Data);
	  }
	  angle_error = pitch-0;
	  if(fabsf(pitch)>55){
		Pid_Angle.output = 0;
	  }else{
		PID_position_PID_calculation_by_error(&Pid_Angle,angle_error);
	  }
	//Target_RPM=Target.Pid_Data;
    PID_incremental_PID_calculation(&R_pid_Speed,r_rpm,Pid_Angle.output);
    PID_incremental_PID_calculation(&L_pid_Speed,l_rpm,Pid_Angle.output);
	if(R_pid_Speed.output>0){
		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		 r_output = R_pid_Speed.output;
	}
	else if(R_pid_Speed.output<0){
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		  r_output = -R_pid_Speed.output;
	}
	 if(L_pid_Speed.output>0){
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
		  l_output = L_pid_Speed.output;
	}
	else if(L_pid_Speed.output<0){
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
		  l_output = -L_pid_Speed.output;
	}
	
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,l_output);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,r_output);
     
//	Vofa_SendFloat(pitch);
//	Vofa_SendFloat(0);
//	Vofa_SendFloat(r_rpm);
//	Vofa_Tail();
	osDelayUntil(&preTime,pdMS_TO_TICKS(4));
	
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
	float r_rpm,l_rpm;
	TickType_t preTime=xTaskGetTickCount();
	
  /* Infinite loop */
  for(;;)
  {
	  
	  TIM3_CNT = __HAL_TIM_GetCounter(&htim3); 
	  TIM4_CNT = __HAL_TIM_GetCounter(&htim4);
	  __HAL_TIM_SetCounter(&htim3,0);
	  __HAL_TIM_SetCounter(&htim4,0);
	  //xSemaphoreTake(Encoder_MutexHandle,portMAX_DELAY);
	  L_RPM = -(float)TIM3_CNT/52/20*1000/3*60.0f;
      R_RPM = (float)TIM4_CNT/52/20*1000/3*60.0f;
	  l_rpm = L_RPM;
	  r_rpm = R_RPM;
	  //xSemaphoreGive(Encoder_MutexHandle);
	   Vofa_SendFloat(Target.Pid_Data);
	   Vofa_SendFloat(r_rpm);
	  Vofa_SendFloat(l_rpm);
	   Vofa_Tail();
	  
	  
 	
	  osDelayUntil(&preTime,pdMS_TO_TICKS(3));
	 
  }
  
  /* USER CODE END StartRPMGet */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

