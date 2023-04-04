/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "gpio.h"
#include "usart.h"
#include "teamATbasic_V1_1.h"
#include "custom_app.h"
#include "app_ble.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BT_CONFIG_TONE_ALERT             0
#define BT_CONFIG_VIBRATION_ALERT        1
#define BT_CONFIG_REPETITION_ALERT       2
#define BT_CONFIG_TIME_ALERT             3
#define BT_CONFIG_ELEVATION_ANGLE_ALERT  4
#define BT_CONFIG_MAX_ANGLE_ALERT        5
#define BT_CONFIG_ELEVATION_ANGLE_VALUE  6
#define BT_CONFIG_TIME_VALUE             9
#define BT_CONFIG_PERIOD_TIME_VALUE      12
#define BT_CONFIG_REP_LIMIT_VALUE        15
#define BT_CONFIG_MAX_ANGLE_VALUE        18
#define BT_CONFIG_DEVICE_NAME             21
#define BT_CONFIG_BUFFER_SIZE            (21+BLUTOOTH_NAME_MAX_SIZE)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
char deviceName[50];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2200 * 4, //128 * 4,
  .priority = (osPriority_t) osPriorityNormal

};

///* Definitions for displayTask */
//osThreadId_t displayTaskHandle;
//const osThreadAttr_t displayTask_attributes = {
//  .name = "displayTask",
//  .stack_size = 5000 * 4,
//  .priority = (osPriority_t) osPriorityBelowNormal
//  };
/* Definitions for displayTask */
//osThreadId_t displayTaskHandle;
//const osThreadAttr_t displayTask_attributes = {
//  .name = "displayTask",
//  .stack_size = 1000 * 4,
//  .priority = (osPriority_t) osPriorityLow
//
//};
/* Definitions for sdLogTasks */
//osThreadId_t sdLogTasksHandle;
//const osThreadAttr_t sdLogTasks_attributes = {
//  .name = "sdLogTasks",
//  .stack_size = 5000 * 4,
//  .priority = (osPriority_t) osPriorityBelowNormal
//};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void sendConfigBluetooth();
//void getConfigBluetooth();
void getConfigBluetooth(char * cfgString, int stringSize);
void getControlBluetooth(char * cfgString, int stringSize);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartDisplayTask(void *argument);
void StartSdLogTasks(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of defaultTask */


  /* creation of displayTask */
  //displayTaskHandle = osThreadNew(StartDisplayTask, NULL, &displayTask_attributes);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of sdLogTasks */
  //sdLogTasksHandle = osThreadNew(StartSdLogTasks, NULL, &sdLogTasks_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

char btString[53];

 int angle = 10;
 int angleMax = 20;

 int repXmin = 5;
 int repLimitXmin = 10;
 int timeElevationXmin = 15;
 int timeLimitXmin = 20;
 int repTotal = 100;
 int timeElevationTotal = 5;




/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
//	sprintf(btString,"%3d,%3d,%3d,%3d,%3d,%3d",
//					 repXmin,
//					 repLimitXmin,
//					 timeElevationXmin,
//					 timeLimitXmin,
//					 repTotal,
//					 timeElevationTotal
//	         );
//
//	connectControlCallback(getControlBluetooth);
//	 connectConfigReceiveCallback(getConfigBluetooth);
//
//	  uint32_t currentTime = 0;
//	  uint32_t lastTime = 0;
//
//	  sprintf(deviceName,"Montre 1");

  /* Infinite loop */
  for(;;)
  {
//	  currentTime = HAL_GetTick();
//	      	  if(currentTime-lastTime >= 1000)
//	      	  {
//	      		  lastTime = currentTime;
//
//	      		  sprintf(btString,"%3d,%3d,%3d,%3d,%3d,%3d,",
//	      		 				 repXmin++,
//	      		 				 repLimitXmin++,
//	      		 				 timeElevationXmin++,
//	      		 				 timeLimitXmin++,
//	      		 				 repTotal++,
//	      		 				 timeElevationTotal++
//	      		           );
//
//	      		  APP_BLE_ConnStatus_t status = APP_BLE_Get_Server_Connection_Status();
//
//	      		  if(status == APP_BLE_CONNECTED_SERVER || status == APP_BLE_CONNECTED_CLIENT )
//	      		  {
//	      			livedataUpdateChar(btString);
//
//	      			sprintf(btString,"%3d,%3d,",
//												 angle++,
//												 angleMax++
//										   );
//
//					  angleUpdateChar(btString);
//					  sendConfigBluetooth();
//
//	      		  }
//
//
//	      	  }

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
	//lowPrioritySetup();
	/* Infinite loop */
  for(;;)
  {
	//lowPriorityLoop();
    osDelay(10);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartSdLogTasks */
/**
* @brief Function implementing the sdLogTasks thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSdLogTasks */
void StartSdLogTasks(void *argument)
{
  /* USER CODE BEGIN StartSdLogTasks */
	//sdTaskSetup();
  /* Infinite loop */
  for(;;)
  {
	//sdTaskLoop();
    osDelay(10);
  }
  /* USER CODE END StartSdLogTasks */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void sendConfigBluetooth()
{
   char cfgString[53];

   for(int i = 0; i < 53; i++)
   {
	   cfgString[i] = '\0';
   }
      if(1)//mainMenu.page_config.pageAlerts.getToneAlertEnable())
      {
          cfgString[BT_CONFIG_TONE_ALERT] = '1';
      }
      else
      {
          cfgString[BT_CONFIG_TONE_ALERT] = '0';
      }

      if(0)//mainMenu.page_config.pageAlerts.getVibrationAlertEnable())
      {
          cfgString[BT_CONFIG_VIBRATION_ALERT] = '1';
      }
      else
      {
          cfgString[BT_CONFIG_VIBRATION_ALERT] = '0';
      }

      if(1)//mainMenu.page_config.pageAlerts.getRepetitionAlertEnable())
      {
          cfgString[BT_CONFIG_REPETITION_ALERT] = '1';
      }
      else
      {
          cfgString[BT_CONFIG_REPETITION_ALERT] = '0';
      }

      if(1)//mainMenu.page_config.pageAlerts.getTimeAlertEnable())
      {
          cfgString[BT_CONFIG_TIME_ALERT] = '1';
      }
      else
      {
          cfgString[BT_CONFIG_TIME_ALERT] = '0';
      }

      if(1)//mainMenu.page_config.pageAlerts.getAngleDetectAlertEnable())
      {
          cfgString[BT_CONFIG_ELEVATION_ANGLE_ALERT] = '1';
      }
      else
      {
          cfgString[BT_CONFIG_ELEVATION_ANGLE_ALERT] = '0';
      }

      if(1)//mainMenu.page_config.pageAlerts.getAngleMaxAlertEnable())
      {
        cfgString[BT_CONFIG_MAX_ANGLE_ALERT] = '1';
      }
      else
      {
        cfgString[BT_CONFIG_MAX_ANGLE_ALERT] = '0';
      }

      uintToArray((uint16_t)12, &cfgString[BT_CONFIG_ELEVATION_ANGLE_VALUE], 3 );

      uintToArray((uint16_t)5, &cfgString[BT_CONFIG_TIME_VALUE], 3);

      uintToArray((uint16_t)10, &cfgString[BT_CONFIG_PERIOD_TIME_VALUE], 3 );

      uintToArray((uint16_t)20, &cfgString[BT_CONFIG_REP_LIMIT_VALUE], 3 );

      uintToArray((uint16_t)145, &cfgString[BT_CONFIG_MAX_ANGLE_VALUE], 3 );



      strcpy(&cfgString[BT_CONFIG_DEVICE_NAME],deviceName);

     //Serial.println("Sent max value:");
      //Serial.println((uint16_t)(mainMenu.page_config.page_angle_max.mAngleLimit));


      bt_updateConfigString(cfgString);

}
char serialOutBuffer[256];

void getControlBluetooth(char * cfgString, int stringSize)
{

	uint8_t controlValue;

	controlValue = (uint8_t)cfgString[0];

	 switch(controlValue)
	      {
	            case 1: // reset zero

	            	sprintf(serialOutBuffer,"\r\nControl: Zero reset");
	            	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	                //elevationFlagFirst = 0; // Reset zero
	            break;

	            case 2: // reset max angle
	            	sprintf(serialOutBuffer,"\r\nControl: Max Angle Reset reset");
	            	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	                //mainMenu.resetMaxAngle();
	            break;

	            case 3: // reset repetitions last x min

	                //TODO reset repetitions last x min
	            break;

	            case 4: // reset time last x min

	              //TODO reset time last x min
	            break;

	             case 5: // reset repetitions total

	                //TODO reset repetitions total
	            break;

	            case 6: // reset time total

	              //TODO reset time total
	            break;

	            default:
	            break;
	      }

}


void getConfigBluetooth(char * cfgString, int stringSize)
{
//   //char cfgString[53];
//
////   if(bt_newConfigPresent())
////   {
////	   bt_readConfigString(cfgString);
////   }
//
//   //mainMenu.page_config.pageAlerts.configChange = true;
//
//	if(cfgString[BT_CONFIG_TONE_ALERT] == '1')
//	{
//		//mainMenu.page_config.pageAlerts.setToneAlertEnable(1);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_TONE_ALERT: On");
//		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//	else
//	{
//		//mainMenu.page_config.pageAlerts.setToneAlertEnable(0);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_TONE_ALERT: Off");
//		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//
//	if(cfgString[BT_CONFIG_VIBRATION_ALERT] == '1')
//	{
//		//mainMenu.page_config.pageAlerts.setVibrationAlertEnable(1);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_VIBRATION_ALERT: On");
//		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//	else
//	{
//		//mainMenu.page_config.pageAlerts.setVibrationAlertEnable(0);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_VIBRATION_ALERT: Off");
//		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//
//	if(cfgString[BT_CONFIG_REPETITION_ALERT] == '1')
//	{
//		//mainMenu.page_config.pageAlerts.setRepetitionAlertEnable(1);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_REPETITION_ALERT: On");
//			HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//	else
//	{
//		//mainMenu.page_config.pageAlerts.setRepetitionAlertEnable(0);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_REPETITION_ALERT: Off");
//		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//
//	if(cfgString[BT_CONFIG_TIME_ALERT] == '1')
//	{
//		//mainMenu.page_config.pageAlerts.setTimeAlertEnable(1);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_TIME_ALERT: On");
//		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//	else
//	{
//		//mainMenu.page_config.pageAlerts.setTimeAlertEnable(0);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_TIME_ALERT: Off");
//		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//
//	if(cfgString[BT_CONFIG_ELEVATION_ANGLE_ALERT] == '1')
//	{
//		//mainMenu.page_config.pageAlerts.setAngleDetectAlertEnable(1);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_ELEVATION_ANGLE_ALERT: On");
//		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//	else
//	{
//		//mainMenu.page_config.pageAlerts.setAngleDetectAlertEnable(0);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_ELEVATION_ANGLE_ALERT: Off");
//		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//
//	if(cfgString[BT_CONFIG_MAX_ANGLE_ALERT] == '1')
//	{
//	  //mainMenu.page_config.pageAlerts.setAngleMaxAlertEnable(1);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_MAX_ANGLE_ALERT: On");
//		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//	else
//	{
//	  //mainMenu.page_config.pageAlerts.setAngleMaxAlertEnable(0);
//		sprintf(serialOutBuffer,"\r\nBT_CONFIG_MAX_ANGLE_ALERT: Off");
//		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//	}
//
//	//mainMenu.page_config.page_angle_detect.mAngleLimit = arrayToUint(&cfgString[BT_CONFIG_ELEVATION_ANGLE_VALUE] , 3);
//	//mainMenu.page_config.page_angle_detect.configChange = true;
//	sprintf(serialOutBuffer,"\r\nBT_CONFIG_ELEVATION_ANGLE_VALUE:%u",arrayToUint(&cfgString[BT_CONFIG_ELEVATION_ANGLE_VALUE],3));
//	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//
//
//	//mainMenu.page_config.page_time.mTimeTargetMilliseconds = arrayToUint(&cfgString[BT_CONFIG_TIME_VALUE] , 3)*60*1000;
//	//mainMenu.page_config.page_time.configChange = true;
//	sprintf(serialOutBuffer,"\r\nBT_CONFIG_TIME_VALUE:%u",arrayToUint(&cfgString[BT_CONFIG_TIME_VALUE],3));
//	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//
//	//mainMenu.page_config.page_Length.mPeriodLength = arrayToUint(&cfgString[BT_CONFIG_PERIOD_TIME_VALUE] , 3)*60*1000;
//	//mainMenu.page_config.page_Length.configChange = true;
//	sprintf(serialOutBuffer,"\r\nBT_CONFIG_PERIOD_TIME_VALUE:%u",arrayToUint(&cfgString[BT_CONFIG_PERIOD_TIME_VALUE],3));
//	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//
//	//mainMenu.page_config.page_rep.mRepLimit = arrayToUint(&cfgString[BT_CONFIG_REP_LIMIT_VALUE] , 3);
//	//mainMenu.page_config.page_rep.configChange = true;
//	sprintf(serialOutBuffer,"\r\nBT_CONFIG_REP_LIMIT_VALUE:%u",arrayToUint(&cfgString[BT_CONFIG_REP_LIMIT_VALUE],3));
//	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//
//	//mainMenu.page_config.page_angle_max.mAngleLimit = arrayToUint(&cfgString[BT_CONFIG_MAX_ANGLE_VALUE] , 3);
//	//mainMenu.page_config.page_angle_max.configChange = true;
//	sprintf(serialOutBuffer,"\r\nBT_CONFIG_MAX_ANGLE_VALUE:%u",arrayToUint(&cfgString[BT_CONFIG_MAX_ANGLE_VALUE],3));
//	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//
//	//mainMenu.page_config.configChange = true;
//	//strcpy(mainMenu.page_config.bluetoothDeviceName,&cfgString[BT_CONFIG_DEVICE_NAME]);
//	sprintf(serialOutBuffer,"\r\nBT_CONFIG_DEVICE_NAME: ");
//	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//
//	sprintf(deviceName,&cfgString[BT_CONFIG_DEVICE_NAME]);
//	sprintf(serialOutBuffer,deviceName);
//	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
//
//	//Serial.print("\n Name received:");
//	//Serial.println(mainMenu.page_config.bluetoothDeviceName);
//
//	//BLE.setDeviceName(mainMenu.page_config.bluetoothDeviceName);
//	//BLE.setDeviceName(DEFAULT_BLUTOOTH_NAME);
//	//BLE.advertise();

}
/* USER CODE END Application */

