/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* ledService */
  /* angleService */
  uint8_t               Angle_c_Notification_Status;
  /* liveDataService */
  uint8_t               Livedata_c_Notification_Status;
  /* configService */
  uint8_t               Configflags_c_Notification_Status;
  /* controlService */
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  //uint8_t ButtonStatus;
  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */
btDataReceiveCallbackFunction_t callback_controlReceive;
btDataReceiveCallbackFunction_t callback_configReceive;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* ledService */
/* angleService */
static void Custom_Angle_c_Update_Char(void);
static void Custom_Angle_c_Send_Notification(void);
/* liveDataService */
static void Custom_Livedata_c_Update_Char(void);
static void Custom_Livedata_c_Send_Notification(void);
/* configService */
static void Custom_Configflags_c_Update_Char(void);
static void Custom_Configflags_c_Send_Notification(void);
/* controlService */

/* USER CODE BEGIN PFP */
//static void P2PS_Send_Notification(void);



/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* ledService */
    case CUSTOM_STM_SWITCH_C_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SWITCH_C_READ_EVT */

      /* USER CODE END CUSTOM_STM_SWITCH_C_READ_EVT */
      break;

    case CUSTOM_STM_SWITCH_C_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SWITCH_C_WRITE_NO_RESP_EVT */

      /* USER CODE END CUSTOM_STM_SWITCH_C_WRITE_NO_RESP_EVT */
      break;

    /* angleService */
    case CUSTOM_STM_ANGLE_C_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ANGLE_C_READ_EVT */

      /* USER CODE END CUSTOM_STM_ANGLE_C_READ_EVT */
      break;

    case CUSTOM_STM_ANGLE_C_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ANGLE_C_NOTIFY_ENABLED_EVT */
    	Custom_App_Context.Angle_c_Notification_Status = 1;
      /* USER CODE END CUSTOM_STM_ANGLE_C_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_ANGLE_C_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ANGLE_C_NOTIFY_DISABLED_EVT */
    	Custom_App_Context.Angle_c_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_ANGLE_C_NOTIFY_DISABLED_EVT */
      break;

    /* liveDataService */
    case CUSTOM_STM_LIVEDATA_C_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_LIVEDATA_C_READ_EVT */

      /* USER CODE END CUSTOM_STM_LIVEDATA_C_READ_EVT */
      break;

    case CUSTOM_STM_LIVEDATA_C_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_LIVEDATA_C_NOTIFY_ENABLED_EVT */
    	Custom_App_Context.Livedata_c_Notification_Status =1;
      /* USER CODE END CUSTOM_STM_LIVEDATA_C_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_LIVEDATA_C_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_LIVEDATA_C_NOTIFY_DISABLED_EVT */
    	Custom_App_Context.Livedata_c_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_LIVEDATA_C_NOTIFY_DISABLED_EVT */
      break;

    /* configService */
    case CUSTOM_STM_CONFIGFLAGS_C_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CONFIGFLAGS_C_READ_EVT */

      /* USER CODE END CUSTOM_STM_CONFIGFLAGS_C_READ_EVT */
      break;

    case CUSTOM_STM_CONFIGFLAGS_C_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CONFIGFLAGS_C_WRITE_EVT */


         //APP_DBG_MSG("\r\n\r** CUSTOM_STM_CONFIGFLAGS_C_WRITE_EVT \n");
         //APP_DBG_MSG("\r\n\r** Length: %d \n", pNotification->DataTransfered.Length);
//         for(int i = 0 ; i < pNotification->DataTransfered.Length; i++)
//         	{
//         		configString[i] = pNotification->DataTransfered.pPayload[i];
//         	}
//
//         	newConfigPresent = true;
    		callback_configReceive(pNotification->DataTransfered.pPayload,pNotification->DataTransfered.Length);

//         if (pNotification->DataTransfered.pPayload[1] == 0x01)
//         {
//           HAL_GPIO_WritePin(Blue_Led_GPIO_Port, Blue_Led_Pin, GPIO_PIN_SET);
//         }
//         if (pNotification->DataTransfered.pPayload[1] == 0x00)
//         {
//           HAL_GPIO_WritePin(Blue_Led_GPIO_Port, Blue_Led_Pin, GPIO_PIN_RESET);
//         }


      /* USER CODE END CUSTOM_STM_CONFIGFLAGS_C_WRITE_EVT */
      break;

    case CUSTOM_STM_CONFIGFLAGS_C_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CONFIGFLAGS_C_NOTIFY_ENABLED_EVT */
    	Custom_App_Context.Configflags_c_Notification_Status = 1;
      /* USER CODE END CUSTOM_STM_CONFIGFLAGS_C_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_CONFIGFLAGS_C_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CONFIGFLAGS_C_NOTIFY_DISABLED_EVT */
    	Custom_App_Context.Configflags_c_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_CONFIGFLAGS_C_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_CONFIGFLAGSREQ_C_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CONFIGFLAGSREQ_C_READ_EVT */

      /* USER CODE END CUSTOM_STM_CONFIGFLAGSREQ_C_READ_EVT */
      break;

    case CUSTOM_STM_CONFIGFLAGSREQ_C_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CONFIGFLAGSREQ_C_WRITE_EVT */
        APP_DBG_MSG("\r\n\r** CUSTOM_STM_CONFIGFLAGSREQ_C_WRITE_EVT \n");
        APP_DBG_MSG("\r\n\r** Length: %d \n", pNotification->DataTransfered.Length);
      /* USER CODE END CUSTOM_STM_CONFIGFLAGSREQ_C_WRITE_EVT */
      break;

    /* controlService */
    case CUSTOM_STM_CONTROL_C_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CONTROL_C_WRITE_EVT */
    	callback_controlReceive(pNotification->DataTransfered.pPayload,pNotification->DataTransfered.Length);
      /* USER CODE END CUSTOM_STM_CONTROL_C_WRITE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	  //Custom_App_Context.ButtonStatus = 0;
	  Custom_App_Context.Angle_c_Notification_Status = 1;
	  Custom_App_Context.Configflags_c_Notification_Status = 1;
	  Custom_App_Context.Livedata_c_Notification_Status = 1;

	  Custom_Configflags_c_Update_Char();
	  Custom_Angle_c_Update_Char();
	  //Custom_Livedata_c_Update_Char();


  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* ledService */
/* angleService */
void Custom_Angle_c_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Angle_c_UC_1*/

  char btString[50];
  sprintf(btString,"%d,%d,",25,120);
	for(int i = 0 ; i < 10; i++)
	{
		UpdateCharData[i] = btString[i];
	}
  updateflag = 1; // Custom_App_Context.Angle_c_Notification_Status

  /* USER CODE END Angle_c_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_ANGLE_C, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Angle_c_UC_Last*/

  /* USER CODE END Angle_c_UC_Last*/
  return;
}

void Custom_Angle_c_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Angle_c_NS_1*/
  UpdateCharData[0] = 29;
  updateflag = 1; // Custom_App_Context.Angle_c_Notification_Status
  /* USER CODE END Angle_c_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_ANGLE_C, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Angle_c_NS_Last*/

  /* USER CODE END Angle_c_NS_Last*/

  return;
}

/* liveDataService */
void Custom_Livedata_c_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Livedata_c_UC_1*/
  char btString[50];
  sprintf(btString,"%3d,%3d,%3d,%3d,%3d,%3d",
          5,
          10,
          2,
          3,
          100,
          25
          );
  	for(int i = 0 ; i < 24; i++)
  	{
  		UpdateCharData[i] = btString[i];
  	}


  updateflag = 1;  //Custom_App_Context.Livedata_c_Notification_Status
  /* USER CODE END Livedata_c_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_LIVEDATA_C, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Livedata_c_UC_Last*/

  /* USER CODE END Livedata_c_UC_Last*/
  return;
}

void Custom_Livedata_c_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Livedata_c_NS_1*/
  UpdateCharData[0] = 0xFF;
  UpdateCharData[1] = 0x02;
  updateflag = 1;  //Custom_App_Context.Livedata_c_Notification_Status
  /* USER CODE END Livedata_c_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_LIVEDATA_C, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Livedata_c_NS_Last*/

  /* USER CODE END Livedata_c_NS_Last*/

  return;
}

/* configService */
void Custom_Configflags_c_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Configflags_c_UC_1*/
//  UpdateCharData[0] = 0x11;
//  UpdateCharData[1] = 0x11;
//  UpdateCharData[2] = 0x11;
//  updateflag = 1;  //Custom_App_Context.Configflags_c_Notification_Status
  /* USER CODE END Configflags_c_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CONFIGFLAGS_C, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Configflags_c_UC_Last*/

  /* USER CODE END Configflags_c_UC_Last*/
  return;
}

void Custom_Configflags_c_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Configflags_c_NS_1*/
  UpdateCharData[0] = 0xFF;
    UpdateCharData[1] = 0xFF;
    UpdateCharData[2] = 0xFF;
    updateflag = 1;  //Custom_App_Context.Configflags_c_Notification_Status
  /* USER CODE END Configflags_c_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CONFIGFLAGS_C, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Configflags_c_NS_Last*/

  /* USER CODE END Configflags_c_NS_Last*/

  return;
}

/* controlService */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/


void livedataUpdateChar(char *btString) /* Property Read */
{

	for(int i = 0 ; i < 24; i++)
	{
		UpdateCharData[i] = btString[i];
	}

    Custom_STM_App_Update_Char(CUSTOM_STM_LIVEDATA_C, (uint8_t *)UpdateCharData);

  return;
}

void angleUpdateChar(char *btString) /* Property Read */
{

	for(int i = 0 ; i < 10; i++)
	{
		UpdateCharData[i] = btString[i];
	}

    Custom_STM_App_Update_Char(CUSTOM_STM_ANGLE_C, (uint8_t *)UpdateCharData);

}

void bt_updateConfigString(char *btString) /* Property Read */
{

	for(int i = 0 ; i < 53; i++)
	{
		UpdateCharData[i] = btString[i];
	}

    Custom_STM_App_Update_Char(CUSTOM_STM_CONFIGFLAGS_C, (uint8_t *)UpdateCharData);

}




void connectControlCallback(btDataReceiveCallbackFunction_t cbFunc)
{
	callback_controlReceive = cbFunc;
}

void connectConfigReceiveCallback(btDataReceiveCallbackFunction_t cbFunc)
{
	callback_configReceive = cbFunc;
}


/* USER CODE END FD_LOCAL_FUNCTIONS*/
