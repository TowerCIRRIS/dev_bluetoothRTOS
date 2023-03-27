/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "dma.h"
#include "rf.h"
#include "rtc.h"
#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32_seq.h"
//#include "otp.h"
#include "teamATbasic_V1_1.h"
#include "custom_app.h"
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

/* USER CODE BEGIN PV */
char deviceName[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
//static void Config_HSE(void);
void sendConfigBluetooth();
//void getConfigBluetooth();
void getConfigBluetooth(char * cfgString, int stringSize);
void getControlBluetooth(char * cfgString, int stringSize);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */
  //Config_HSE();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_RF_Init();
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_DMA_Init();
  /* USER CODE BEGIN 2 */
 // HAL_GPIO_WritePin(OUT_DEV_PWR_EN_GPIO_Port, OUT_DEV_PWR_EN_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  char btString[53];

  int angle = 10;
  int angleMax = 20;

  int repXmin = 5;
  int repLimitXmin = 10;
  int timeElevationXmin = 15;
  int timeLimitXmin = 20;
  int repTotal = 100;
  int timeElevationTotal = 5;


		 sprintf(btString,"%3d,%3d,%3d,%3d,%3d,%3d",
				 repXmin,
				 repLimitXmin,
				 timeElevationXmin,
				 timeLimitXmin,
				 repTotal,
				 timeElevationTotal
          );

  //livedataUpdateChar(btString);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

 connectControlCallback(getControlBluetooth);
 connectConfigReceiveCallback(getConfigBluetooth);

  uint32_t currentTime = 0;
  uint32_t lastTime = 0;

  sprintf(deviceName,"Montre 1");
  while (1)
  {


	  //UTIL_SEQ_Run( UTIL_SEQ_DEFAULT );
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */

    currentTime = HAL_GetTick();
    	  if(currentTime-lastTime >= 1000)
    	  {
    		  lastTime = currentTime;

    		  sprintf(btString,"%3d,%3d,%3d,%3d,%3d,%3d,",
    		 				 repXmin++,
    		 				 repLimitXmin++,
    		 				 timeElevationXmin++,
    		 				 timeLimitXmin++,
    		 				 repTotal++,
    		 				 timeElevationTotal++
    		           );

    		  livedataUpdateChar(btString);

    		  sprintf(btString,"%3d,%3d,",
    		    		 				 angle++,
    		    		 				 angleMax++
    		    		           );

    		  angleUpdateChar(btString);
    		  sendConfigBluetooth();

    	  }

//    	  if(bt_newConfigPresent())
//    	  {
//    		  getConfigBluetooth();
//    	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */




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
   //char cfgString[53];

//   if(bt_newConfigPresent())
//   {
//	   bt_readConfigString(cfgString);
//   }

   //mainMenu.page_config.pageAlerts.configChange = true;

	if(cfgString[BT_CONFIG_TONE_ALERT] == '1')
	{
		//mainMenu.page_config.pageAlerts.setToneAlertEnable(1);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_TONE_ALERT: On");
		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}
	else
	{
		//mainMenu.page_config.pageAlerts.setToneAlertEnable(0);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_TONE_ALERT: Off");
		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}

	if(cfgString[BT_CONFIG_VIBRATION_ALERT] == '1')
	{
		//mainMenu.page_config.pageAlerts.setVibrationAlertEnable(1);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_VIBRATION_ALERT: On");
		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}
	else
	{
		//mainMenu.page_config.pageAlerts.setVibrationAlertEnable(0);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_VIBRATION_ALERT: Off");
		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}

	if(cfgString[BT_CONFIG_REPETITION_ALERT] == '1')
	{
		//mainMenu.page_config.pageAlerts.setRepetitionAlertEnable(1);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_REPETITION_ALERT: On");
			HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}
	else
	{
		//mainMenu.page_config.pageAlerts.setRepetitionAlertEnable(0);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_REPETITION_ALERT: Off");
		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}

	if(cfgString[BT_CONFIG_TIME_ALERT] == '1')
	{
		//mainMenu.page_config.pageAlerts.setTimeAlertEnable(1);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_TIME_ALERT: On");
		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}
	else
	{
		//mainMenu.page_config.pageAlerts.setTimeAlertEnable(0);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_TIME_ALERT: Off");
		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}

	if(cfgString[BT_CONFIG_ELEVATION_ANGLE_ALERT] == '1')
	{
		//mainMenu.page_config.pageAlerts.setAngleDetectAlertEnable(1);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_ELEVATION_ANGLE_ALERT: On");
		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}
	else
	{
		//mainMenu.page_config.pageAlerts.setAngleDetectAlertEnable(0);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_ELEVATION_ANGLE_ALERT: Off");
		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}

	if(cfgString[BT_CONFIG_MAX_ANGLE_ALERT] == '1')
	{
	  //mainMenu.page_config.pageAlerts.setAngleMaxAlertEnable(1);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_MAX_ANGLE_ALERT: On");
		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}
	else
	{
	  //mainMenu.page_config.pageAlerts.setAngleMaxAlertEnable(0);
		sprintf(serialOutBuffer,"\r\nBT_CONFIG_MAX_ANGLE_ALERT: Off");
		HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);
	}

	//mainMenu.page_config.page_angle_detect.mAngleLimit = arrayToUint(&cfgString[BT_CONFIG_ELEVATION_ANGLE_VALUE] , 3);
	//mainMenu.page_config.page_angle_detect.configChange = true;
	sprintf(serialOutBuffer,"\r\nBT_CONFIG_ELEVATION_ANGLE_VALUE:%u",arrayToUint(&cfgString[BT_CONFIG_ELEVATION_ANGLE_VALUE],3));
	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);


	//mainMenu.page_config.page_time.mTimeTargetMilliseconds = arrayToUint(&cfgString[BT_CONFIG_TIME_VALUE] , 3)*60*1000;
	//mainMenu.page_config.page_time.configChange = true;
	sprintf(serialOutBuffer,"\r\nBT_CONFIG_TIME_VALUE:%u",arrayToUint(&cfgString[BT_CONFIG_TIME_VALUE],3));
	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);

	//mainMenu.page_config.page_Length.mPeriodLength = arrayToUint(&cfgString[BT_CONFIG_PERIOD_TIME_VALUE] , 3)*60*1000;
	//mainMenu.page_config.page_Length.configChange = true;
	sprintf(serialOutBuffer,"\r\nBT_CONFIG_PERIOD_TIME_VALUE:%u",arrayToUint(&cfgString[BT_CONFIG_PERIOD_TIME_VALUE],3));
	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);

	//mainMenu.page_config.page_rep.mRepLimit = arrayToUint(&cfgString[BT_CONFIG_REP_LIMIT_VALUE] , 3);
	//mainMenu.page_config.page_rep.configChange = true;
	sprintf(serialOutBuffer,"\r\nBT_CONFIG_REP_LIMIT_VALUE:%u",arrayToUint(&cfgString[BT_CONFIG_REP_LIMIT_VALUE],3));
	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);

	//mainMenu.page_config.page_angle_max.mAngleLimit = arrayToUint(&cfgString[BT_CONFIG_MAX_ANGLE_VALUE] , 3);
	//mainMenu.page_config.page_angle_max.configChange = true;
	sprintf(serialOutBuffer,"\r\nBT_CONFIG_MAX_ANGLE_VALUE:%u",arrayToUint(&cfgString[BT_CONFIG_MAX_ANGLE_VALUE],3));
	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);

	//mainMenu.page_config.configChange = true;
	//strcpy(mainMenu.page_config.bluetoothDeviceName,&cfgString[BT_CONFIG_DEVICE_NAME]);
	sprintf(serialOutBuffer,"\r\nBT_CONFIG_DEVICE_NAME: ");
	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);

	sprintf(deviceName,&cfgString[BT_CONFIG_DEVICE_NAME]);
	sprintf(serialOutBuffer,deviceName);
	HAL_UART_Transmit(&huart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), HAL_MAX_DELAY);

	//Serial.print("\n Name received:");
	//Serial.println(mainMenu.page_config.bluetoothDeviceName);

	//BLE.setDeviceName(mainMenu.page_config.bluetoothDeviceName);
	//BLE.setDeviceName(DEFAULT_BLUTOOTH_NAME);
	//BLE.advertise();

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
