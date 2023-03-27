/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.c
  * @author  MCD Application Team
  * @brief   Custom Example Service.
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
#include "common_blesvc.h"
#include "custom_stm.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  CustomLedsHdle;                    /**< ledService handle */
  uint16_t  CustomSwitch_CHdle;                  /**< switchCharacteristic handle */
  uint16_t  CustomAnglesHdle;                    /**< angleService handle */
  uint16_t  CustomAngle_CHdle;                  /**< AngleCharacteristic handle */
  uint16_t  CustomLivedatasHdle;                    /**< liveDataService handle */
  uint16_t  CustomLivedata_CHdle;                  /**< liveDataCharacteristic handle */
  uint16_t  CustomConfigsHdle;                    /**< configService handle */
  uint16_t  CustomConfigflags_CHdle;                  /**< configFlagsCharacteristic handle */
  uint16_t  CustomConfigflagsreq_CHdle;                  /**< configFlagsRequestCharacteristic handle */
  uint16_t  CustomControlsHdle;                    /**< controlService handle */
  uint16_t  CustomControl_CHdle;                  /**< controlCharacteristic handle */
}CustomContext_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
/* USER CODE BEGIN PM */
//#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
//#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
uint8_t SizeSwitch_C = 1;
uint8_t SizeAngle_C = 10;
uint8_t SizeLivedata_C = 24;
uint8_t SizeConfigflags_C = 53;
uint8_t SizeConfigflagsreq_C = 1;
uint8_t SizeControl_C = 1;

/**
 * START of Section BLE_DRIVER_CONTEXT
 */
static CustomContext_t CustomContext;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *pckt);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */

/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Hardware Characteristics Service */
/*
 The following 128bits UUIDs have been generated from the random UUID
 generator:
 D973F2E0-B19E-11E2-9E96-0800200C9A66: Service 128bits UUID
 D973F2E1-B19E-11E2-9E96-0800200C9A66: Characteristic_1 128bits UUID
 D973F2E2-B19E-11E2-9E96-0800200C9A66: Characteristic_2 128bits UUID
 */
#define COPY_LEDSERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x19,0xb1,0x00,0x00,0xe8,0xf2,0x53,0x7e,0x4f,0x6c,0xd1,0x04,0x76,0x8a,0x12,0x14)
#define COPY_SWITCHCHARACTERISTIC_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x19,0xb1,0x00,0x01,0xe8,0xf2,0x53,0x7e,0x4f,0x6c,0xd1,0x04,0x76,0x8a,0x12,0x14)
#define COPY_ANGLESERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x20,0xb1,0x00,0x00,0xe8,0xf2,0x53,0x7e,0x4f,0x6c,0xd1,0x04,0x76,0x8a,0x12,0x14)
#define COPY_ANGLECHARACTERISTIC_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x20,0xb1,0x00,0x01,0xe8,0xf2,0x53,0x7e,0x4f,0x6c,0xd1,0x04,0x76,0x8a,0x12,0x14)
#define COPY_LIVEDATASERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x22,0xb1,0x00,0x00,0xe8,0xf2,0x53,0x7e,0x4f,0x6c,0xd1,0x04,0x76,0x8a,0x12,0x14)
#define COPY_LIVEDATACHARACTERISTIC_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x22,0xb1,0x00,0x01,0xe8,0xf2,0x53,0x7e,0x4f,0x6c,0xd1,0x04,0x76,0x8a,0x12,0x14)
#define COPY_CONFIGSERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x23,0xb1,0x00,0x00,0xe8,0xf2,0x53,0x7e,0x4f,0x6c,0xd1,0x04,0x76,0x8a,0x12,0x14)
#define COPY_CONFIGFLAGSCHARACTERISTIC_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x23,0xb1,0x00,0x01,0xe8,0xf2,0x53,0x7e,0x4f,0x6c,0xd1,0x04,0x76,0x8a,0x12,0x14)
#define COPY_CONFIGFLAGSREQUESTCHARACTERISTIC_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x23,0xb1,0x00,0x02,0xe8,0xf2,0x53,0x7e,0x4f,0x6c,0xd1,0x04,0x76,0x8a,0x12,0x14)
#define COPY_CONTROLSERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x28,0xb1,0x00,0x00,0xe8,0xf2,0x53,0x7e,0x4f,0x6c,0xd1,0x04,0x76,0x8a,0x12,0x14)
#define COPY_CONTROLCHARACTERISTIC_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x28,0xb1,0x00,0x01,0xe8,0xf2,0x53,0x7e,0x4f,0x6c,0xd1,0x04,0x76,0x8a,0x12,0x14)

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  Custom_STM_App_Notification_evt_t     Notification;
  /* USER CODE BEGIN Custom_STM_Event_Handler_1 */
//  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
//  Custom_STM_App_Notification_evt_t     Notification;
  /* USER CODE END Custom_STM_Event_Handler_1 */

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch (event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
      switch (blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
          if (attribute_modified->Attr_Handle == (CustomContext.CustomAngle_CHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_2_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_2_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_ANGLE_C_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_ANGLE_C_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomAngle_CHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomLivedata_CHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_3_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_3_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_LIVEDATA_C_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_LIVEDATA_C_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomLivedata_CHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomConfigflags_CHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_4_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_4_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_CONFIGFLAGS_C_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_CONFIGFLAGS_C_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_4_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomConfigflags_CHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSwitch_CHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */

            /* USER CODE END CUSTOM_STM_Service_1_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomSwitch_CHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomConfigflags_CHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_4_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            //AJOUT SIMON
            Notification.Custom_Evt_Opcode = CUSTOM_STM_CONFIGFLAGS_C_WRITE_EVT;
            Notification.DataTransfered.Length=attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload=attribute_modified->Attr_Data;
			Custom_STM_App_Notification(&Notification);
            /* USER CODE END CUSTOM_STM_Service_4_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomConfigflags_CHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomControl_CHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_5_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            //AJOUT SIMON
			Notification.Custom_Evt_Opcode = CUSTOM_STM_CONTROL_C_WRITE_EVT;
			Notification.DataTransfered.Length=attribute_modified->Attr_Data_Length;
			Notification.DataTransfered.pPayload=attribute_modified->Attr_Data;
			Custom_STM_App_Notification(&Notification);
            /* USER CODE END CUSTOM_STM_Service_5_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomControl_CHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_END */
          break;

        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */
          break;
        /* USER CODE BEGIN BLECORE_EVT */

        /* USER CODE END BLECORE_EVT */
        default:
          /* USER CODE BEGIN EVT_DEFAULT */

          /* USER CODE END EVT_DEFAULT */
          break;
      }
      /* USER CODE BEGIN EVT_VENDOR*/

      /* USER CODE END EVT_VENDOR*/
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT_CASES*/

      /* USER CODE END EVENT_PCKT_CASES*/

    default:
      /* USER CODE BEGIN EVENT_PCKT*/

      /* USER CODE END EVENT_PCKT*/
      break;
  }

  /* USER CODE BEGIN Custom_STM_Event_Handler_2 */

  /* USER CODE END Custom_STM_Event_Handler_2 */

  return(return_value);
}/* end Custom_STM_Event_Handler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitCustomSvc(void)
{

  Char_UUID_t  uuid;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN SVCCTL_InitCustomSvc_1 */

  /* USER CODE END SVCCTL_InitCustomSvc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

  /**
   *          ledService
   *
   * Max_Attribute_Records = 1 + 2*1 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for ledService +
   *                                2 for switchCharacteristic +
   *                              = 3
   */

  COPY_LEDSERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             3,
                             &(CustomContext.CustomLedsHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: ledS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: ledS \n\r");
  }

  /**
   *  switchCharacteristic
   */
  COPY_SWITCHCHARACTERISTIC_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomLedsHdle,
                          UUID_TYPE_128, &uuid,
                          SizeSwitch_C,
                          CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomSwitch_CHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : SWITCH_C, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : SWITCH_C \n\r");
  }

  /**
   *          angleService
   *
   * Max_Attribute_Records = 1 + 2*1 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for angleService +
   *                                2 for AngleCharacteristic +
   *                                1 for AngleCharacteristic configuration descriptor +
   *                              = 4
   */

  COPY_ANGLESERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             4,
                             &(CustomContext.CustomAnglesHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: angleS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: angleS \n\r");
  }

  /**
   *  AngleCharacteristic
   */
  COPY_ANGLECHARACTERISTIC_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomAnglesHdle,
                          UUID_TYPE_128, &uuid,
                          SizeAngle_C,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomAngle_CHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : ANGLE_C, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : ANGLE_C \n\r");
  }

  /**
   *          liveDataService
   *
   * Max_Attribute_Records = 1 + 2*1 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for liveDataService +
   *                                2 for liveDataCharacteristic +
   *                                1 for liveDataCharacteristic configuration descriptor +
   *                              = 4
   */

  COPY_LIVEDATASERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             4,
                             &(CustomContext.CustomLivedatasHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: liveDataS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: liveDataS \n\r");
  }

  /**
   *  liveDataCharacteristic
   */
  COPY_LIVEDATACHARACTERISTIC_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomLivedatasHdle,
                          UUID_TYPE_128, &uuid,
                          SizeLivedata_C,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomLivedata_CHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : LIVEDATA_C, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : LIVEDATA_C \n\r");
  }

  /**
   *          configService
   *
   * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for configService +
   *                                2 for configFlagsCharacteristic +
   *                                2 for configFlagsRequestCharacteristic +
   *                                1 for configFlagsCharacteristic configuration descriptor +
   *                              = 6
   */

  COPY_CONFIGSERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             6,
                             &(CustomContext.CustomConfigsHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: configS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: configS \n\r");
  }

  /**
   *  configFlagsCharacteristic
   */
  COPY_CONFIGFLAGSCHARACTERISTIC_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomConfigsHdle,
                          UUID_TYPE_128, &uuid,
                          SizeConfigflags_C,
                          CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomConfigflags_CHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : CONFIGFLAGS_C, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : CONFIGFLAGS_C \n\r");
  }
  /**
   *  configFlagsRequestCharacteristic
   */
  COPY_CONFIGFLAGSREQUESTCHARACTERISTIC_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomConfigsHdle,
                          UUID_TYPE_128, &uuid,
                          SizeConfigflagsreq_C,
                          CHAR_PROP_READ | CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomConfigflagsreq_CHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : CONFIGFLAGSREQ_C, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : CONFIGFLAGSREQ_C \n\r");
  }

  /**
   *          controlService
   *
   * Max_Attribute_Records = 1 + 2*1 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for controlService +
   *                                2 for controlCharacteristic +
   *                              = 3
   */

  COPY_CONTROLSERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             3,
                             &(CustomContext.CustomControlsHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: controlS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: controlS \n\r");
  }

  /**
   *  controlCharacteristic
   */
  COPY_CONTROLCHARACTERISTIC_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomControlsHdle,
                          UUID_TYPE_128, &uuid,
                          SizeControl_C,
                          CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomControl_CHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : CONTROL_C, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : CONTROL_C \n\r");
  }

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_2 */

  /* USER CODE END SVCCTL_InitCustomSvc_2 */

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_1 */

  /* USER CODE END Custom_STM_App_Update_Char_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_SWITCH_C:
      ret = aci_gatt_update_char_value(CustomContext.CustomLedsHdle,
                                       CustomContext.CustomSwitch_CHdle,
                                       0, /* charValOffset */
                                       SizeSwitch_C, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value SWITCH_C command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value SWITCH_C command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_1*/
      break;

    case CUSTOM_STM_ANGLE_C:
      ret = aci_gatt_update_char_value(CustomContext.CustomAnglesHdle,
                                       CustomContext.CustomAngle_CHdle,
                                       0, /* charValOffset */
                                       SizeAngle_C, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value ANGLE_C command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value ANGLE_C command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_1*/
      break;

    case CUSTOM_STM_LIVEDATA_C:
      ret = aci_gatt_update_char_value(CustomContext.CustomLivedatasHdle,
                                       CustomContext.CustomLivedata_CHdle,
                                       0, /* charValOffset */
                                       SizeLivedata_C, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value LIVEDATA_C command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value LIVEDATA_C command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_3_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_3_Char_1*/
      break;

    case CUSTOM_STM_CONFIGFLAGS_C:
      ret = aci_gatt_update_char_value(CustomContext.CustomConfigsHdle,
                                       CustomContext.CustomConfigflags_CHdle,
                                       0, /* charValOffset */
                                       SizeConfigflags_C, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value CONFIGFLAGS_C command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value CONFIGFLAGS_C command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_4_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_4_Char_1*/
      break;

    case CUSTOM_STM_CONFIGFLAGSREQ_C:
      ret = aci_gatt_update_char_value(CustomContext.CustomConfigsHdle,
                                       CustomContext.CustomConfigflagsreq_CHdle,
                                       0, /* charValOffset */
                                       SizeConfigflagsreq_C, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value CONFIGFLAGSREQ_C command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value CONFIGFLAGSREQ_C command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_4_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_4_Char_2*/
      break;

    case CUSTOM_STM_CONTROL_C:
      ret = aci_gatt_update_char_value(CustomContext.CustomControlsHdle,
                                       CustomContext.CustomControl_CHdle,
                                       0, /* charValOffset */
                                       SizeControl_C, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value CONTROL_C command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value CONTROL_C command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_5_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_5_Char_1*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_2 */

  /* USER CODE END Custom_STM_App_Update_Char_2 */

  return ret;
}
