/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#define CAN_BASE_ID 0						///< CAN��׼ID�����11λ��Ҳ����0x7FF

#define CAN_FILTER_MODE_MASK_ENABLE 0		///< CAN������ģʽѡ��=0���б�ģʽ  =1������ģʽ

#define CAN_ID_TYPE_STD_ENABLE      1       ///< CAN����ID����ѡ��=1����׼ID��=0����չID

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CANSendData_fixed_data(void);
void CANSendData_data(uint8_t *data,uint32_t TXID);
void CAN_TxMsg(CAN_HandleTypeDef *hcan_tmp,uint32_t TXID,uint8_t *data,short len);
void CAN_TxRTR(CAN_HandleTypeDef *hcan_tmp,uint32_t TXID);
void CAN_Init(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

