/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TXHeader;  //发送包头


typedef struct
{
	CAN_RxHeaderTypeDef hdr;
	uint8_t payload[8];
} CAN_RxPacketTypeDef;

uint8_t TXmessage[8] = {0};

/// CAN过滤器寄存器位宽类型定义
typedef union
{
    __IO uint32_t value;
    struct
    {
        uint8_t REV : 1;			///< [0]    ：未使用
        uint8_t RTR : 1;			///< [1]    : RTR（数据帧或远程帧标志位）
        uint8_t IDE : 1;			///< [2]    : IDE（标准帧或扩展帧标志位）
        uint32_t EXID : 18;			///< [21:3] : 存放扩展帧ID
        uint16_t STID : 11;			///< [31:22]: 存放标准帧ID
    } Sub;
} CAN_FilterRegTypeDef;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 7;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CANSendData_fixed_data(void)
{
	uint32_t pTxMailBox;
	
    TXHeader.StdId=0x401 ;
	TXHeader.ExtId=0;
	TXHeader.DLC=8;
	TXHeader.IDE=CAN_ID_STD;
	TXHeader.RTR=CAN_RTR_DATA;
	TXHeader.TransmitGlobalTime = DISABLE;
	

	TXmessage[0] = 1;//Fan working
	TXmessage[1] = 2;
	TXmessage[2] = 3;
	TXmessage[3] = 4;
	TXmessage[4] = 5;
	TXmessage[5] = 6;
	TXmessage[6] = 7;
	TXmessage[7] = 8;
	HAL_CAN_AddTxMessage(&hcan1,&TXHeader,TXmessage,&pTxMailBox);	
}

void CANSendData_data(uint8_t *data,uint32_t TXID)
{
	uint32_t pTxMailBox;
	
    TXHeader.StdId=TXID ;
	TXHeader.ExtId=0;
	TXHeader.DLC=8;
	TXHeader.IDE=CAN_ID_STD;
	TXHeader.RTR=CAN_RTR_DATA;
	TXHeader.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan1, &TXHeader, data, &pTxMailBox);
}


static void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;
    CAN_FilterRegTypeDef IDH = {0};
    CAN_FilterRegTypeDef IDL = {0};

#if CAN_ID_TYPE_STD_ENABLE
    IDH.Sub.STID = (CAN_BASE_ID >> 16) & 0xFFFF;		// 标准ID高16位
    IDL.Sub.STID = (CAN_BASE_ID & 0xFFFF);				// 标准ID低16位
#else
    IDH.Sub.EXID = (CAN_BASE_ID >> 16) & 0xFFFF;		// 扩展ID高16位
    IDL.Sub.EXID = (CAN_BASE_ID & 0xFFFF);				// 扩展ID低16位
    IDL.Sub.IDE  = 1;									// 扩展帧标志位置位
#endif
    sFilterConfig.FilterBank           = 0;												// 设置过滤器组编号
#if CAN_FILTER_MODE_MASK_ENABLE
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;							// 屏蔽位模式
#else
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;							// 列表模式
#endif
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_16BIT;							// 16位宽
    sFilterConfig.FilterIdHigh         = 0xa9 << 5;										// 标识符寄存器一ID高十六位，放入扩展帧位
    sFilterConfig.FilterIdLow          = 0xb7 << 5;										// 标识符寄存器一ID低十六位，放入扩展帧位
    sFilterConfig.FilterMaskIdHigh     = 0x1200 << 5;										// 标识符寄存器二ID高十六位，放入扩展帧位
    sFilterConfig.FilterMaskIdLow      = 0x1201 << 5;										// 标识符寄存器二ID低十六位，放入扩展帧位
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;									// 过滤器组关联到FIFO0
    sFilterConfig.FilterActivation     = ENABLE;										// 激活过滤器
    sFilterConfig.SlaveStartFilterBank = 14;											// 设置从CAN的起始过滤器编号
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_Init(void)
{
    MX_CAN1_Init();
    CAN_Filter_Config();
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);					// 使能CAN接收中断
}

void CAN_TxMsg(CAN_HandleTypeDef *hcan_tmp,uint32_t TXID,uint8_t *data,short len)
{
	uint32_t pTxMailBox;
    TXHeader.StdId=TXID ;
	TXHeader.ExtId=0;
	TXHeader.DLC=len;
	TXHeader.IDE=CAN_ID_STD;
	TXHeader.RTR=CAN_RTR_DATA;
	TXHeader.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(hcan_tmp, &TXHeader, data, &pTxMailBox);
}

void CAN_TxRTR(CAN_HandleTypeDef *hcan_tmp,uint32_t TXID)
{
	uint32_t pTxMailBox;
	TXHeader.StdId=TXID;
	TXHeader.ExtId=0;
	TXHeader.DLC=0;
	TXHeader.IDE=CAN_ID_STD;
    TXHeader.RTR=CAN_RTR_REMOTE;
	TXHeader.TransmitGlobalTime = DISABLE;
	HAL_CAN_AddTxMessage(hcan_tmp, &TXHeader, NULL, &pTxMailBox);
}


//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
//{
//	static CAN_RxPacketTypeDef packet;
//	
//    // CAN数据接收
//    if (canHandle->Instance == hcan1.Instance)
//    {
//        if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &packet.hdr, packet.payload) == HAL_OK)		// 获得接收到的数据头和数据
//        {
//			printf("\r\n\r\n\r\n################### CAN RECV ###################\r\n");
//			printf("STID:0x%X\r\n",packet.hdr.StdId);
//			printf("EXID:0x%X\r\n",packet.hdr.ExtId);
//			printf("DLC :%d\r\n", packet.hdr.DLC);
//			printf("DATA:");
//			for(int i = 0; i < packet.hdr.DLC; i++)
//			{
//				printf("0x%02X ", packet.payload[i]);
//			}
//           HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);						// 再次使能FIFO0接收中断
//        }
//    }
//}

/* USER CODE END 1 */
