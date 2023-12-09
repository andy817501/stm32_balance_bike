#ifndef __SERIAL_BOOT_H
#define __SERIAL_BOOT_H
#include <stdint.h>
//未初始化
#define FDILink_Status_Uninitialized 		0
//运行
#define FDILink_Status_Running 				2
//正忙
#define FDILink_Status_Busy 				3

#define FDILink_Frame_Start					0
#define FDILink_Frame_CMD					1
#define FDILink_Frame_Length				2
#define FDILink_Frame_SerialNumber			3
#define FDILink_Frame_CRC8					4
#define FDILink_Frame_CRC16H				5
#define FDILink_Frame_CRC16L				6
#define FDILink_Frame_Data					7
#define FDILink_Frame_End					8
/***************************************************************
*	帧符号
***************************************************************/
#define FDILink_STX_Flag 0xFC
#define FDILink_EDX_Flag 0xFD
#define FDILink_Connect_Flag 0xFD

typedef struct FDILink_Status
{
	int 				BootStatus;
	int					RxStatus;
	int 				RxType;
	int 				RxDataLeft;
	int                 RxLost;
	int		 			RxNumber;
	int                 RxTotalBytes;
	int                 RxOKBytes;
	int		 			TxNumber;
	int 				CRC8_Verify;
	int 				CRC16_Verify;
	uint32_t 			BufferIndex;
	uint8_t 			FDILink_Frame_Buffer[12];
	uint8_t 			Buffer[256];
}FDILink_t;


int FDILink_Receive(FDILink_t* FDILink, uint8_t value);
#endif
