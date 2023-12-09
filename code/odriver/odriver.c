#include "odriver.h"



void OdriveInit(Odrv_t* odrive,uint16_t axis_id,float maximum_velocity,float maximum_current,uint8_t initial_input_mode,uint8_t initial_control_mode){

	uint8_t TxBuffer[8]={0};

	odrive->Instance = axis_id;

	odrive->control_mode=	initial_control_mode;

	odrive->input_mode = initial_input_mode;

	odrive->max_current=maximum_current;

	odrive->max_velocity=maximum_velocity;

//	memcpy(&TxBuffer[0],&maximum_velocity,4);

//	memcpy(&TxBuffer[4],&maximum_current,4);


#if defined USED_CAN1


//	CAN_TxMsg(&hcan1, odrive->Instance<<5 |
//			((uint16_t)SET_LIMITS), TxBuffer, sizeof(TxBuffer));


#elif defined USED_CAN2


	CAN_TxMsg(&hcan2, odrive->Instance<<5 |
			((uint16_t)SET_LIMITS), TxBuffer, sizeof(TxBuffer));



#endif

}

void OdriveSetControlMode(Odrv_t* odrive,uint8_t requested_control_mode)
{

	uint8_t TxBuffer[8];

	if(odrive->control_mode!=requested_control_mode){


		TxBuffer[0] = requested_control_mode;

		odrive->control_mode=requested_control_mode;

#if defined USED_CAN1

		CAN_TxMsg(&hcan1, odrive->Instance<<5 |
				((uint16_t)SET_CONTROLLER_MODES),TxBuffer, 8);

#elif defined USED_CAN2

		CAN_TxMsg(&hcan2, odrive->Instance<<5 |
				((uint16_t)SET_CONTROLLER_MODES),TxBuffer, 8);

#endif

	}

}

void OdriveSetInputMode(Odrv_t* odrive,uint8_t requested_input_mode)
{

	uint8_t TxBuffer[8];

	if(odrive->input_mode!=requested_input_mode)
	{
		TxBuffer[4]  = requested_input_mode;

		odrive->input_mode=requested_input_mode;

#if defined USED_CAN1

		CAN_TxMsg(&hcan1, odrive->Instance<<5 |
				((uint16_t)SET_CONTROLLER_MODES),TxBuffer, 8);

#elif defined USED_CAN2
		CAN_TxMsg(&hcan2, odrive->Instance<<5 |
				((uint16_t)SET_CONTROLLER_MODES),TxBuffer, 8);

#endif
	}
}


void OdriveSetControlInputMode(Odrv_t* odrive,uint8_t requested_control_mode,uint8_t requested_input_mode)
{
	uint8_t TxBuffer[8];


	if(odrive->control_mode!=requested_control_mode||odrive->input_mode!=requested_input_mode)
	{

		TxBuffer[0]  = requested_control_mode;


		TxBuffer[4]  = requested_input_mode;

		odrive->control_mode=requested_control_mode;


		odrive->input_mode=requested_input_mode;

#if defined USED_CAN1

		CAN_TxMsg(&hcan1, odrive->Instance<<5 |
				((uint16_t)SET_CONTROLLER_MODES),TxBuffer, 8);

#elif defined USED_CAN2
		CAN_TxMsg(&hcan2, odrive->Instance<<5 |
				((uint16_t)SET_CONTROLLER_MODES),TxBuffer, 8);

#endif
	}


}



void Odriveturncount(Odrv_t* odrive, float count_num,uint8_t mode)//turn number of turns
{

	uint8_t TxBuffer[8];

	OdriveClearError(odrive);

	OdriveSetControlInputMode(odrive,POSITION_CONTROL,mode);

	memset(&TxBuffer[0],0,8);

	memcpy(&TxBuffer[0],&count_num,4);

#if defined USED_CAN1

	CAN_TxMsg(&hcan1, odrive->Instance<<5 |
			((uint16_t)SET_INPUT_POS), TxBuffer, 8);

#elif defined USED_CAN2

	CAN_TxMsg(&hcan2, odrive->Instance<<5 |
			((uint16_t)SET_INPUT_POS), TxBuffer, sizeof(TxBuffer));

#endif



}





void OdriveVelocity(Odrv_t* odrive, float velocity)//turn odrive velocity
{

	uint8_t TxBuffer[8];

	if(velocity>odrive->max_velocity)
		velocity=odrive->max_velocity;


	OdriveSetControlInputMode(odrive,VELOCITY_CONTROL,VEL_RAMP);

	memset(&TxBuffer[0],0,8);


	memcpy(&TxBuffer[0],&velocity,4);

#if defined USED_CAN1

	CAN_TxMsg(&hcan1, odrive->Instance<<5 |
			((uint16_t)SET_INPUT_VEL),TxBuffer, 8);

#elif defined USED_CAN2

	CAN_TxMsg(&hcan2, odrive->Instance<<5 |
			((uint16_t)SET_INPUT_VEL), TxBuffer, sizeof(TxBuffer));

#endif

}





void OdriveTorque(Odrv_t* odrive, float torque)//torque control
{

	uint8_t TxBuffer[4];


//	OdriveSetControlInputMode(odrive,TORQUE_CONTROL,PASSTHROUGH);


	memset(&TxBuffer[0],0,4);


	memcpy(&TxBuffer[0],&torque,sizeof(float));

#if defined USED_CAN1

	CAN_TxMsg(&hcan1, odrive->Instance<<5 |
			((uint16_t)SET_INPUT_TORQUE),TxBuffer, sizeof(TxBuffer));

#elif defined USED_CAN2

	CAN_TxMsg(&hcan2, odrive->Instance<<5 |
			((uint16_t)SET_INPUT_TORQUE), TxBuffer, sizeof(TxBuffer));

#endif

}



void OdriveStop(Odrv_t* odrive)
{
	OdriveVelocity(odrive, 0.0);
}

void OdriveRestart(Odrv_t* odrive)
{

	uint8_t TxBuffer[8];
	CAN_TxMsg(&hcan1, odrive->Instance<<5 |
				((uint16_t)Reboot),TxBuffer, sizeof(TxBuffer));

}

void OdriveClearError(Odrv_t* odrive)
{

	uint8_t TxBuffer[8];

	CAN_TxMsg(&hcan1, odrive->Instance<<5 |
				((uint16_t)	CLEAR_ERROR),TxBuffer, sizeof(TxBuffer));

}

void OdriveSetAbsolutePosition(Odrv_t* odrive)
{
	uint8_t TxBuffer[8];

	CAN_TxMsg(&hcan1, odrive->Instance<<5 |
				((uint16_t)	SET_ABSOLUTE_POSITION),TxBuffer, sizeof(TxBuffer));

}

void OdriveGetEncoderFeedback(Odrv_t* odrive)//turn odrive velocity
{
#if defined USED_CAN1
	CAN_TxRTR(&hcan1, odrive->Instance<<5 |
			((uint16_t)GET_ENCODER_ESTIMATE));
#elif defined USED_CAN2

	CAN_TxRTR(&hcan2, odrive->Instance<<5 |
			((uint16_t)GET_ENCODER_ESTIMATE));

#endif

}




void OdriveGetTemperatureFeedback(Odrv_t* odrive)//turn odrive velocity
{
#if defined USED_CAN1
	CAN_TxRTR(&hcan1, odrive->Instance<<5 |
			((uint16_t)GET_TEMPERATURE));
#elif defined USED_CAN2

	CAN_TxRTR(&hcan2, odrive->Instance<<5 |
			((uint16_t)GET_TEMPERATURE));

#endif

}


void OdriveGet_VOLTAGE_Gurren(Odrv_t* odrive)      //turn odrive velocity
{
#if defined USED_CAN1
	CAN_TxRTR(&hcan1, odrive->Instance<<5 |
			((uint16_t)GET_BUS_VOLTAGE_Gurren));
#elif defined USED_CAN2

	CAN_TxRTR(&hcan2, odrive->Instance<<5 |
			((uint16_t)GET_TEMPERATURE));

#endif
}


void decode_Odrive(Odrv_t* odrive,bike_state* bike_State)
{
		
	float data=0;

	uint16_t id = Odrvmsg.RXmsg.StdId ;

	uint16_t mask = 0x00F;

	uint16_t command=id&mask;
	printf("command:0x%X\r\n",command);


	if(command==GET_ENCODER_ESTIMATE){

		memcpy(&data,&Odrvmsg.Data[0],sizeof(float));
		odrive->feedback.encoder=data;
		bike_State->Balance_Motor_rpm = data;
		memcpy(&data,&Odrvmsg.Data[4]      ,sizeof(float));
		odrive->feedback.velocity=data;
		bike_State->Bicycle_voltage = data;
	}
	
	else if(command==SET_AXIS_STATE){
		memcpy(&data,&Odrvmsg.Data[0],sizeof(float));
		odrive->feedback.Bus_Voltage=data;
		bike_State->Bicycle_voltage = data;
		memcpy(&data,&Odrvmsg.Data[4]      ,sizeof(float));
		odrive->feedback.Bus_Current=data;
		bike_State->Bicycle_current = data;
	}

	else if(command==GET_TEMPERATURE){

		memcpy(&data,&Odrvmsg.Data[0],sizeof(float));
		odrive->feedback.inverter_temperature=data;
		memcpy(&data,&Odrvmsg.Data[4]      ,sizeof(float));
		odrive->feedback.motor_temperature=data;
	}
}


void OdriveSetAxisState(Odrv_t* odrive,uint8_t AxisRequested_state)
{

	odrive->current_state=AxisRequested_state;

	uint8_t TxBuffer[8];

	memcpy(&TxBuffer[0],&AxisRequested_state,1);


#if defined USED_CAN1

	CAN_TxMsg(&hcan1, odrive->Instance<<5 |
			((uint16_t)AxisRequested_state),TxBuffer, sizeof(TxBuffer));

#elif defined USED_CAN2

	CAN_TxMsg(&hcan2, odrive->Instance<<5 |
			((uint16_t)AxisRequested_state), TxBuffer, sizeof(TxBuffer));

#endif
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
//{
//	
//    // CAN���ݽ���
//    if (canHandle->Instance == hcan1.Instance)
//    {
//        if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &Odrvmsg.RXmsg, Odrvmsg.Data) == HAL_OK)		// ��ý��յ�������ͷ������
//        {
//			printf("\r\n\r\n\r\n################### CAN RECV ###################\r\n");
//			printf("STID:0x%X\r\n",Odrvmsg.RXmsg.StdId);
//			printf("EXID:0x%X\r\n",Odrvmsg.RXmsg.ExtId);
//			printf("DLC :%d\r\n", Odrvmsg.RXmsg.DLC);
//			printf("DATA:");
//			for(int i = 0; i < Odrvmsg.RXmsg.DLC; i++)
//			{
//				printf("0x%02X ", Odrvmsg.Data[i]);
//			}
//			decode_Odrive(&odrive_andy);
//			printf("\n get_date Pos_Estimate is %.2f and get_date Get_Bus_Voltage_Current is %.2f v %.2f A\n",odrive_andy.feedback.encoder,odrive_andy.feedback.Bus_Voltage,odrive_andy.feedback.Bus_Current);
//           HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);						// �ٴ�ʹ��FIFO0�����ж�
//        }
//    }
//}

