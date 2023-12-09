/*
 * odriver.h
 *
 *  Created on: Oct 6, 2022
 *      Author: heheibhoi
 */
#ifndef SRC_ODRIVE_ODRIVER_H_
#define SRC_ODRIVE_ODRIVER_H_

#include "can.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <balance.h>


#define USED_CAN1



enum Commands{

	FULL_MOTOR_CALIBRATION=4,

	SET_AXIS_STATE=7,

	GET_ENCODER_ESTIMATE=9,  //��ȡ����������

	SET_CONTROLLER_MODES=11,

	SET_INPUT_POS=12,

	SET_INPUT_VEL=13,

	SET_INPUT_TORQUE=14,

	SET_LIMITS=15,
	
	Get_Iq=20,

	GET_TEMPERATURE=21,
	
	Reboot=22,

	GET_BUS_VOLTAGE_Gurren=23,   //��ȡ��ѹ����

	CLEAR_ERROR = 24,

	SET_ABSOLUTE_POSITION,
};//basic commands




enum ControlMode{

//	VOLTAGE_CONTROL=0,

	TORQUE_CONTROL=1,

	VELOCITY_CONTROL,

	POSITION_CONTROL

};//control mode state

enum AxisRequestedState{

	IDLE_STATE=1,

	STARTUP_SEQUENCE,

	FULL_CALIBRATION_SEQUENCE,

	MOTOR_CALIBRATION,

	ENCODER_INDEX_SEARCH,


	ENCODER_OFFSET_CALIBRATION,

	CLOSED_LOOP_CONTROL,

	LOCKIN_SPIN,

	ENCODER_DIR_FIND,
	HOMING,

	ENCODER_HALL_POLARITY_CALIBRATION,

	ENCODER_HALL_PHASE_CALIBRATION,

};//axis requested state



enum InputMode{

	PASSTHROUGH=1,

	VEL_RAMP,

	POS_FILTER,

	TRAPEZOIDAL_TRAJECTORY=5,

	TORQUE_RAMP,

};//input mode


struct Odrv_feedback{

	float encoder;

	float velocity;

	float inverter_temperature;

	float motor_temperature;
	
	float Bus_Voltage;
	
	float Bus_Current;
};

typedef struct {
	CAN_RxHeaderTypeDef RXmsg;
	uint8_t Data[8];
} odrvmsg;

typedef struct{

	float max_velocity;

	float max_current;

	uint16_t Instance;

	uint8_t MAX_param_buffer[8];

	uint8_t CAN_Tx_buffer[8];

	enum InputMode input_mode;

	enum ControlMode control_mode;

	enum AxisRequestedState current_state;

	struct Odrv_feedback feedback;

}Odrv_t;

extern odrvmsg Odrvmsg;


void OdriveInit(Odrv_t* odrive,uint16_t axis_id,float maximum_velocity,float maximum_current,uint8_t initial_input_mode,uint8_t initial_control_mode);


void Odriveturncount( Odrv_t* odrive , float count_num , uint8_t mode);


void OdriveVelocity(Odrv_t* odrive, float velocity);

void OdriveTorque(Odrv_t* odrive, float torque);

void decode_Odrive(Odrv_t* odrive,bike_state* bike_State);

void OdriveSetControlMode(Odrv_t* odrive,uint8_t control_mode);

void OdriveSetInputMode(Odrv_t* odrive,uint8_t input_mode);

void OdriveSetAxisState(Odrv_t* odrive,uint8_t AxisRequested_state);

void OdriveSetControlInputMode(Odrv_t* odrive,uint8_t control_mode,uint8_t input_mode);

void OdriveClearError(Odrv_t* odrive);

void OdriveStop(Odrv_t* odrive);

void OdriveSetAbsolutePosition(Odrv_t* odrive);

void OdriveGetTemperatureFeedback(Odrv_t* odrive);

void OdriveGetEncoderFeedback(Odrv_t* odrive);

void OdriveGet_VOLTAGE_Gurren(Odrv_t* odrive);

#endif /* SRC_ODRIVE_ODRIVER_H_ */
