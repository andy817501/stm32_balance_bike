#include "fdilink.h"
#include "fdilink_decode.h"


FDILink_IMUData_Packet_t IMU_Recive;
FDILink_AHRSData_Packet_t AHRS_Recive;
FDILink_INSGPSData_Packet_t INSGPS_Recive;
System_State_Packet_t System_State_Recive;
Unix_Time_Packet_t Unix_Time_Recive;
Formatted_Time_Packet_t Formatted_Time_Recive;
Status_Packet_t Status_Recive;
Position_Standard_Deviation_Packet_t Position_Standard_Deviation_Recive;
Velocity_Standard_Deviation_Packet_t Velocity_Standard_Deviation_Recive;
Euler_Orientation_Standard_Deviation_Packet_t Euler_Orientation_Standard_Deviation_Recive;
Quaternion_Orientation_Standard_Deviation_Packet_t Quaternion_Orientation_Standard_Deviation_Recive;
Raw_Sensors_Packet_t Raw_Sensors_Recive;
Raw_GNSS_Packet_t Raw_GNSS_Recive;
Satellites_Packet_t Satellites_Recive;
Detailed_Satellites_Packet_t Detailed_Satellites_Recive;
Geodetic_Position_Packet_t Geodetic_Position_Recive;
ECEF_Position_Packet_t ECEF_Position_Recive;
UTM_Position_Packet_t UTM_Position_Recive;
NED_Velocity_Packet_t NED_Velocity_Recive;
Body_Velocity_Packet_t Body_Velocity_Recive;
Acceleration_Packet_t Acceleration_Recive;
Body_Acceleration_Packet_t Body_Acceleration_Recive;
Euler_Orientation_Packet_t Euler_Orientation_Recive;
Quaternion_Orientation_Packet_t Quaternion_Orientation_Recive;
DCM_Orientation_Packet_t DCM_Orientation_Recive;
Angular_Velocity_Packet_t Angular_Velocity_Recive;
Angular_Acceleration_Packet_t Angular_Acceleration_Recive;
External_Position_And_Velocity_Packet_t External_Position_And_Velocity_Recive;
External_Position_Packet_t External_Position_Recive;
External_Velocity_Packet_t External_Velocity_Recive;
External_Body_Velocity_Packet_t External_Body_Velocity_Recive;
External_Heading_Packet_t External_Heading_Recive;
Running_Time_Packet_t Running_Time_Recive;
Local_Magnetic_Field_Packet_t Local_Magnetic_Field_Recive;
Odometer_State_Packet_t Odometer_State_Recive;
External_Time_Packet_t External_Time_Recive;
External_Depth_Packet_t External_Depth_Recive;
Geoid_Height_Packet_t Geoid_Height_Recive;
RTCM_Corrections_Packet_t RTCM_Corrections_Recive;
External_Pitot_Pressure_Packet_t External_Pitot_Pressure_Recive;
Wind_Packet_t Wind_Recive;
Heave_Packet_t Heave_Recive;
Raw_Satellite_Data_Packet_t Raw_Satellite_Data_Recive;
External_Odometer_Packet_t External_Odometer_Recive;
External_Air_Data_Packet_t External_Air_Data_Recive;
Gimbal_State_Packet_t Gimbal_State_Recive;
Automotive_Packet_t Automotive_Recive;
Packet_Timer_Period_Packet_t Packet_Timer_Period_Recive;
Packets_Period_Packet_t Packets_Period_Recive;
Installation_Alignment_Packet_t Installation_Alignment_Recive;
Filter_Options_Packet_t Filter_Options_Recive;
Magnetic_Calibration_Values_Packet_t Magnetic_Calibration_Values_Recive;
Magnetic_Calibration_Configuration_Packet_t Magnetic_Calibration_Configuration_Recive;
Magnetic_Calibration_Status_Packet_t Magnetic_Calibration_Status_Recive;
Odometer_Configuration_Packet_t Odometer_Configuration_Recive;
Set_Zero_Orientation_Alignment_Packet_t Set_Zero_Orientation_Alignment_Recive;
User_Data_Packet_t User_Data_Recive;
SOTM_Packet_t SOTM_Recive;

void* const FDILink_Buffer_List[256] = 
{
	[FDILINK_IMUDATA_PACKET_ID]                           = &IMU_Recive,
	[FDILINK_AHRSDATA_PACKET_ID]                          = &AHRS_Recive,
	[FDILINK_INSGPSDATA_PACKET_ID]                        = &INSGPS_Recive,
	[System_State_Packet_ID]                              = &System_State_Recive,
	[Unix_Time_Packet_ID]                                 = &Unix_Time_Recive,
	[Formatted_Time_Packet_ID]                            = &Formatted_Time_Recive,
	[Status_Packet_ID]                                    = &Status_Recive,
	[Position_Standard_Deviation_Packet_ID]               = &Position_Standard_Deviation_Recive,
	[Velocity_Standard_Deviation_Packet_ID]               = &Velocity_Standard_Deviation_Recive,
	[Euler_Orientation_Standard_Deviation_Packet_ID]      = &Euler_Orientation_Standard_Deviation_Recive,
	[Quaternion_Orientation_Standard_Deviation_Packet_ID] = &Quaternion_Orientation_Standard_Deviation_Recive,
	[Raw_Sensors_Packet_ID]                               = &Raw_Sensors_Recive,
	[Raw_GNSS_Packet_ID]                                  = &Raw_GNSS_Recive,
	[Satellites_Packet_ID]                                = &Satellites_Recive,
	[Detailed_Satellites_Packet_ID]                       = &Detailed_Satellites_Recive,
	[Geodetic_Position_Packet_ID]                         = &Geodetic_Position_Recive,
	[ECEF_Position_Packet_ID]                             = &ECEF_Position_Recive,
	[UTM_Position_Packet_ID]                              = &UTM_Position_Recive,
	[NED_Velocity_Packet_ID]                              = &NED_Velocity_Recive,
	[Body_Velocity_Packet_ID]                             = &Body_Velocity_Recive,
	[Acceleration_Packet_ID]                              = &Acceleration_Recive,
	[Body_Acceleration_Packet_ID]                         = &Body_Acceleration_Recive,
	[Euler_Orientation_Packet_ID]                         = &Euler_Orientation_Recive,
	[Quaternion_Orientation_Packet_ID]                    = &Quaternion_Orientation_Recive,
	[DCM_Orientation_Packet_ID]                           = &DCM_Orientation_Recive,
	[Angular_Velocity_Packet_ID]                          = &Angular_Velocity_Recive,
	[Angular_Acceleration_Packet_ID]                      = &Angular_Acceleration_Recive,
	[External_Position_And_Velocity_Packet_ID]            = &External_Position_And_Velocity_Recive,
	[External_Position_Packet_ID]                         = &External_Position_Recive,
	[External_Velocity_Packet_ID]                         = &External_Velocity_Recive,
	[External_Body_Velocity_Packet_ID]                    = &External_Body_Velocity_Recive,
	[External_Heading_Packet_ID]                          = &External_Heading_Recive,
	[Running_Time_Packet_ID]                              = &Running_Time_Recive,
	[Local_Magnetic_Field_Packet_ID]                      = &Local_Magnetic_Field_Recive,
	[Odometer_State_Packet_ID]                            = &Odometer_State_Recive,
	[External_Time_Packet_ID]                             = &External_Time_Recive,
	[External_Depth_Packet_ID]                            = &External_Depth_Recive,
	[Geoid_Height_Packet_ID]                              = &Geoid_Height_Recive,
	[RTCM_Corrections_Packet_ID]                          = &RTCM_Corrections_Recive,
	[External_Pitot_Pressure_Packet_ID]                   = &External_Pitot_Pressure_Recive,
	[Wind_Packet_ID]                                      = &Wind_Recive,
	[Heave_Packet_ID]                                     = &Heave_Recive,
	[Raw_Satellite_Data_Packet_ID]                        = &Raw_Satellite_Data_Recive,
	[External_Odometer_Packet_ID]                         = &External_Odometer_Recive,
	[External_Air_Data_Packet_ID]                         = &External_Air_Data_Recive,
	[Gimbal_State_Packet_ID]                              = &Gimbal_State_Recive,
	[Automotive_Packet_ID]                                = &Automotive_Recive,
	[Packet_Timer_Period_Packet_ID]                       = &Packet_Timer_Period_Recive,
	[Packets_Period_Packet_ID]                            = &Packets_Period_Recive,
	[Installation_Alignment_Packet_ID]                    = &Installation_Alignment_Recive,
	[Filter_Options_Packet_ID]                            = &Filter_Options_Recive,
	[Magnetic_Calibration_Values_Packet_ID]               = &Magnetic_Calibration_Values_Recive,
	[Magnetic_Calibration_Configuration_Packet_ID]        = &Magnetic_Calibration_Configuration_Recive,
	[Magnetic_Calibration_Status_Packet_ID]               = &Magnetic_Calibration_Status_Recive,
	[Odometer_Configuration_Packet_ID]                    = &Odometer_Configuration_Recive,
	[Set_Zero_Orientation_Alignment_Packet_ID]            = &Set_Zero_Orientation_Alignment_Recive,
	[User_Data_Packet_ID]                                 = &User_Data_Recive,
	[SOTM_Packet_ID]                                      = &SOTM_Recive, 

};

FDILink_t FDILink_Handler;
uint8_t FDILink_Recive_Buffer[256];
int FDILink_Index = 0;
#include "stm32f4xx_hal.h"
#include "string.h"
#include "math.h"
extern UART_HandleTypeDef huart3;
static void quaternio_to_euler(float qw,float qx,float qy,float qz,volatile float* roll,volatile float* pitch,volatile float* yaw)
{

*roll = atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy));
*pitch = asin(2*(qw*qy-qx*qz));
*yaw = atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));	
}
volatile static float roll,pitch,yaw;
//void imu(void)
//{
//		HAL_UART_Receive_DMA(&huart3, FDILink_Recive_Buffer, 256);
//	while(1)
//	{
//		int index = 256 - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
//		if(index != FDILink_Index)
//		{
//			int c = FDILink_Recive_Buffer[FDILink_Index];
//			FDILink_Index = (FDILink_Index + 1) % 256;
//			if(FDILink_Receive(&FDILink_Handler, c) > 0)
//			{
//				void* p = FDILink_Buffer_List[FDILink_Handler.RxType];
//				
//				quaternio_to_euler(AHRS_Recive.Q1,AHRS_Recive.Q2,AHRS_Recive.Q3,AHRS_Recive.Q4,&roll,&pitch,&yaw);
//				 printf("roll: %.3f, pitch: %.3f, yaw: %.3f\n", roll, pitch, yaw);
//				if(p)
//				{
//					memcpy(p, FDILink_Handler.Buffer, FDILink_Handler.BufferIndex);
//				}
//			}
//		}
//	}
//}


void dma_usrat3_init(void)
{
__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);  //开启空闲中断
HAL_UART_Receive_DMA(&huart3,FDILink_Recive_Buffer,256);  //开启DMA接收
}

void interrupt_imu_date(bike_state* bike_State)
{
		int index = 256 - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
		if(index != FDILink_Index)
		{
			int c = FDILink_Recive_Buffer[FDILink_Index];
			FDILink_Index = (FDILink_Index + 1) % 256;
			if(FDILink_Receive(&FDILink_Handler, c) > 0)
			{
				void* p = FDILink_Buffer_List[FDILink_Handler.RxType];

				quaternio_to_euler(AHRS_Recive.Q1,AHRS_Recive.Q2,AHRS_Recive.Q3,AHRS_Recive.Q4,&roll,&pitch,&yaw);
				bike_State->Roll = roll;
				if(p)
				{
					memcpy(p, FDILink_Handler.Buffer, FDILink_Handler.BufferIndex);
				}
			}
			HAL_UART_Receive_DMA(&huart3,FDILink_Recive_Buffer,256); 
		}

}
