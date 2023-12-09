#include <balance.h>


bike_state andy_bike;

float  LQR_operation(bike_state* bike_State)
{
    static float Motor_Set_Rpm,Motor_Set_Rpm_set;
    Motor_Set_Rpm = (bike_State->Roll * bike_State->LQR_Kp + bike_State->Roll_Angular_Velocity * bike_State->LQR_Kv 
    + bike_State->Balance_Motor_rpm * bike_State->LQR_Ks);

	Motor_Set_Rpm = (float)(0.3 * (double)Motor_Set_Rpm_set + 0.7 * (double)Motor_Set_Rpm);

    Motor_Set_Rpm = Motor_Set_Rpm < 5 ? Motor_Set_Rpm > -5 ? Motor_Set_Rpm : -5 : 5;
    return Motor_Set_Rpm;
}


float  Speed_PID_loop(float Target,float Measurements,bike_state* bike_State){

    float out = bike_State->Speed_Kp * (Target - Measurements);
    return  out;
}

float  Angle_PID_loop(float Target,float Measurements,bike_state* bike_State){
    float ep;
    static float ei = 0.0f;
    ep =  Target - Measurements;
    ei += ep;

    ei = ei < 10 ? ei > -10 ? ei : -10 : 10;
    float out = bike_State->Angle_Kp * ep + bike_State->Angle_Ki * ei + bike_State->Angle_Kd * bike_State->Roll_Angular_Velocity;

    return  out;
}

float  Angle_Vel_PID_loop(float Target,float Measurements,bike_state* bike_State){
    volatile static float Angle_Vel_thiserror=0.0f,Angle_Vel_lasterror=0.0f,Angle_Vel_preerror=0.0f;
    static float Angle_Vel_ep=0.0f,Angle_Vel_ei=0.0f,Angle_Vel_ed=0.0f;
    // if(Measurements>0 && Measurements<1.0f) Measurements = 0;
    // if(Measurements<0 && Measurements>-1.0f) Measurements = 0;
    Angle_Vel_thiserror = Measurements-Target;
    Angle_Vel_ep=Angle_Vel_thiserror-Angle_Vel_lasterror;
    Angle_Vel_ei=Angle_Vel_thiserror;
    Angle_Vel_ed=Angle_Vel_thiserror-2*Angle_Vel_lasterror+Angle_Vel_preerror;
    Angle_Vel_lasterror=Angle_Vel_thiserror;
    Angle_Vel_preerror=Angle_Vel_lasterror;
    float out = bike_State->Angle_Vel_Kp * Angle_Vel_ep + bike_State->Angle_Vel_Ki * Angle_Vel_ei + bike_State->Angle_Vel_Kd * Angle_Vel_ed;
    return out;
}

