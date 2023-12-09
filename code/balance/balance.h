#ifndef BALANCE_H
#define BALANCE_H


__packed typedef struct{
  double LQR_Kp;
  double LQR_Kv;
  double LQR_Ks;

  double Speed_Kp;
  double Speed_Ki;
  double Speed_Kd;

  double Angle_Kp;
  double Angle_Ki;
  double Angle_Kd;

  double Angle_Vel_Kp;
  double Angle_Vel_Ki;
  double Angle_Vel_Kd;

  double Angle_Offsett;

  float Balance_Motor_rpm;
  float Bicycle_voltage;
  float Bicycle_current;
  float Roll_Angular_Velocity;
  float Roll;
}bike_state;

extern bike_state andy_bike;
float  LQR_operation(bike_state* bike_State);

#endif 

