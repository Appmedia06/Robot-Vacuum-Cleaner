#ifndef __MOTER_H
#define __MOTER_H

#define Wheel_Normal_Speed 1800
#define Wheel_Turn_Low 500
#define Wheel_Turn_High 2000
#define Brush_Speed 1000
#define Fan_Speed 1600

void Motor_Init(void);
void MotorA_SetSpeed(int16_t Speed);
void MotorB_SetSpeed(int16_t Speed);

void brush_Strat(void);
void brush_Stop(void);

void fan_Start(void);
void fan_Stop(void);

void Forward(int16_t leftSpeed, int16_t rightSpeed);
void Backward(int16_t leftSpeed, int16_t rightSpeed);
void TurnLeft(int16_t angle);
void TurnRight(int16_t angle);
void Motor_Stop(void);

#endif
