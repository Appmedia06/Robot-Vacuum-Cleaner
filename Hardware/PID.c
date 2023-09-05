#include "stm32f10x.h"                  // Device header
#include "MPU6050.h"
#include "kalman.h"
#include "Motor.h"
#include "OLED.h"

float Kp  = 15;
float Ki = 0.05;
float Kd = 250;

static int16_t left_curr_err;
static int16_t left_prev_err;
static int16_t left_err_accum = 0;
static int16_t left_PWM_change;

static int16_t right_curr_err;
static int16_t right_prev_err;
static int16_t right_err_accum = 0;
static int16_t right_PWM_change;

static float current_Yaw; // 現在的Yaw
static float previous_Yaw = 0; // 上一次的Yaw
static float difference_Yaw; // Yaw的差值

void L_PID_Control(void)
{
	current_Yaw = Yaw;
	left_curr_err = current_Yaw - previous_Yaw;
	
	left_PWM_change = (Kp * left_curr_err) + (Ki * left_err_accum) + (Kd * (left_curr_err - left_prev_err));
	
	
	int16_t leftSpeed = Wheel_Normal_Speed + left_PWM_change;
	if(leftSpeed > 1950)
	{
		leftSpeed = 1950;
	}
	else if(leftSpeed < 1650)
	{
		leftSpeed = 1650;
	}
	
	OLED_ShowSignedNum(3, 1, leftSpeed, 4);
	OLED_ShowSignedNum(3, 7, left_PWM_change, 4);
	MotorA_SetSpeed(leftSpeed);
	
	left_prev_err = left_curr_err;
	left_err_accum += left_curr_err;
}


void R_PID_Control(void)
{
	current_Yaw = Yaw;
	right_curr_err = previous_Yaw - current_Yaw;
	
	right_PWM_change = (Kp * right_curr_err) + (Ki * right_err_accum) + (Kd * (right_curr_err - right_prev_err));
	
	
	
	
	int16_t rightSpeed = Wheel_Normal_Speed + right_PWM_change;
	if(rightSpeed > 750)
	{
		rightSpeed = 750;
	}
	else if(rightSpeed < 450)
	{
		rightSpeed = 450;
	}
	
	OLED_ShowSignedNum(3, 1, rightSpeed, 5);
	OLED_ShowSignedNum(4, 1, right_PWM_change, 5);
	MotorB_SetSpeed(rightSpeed);
	
	right_prev_err = right_curr_err;
	right_err_accum += right_curr_err;
}


