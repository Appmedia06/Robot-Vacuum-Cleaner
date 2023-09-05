#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "Timer.h"
#include "Pin.h"
#include "Delay.h"
#include "Algorithm.h"

void Motor_Init(void)
{
	// 電機方向控制腳初始化
	
	// MotorA (AIN1:PB0, AIN2:PB1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = MotorA_AIN1_Pin | MotorA_AIN2_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MotorA_AIN_Port, &GPIO_InitStructure);
	
	// MotorB (BIN1:PB12, AIN2:PB13)
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = MotorB_BIN1_Pin | MotorB_BIN2_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MotorB_BIN_Port, &GPIO_InitStructure);
	
	
	TIM1_Init(1999, 719);
}

// 左輪
void MotorA_SetSpeed(int16_t Speed)
{
	if(Speed > 0)
	{
		GPIO_SetBits(MotorA_AIN_Port, MotorA_AIN2_Pin);
		GPIO_ResetBits(MotorA_AIN_Port, MotorA_AIN1_Pin);
		PWM_SetCompare1(Speed);
	}
	else
	{
		GPIO_SetBits(MotorA_AIN_Port, MotorA_AIN1_Pin);
		GPIO_ResetBits(MotorA_AIN_Port, MotorA_AIN2_Pin);
		PWM_SetCompare1(Speed); // 負轉正
	}
}
// 右輪
void MotorB_SetSpeed(int16_t Speed)
{
	if(Speed > 0)
	{
		GPIO_SetBits(MotorB_BIN_Port, MotorB_BIN2_Pin);
		GPIO_ResetBits(MotorB_BIN_Port, MotorB_BIN1_Pin);
		PWM_SetCompare3(Speed);
	}
	else
	{
		GPIO_SetBits(MotorB_BIN_Port, MotorB_BIN1_Pin);
		GPIO_ResetBits(MotorB_BIN_Port, MotorB_BIN2_Pin);
		PWM_SetCompare3(Speed); // 負轉正
	}
}




void Forward(int16_t leftSpeed, int16_t rightSpeed)
{
	work_status = 1;
	MotorA_SetSpeed(leftSpeed);
	MotorB_SetSpeed(rightSpeed);
}

void Backward(int16_t leftSpeed, int16_t rightSpeed)
{
	MotorA_SetSpeed(-leftSpeed);
	MotorB_SetSpeed(-rightSpeed);
}

void Motor_Stop(void)
{
	work_status = 0;
	MotorA_SetSpeed(0);
	MotorB_SetSpeed(0);
}

void TurnLeft(int16_t angle)
{
	MotorA_SetSpeed(Wheel_Turn_Low);
	MotorB_SetSpeed(Wheel_Turn_High);
	
	work_status = 4; // 左轉:不能使用PID
	// delay_ms(angle + 10); // 左轉時間
	TIM4_Delay_ms(angle * 33 + 500); 
	
	Get_YawOffset(); 
	
	Forward(Wheel_Normal_Speed, Wheel_Normal_Speed);
}

void TurnRight(int16_t angle)
{
	MotorA_SetSpeed(Wheel_Turn_High);
	MotorB_SetSpeed(Wheel_Turn_Low);
	
	work_status = 5; // 右轉:不能使用PID
	// delay_ms(angle + 10); // 右轉時間
	TIM4_Delay_ms(angle * 33 + 500); 
	
	Get_YawOffset(); 
	
	// 結束回歸直行
	Forward(Wheel_Normal_Speed, Wheel_Normal_Speed);
}







// 邊刷啟動
void brush_Strat(void)
{
	GPIO_SetBits(Brush_AIN_Port, Brush_AIN1_Pin);
	GPIO_ResetBits(Brush_AIN_Port, Brush_AIN2_Pin);
	PWM_SetCompare4(1000 - Brush_Speed);
}

// 邊刷關閉
void brush_Stop(void)
{
	PWM_SetCompare4(0);
}

// 風扇開啟
void fan_Start(void)
{
	PWM_SetCompare4(Fan_Speed);
}
// 風扇關閉
void fan_Stop(void)
{
	PWM_SetCompare4(0);
}

