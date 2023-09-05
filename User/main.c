#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "MPU6050.h"
#include "Key.h"
#include "kalman.h"
#include "Timer.h"
#include "Motor.h"
#include "SR04.h"
#include "PID.h"
#include "Algorithm.h"
#include "LPM.h"
#include "LED.h"

uint8_t Once_Setting = 1; // 用來設置一次性設置
float Pitch, Roll, Yaw;

void display_Running(void);


int main(void)
{
	Key_Init();
	OLED_Init();
	LED_Init();
	TIM4_Init(5, 14400);
	delay_init();
	LPM_Init();
	
	OLED_Clear();
	OLED_ShowString(1, 1, "Press Button...");
	
	Start_StopMode(); // 上電後自動進入停止模式(等待按鍵觸發)
	
	OLED_Clear();
	OLED_ShowString(1, 1, "Launch");
	delay_ms(1000);
	OLED_Clear();
	
	while(1)
	{
		if(Once_Setting)
		{
			TIM4_Delay_ms(5000);
			
			/* 超聲波傳感器初始化 */
			HCSR04_Init();
			Echo_number = 0;
			
			/* MPU6050初始化 */
			IIC_Init();
			MPU6050_initialize();     
			DMP_Init();
			/* 定時抓取MPU6050資料並做PID */
			TIM3_Init(499, 7199);  
			
			/* 馬達初始化 */
			Motor_Init();
			// 開啟馬達
			Forward(Wheel_Normal_Speed, Wheel_Normal_Speed);
			// brush_Strat();
			fan_Start();


			work_status = 1; 
			Once_Setting = 0;
			
			TIM4_Delay_ms(500);
			
			OLED_Clear();
		}
		
		if(work_status != 0)
			interrupt_Disable();
		
		Movement_Algo(); // 掃地機器人的運行演算法	
		LED1_Turn();
		
	
		if(get_KeyFlag())
		{
			Motor_Stop();
			fan_Stop();
			Once_Setting = 1;		
			
			Entry_StopMode();
		}
	}
}




