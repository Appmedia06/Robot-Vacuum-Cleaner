#include "stm32f10x.h"                  // Device header
#include "Algorithm.h"
#include "SR04.h"
#include "Motor.h"
#include "OLED.h"

#define F_BorderDistance 15  // 前面邊界距離限制
#define LR_BorderDistance 5  // 左右邊界距離限制
#define ObstacleDistance 5


float distance;
uint8_t Turn = 0; // 0:上次是左轉, 1:上次是右轉
uint8_t work_status;


void Movement_Algo(void)
{
	distance = HCSR04_GetValue();
	OLED_ShowNum(4, 8, distance, 3);
	
	if(work_status == 1) // 一般前行的模式
	{
		if(distance < F_BorderDistance)
		{
			Motor_Stop();
			if(Turn == 0) // 上一次是左轉
			{
				// 判斷右邊有無障礙物
				Echo_number = 2;
				distance = HCSR04_GetValue();
				
				if(distance > ObstacleDistance) // 無障礙物:右迴轉
				{
					TurnRight(180); // 180度右迴轉
					Echo_number = 0; // 前感測
					work_status = 1; // 設回一般模式
					Turn = 1; // 這次是右轉
				}
				else              // 有障礙物:左轉，直到右邊沒東西
				{
					TurnLeft(90); // 90度左轉
					//Echo_number = 2; // 前面已經設過
					work_status = 3; // 檢測右方
					Turn = 0; // 這次是左轉
				}
			}
			else  // 上一次是右轉
			{
				// 判斷左邊有無障礙物
				Echo_number = 1;
				distance = HCSR04_GetValue();
				
				if(distance > ObstacleDistance) // 無障礙物:左迴轉
				{
					TurnLeft(180); // 180度迴轉
					Echo_number = 0; // 前感測
					work_status = 1; // 設回一般模式
					Turn = 0; // 這次是左轉
				}
				else              // 有障礙物:右轉，直到左邊沒東西
				{
					TurnRight(90); // 90度右轉
					//Echo_number = 1; // 前面已經設過
					work_status = 2; // // 檢測左方
					Turn = 1; // 這次是右轉
				}
			}			
		}
	}
	else if(work_status == 2 || work_status == 3) // 檢測左方or右方
	{
		if(distance > LR_BorderDistance)
		{
			if(Turn == 0)
			{
				TurnRight(90); 
				Echo_number = 0; // 前感測
				work_status = 1; // 設回一般模式
				Turn = 1;
			}
			else
			{
				TurnLeft(90); 
				Echo_number = 0; // 前感測
				work_status = 1; // 設回一般模式
				Turn = 0;
			}
		}
	}	
}

