#include "stm32f10x.h"                  // Device header
#include "SR04.h"
#include "Pin.h"
#include "Delay.h"
#include "Timer.h"


uint16_t Time;

void HCSR04_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// SCL(PA0)
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = HCSR04_Trig_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HCSR04_Trig_Port, &GPIO_InitStructure);
	
	// SDA_1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = HCSR04_Echo_1_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HCSR04_Echo_1_Port, &GPIO_InitStructure);
	
	// SDA_2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = HCSR04_Echo_2_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HCSR04_Echo_2_Port, &GPIO_InitStructure);
	
	// SDA_3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = HCSR04_Echo_3_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HCSR04_Echo_3_Port, &GPIO_InitStructure);
	
	GPIO_ResetBits(HCSR04_Trig_Port, HCSR04_Trig_Pin); // 觸發線置低電平
}

void HCSR04_Start(void)
{
	GPIO_SetBits(HCSR04_Trig_Port, HCSR04_Trig_Pin);
	delay_us(45);
	GPIO_ResetBits(HCSR04_Trig_Port, HCSR04_Trig_Pin);
	TIM2_Init();
}

float HCSR04_GetValue(void)
{
	HCSR04_Start();
	delay_ms(100);
	/* 
	聲波距離公式(by 手冊) 距離 = 高電平時間 * 聲速(340m/s) / 2 
	除2:聲波來回
	*/
	return (((float)Time * 0.0001) * 34000) / 2;
}

