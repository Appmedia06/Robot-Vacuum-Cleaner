#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Key.h"


void LPM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // 開啟PWR時鐘
}

void Start_StopMode(void)
{
	PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI); // 進入停止模式(等待外部按鍵中斷)
	SystemInit(); // 改回72M
}

void Entry_StopMode(void)
{
	OLED_Clear();
	OLED_ShowString(1, 1, "Stop working...");
	delay_ms(1000);
	
	uint8_t count_KeyTime = 10;
	while(count_KeyTime)
	{
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 1)
			count_KeyTime--;
	}
	
	interrupt_Enable();
	Start_StopMode(); // 開啟停止模式
	OLED_Clear();
	OLED_ShowString(1, 1, "Launch");
}

