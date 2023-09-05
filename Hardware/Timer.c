#include "stm32f10x.h"                  // Device header
#include "Pin.h"
#include "Timer.h"
#include "MPU6050.h"
#include "Algorithm.h"
#include "PID.h"
#include "SR04.h"
#include "OLED.h"
#include "LED.h"

uint8_t Echo_number; // 0:前, 1:左, 2:右


uint8_t offset_flag;
float Yaw_offset;
int8_t count_offserTime; 
int8_t count_algoTime;
float prev_Yaw = 0;

/* Timer3:定時去抓取MPU6050資料*/
void TIM3_Init(uint16_t apr, uint16_t psc)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 開啟時鐘
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 不分頻
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 上數
	TIM_TimeBaseStructure.TIM_Period = apr; // 設置在下一個更新事件裝入活動的自動重裝載寄存器週期的值
	TIM_TimeBaseStructure.TIM_Prescaler = psc; // 設置用來作為TIM3時鐘頻率除數的預分頻值
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); // 中斷始能
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; // 通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 使能IRQ通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 先佔優先級=0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; // 從佔優先級=0
	NVIC_Init(&NVIC_InitStructure);  // 初始化NVIC寄存器
	
	TIM_Cmd(TIM3, ENABLE);  // 使能TIM3
	
	count_offserTime = 100; // 50ms一次，100次就是5秒
	count_algoTime = 10; // 0.2s一次
}



// 定時器中斷函數
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否
		{
			//LED1_Turn();
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志
			
			if(work_status == 1 || work_status == 2 || work_status == 3)
			{
				Read_DMP(&Pitch, &Roll, &Yaw);
				
				if(count_offserTime)
				{
					count_offserTime--;
					Yaw_offset += (Yaw - prev_Yaw);
					prev_Yaw = Yaw;
					OLED_ShowString(4, 1, "OFFSET");
					
					if(count_offserTime == 0)
					{
//						Yaw_offset *= 0.01667;
						OLED_ShowString(4, 1, "      ");
					}
				}
				else
				{
					Yaw -= Yaw_offset;
					L_PID_Control();
				}
				
				OLED_ShowSignedNum(2, 1, Yaw, 3);
				OLED_ShowSignedNum(2, 7, Yaw_offset, 5);
				
			}
		}
}

void Get_YawOffset(void)
{
	Read_DMP(&Pitch, &Roll, &Yaw);
	Yaw_offset = Yaw;
}


/* 控制直流電機馬達PWM */
void TIM1_Init(uint16_t apr, uint16_t psc)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // 開啟TIM1時鐘
	
	// GPIO Init
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	// MotorA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 復用推挽輸出(由片上外設TIM控制輸出)
	GPIO_InitStructure.GPIO_Pin = MotorA_PWM_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MotorA_PWM_Port, &GPIO_InitStructure);
	// MotorB
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 復用推挽輸出(由片上外設TIM控制輸出)
	GPIO_InitStructure.GPIO_Pin = MotorB_PWM_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MotorB_PWM_Port, &GPIO_InitStructure);
	// 邊刷
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 復用推挽輸出(由片上外設TIM控制輸出)
	GPIO_InitStructure.GPIO_Pin = Brush_PWM_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Brush_PWM_Port, &GPIO_InitStructure);
	// 風扇
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 復用推挽輸出(由片上外設TIM控制輸出)
	GPIO_InitStructure.GPIO_Pin = fan_PWM_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(fan_PWM_Port, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM1); // 使用內部時鐘(默認也是內部時鐘，所以不寫也行)
	
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// freq:10k, reso:1%, duty:by compare
	TIM_TimeBaseInitStructure.TIM_Period = apr;  //ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc; //PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
	
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure); //給結構體賦初始值，再給部分需要改的變數做修正
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; // CCR的值(between 0x0000 and 0xFFFF)
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); // TIM1通道1
	
	TIM_OCStructInit(&TIM_OCInitStructure); //給結構體賦初始值，再給部分需要改的變數做修正
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; // CCR的值(between 0x0000 and 0xFFFF)
	TIM_OC2Init(TIM1, &TIM_OCInitStructure); // TIM1通道2
	
	TIM_OCStructInit(&TIM_OCInitStructure); //給結構體賦初始值，再給部分需要改的變數做修正
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; // CCR的值(between 0x0000 and 0xFFFF)
	TIM_OC3Init(TIM1, &TIM_OCInitStructure); // TIM1通道3
	
	TIM_OCStructInit(&TIM_OCInitStructure); //給結構體賦初始值，再給部分需要改的變數做修正
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; // CCR的值(between 0x0000 and 0xFFFF)
	TIM_OC4Init(TIM1, &TIM_OCInitStructure); // TIM1通道4
	
	TIM_Cmd(TIM1, ENABLE); // 啟動定時器
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

// 設置duty cycle(設置速度)
void PWM_SetCompare1(uint16_t Compare) // 小車左輪
{
	TIM_SetCompare1(TIM1, Compare);
}
void PWM_SetCompare2(uint16_t Compare) // 小車右輪
{
	TIM_SetCompare2(TIM1, Compare);
}
void PWM_SetCompare3(uint16_t Compare) // 邊刷馬達
{
	TIM_SetCompare3(TIM1, Compare);
}
void PWM_SetCompare4(uint16_t Compare) // 風扇
{
	TIM_SetCompare4(TIM1, Compare);
}



extern uint16_t Time;

/* HCSR-04(超聲波傳感器)*/
void TIM2_Init(void)
{
	Time = 0;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		
	TIM_InternalClockConfig(TIM2); // 使用內部時鐘(默認也是內部時鐘，所以不寫也行)
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ARR
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// 1us計數一次(定時頻率=10000)
	TIM_TimeBaseInitStructure.TIM_Period = 7199;  // APR 1 = 0.0001s
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;  // PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
//	// input capture(SDA)
//	TIM_ICInitTypeDef TIM_ICInitStructure;
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
//	TIM_ICInitStructure.TIM_ICFilter = 0x00;
//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		if(Echo_number == 0)
		{
			if(GPIO_ReadInputDataBit(HCSR04_Echo_1_Port, HCSR04_Echo_1_Pin) == 1)
			{
				Time++;
			}
		}
		else if(Echo_number == 1)
		{
			if(GPIO_ReadInputDataBit(HCSR04_Echo_2_Port, HCSR04_Echo_2_Pin) == 1)
			{
				Time++;
			}
		}
		else if(Echo_number == 2)
		{
			if(GPIO_ReadInputDataBit(HCSR04_Echo_3_Port, HCSR04_Echo_3_Pin) == 1)
			{
				Time++;
			}			
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

void TIM4_Init(uint16_t apr, uint16_t psc)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 開啟時鐘
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 不分頻
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 上數
	TIM_TimeBaseStructure.TIM_Period = apr; // 設置在下一個更新事件裝入活動的自動重裝載寄存器週期的值
	TIM_TimeBaseStructure.TIM_Prescaler = psc; // 設置用來作為TIM4時鐘頻率除數的預分頻值
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
}

void TIM4_Interrupt_Enable(void)
{
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); // 中斷始能
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; // 通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 使能IRQ通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 先佔優先級=0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; // 從佔優先級=0
	NVIC_Init(&NVIC_InitStructure);  // 初始化NVIC寄存器
	
	TIM_Cmd(TIM4, ENABLE);  // 使能TIM4
}

void TIM4_Interrupt_Disable(void)
{
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE); // 中斷始能
	
	TIM_Cmd(TIM4, DISABLE);  // 使能TIM4
}

/* nms最大到13107，也就是13.107s */
//uint16_t fixed_PSC = 14400;
// uint8_t delay_flag = 0; // 結束計時時，置1
//void TIM_Delay_ms(uint16_t nms)
//{	
//	uint16_t cal_APR = (72000000 / fixed_PSC) / (1 / (nms * 0.001));
//	OLED_ShowNum(2, 1, cal_APR, 5);
//	TIM4_Init(499, 7199);
//	while(delay_flag == 0)
//	{}
//	delay_flag = 0;
//}

uint8_t delay_flag = 0; // 結束計時時，置1

void TIM4_Delay_1ms(void)
{
	while(delay_flag == 0)
	{
		LED1_ON();
	}
	delay_flag = 0;
}

void TIM4_Delay_ms(uint16_t ns)
{
	TIM4_Interrupt_Enable();
	while(ns--)
	{
		TIM4_Delay_1ms();
	}
	TIM4_Interrupt_Disable();
}
		

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除TIMx更新中断标志
		delay_flag = 1;
	}
}



