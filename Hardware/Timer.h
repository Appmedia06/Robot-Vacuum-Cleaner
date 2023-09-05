#ifndef __TIMER_H
#define __TIMER_H

extern float Pitch,Roll,Yaw;
extern uint8_t offset_flag;

void TIM3_Init(uint16_t apr, uint16_t psc);
void Get_YawOffset(void);

void TIM1_Init(uint16_t apr, uint16_t psc);
void PWM_SetCompare1(uint16_t Compare); // 小車左輪
void PWM_SetCompare2(uint16_t Compare); // 小車右輪
void PWM_SetCompare3(uint16_t Compare); // 邊刷馬達
void PWM_SetCompare4(uint16_t Compare); // 風扇

void TIM2_Init(void);

void TIM4_Init(uint16_t apr, uint16_t psc);
void TIM4_Delay_ms(uint16_t ns);
void TIM4_Interrupt_Enable(void);

#endif
