#ifndef __SR04_H
#define __SR04_H


extern uint8_t Echo_number; // 0:中, 1:左, 2:右

void SR04_Init(void);
void SR04_Trig(void);
float SR04_EchoDistance(void);


void HCSR04_Init(void);
void HCSR04_Start(void);
float HCSR04_GetValue(void);

#endif
