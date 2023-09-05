#ifndef __KEY_H
#define __KEY_H

void Key_Init(void);
void interrupt_Enable(void);
void interrupt_Disable(void);
uint8_t get_KeyFlag(void);

#endif
