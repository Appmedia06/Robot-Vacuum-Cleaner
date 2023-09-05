#ifndef __ALGORITHM_H
#define __ALGORITHM_H

// 0:停止, 1:前進(檢測前方), 2:前進(檢測左方), 3:前進(檢測右方), 4:左轉, 5:右轉
extern uint8_t work_status; 


void Movement_Algo(void);

#endif
