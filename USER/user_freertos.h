#ifndef __USER_FREERTOS_H
#define __USER_FREERTOS_H


#include "scheduler.h"


#define START_TASK_STACK_SIZE    256    // 启动任务栈增大
#define START_TASK_PRIORITY     1       // 启动任务低优先级
#define TASK_2MS_STACK_SIZE     512     // 核心任务栈增大
#define TASK_2MS_PRIORITY       10      // 最高优先级（核心控制）
#define TASK_4MS_STACK_SIZE     256
#define TASK_4MS_PRIORITY       9
#define TASK_6MS_STACK_SIZE     256
#define TASK_6MS_PRIORITY       8
#define TASK_10MS_STACK_SIZE    256
#define TASK_10MS_PRIORITY      7
#define TASK_20MS_STACK_SIZE    256
#define TASK_20MS_PRIORITY      6
#define TASK_50MS_STACK_SIZE    256
#define TASK_50MS_PRIORITY      5
#define TASK_500MS_STACK_SIZE   128
#define TASK_500MS_PRIORITY     4
#define TASK_1000MS_STACK_SIZE  128
#define TASK_1000MS_PRIORITY    3


void user_freertos_start(void);

#endif // !__USER_FREERTOS_H
