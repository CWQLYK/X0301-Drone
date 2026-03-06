#ifndef __IWDG_H
#define __IWDG_H
void IWDG_Init(uint8_t prer, uint16_t rlr);            //初始化独立看门狗
void IWDG_Feed(void);                                  //喂狗
#endif

