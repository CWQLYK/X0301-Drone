#ifndef _REMOTE_H
#define _REMOTE_H
#include "ALL_DEFINE.h"

// 全局标志位声明（供其他模块调用）
extern u16 test_flag;   // 测试功能标志位（预留扩展）
extern u16 set_flag;    // 参数设置标志位（预留扩展）

// 函数声明（遥控模块核心接口）
extern void RC_Analy(void);    // 遥控数据解析与异常处理

#endif

