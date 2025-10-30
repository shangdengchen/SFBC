#ifndef PRE_SYSTEM_H
#define PRE_SYSTEM_H

#include "include/precompiled.h"

extern bool Ready;
extern bool systemInitialized;

void Setup_System();
void Init_Loop();

void PrintTestResult(); // 声明打印测试结果的函数
void LogInitialization(const char* module, bool result); // 声明记录初始化结果的函数
void EmitShutdownSequence(); // 声明执行关机序列的函数
#endif