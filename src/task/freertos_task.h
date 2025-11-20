//
// Created by MGJ on 2025/4/23.
//

#ifndef FOC_CODE_FREERTOS_TASK_H
#define FOC_CODE_FREERTOS_TASK_H

#include "include/precompiled.h"


extern TaskHandle_t Task0;
extern TaskHandle_t Task1;
extern TaskHandle_t Task2;
extern TaskHandle_t Task3;
extern TaskHandle_t Task4;
extern TaskHandle_t Task5;



void RTOS_Task();
void Create_LED_tasks();
void Led_Loop(void *pvParameters);
extern void Create_ButtonBatteryTask();


#endif //FOC_CODE_FREERTOS_TASK_H