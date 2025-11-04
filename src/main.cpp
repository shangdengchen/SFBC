#include "./Include/project_config.h"


void setup() {
    //初始化系统
    Setup_System();
    //freertos任务
    RTOS_Task();
}

void loop() {
    //初次启动
    Init_Loop();
    //小车控制
    Control_Loop();
    // 定期重置看门狗
    esp_task_wdt_reset();
}