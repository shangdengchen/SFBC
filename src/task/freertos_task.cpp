#include "freertos_task.h"


#include "foc/foc_drive.h"
#include "handle/xbox_controls.h"
#include "power/BatteryAndButton.h"
#include "nvs/nvs.h"
#include "led/led.h"


#include "esp_task_wdt.h"
#include "Init/init_System.h"
#include "web/web.h"

TaskHandle_t Task0;
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t Task5;

//=====================================核心0=========================================

// 创建 Foc_A 初始化任务
void Create_Foc_A_Initialize_Task() {
    xTaskCreatePinnedToCore(Foc_A_Initialize,
                            "Foc_A_Initialize",
                            2048,
                            NULL,
                            4,
                            &Task1,
                            0);
}

// 创建 Foc_B 初始化任务
void Create_Foc_B_Initialize_Task() {
    xTaskCreatePinnedToCore(Foc_B_Initialize,
                            "Foc_B_Initialize",
                            2048,
                            NULL,
                            3,
                            &Task2,
                            0);
}

// 创建按键处理任务
void Create_ButtonBatteryTask() {
    xTaskCreatePinnedToCore(ButtonBatteryTask,
                            "Button_Event",
                            2048,
                            NULL,
                            2,
                            &Task0,
                            0);
}


//=====================================核心1=========================================
// 创建手柄任务
void Create_Handle_control_tasks() {
    if (!xbox_Handle_control_switch) return;
    xTaskCreatePinnedToCore(Handle_control_tasks,
                            "Handle_control",
                            4096,
                            NULL,
                            2,
                            &Task3,
                            1);
}

void Create_Web_tasks() {
    if (xbox_Handle_control_switch)return;
    xTaskCreatePinnedToCore(Web_loop,
                            "Handle_control",
                            4096,
                            NULL,
                            2,
                            &Task4,
                            1);
}

// 创建LED任务
void Create_LED_tasks() {
    //LED显示
    xTaskCreatePinnedToCore(Led_Loop,
                            "Led_Loop",
                            2048,
                            NULL,
                            1,
                            &Task5,
                            1);
}

// 主任务创建函数
void RTOS_Task() {

    // 创建核心0的任务
    Create_Foc_A_Initialize_Task();
    Create_Foc_B_Initialize_Task();


    // 创建核心1的任务
    Create_Handle_control_tasks();
    Create_Web_tasks();
    Create_LED_tasks();

}