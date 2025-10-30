#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "../Include/common_defs.h"
#include "LEDStripController.h"

// 灯光类型枚举
enum LightType {
    HEADLIGHT = 0,      // 大灯
    TURN_LEFT,          // 左转灯
    TURN_RIGHT,         // 右转灯
    BRAKE_LIGHT,        // 刹车灯
    HAZARD_LIGHT,       // 双闪/危险警示灯
    WARNING_LIGHT       // 报警灯
};

// 灯光状态枚举
enum LightStatus {
    OFF = 0,
    ON,
    BLINKING
};

// 函数声明
void initLights(); // 初始化灯光系统(strip0: 前后灯带1-6前部/7-8后部, strip1: 左右灯带1-3左侧/4-6右侧)
void turnOnHeadlight(); // 打开大灯(控制前后灯带的1-6号灯珠)
void turnOffHeadlight(); // 关闭大灯
void turnOnBrakeLight(); // 打开刹车灯(控制前后灯带的7-8号灯珠)
void turnOffBrakeLight(); // 关闭刹车灯
void turnOnLeftIndicator(); // 打开左转灯(控制左右灯带的1-3号灯珠)
void turnOffLeftIndicator(); // 关闭左转灯
void turnOnRightIndicator(); // 打开右转灯(控制左右灯带的4-6号灯珠)
void turnOffRightIndicator(); // 关闭右转灯
void turnOnHazardLights(); // 打开双闪灯（危险警示灯）
void turnOffHazardLights(); // 关闭双闪灯（危险警示灯）
void turnOnWarningLights(); // 打开报警灯
void turnOffWarningLights(); // 关闭报警灯
void updateLights(); // 更新灯光状态（主要用于处理闪烁效果）

#endif