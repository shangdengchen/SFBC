#include "led.h"

#include <Arduino.h>
#include "control/control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// LED控制器实例指针
static LEDStripController* ledStripFront = nullptr;   // 前后LED灯带(1-6前部, 7-8后部)
static LEDStripController* ledStripSide = nullptr;    // 左右LED灯带(1-3左侧, 4-6右侧)

// 灯光亮度变量
static uint8_t headlightBrightness = 100;     // 大灯亮度
static uint8_t brakeLightBrightness = 150;    // 刹车灯亮度
static uint8_t taillightBrightness = 50;      // 尾灯亮度
static uint8_t ambientLightBrightness = 60;  // 氛围灯亮度

// 灯光状态变量
static bool headlightStatus = false;
static bool brakeLightStatus = false;
static bool smallLightStatus = false;
static bool leftIndicatorStatus = false;
static bool rightIndicatorStatus = false;
static bool hazardLightsStatus = false;
static bool warningLightsStatus = false;
static bool ambientLightStatus = false;

// 闪烁控制变量
static unsigned long lastBlinkTime = 0;
static bool blinkState = false;
static const unsigned long BLINK_INTERVAL = 500; // 500毫秒闪烁间隔

// 辅助函数：恢复灯光状态
static void restoreLightStatus() {
    // 根据其他灯光状态恢复显示
    if (headlightStatus) {
        turnOnHeadlight();
    }
    
    if (brakeLightStatus) {
        turnOnBrakeLight();
    }
    
    if (leftIndicatorStatus) {
        turnOnLeftIndicator();
    }
    
    if (rightIndicatorStatus) {
        turnOnRightIndicator();
    }
    
    if (smallLightStatus) {
        turnOnSmallLight();
    }
    
    if (ambientLightStatus) {
        // 通过临时将ambientLightStatus取反再恢复的方式重新激活氛围灯
        ambientLightStatus = false;// 紫罗兰色氛围灯
    }
}

// 辅助函数：设置前部灯光（用于大灯和转向灯）
static void setFrontLights(CRGB color) {
    if (ledStripFront != nullptr) {
        for (int i = 0; i < 6 && i < ledStripFront->getNumLeds(); i++) {
            ledStripFront->setPixelColor(i, color);
        }
        ledStripFront->show();
    }
}

// 辅助函数：设置后部灯光
static void setRearLights(CRGB color, uint8_t brightness) {
    if (ledStripFront != nullptr) {
        for (int i = 6; i < 8 && i < ledStripFront->getNumLeds(); i++) {
            ledStripFront->setPixelColor(i, color);
        }
        ledStripFront->setBrightness(brightness);
        ledStripFront->show();
    }
}

// 辅助函数：设置左侧氛围灯
static void setLeftSideLights(CRGB color) {
    if (ledStripSide != nullptr) {
        for (int i = 0; i < 3 && i < ledStripSide->getNumLeds(); i++) {
            ledStripSide->setPixelColor(i, color);
        }
        ledStripSide->show();
    }
}

// 辅助函数：设置右侧氛围灯
static void setRightSideLights(CRGB color) {
    if (ledStripSide != nullptr) {
        for (int i = 3; i < 6 && i < ledStripSide->getNumLeds(); i++) {
            ledStripSide->setPixelColor(i, color);
        }
        ledStripSide->show();
    }
}

// 初始化灯光系统
void initLights() {
    // 创建LEDStripController实例
    static LEDStripController strip0(NUM_FB_LEDS, LED_FB_PIN);   // 前后LED灯带
    static LEDStripController strip1(NUM_LR_LEDS, LED_LR_PIN);   // 左右LED灯带

    ledStripFront = &strip0;
    ledStripSide = &strip1;
    // 初始化LED灯带
    ledStripFront->begin();
    ledStripSide->begin();
    // 清除所有灯光
    ledStripFront->clear();
    ledStripSide->clear();
    ledStripFront->show();
    ledStripSide->show();
}

// 打开大灯 (控制前后灯带的1-6号灯珠)
void turnOnHeadlight() {
    if (ledStripFront != nullptr) {
        setFrontLights(CRGB::White);
        ledStripFront->setBrightness(headlightBrightness);
        ledStripFront->show();
        headlightStatus = true;
    }
}

// 关闭大灯
void turnOffHeadlight() {
    if (ledStripFront != nullptr && !hazardLightsStatus && !warningLightsStatus) {
        // 直接关闭前部灯光，不考虑转向灯状态，因为它们使用不同的灯带
        setFrontLights(CRGB::Black);
        headlightStatus = false;
    }
}

// 打开刹车灯 (控制前后灯带的7-8号灯珠)
void turnOnBrakeLight() {
    setRearLights(CRGB::Red, brakeLightBrightness);
    brakeLightStatus = true;
}

// 关闭刹车灯
void turnOffBrakeLight() {
    if (!hazardLightsStatus && !warningLightsStatus) {
        if (headlightStatus || smallLightStatus) {
            // 如果大灯或小灯开着，恢复尾灯(7-8号灯珠为红色，但亮度较低)
            setRearLights(CRGB::Red, taillightBrightness);
        } else {
            // 否则完全关闭后部灯光(7-8号灯珠)
            setRearLights(CRGB::Black, taillightBrightness);
        }
        brakeLightStatus = false;
    }
}

// 打开小灯 (控制前后灯带的7-8号灯珠，亮度较低)
void turnOnSmallLight() {
    if (!brakeLightStatus) {
        setRearLights(CRGB::Red, taillightBrightness);
        smallLightStatus = true;
    }
}

// 关闭小灯
void turnOffSmallLight() {
    if (!brakeLightStatus && !hazardLightsStatus && !warningLightsStatus) {
        if (headlightStatus) {
            // 如果大灯开着，保持尾灯状态(7-8号灯珠为红色，但亮度较低)
            setRearLights(CRGB::Red, taillightBrightness);
        } else {
            // 否则完全关闭后部灯光(7-8号灯珠)
            setRearLights(CRGB::Black, taillightBrightness);
        }
        smallLightStatus = false;
    }
}

// 打开左转灯 (控制左右灯带的1-3号灯珠)
void turnOnLeftIndicator() {
    setLeftSideLights(CRGB(255, 255, 0)); // 纯正黄色
    leftIndicatorStatus = true;
}

// 关闭左转灯
void turnOffLeftIndicator() {
    // 直接关闭左侧灯光，不考虑大灯状态，因为它们使用不同的灯带
    setLeftSideLights(CRGB::Black);
    leftIndicatorStatus = false;
    
    // 恢复其他灯光状态（包括氛围灯）
    restoreLightStatus();
}

// 打开右转灯 (控制左右灯带的4-6号灯珠)
void turnOnRightIndicator() {
    setRightSideLights(CRGB(255, 255, 0)); // 纯正黄色
    rightIndicatorStatus = true;
}

// 关闭右转灯
void turnOffRightIndicator() {
    // 直接关闭右侧灯光，不考虑大灯状态，因为它们使用不同的灯带
    setRightSideLights(CRGB::Black);
    rightIndicatorStatus = false;
    
    // 恢复其他灯光状态（包括氛围灯）
    restoreLightStatus();
}

// 打开双闪灯（危险警示灯）
void turnOnHazardLights() {
    if (ledStripFront != nullptr && ledStripSide != nullptr) {
        // 设置所有LED为黄色闪烁
        // 前后灯带(1-8号灯珠)
        for (int i = 0; i < ledStripFront->getNumLeds(); i++) {
            ledStripFront->setPixelColor(i, CRGB(255, 255, 0)); // 纯正黄色
        }
        // 左右灯带(1-6号灯珠)
        for (int i = 0; i < ledStripSide->getNumLeds(); i++) {
            ledStripSide->setPixelColor(i, CRGB(255, 255, 0)); // 纯正黄色
        }
        ledStripFront->show();
        ledStripSide->show();
        hazardLightsStatus = true;
    }
}

// 关闭双闪灯（危险警示灯）
void turnOffHazardLights() {
    if (ledStripFront != nullptr && ledStripSide != nullptr) {
        // 恢复之前的状态
        // 清除所有灯光
        ledStripFront->clear();
        ledStripSide->clear();
        
        restoreLightStatus();
        
        ledStripFront->show();
        ledStripSide->show();
        hazardLightsStatus = false;
    }
}

// 打开报警灯
void turnOnWarningLights() {
    if (ledStripFront != nullptr && ledStripSide != nullptr) {
        // 设置所有LED为红色闪烁
        // 前后灯带(1-8号灯珠)
        for (int i = 0; i < ledStripFront->getNumLeds(); i++) {
            ledStripFront->setPixelColor(i, CRGB::Red);
        }
        // 左右灯带(1-6号灯珠)
        for (int i = 0; i < ledStripSide->getNumLeds(); i++) {
            ledStripSide->setPixelColor(i, CRGB::Red);
        }
        ledStripFront->show();
        ledStripSide->show();
        warningLightsStatus = true;
    }
}

// 关闭报警灯
void turnOffWarningLights() {
    if (ledStripFront != nullptr && ledStripSide != nullptr) {
        // 恢复之前的状态
        ledStripFront->clear();
        ledStripSide->clear();
        
        restoreLightStatus();
        
        ledStripFront->show();
        ledStripSide->show();
        warningLightsStatus = false;
    }
}

// 设置氛围灯颜色 (控制左右灯带的所有LED)
void setAmbientLight(uint8_t r, uint8_t g, uint8_t b) {
    if (ledStripSide != nullptr) {
        // 设置所有左右LED为指定颜色
        for (int i = 0; i < ledStripSide->getNumLeds(); i++) {
            ledStripSide->setPixelColor(i, r, g, b);
        }
        ledStripSide->setBrightness(ambientLightBrightness);
        ledStripSide->show();
        ambientLightStatus = true;
    }
}

// 关闭氛围灯
void turnOffAmbientLight() {
    if (ledStripSide != nullptr && !leftIndicatorStatus && !rightIndicatorStatus && 
        !hazardLightsStatus && !warningLightsStatus) {
        // 只有当没有其他灯光开启时才关闭氛围灯
        for (int i = 0; i < ledStripSide->getNumLeds(); i++) {
            ledStripSide->setPixelColor(i, CRGB::Black);
        }
        ledStripSide->show();
        ambientLightStatus = false;
    }
}

// 更新灯光状态（主要用于处理闪烁效果）
void updateLights() {
    unsigned long currentTime = millis();
    
    // 处理闪烁逻辑
    if ((hazardLightsStatus || warningLightsStatus || leftIndicatorStatus || rightIndicatorStatus) && 
        (currentTime - lastBlinkTime >= BLINK_INTERVAL)) {
        
        blinkState = !blinkState;
        lastBlinkTime = currentTime;
        
        if (hazardLightsStatus) {
            if (blinkState) {
                // 双闪灯亮
                setFrontLights(CRGB::Orange);
                setLeftSideLights(CRGB::Orange);
                setRightSideLights(CRGB::Orange);
            } else {
                // 双闪灯灭
                ledStripFront->clear();
                ledStripSide->clear();
            }
            ledStripFront->show();
            ledStripSide->show();
        } else if (warningLightsStatus) {
            if (blinkState) {
                // 报警灯亮
                setFrontLights(CRGB::Red);
                setLeftSideLights(CRGB::Red);
                setRightSideLights(CRGB::Red);
            } else {
                // 报警灯灭
                ledStripFront->clear();
                ledStripSide->clear();
            }
            ledStripFront->show();
            ledStripSide->show();
        } else if (leftIndicatorStatus) {
            if (blinkState) {
                // 左转灯亮
                setLeftSideLights(CRGB(255, 255, 0)); // 纯正黄色
            } else {
                // 左转灯灭
                // 直接关闭左侧灯光，不考虑大灯状态，因为它们使用不同的灯带
                setLeftSideLights(CRGB::Black);
            }
        } else if (rightIndicatorStatus) {
            if (blinkState) {
                // 右转灯亮
                setRightSideLights(CRGB(255, 255, 0)); // 纯正黄色
            } else {
                // 右转灯灭
                // 直接关闭右侧灯光，不考虑大灯状态，因为它们使用不同的灯带
                setRightSideLights(CRGB::Black);
            }
        }
    }
}

// LED任务主循环
void Led_Loop(void *pvParameters) {
    // 初始化时不再直接开启任何灯光，改为根据状态控制
    
    for (;;) {
        // 获取当前状态
        System_Status current_status = Car_Info.Status;
        float target_speed = Car_Info.TargetSpeed;
        float target_steer = Car_Info.TargetSteer;
        
        // 根据小车状态控制大灯
        // 当小车状态为Open_Output时开启大灯，表示小车处于运行状态
        if (current_status == Open_Output && !headlightStatus) {
            turnOnHeadlight();
        } else if (current_status != Open_Output && headlightStatus) {
            turnOffHeadlight();
        }
        
        // 根据小车状态控制小灯
        // 当小车状态为Open_Output时开启小灯，表示系统正常运行
        if (current_status == Open_Output && !smallLightStatus) {
            turnOnSmallLight();
        } else if (current_status != Open_Output && smallLightStatus) {
            turnOffSmallLight();
        }
        
        // 根据小车状态控制氛围灯
        // 当小车状态为Open_Output或Disable_Output时开启紫罗兰色氛围灯
        if ((current_status == Open_Output || current_status == Disable_Output) && !ambientLightStatus) {
            setAmbientLight(138, 43, 226); // 紫罗兰色氛围灯
        } else if (current_status != Open_Output && current_status != Disable_Output && ambientLightStatus) {
            turnOffAmbientLight();
        }
        
        // 根据小车状态控制刹车灯
        // 当小车处于Disable_Output状态时打开刹车灯，表示停止状态
        if (current_status == Disable_Output && !brakeLightStatus) {
            turnOnBrakeLight();
        } else if (current_status != Disable_Output && brakeLightStatus) {
            turnOffBrakeLight();
        }
        
        // 判断是否需要开启转向灯
        if (target_steer > 0.1f && !rightIndicatorStatus) {  // 右转阈值
            turnOnRightIndicator();
        } else if (target_steer <= 0.05f && rightIndicatorStatus) {  // 关闭阈值
            turnOffRightIndicator();
        }
        
        if (target_steer < -0.1f && !leftIndicatorStatus) {  // 左转阈值
            turnOnLeftIndicator();
        } else if (target_steer >= -0.05f && leftIndicatorStatus) {  // 关闭阈值
            turnOffLeftIndicator();
        }
        
        // 更新灯光状态（主要是闪烁效果）
        updateLights();
        
        // 延迟100ms
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
