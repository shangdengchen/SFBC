#include "led.h"
#include <FastLED.h>
#include <Arduino.h>
#include "LEDStripController.h"

// LED控制器实例指针
static LEDStripController* ledStripFront = nullptr;   // 前后LED灯带(1-6前部, 7-8后部)
static LEDStripController* ledStripSide = nullptr;    // 左右LED灯带(1-3左侧, 4-6右侧)

// 灯光状态变量
static bool headlightStatus = false;
static bool brakeLightStatus = false;
static bool leftIndicatorStatus = false;
static bool rightIndicatorStatus = false;
static bool hazardLightsStatus = false;
static bool warningLightsStatus = false;

// 闪烁控制变量
static unsigned long lastBlinkTime = 0;
static bool blinkState = false;
static const unsigned long BLINK_INTERVAL = 500; // 500毫秒闪烁间隔

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
        // 设置前部LED(1-6号灯珠)为白色，模拟大灯效果
        for (int i = 0; i < 6 && i < ledStripFront->getNumLeds(); i++) {
            ledStripFront->setPixelColor(i, CRGB::White);
        }
        ledStripFront->setBrightness(200); // 较亮的亮度
        ledStripFront->show();
        headlightStatus = true;
    }
}

// 关闭大灯
void turnOffHeadlight() {
    if (ledStripFront != nullptr && !hazardLightsStatus && !warningLightsStatus) {
        // 只有当没有其他重要灯光开启时才关闭前部灯光
        if (!leftIndicatorStatus && !rightIndicatorStatus) {
            // 关闭前部灯光(1-6号灯珠)
            for (int i = 0; i < 6 && i < ledStripFront->getNumLeds(); i++) {
                ledStripFront->setPixelColor(i, CRGB::Black);
            }
        } else if (leftIndicatorStatus) {
            // 如果左转灯开着，恢复左转灯显示
            for (int i = 0; i < 6 && i < ledStripFront->getNumLeds(); i++) {
                ledStripFront->setPixelColor(i, CRGB::Black);
            }
            // 设置左侧转向灯(1-3号灯珠)
            for (int i = 0; i < 3 && i < ledStripSide->getNumLeds(); i++) {
                ledStripSide->setPixelColor(i, CRGB::Orange);
            }
        } else if (rightIndicatorStatus) {
            // 如果右转灯开着，恢复右转灯显示
            for (int i = 0; i < 6 && i < ledStripFront->getNumLeds(); i++) {
                ledStripFront->setPixelColor(i, CRGB::Black);
            }
            // 设置右侧转向灯(4-6号灯珠)
            for (int i = 3; i < 6 && i < ledStripSide->getNumLeds(); i++) {
                ledStripSide->setPixelColor(i, CRGB::Orange);
            }
        }
        ledStripFront->show();
        headlightStatus = false;
    }
}

// 打开刹车灯 (控制前后灯带的7-8号灯珠)
void turnOnBrakeLight() {
    if (ledStripFront != nullptr) {
        // 设置后部LED(7-8号灯珠)为红色，模拟刹车灯效果
        for (int i = 6; i < 8 && i < ledStripFront->getNumLeds(); i++) {
            ledStripFront->setPixelColor(i, CRGB::Red);
        }
        ledStripFront->setBrightness(255); // 最亮
        ledStripFront->show();
        brakeLightStatus = true;
    }
}

// 关闭刹车灯
void turnOffBrakeLight() {
    if (ledStripFront != nullptr && !hazardLightsStatus && !warningLightsStatus) {
        if (headlightStatus) {
            // 如果大灯开着，恢复尾灯(7-8号灯珠为红色，但亮度较低)
            for (int i = 6; i < 8 && i < ledStripFront->getNumLeds(); i++) {
                ledStripFront->setPixelColor(i, CRGB::Red);
            }
            ledStripFront->setBrightness(50); // 尾灯亮度较低
        } else {
            // 否则完全关闭后部灯光(7-8号灯珠)
            for (int i = 6; i < 8 && i < ledStripFront->getNumLeds(); i++) {
                ledStripFront->setPixelColor(i, CRGB::Black);
            }
        }
        ledStripFront->show();
        brakeLightStatus = false;
    }
}

// 打开左转灯 (控制左右灯带的1-3号灯珠)
void turnOnLeftIndicator() {
    if (ledStripSide != nullptr) {
        // 清除左侧显示并仅点亮左侧LED(1-3号灯珠)
        for (int i = 0; i < 3 && i < ledStripSide->getNumLeds(); i++) {
            ledStripSide->setPixelColor(i, CRGB::Orange);
        }
        ledStripSide->show();
        leftIndicatorStatus = true;
    }
}

// 关闭左转灯
void turnOffLeftIndicator() {
    if (ledStripSide != nullptr) {
        if (headlightStatus) {
            // 如果大灯开着，恢复大灯显示
            turnOnHeadlight();
        } else {
            // 否则关闭所有左侧灯光(1-3号灯珠)
            for (int i = 0; i < 3 && i < ledStripSide->getNumLeds(); i++) {
                ledStripSide->setPixelColor(i, CRGB::Black);
            }
            ledStripSide->show();
        }
        leftIndicatorStatus = false;
    }
}

// 打开右转灯 (控制左右灯带的4-6号灯珠)
void turnOnRightIndicator() {
    if (ledStripSide != nullptr) {
        // 清除右侧显示并仅点亮右侧LED(4-6号灯珠)
        for (int i = 3; i < 6 && i < ledStripSide->getNumLeds(); i++) {
            ledStripSide->setPixelColor(i, CRGB::Orange);
        }
        ledStripSide->show();
        rightIndicatorStatus = true;
    }
}

// 关闭右转灯
void turnOffRightIndicator() {
    if (ledStripSide != nullptr) {
        if (headlightStatus) {
            // 如果大灯开着，恢复大灯显示
            turnOnHeadlight();
        } else {
            // 否则关闭所有右侧灯光(4-6号灯珠)
            for (int i = 3; i < 6 && i < ledStripSide->getNumLeds(); i++) {
                ledStripSide->setPixelColor(i, CRGB::Black);
            }
            ledStripSide->show();
        }
        rightIndicatorStatus = false;
    }
}

// 打开双闪灯（危险警示灯）
void turnOnHazardLights() {
    if (ledStripFront != nullptr && ledStripSide != nullptr) {
        // 设置所有LED为橙色闪烁
        // 前后灯带(1-8号灯珠)
        for (int i = 0; i < ledStripFront->getNumLeds(); i++) {
            ledStripFront->setPixelColor(i, CRGB::Orange);
        }
        // 左右灯带(1-6号灯珠)
        for (int i = 0; i < ledStripSide->getNumLeds(); i++) {
            ledStripSide->setPixelColor(i, CRGB::Orange);
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
        
        ledStripFront->show();
        ledStripSide->show();
        warningLightsStatus = false;
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
                // 前后灯带(1-8号灯珠)
                for (int i = 0; i < ledStripFront->getNumLeds(); i++) {
                    ledStripFront->setPixelColor(i, CRGB::Orange);
                }
                // 左右灯带(1-6号灯珠)
                for (int i = 0; i < ledStripSide->getNumLeds(); i++) {
                    ledStripSide->setPixelColor(i, CRGB::Orange);
                }
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
                // 前后灯带(1-8号灯珠)
                for (int i = 0; i < ledStripFront->getNumLeds(); i++) {
                    ledStripFront->setPixelColor(i, CRGB::Red);
                }
                // 左右灯带(1-6号灯珠)
                for (int i = 0; i < ledStripSide->getNumLeds(); i++) {
                    ledStripSide->setPixelColor(i, CRGB::Red);
                }
            } else {
                // 报警灯灭
                ledStripFront->clear();
                ledStripSide->clear();
            }
            ledStripFront->show();
            ledStripSide->show();
        } else if (leftIndicatorStatus && !hazardLightsStatus) {
            if (blinkState) {
                // 左转灯亮
                if (ledStripSide != nullptr) {
                    for (int i = 0; i < 3 && i < ledStripSide->getNumLeds(); i++) {
                        ledStripSide->setPixelColor(i, CRGB::Orange);
                    }
                    ledStripSide->show();
                }
            } else {
                // 左转灯灭
                if (ledStripSide != nullptr) {
                    if (headlightStatus) {
                        // 如果大灯开着，保持前部灯光
                        turnOnHeadlight();
                    } else {
                        // 否则关闭左侧灯光
                        for (int i = 0; i < 3 && i < ledStripSide->getNumLeds(); i++) {
                            ledStripSide->setPixelColor(i, CRGB::Black);
                        }
                        ledStripSide->show();
                    }
                }
            }
        } else if (rightIndicatorStatus && !hazardLightsStatus) {
            if (blinkState) {
                // 右转灯亮
                if (ledStripSide != nullptr) {
                    for (int i = 3; i < 6 && i < ledStripSide->getNumLeds(); i++) {
                        ledStripSide->setPixelColor(i, CRGB::Orange);
                    }
                    ledStripSide->show();
                }
            } else {
                // 右转灯灭
                if (ledStripSide != nullptr) {
                    if (headlightStatus) {
                        // 如果大灯开着，保持前部灯光
                        turnOnHeadlight();
                    } else {
                        // 否则关闭右侧灯光
                        for (int i = 3; i < 6 && i < ledStripSide->getNumLeds(); i++) {
                            ledStripSide->setPixelColor(i, CRGB::Black);
                        }
                        ledStripSide->show();
                    }
                }
            }
        }
    }
}