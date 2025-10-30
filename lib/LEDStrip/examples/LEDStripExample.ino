/*
 * LEDStripController 使用示例
 * 
 * 这个示例演示了如何使用LEDStripController类控制WS2812 LED灯带
 */

#include "LEDStripController.h"

// 创建LEDStripController实例
// 参数1: LED数量
// 参数2: 数据引脚
LEDStripController ledStrip(30, 13);

void setup() {
    // 初始化LED灯带
    ledStrip.begin();
    
    // 设置所有LED为红色
    ledStrip.setColor(255, 0, 0);
    ledStrip.show();
    delay(1000);
    
    // 设置所有LED为绿色
    ledStrip.setColor(0, 255, 0);
    ledStrip.show();
    delay(1000);
    
    // 设置所有LED为蓝色
    ledStrip.setColor(0, 0, 255);
    ledStrip.show();
    delay(1000);
    
    // 清除所有LED
    ledStrip.clear();
    ledStrip.show();
}

void loop() {
    // 彩虹效果演示
    for(int i = 0; i < 256; i++) {
        ledStrip.rainbow();
        ledStrip.show();
        delay(20);
    }
    
    // 剧场追逐效果演示
    for(int i = 0; i < 100; i++) {
        ledStrip.theaterChase(CRGB::Blue);
        ledStrip.show();
        delay(100);
    }
    
    // 清除LED
    ledStrip.clear();
    ledStrip.show();
    delay(1000);
}