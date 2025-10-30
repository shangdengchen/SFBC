#ifndef LED_STRIP_CONTROLLER_H
#define LED_STRIP_CONTROLLER_H

#include <FastLED.h>

class LEDStripController {
private:
    CRGB* leds;
    int numLeds;
    int dataPin;
    
public:
    // 构造函数
    LEDStripController(int numLeds, int dataPin);
    
    // 初始化函数
    void begin();
    
    // 基本控制函数
    void setPixelColor(int index, CRGB color);
    void setPixelColor(int index, uint8_t r, uint8_t g, uint8_t b);
    void setColor(CRGB color);
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setBrightness(uint8_t brightness);
    void clear();
    void show();
    
    // 效果函数
    void rainbow();
    void rainbowCycle();
    void theaterChase(CRGB color);
    void fadeToBlack(int ledIndex, uint8_t fadeValue);
    
    // 工具函数
    int getNumLeds() const { return numLeds; }
    CRGB* getLeds() const { return leds; }
};

#endif