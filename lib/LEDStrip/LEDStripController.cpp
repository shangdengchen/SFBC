#include "LEDStripController.h"
#include <Arduino.h>

// 构造函数
LEDStripController::LEDStripController(int numLeds, int dataPin) {
    this->numLeds = numLeds;
    this->dataPin = dataPin;
    this->leds = new CRGB[numLeds];
}

// 初始化函数
void LEDStripController::begin() {
    switch(dataPin) {
        case 38:
            FastLED.addLeds<WS2812, 38, GRB>(leds, numLeds);
            break;
        case 39:
            FastLED.addLeds<WS2812, 39, GRB>(leds, numLeds);
            break;
        default:
            FastLED.addLeds<WS2812, 38, GRB>(leds, numLeds);
            break;
    }
    FastLED.setBrightness(50);
    clear();
}

// 设置特定像素的颜色
void LEDStripController::setPixelColor(int index, CRGB color) {
    if (index >= 0 && index < numLeds) {
        leds[index] = color;
    }
}

// 设置特定像素的RGB颜色
void LEDStripController::setPixelColor(int index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= 0 && index < numLeds) {
        leds[index] = CRGB(r, g, b);
    }
}

// 设置所有LED为同一颜色
void LEDStripController::setColor(CRGB color) {
    for (int i = 0; i < numLeds; i++) {
        leds[i] = color;
    }
}

// 设置所有LED为同一RGB颜色
void LEDStripController::setColor(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < numLeds; i++) {
        leds[i] = CRGB(r, g, b);
    }
}

// 设置亮度
void LEDStripController::setBrightness(uint8_t brightness) {
    FastLED.setBrightness(brightness);
}

// 清除所有LED（设为黑色）
void LEDStripController::clear() {
    for (int i = 0; i < numLeds; i++) {
        leds[i] = CRGB::Black;
    }
}

// 显示LED状态
void LEDStripController::show() {
    FastLED.show();
}

// 彩虹效果
void LEDStripController::rainbow() {
    static uint8_t hue = 0;
    for (int i = 0; i < numLeds; i++) {
        leds[i] = CHSV(hue + (i * 256 / numLeds), 255, 255);
    }
    hue++;
}

// 循环彩虹效果
void LEDStripController::rainbowCycle() {
    static uint8_t hue = 0;
    for (int i = 0; i < numLeds; i++) {
        leds[i] = CHSV((i * 256 / numLeds) + hue, 255, 255);
    }
    hue++;
}

// 剧场追逐效果
void LEDStripController::theaterChase(CRGB color) {
    static uint8_t chasePosition = 0;
    clear();
    
    for (int i = 0; i < numLeds; i += 3) {
        leds[i + chasePosition] = color;
    }
    
    chasePosition = (chasePosition + 1) % 3;
}

// 淡出特定LED
void LEDStripController::fadeToBlack(int ledIndex, uint8_t fadeValue) {
    if (ledIndex >= 0 && ledIndex < numLeds) {
        leds[ledIndex].nscale8(fadeValue);
    }
}