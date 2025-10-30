#include <Arduino.h>
#include "../src/led/led.h"

// 测试状态
enum TestState {
  INIT,
  HEADLIGHT_TEST,
  BRAKE_LIGHT_TEST,
  LEFT_INDICATOR_TEST,
  RIGHT_INDICATOR_TEST,
  HAZARD_LIGHT_TEST,
  WARNING_LIGHT_TEST,
  COMPLETE
};

TestState ledTestState  = INIT;
unsigned long stateStartTime = 0;
const unsigned long STATE_DURATION = 3000; // 每个测试状态持续3秒

void setup() {
  Serial.begin(115200);
  Serial.println("LED功能测试开始");
  
  // 初始化LED系统
  initLights();
  stateStartTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  switch (ledTestState) {
    case INIT:
      Serial.println("初始化完成，开始测试大灯...");
      ledTestState = HEADLIGHT_TEST;
      turnOnHeadlight();
      stateStartTime = currentTime;
      break;
      
    case HEADLIGHT_TEST:
      if (currentTime - stateStartTime >= STATE_DURATION) {
        turnOffHeadlight();
        Serial.println("大灯测试完成，开始测试刹车灯...");
        ledTestState = BRAKE_LIGHT_TEST;
        turnOnBrakeLight();
        stateStartTime = currentTime;
      }
      break;
      
    case BRAKE_LIGHT_TEST:
      if (currentTime - stateStartTime >= STATE_DURATION) {
        turnOffBrakeLight();
        Serial.println("刹车灯测试完成，开始测试左转灯...");
        ledTestState = LEFT_INDICATOR_TEST;
        turnOnLeftIndicator();
        stateStartTime = currentTime;
      }
      break;
      
    case LEFT_INDICATOR_TEST:
      // 更新灯光状态以处理闪烁效果
      updateLights();
      
      if (currentTime - stateStartTime >= STATE_DURATION) {
        turnOffLeftIndicator();
        Serial.println("左转灯测试完成，开始测试右转灯...");
        ledTestState = RIGHT_INDICATOR_TEST;
        turnOnRightIndicator();
        stateStartTime = currentTime;
      }
      break;
      
    case RIGHT_INDICATOR_TEST:
      // 更新灯光状态以处理闪烁效果
      updateLights();
      
      if (currentTime - stateStartTime >= STATE_DURATION) {
        turnOffRightIndicator();
        Serial.println("右转灯测试完成，开始测试双闪灯...");
        ledTestState = HAZARD_LIGHT_TEST;
        turnOnHazardLights();
        stateStartTime = currentTime;
      }
      break;
      
    case HAZARD_LIGHT_TEST:
      // 更新灯光状态以处理闪烁效果
      updateLights();
      
      if (currentTime - stateStartTime >= STATE_DURATION) {
        turnOffHazardLights();
        Serial.println("双闪灯测试完成，开始测试报警灯...");
        ledTestState = WARNING_LIGHT_TEST;
        turnOnWarningLights();
        stateStartTime = currentTime;
      }
      break;
      
    case WARNING_LIGHT_TEST:
      // 更新灯光状态以处理闪烁效果
      updateLights();
      
      if (currentTime - stateStartTime >= STATE_DURATION) {
        turnOffWarningLights();
        Serial.println("报警灯测试完成，所有测试结束。");
        ledTestState = COMPLETE;
      }
      break;
      
    case COMPLETE:
      // 测试完成，系统空闲
      break;
  }
  
  delay(50); // 短暂延迟以确保系统稳定
}