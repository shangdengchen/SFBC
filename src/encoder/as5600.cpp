//
// Created by MGJ on 2025/5/23.
//
#include "encoder.h"
#include "I2C/I2C_Manage.h"

#define AS5600_ADDR       0x36    // AS5600 的 I2C 地址
#define CHIP_ID_REG       0x0F    // 芯片 ID 寄存器地址


MagneticSensorI2C AS5600_I2C_A = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C AS5600_I2C_B = MagneticSensorI2C(AS5600_I2C);


bool Check_AS5600_A() {
    TwoWire *wire_a = &I2C_A;
    // 检查第一个编码器（AS5600_A）
    wire_a->beginTransmission(AS5600_ADDR);
    wire_a->write(CHIP_ID_REG);
    if (wire_a->endTransmission() != 0) { // I2C通信失败
        Serial.println("[AS5600_A]: 未发现");
        return false;
    }
    if (wire_a->requestFrom(AS5600_ADDR, 1) != 1) { // 请求数据失败
        Serial.println("[AS5600_A]: 数据读取失败!");
        return false;
    }
    Serial.println("[AS5600_A]: 连接成功!");
    return true;
}


bool Check_AS5600_B() {
    TwoWire *wire_b = &I2C_B;
    // 检查第二个编码器（AS5600_B）
    wire_b->beginTransmission(AS5600_ADDR);
    wire_b->write(CHIP_ID_REG);
    if (wire_b->endTransmission() != 0) {
        Serial.println("[AS5600_B]: 未发现");
        return false;
    }
    if (wire_b->requestFrom(AS5600_ADDR, 1) != 1) {
        Serial.println("[AS5600_B]: 数据读取失败!");
        return false;
    }
    Serial.println("[AS5600_B]: 连接成功!");
    return true;
}



