//
// Created by MGJ on 2025/5/23.
//
#include "encoder.h"
#include "I2C/I2C_Manage.h"

// MT6701 配置
#define MT6701_I2C_ADDR   0x06  // 7位I2C地址
#define ANGLE_REG_HIGH    0x03   // 角度高8位寄存器
#define ANGLE_REG_LOW     0x04   // 角度低6位寄存器


TwoWire *Wire_A;
TwoWire *Wire_B;


GenericSensor MT6701_I2C_A = GenericSensor(Read_MT6701_I2C_A, Init_MT6701_I2C_A);
GenericSensor MT6701_I2C_B = GenericSensor(Read_MT6701_I2C_B, Init_MT6701_I2C_B);


bool Check_MT6701_A() {
    Init_MT6701_I2C_A();
    Wire_A->beginTransmission(MT6701_I2C_ADDR);    // 发起I2C传输到设备地址
    uint8_t error = Wire_A->endTransmission();     // 结束传输并获取错误码
    if (error == 0)
    {
        Serial.println("[MT6701_A]: 连接成功!");
        return true;
    } else
    {
        Serial.println("[MT6701_A]: 未发现");
        return false;
    }

}


bool Check_MT6701_B() {
    Init_MT6701_I2C_B();
    Wire_B->beginTransmission(MT6701_I2C_ADDR);    // 发起I2C传输到设备地址
    uint8_t error = Wire_B->endTransmission();     // 结束传输并获取错误码
    if (error == 0)
    {
        Serial.println("[MT6701_B]: 连接成功!");
        return true;
    } else
    {
        Serial.println("[MT6701_B]: 未发现");
        return false;
    }
}


void Init_MT6701_I2C_A() {
    Wire_A = &I2C_A;
}
void Init_MT6701_I2C_B() {
    Wire_B = &I2C_B;
}

float Read_MT6701_I2C_A() {
    // === 第一步：读取高8位（0x03寄存器） ===
    Wire_A->beginTransmission(MT6701_I2C_ADDR);
    Wire_A->write(ANGLE_REG_HIGH);
    Wire_A->endTransmission(false);
    Wire_A->requestFrom(MT6701_I2C_ADDR, 1);
    if (!Wire_A->available()) return -1;
    uint8_t highByte = Wire_A->read();
    // === 第二步：读取低6位（0x04寄存器） ===
    Wire_A->beginTransmission(MT6701_I2C_ADDR);
    Wire_A->write(ANGLE_REG_LOW);
    Wire_A->endTransmission(false);
    Wire_A->requestFrom(MT6701_I2C_ADDR, 1);
    if (!Wire_A->available()) return -1;
    uint8_t lowByte = Wire_A->read();
    uint16_t rawAngle = (highByte << 6) | (lowByte >> 2);

    float rad = ((float)rawAngle * 2.0f *(float)PI) / 16384.0f; // 使用16384作为满量程值
    if (rad < 0) {
        rad +=2 * PI;
    }
    return rad;
}



float Read_MT6701_I2C_B() {
    // === 第一步：读取高8位（0x03寄存器） ===
    Wire_B->beginTransmission(MT6701_I2C_ADDR);
    Wire_B->write(ANGLE_REG_HIGH);
    Wire_B->endTransmission(false);
    Wire_B->requestFrom(MT6701_I2C_ADDR, 1);
    if (!Wire_B->available()) return -1;
    uint8_t highByte = Wire_B->read();
    // === 第二步：读取低6位（0x04寄存器） ===
    Wire_B->beginTransmission(MT6701_I2C_ADDR);
    Wire_B->write(ANGLE_REG_LOW);
    Wire_B->endTransmission(false);
    Wire_B->requestFrom(MT6701_I2C_ADDR, 1);
    if (!Wire_B->available()) return -1;
    uint8_t lowByte = Wire_B->read();
    uint16_t rawAngle = (highByte << 6) | (lowByte >> 2);

    float rad = ((float)rawAngle * 2.0f *(float)PI) / 16384.0f; // 使用16384作为满量程值
    if (rad < 0) {
        rad +=2 * PI;
    }
    return rad;
}


