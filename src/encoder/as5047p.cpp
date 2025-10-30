//
// Created by MGJ on 2025/5/23.
//
#include "encoder.h"
#include <SPI.h>

#define ANGLECOM_ADDR 0x3FFF  // 含动态补偿的角度值
#define ERRFL_ADDR    0x0001  // 错误标志寄存器
#define DIAAGC_ADDR   0x3FFC  // 诊断和AGC寄存器
#define NOP_ADDR       0x0000

#ifdef CS_1_GPIO

const uint32_t SPI_FREQ = 1000000;
const uint8_t SPI_MODE = SPI_MODE1;

SPIClass HSPI_AB(HSPI);


GenericSensor AS5047P_SPI_A = GenericSensor(Read_AS5047P_SPI_A, Init_AS5047P_SPI_A);
GenericSensor AS5047P_SPI_B = GenericSensor(Read_AS5047P_SPI_B, Init_AS5047P_SPI_B);


bool calcEvenParity(uint16_t data) {
    uint8_t count = 0;
    for (int i = 0; i < 15; i++) {
        if (data & (1 << i)) count++;
    }
    return (count % 2); // 返回1表示需要补1使总位数为偶
}

uint16_t readRegister(uint16_t regAddr, uint8_t cs_select) {
    // 步骤1: 构造读命令帧（16-bit）
    uint16_t cmd = (regAddr & 0x3FFF) | (1 << 14); // R/W=1（读）
    cmd |= (calcEvenParity(cmd) << 15);            // 设置PARC校验位

    // 步骤2: 发送读命令（CSn拉低→发送→拉高）
    digitalWrite(cs_select, LOW);
    delayMicroseconds(1); // 满足tL时间（>350ns）
    HSPI_AB.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE));
    HSPI_AB.transfer16(cmd); // 发送命令，不关心返回值
    HSPI_AB.endTransaction();
    digitalWrite(cs_select, HIGH);
    delayMicroseconds(1); // 满足tCSn时间（>350ns）

    // 步骤3: 发送NOP命令获取数据
    digitalWrite(cs_select, LOW);
    delayMicroseconds(1);
    HSPI_AB.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE));
    uint16_t response = HSPI_AB.transfer16(0x0000); // NOP命令
    HSPI_AB.endTransaction();
    digitalWrite(cs_select, HIGH);

    // 步骤4: 解析响应数据
    bool efError = (response >> 14) & 0x1;     // EF位（bit14）
    bool pard = (response >> 15) & 0x1;        // PARD校验位
    uint16_t data = response & 0x3FFF;         // 有效数据（bit13-0）
    bool parityValid = (calcEvenParity(response & 0x7FFF) == pard);

    return (efError || !parityValid) ? 0xFFFF : data;
}

bool Check_A5047P_A() {
    Init_AS5047P_SPI_A();
    if (Read_AS5047P_SPI_A() == 0.0f) {
        Serial.println("[A5047P_A]: 未发现");
        return false;
    }

    Serial.println("[A5047P_A]: 连接成功!");
    return true; // 所有检测通过

}


bool Check_A5047P_B() {
    Init_AS5047P_SPI_B();

    if (Read_AS5047P_SPI_B() == 0.0f) {
        Serial.println("[A5047P_B]: 未发现");
        return false;
    }

    Serial.println("[A5047P_B]: 连接成功!");
    return true; // 所有检测通过
}


void Init_AS5047P_SPI_A() {
    pinMode(CS_1_GPIO, OUTPUT);
    digitalWrite(CS_1_GPIO, HIGH);
    HSPI_AB.begin(CLK_GPIO, MISO_GPIO, MOSI_GPIO, CS_1_GPIO);
}

void Init_AS5047P_SPI_B() {
    pinMode(CS_2_GPIO, OUTPUT);
    digitalWrite(CS_2_GPIO, HIGH);
    HSPI_AB.begin(CLK_GPIO, MISO_GPIO, MOSI_GPIO, CS_2_GPIO);
}


float Read_AS5047P_SPI_A() {
    uint16_t angle = readRegister(ANGLECOM_ADDR, CS_1_GPIO); // 开启调试
    if (angle != 0xFFFF) {
        float rad = ((float) angle * 2.0f * (float) PI) / 16384.0f; // 使用16384作为满量程值
        if (rad < 0) {
            rad += 2 * PI;
        }
        return rad;
    }
    return -1;
}


float Read_AS5047P_SPI_B() {
    uint16_t angle = readRegister(ANGLECOM_ADDR, CS_2_GPIO); // 开启调试
    if (angle != 0xFFFF) {
        float rad = ((float) angle * 2.0f * (float) PI) / 16384.0f; // 使用16384作为满量程值
        if (rad < 0) {
            rad += 2 * PI;
        }
        return rad;
    }
    return -1;
}
#endif
