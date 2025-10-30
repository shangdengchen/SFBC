//
// Created by MGJ on 2025/5/23.
//

#include "encoder.h"


#include "I2C/I2C_Manage.h"


Encoder_Type Car_Encoder_A;
Encoder_Type Car_Encoder_B;

bool encoder_init_A()
{
    Car_Encoder_A=UNKNOWN_Encoder;

    if (Check_AS5600_A()) Car_Encoder_A=AS5600;
    if (Check_MT6701_A()) Car_Encoder_A=MT6701;
#ifdef CS_1_GPIO
    if (Check_A5047P_A()) Car_Encoder_A=AS5047P;
#endif
    if (Car_Encoder_A==UNKNOWN_Encoder)
    {
        Serial.println("[编码器A]: 未连接编码器");
        return false;
    }
    return true;
}


bool encoder_init_B()
{
    Car_Encoder_B=UNKNOWN_Encoder;

    if (Check_AS5600_B()) Car_Encoder_B=AS5600;
    if (Check_MT6701_B()) Car_Encoder_B=MT6701;
#ifdef CS_1_GPIO
    if (Check_A5047P_B()) Car_Encoder_B=AS5047P;
#endif
    if (Car_Encoder_B==UNKNOWN_Encoder)
    {
        Serial.println("[编码器B]: 未连接编码器");
        return false;
    }
    return true;
}


bool encoder_init()
{
    bool init= true;
    init*=encoder_init_A();
    init*=encoder_init_B();
    return init;
}

