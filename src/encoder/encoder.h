
#ifndef SFBC_ENCODER_H
#define SFBC_ENCODER_H

#include "include/precompiled.h"


enum Encoder_Type {
    MT6701,
    AS5600,
    AS5047P,
    UNKNOWN_Encoder,
};

extern Encoder_Type Car_Encoder_A;
extern Encoder_Type Car_Encoder_B;


extern MagneticSensorI2C AS5600_I2C_A;
extern MagneticSensorI2C AS5600_I2C_B;

extern GenericSensor MT6701_I2C_A;
extern GenericSensor MT6701_I2C_B;

extern GenericSensor AS5047P_SPI_A;
extern GenericSensor AS5047P_SPI_B;

bool encoder_init();

bool Check_AS5600_A();
bool Check_AS5600_B();

bool Check_MT6701_A();
bool Check_MT6701_B();

bool Check_A5047P_A();
bool Check_A5047P_B();

void Init_MT6701_I2C_A();
void Init_MT6701_I2C_B();

float Read_MT6701_I2C_A();
float Read_MT6701_I2C_B();

void Init_AS5047P_SPI_A();
void Init_AS5047P_SPI_B();

float Read_AS5047P_SPI_A();
float Read_AS5047P_SPI_B();


#endif
