//
// Created by MGJ on 2025/4/23.
//

#ifndef FOC_CODE_FOC_DRIVE_H
#define FOC_CODE_FOC_DRIVE_H

#include "include/precompiled.h"

#include "mpu/MPU6050.h"

#define OPEN_MOTOR_FIRST    0

#define PID_CS_P_A          0.4f
#define PID_CS_I_A          0.2f
#define PID_CS_D_A          0.0005f
#define   VEL_Kp_A          0.1f
#define   VEL_Ki_A          0.0001f
#define   VEL_Kd_A          0.00f

#define PID_CS_P_B          PID_CS_P_A
#define PID_CS_I_B          PID_CS_I_A
#define PID_CS_D_B          PID_CS_D_A
#define   VEL_Kp_B            VEL_Kp_A
#define   VEL_Ki_B            VEL_Ki_A
#define   VEL_Kd_B            VEL_Kd_A


#define LPF_velocity_Tf     0.150f
#define LPF_current_d_Tf    0.1f
#define LPF_current_q_Tf    0.1f
#define LPF_angle_Tf        0.05f
#define MAX_SPEED           130.0f


// MPU6050
extern MPU6050 mpu6050;

// 无刷电机
extern BLDCMotor motor_A;
extern BLDCMotor motor_B;

// 电机驱动器
extern BLDCDriver3PWM driver_A;
extern BLDCDriver3PWM driver_B;

// 电流检测
extern LowsideCurrentSense cs_A;
extern LowsideCurrentSense cs_B;


void MotorOpen();

void MotorClose();

void Foc_Parameters_init();

void onTarget(char *cmd);


void Foc_A_Initialize(void *pvParameters);

void Foc_B_Initialize(void *pvParameters);


#endif //FOC_CODE_FOC_DRIVE_H
