#include "foc_drive.h"


#include "task/freertos_task.h"
#include "control/control.h"
#include "nvs/nvs.h"
#include "power/BatteryAndButton.h"
#include "Init/init_System.h"
#include "I2C/I2C_Manage.h"
#include "mpu/MPU6050.h"
#include "pid/pid_adjust.h"
#include "encoder/encoder.h"

BLDCMotor motor_A = BLDCMotor(7); // 电机A
BLDCMotor motor_B = BLDCMotor(7); // 电机B


#if   MPU_I2C == 1
MPU6050 mpu6050(I2C_A);
#elif MPU_I2C == 2
MPU6050 mpu6050(I2C_B);
#endif


BLDCDriver3PWM driver_A = BLDCDriver3PWM(PWM_A_1_GPIO, PWM_A_2_GPIO, PWM_A_3_GPIO); // 电机A的驱动器
BLDCDriver3PWM driver_B = BLDCDriver3PWM(PWM_B_1_GPIO, PWM_B_2_GPIO, PWM_B_3_GPIO); // 电机B的驱动器


// 用于检测电机电流，参数分别为：电阻值、增益、电流检测引脚、电压检测引脚、备用引脚
LowsideCurrentSense cs_A = LowsideCurrentSense(0.005f, 50.0f, ADC_A_1_GPIO, ADC_A_2_GPIO, _NC); // 电机A的电流检测
LowsideCurrentSense cs_B = LowsideCurrentSense(0.005f, 50.0f, ADC_B_1_GPIO, ADC_B_2_GPIO, _NC); // 电机B的电流检测


void MotorOpen() {
    //清除位置记录
    SpeedPID.Ki_Out = 0.0f;
#ifdef LED_1_GPIO
    G_LED.doubleBlink(100, 900);
    R_LED.off();
#endif
    motor_A.enable();
    motor_B.enable();
    Car_Info.ControlOut_A = 0;
    Car_Info.ControlOut_B = 0;
    Car_Info.Status = Open_Output;
}


void MotorClose() {
#ifdef LED_1_GPIO
    G_LED.off();
    R_LED.blink();
#endif
    motor_A.disable();
    motor_B.disable();
    Car_Info.ControlOut_A = 0;
    Car_Info.ControlOut_B = 0;
    Car_Info.Status = Disable_Output;

}

Commander command = Commander(Serial);

void doTarget(char *cmd) { command.scalar(&motor_A.target, cmd); }    // 串口控制指令：目标值

void Foc_Parameters_init() {
    //==============================================================================
    motor_A.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_A.torque_controller = TorqueControlType::foc_current;
    motor_A.controller = MotionControlType::velocity;

    //速度PID
    motor_A.PID_velocity.P = VEL_Kp_A;
    motor_A.PID_velocity.I = VEL_Ki_A;
    motor_A.PID_velocity.D = VEL_Kd_A;
    motor_A.PID_velocity.output_ramp = 1000.0f;

    //电流d PID
    motor_A.PID_current_d.P = PID_CS_P_A;   // 降低P减少震荡
    motor_A.PID_current_d.I = PID_CS_I_A;   // 增加I加速误差消除
    motor_A.PID_current_d.D = PID_CS_D_A;   // 暂时禁用D项
    motor_A.PID_current_d.output_ramp = 1000.0f;

    //电流q PID
    motor_A.PID_current_q.P = PID_CS_P_A;
    motor_A.PID_current_q.I = PID_CS_I_A;
    motor_A.PID_current_q.D = PID_CS_D_A;
    motor_A.PID_current_q.output_ramp = 1000.0f;

    //滤波
    motor_A.LPF_velocity.Tf = LPF_velocity_Tf;  //速度滤波系数
    motor_A.LPF_current_d.Tf = LPF_current_d_Tf; //current_d滤波系数
    motor_A.LPF_current_q.Tf = LPF_current_q_Tf; //current_d滤波系数
    motor_A.LPF_angle.Tf = LPF_angle_Tf;     //角度滤波系数


    //限制
    motor_A.voltage_limit = 0.6f;
    motor_A.current_limit = 6.5f;
    motor_A.velocity_limit = MAX_SPEED;
    motor_A.PID_current_d.limit = 3.0f;
    motor_A.PID_current_q.limit = 3.0f;

    //电机参数
    motor_A.KV_rating = 700.0f;
    motor_A.phase_resistance = 0.14f;
    motor_A.phase_inductance = 25e-6;  // 25μH → 25e-6 Henry
    driver_A.voltage_power_supply = 8.0f;
    motor_A.velocity_index_search = 2.0f;

    cs_A.gain_a *= -1;
    cs_A.gain_b *= -1;
    cs_A.gain_c *= -1;


    //==============================================================================
    motor_B.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_B.torque_controller = TorqueControlType::foc_current;
    motor_B.controller = MotionControlType::velocity;

    //速度PID
    motor_B.PID_velocity.P = VEL_Kp_B;
    motor_B.PID_velocity.I = VEL_Ki_B;
    motor_B.PID_velocity.D = VEL_Kd_B;
    motor_B.PID_velocity.output_ramp = 1000.0f;

    //电流d PID
    motor_B.PID_current_d.P = PID_CS_P_B;   // 降低P减少震荡
    motor_B.PID_current_d.I = PID_CS_I_B;   // 增加I加速误差消除
    motor_B.PID_current_d.D = PID_CS_D_B;   // 暂时禁用D项
    motor_B.PID_current_d.output_ramp = 1000.0f;

    //电流q PID
    motor_B.PID_current_q.P = PID_CS_P_B;
    motor_B.PID_current_q.I = PID_CS_I_B;
    motor_B.PID_current_q.D = PID_CS_D_B;
    motor_B.PID_current_q.output_ramp = 1000.0f;

    //滤波
    motor_B.LPF_velocity.Tf = LPF_velocity_Tf;  //速度滤波系数
    motor_B.LPF_current_d.Tf = LPF_current_d_Tf; //current_d滤波系数
    motor_B.LPF_current_q.Tf = LPF_current_q_Tf; //current_d滤波系数
    motor_B.LPF_angle.Tf = LPF_angle_Tf;     //角度滤波系数


    //限制
    motor_B.voltage_limit = 0.6f;
    motor_B.current_limit = 6.5f;
    motor_B.velocity_limit = MAX_SPEED;
    motor_B.PID_current_d.limit = 3.0f;
    motor_B.PID_current_q.limit = 3.0f;

    //电机参数
    motor_B.KV_rating = 700.0f;
    motor_B.phase_resistance = 0.14f;
    motor_B.phase_inductance = 25e-6;  // 25μH → 25e-6 Henry
    driver_B.voltage_power_supply = 8.0f;
    motor_B.velocity_index_search = 2.0f;

    cs_B.gain_a *= -1;
    cs_B.gain_b *= -1;
    cs_B.gain_c *= -1;
    command.add('T', doTarget, "target angle");
}


void Foc_A_Initialize(void *pvParameters) {

#if MORE_FOC_INIT_INFO
    motor_A.useMonitoring(Serial);
#endif
    if (!driver_A.init()) {
        Serial.println("[驱动器A]:初始化失败");
    } else {
        Serial.println("[驱动器A]:初始化成功");
    }

    switch (Car_Encoder_A) {
        case AS5600: {
            Serial.println("[编码器A]:AS5600");
            AS5600_I2C_A.init(&I2C_A);
            motor_A.linkSensor(&AS5600_I2C_A);
            break;
        }
        case MT6701: {
            Serial.println("[编码器A]:MT6701");
            MT6701_I2C_A.init();
            motor_A.linkSensor(&MT6701_I2C_A);
            break;
        }
#ifdef CS_1_GPIO
        case AS5047P: {
            Serial.println("[编码器A]:AS5047P");
            AS5047P_SPI_A.init();
            motor_A.linkSensor(&AS5047P_SPI_A);
            break;
        }
#endif
        case UNKNOWN_Encoder:
            break;
    }


    motor_A.linkDriver(&driver_A);

    cs_A.linkDriver(&driver_A);
    Serial.println("[电流传感器A]:正在校准电流采样...");
    motor_A.init();
    if (!cs_A.init()) {
        Serial.println("[电流传感器A]:初始化失败");
    } else {
        Serial.println("[电流传感器A]:初始化成功");
    }
    motor_A.linkCurrentSense(&cs_A);
    Serial.println("[电机A]:正在对齐编码器,驱动相位...");
    if (!motor_A.initFOC()) {
        Serial.println("[电机A]:初始化失败");
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
        portENTER_CRITICAL(&mux);
        systemInitialized = false; // 将系统初始化标志设置为false
        portEXIT_CRITICAL(&mux);
    } else {
        Serial.println("[电机A]:初始化成功");
    }
    vTaskDelete(Task1);
}

void Foc_B_Initialize(void *pvParameters) {
#if MORE_FOC_INIT_INFO
    motor_B.useMonitoring(Serial);
#endif
    if (!driver_B.init()) {
        Serial.println("[驱动器B]:初始化失败");
    } else {
        Serial.println("[驱动器B]:初始化成功");
    }
    switch (Car_Encoder_B) {
        case AS5600: {
            Serial.println("[编码器A]:AS5600");
            AS5600_I2C_B.init(&I2C_B);
            motor_B.linkSensor(&AS5600_I2C_B);
            break;
        }

        case MT6701: {
            Serial.println("[编码器B]:MT6701");
            MT6701_I2C_B.init();
            motor_B.linkSensor(&MT6701_I2C_B);
            break;
        }
#ifdef CS_1_GPIO
        case AS5047P: {
            Serial.println("[编码器B]:AS5047P");
            AS5047P_SPI_B.init();
            motor_B.linkSensor(&AS5047P_SPI_B);
            break;
        }
#endif
        case UNKNOWN_Encoder:
            break;
    }
    motor_B.linkDriver(&driver_B);
    cs_B.linkDriver(&driver_B);
    Serial.println("[电流传感器B]:正在校准电流采样...");
    motor_B.init();
    if (!cs_B.init()) {
        Serial.println("[电流传感器B]:初始化失败");
    } else {
        Serial.println("[电流传感器B]:初始化成功");
    }
    motor_B.linkCurrentSense(&cs_B);
    Serial.println("[电机B]:正在对齐编码器,驱动相位...");
    if (!motor_B.initFOC()) {
        Serial.println("[电机B]:初始化失败");
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
        portENTER_CRITICAL(&mux);
        systemInitialized = false; // 将系统初始化标志设置为false
        portEXIT_CRITICAL(&mux);
    } else {
        Serial.println("[电机B]:初始化成功");
    }
    vTaskDelete(Task2);
}
