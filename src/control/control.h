#ifndef FOC_CODE_CONTROL_H
#define FOC_CODE_CONTROL_H

#include "include/precompiled.h"




#define MAX_C_SPEED  0.5f
#define MAX_C_STEER  0.5f
#define MAX_PROTECT_SPEED 125.0f




enum System_Status {
    //关闭输出
    Disable_Output,
    //打开输出
    Open_Output,
    //错误
    Error_Unknown,
    Error_Init,
    Error_Power,
    Error_Mpu,
    Error_Encoder,
};


typedef struct {
    //小车状态
    System_Status Status;
    //灵敏度
    float SpeedSensitivity;
    //灵敏度
    float SteerSensitivity;
    //当前速度
    float NowSpeed;
    //当前角度
    float NowPitch;
    //当前速度差
    float NowSpeed_Erorr;
    //机械角度
    float Angle_Deviation;
    //遥控速度
    float TargetSpeed;
    //遥控转弯
    float TargetSteer;
    //加速
    float BoostSpeed;
    //自动立
    bool AutoStandUp;
    //输出A
    float ControlOut_A;
    //输出B
    float ControlOut_B;
    //加速度保护
    bool Acc_Protect;
    //是否连接
    int Control_Connected;
} Car_t;

extern Car_t Car_Info;


extern SimpleKalmanFilter KalmanFilter_mpu;
// 低通滤波器
extern LowPassFilter lpf_speed;
extern LowPassFilter lpf_speed_error;
extern LowPassFilter lpf_gyro_x;
extern LowPassFilter lpf_control_steer;
extern LowPassFilter lpf_control_speed;


extern float K[4];

bool Car_Control_init();

void AbnormalSpinDetect();

void LandingDetect();

void Control_Loop();

bool Control_interface(float steer, float speed, float dead_zone, float Speed_Sensitivity, float Steer_Sensitivity);


#endif //FOC_CODE_CONTROL_H









//
//
//void task_control(void *pvParameters)
//{
//    while (1)
//    {
//        angle_pitch = mpu.getAngleY();
//        // Serial.printf("d:%f\n",angle_pitch);
//        acc_Z = mpu.getAccZ();
//        if (running_state == STATE_STOP)
//        {
//            if (abs(angle_pitch - angle_pitch_offset) < 0.5)
//            {
//                running_state = STATE_RUNNING;
//            }
//        }
//        else if (running_state == STATE_RUNNING)
//        {
//            speed_average = (speed_right + speed_left) / 2;
//            if (abs(angle_pitch - angle_pitch_offset) > 50 )
//            {
//                speed_I_sum=0;//清空积分系数
//                running_state = STATE_PICKUP;
//            }
//            else
//            {
//                speed_target = translate_speed;
//
//                speed_error = speed_target - speed_average;
//
//                speed_I_sum+=(speed_error*speed_I);
//                if(speed_I_sum>15){
//                    speed_I_sum = 15;
//                }
//                if(speed_I_sum<-15){
//                    speed_I_sum = -15;
//                }
//                angle_target_pitch = speed_P * speed_error + speed_I_sum;
//
//                motor_output_left = angle_P * ((angle_pitch - angle_pitch_offset) - angle_target_pitch) + angle_D * mpu.getGyroY()+rotate_speed; //+ angle_D * mpu.getGyroY()
//                motor_output_right = angle_P * ((angle_pitch - angle_pitch_offset) - angle_target_pitch) + angle_D * mpu.getGyroY()-rotate_speed;
//                angle_last_pitch = angle_pitch;
//            }
//        }
//        else if (running_state == STATE_PICKUP)
//        {
//            motor_output_left = 0;
//            motor_output_right = 0;
//            if (speed_right > 3.14 && speed_left < -3.14)
//            {
//                running_state = STATE_STOP;
//            }
//        }
//        vTaskDelay(1);
//    }
//}











//// ================== LQR参数配置区 ==================
//// 调试时主要调整这些参数
//#define WHEEL_RADIUS   0.0315f   // 轮子半径(m) 6.3cm/2
//#define MASS           0.45f     // 总质量(kg)
//#define G              9.81f
//#define DT             0.005f    // 控制周期(秒)
//
//// LQR增益矩阵（调试重点！）
////float K[4] = { -12.0, -3.0, -0.8, -0.2 }; // [角度误差, 角速度, 位置误差, 速度误差]
//
//float K[4] = { -12, 1.45, -0, 50.0 }; // 先关闭位置和速度环
//
//// 状态观测器参数
//float pos_lpf = 0.85;      // 位置低通滤波系数
//LowPassFilter angle_lpf  = LowPassFilter(0.02);
//
//// ================== 状态变量 ==================
//struct {
//    float angle;       // 车体倾角(rad)
//    float angle_vel;   // 角速度(rad/s)
//    float position;    // 位置(m)
//    float speed;       // 速度(m/s)
//} state;
//
//// ================== LQR控制器 ==================
//float lqr_control() {
//    // 目标状态（保持直立静止）
//    const float target_angle = 0.0f;
//    const float target_speed = 0.0f;
//
//    // 计算状态误差
//    float angle_error = target_angle - state.angle;
//    float speed_error = target_speed - state.speed;
//
//    // 位置误差通过积分速度得到（近似）
//    static float position_error = 0;
//    position_error = position_error * 0.95f + speed_error * DT;
//
//    // LQR控制量计算 u = -K*x
//    float control_output = -(K[0] * angle_error +
//                             K[1] * state.angle_vel +
//                             K[2] * position_error +
//                             K[3] * speed_error);
//
//    return control_output;
//}

//// ================== 主控制循环 ==================
//void Control_Loop() {
//    // 原有初始化检查
//    if (motor_A.motor_status != motor_ready || motor_B.motor_status != motor_ready) return;
//
//    //================ 传感器数据处理 ================
//    // 更新MPU数据
//    mpu6050.update();
//
//    // 角度处理（关键！）
//    Now_Pitch= (mpu6050.getAngleX()+ Offset_parameters) * DEG_TO_RAD; // 转换为弧度
//    state.angle = angle_lpf(Now_Pitch);
//    Now_Pitch=state.angle;
//
//    // 角速度处理
//    float raw_gyro = mpu6050.getGyroX() * DEG_TO_RAD;  // 转换为rad/s
//    state.angle_vel = state.angle_vel * 0.90 + raw_gyro * 0.10;
//
//    //================ 速度/位置估计 ================
//    // 计算电机转速（转换为m/s）
//    float wheel_rpm = (motor_A.shaft_velocity - motor_B.shaft_velocity) / 2.0f;
//    float instant_speed = wheel_rpm * (2.0f * PI * WHEEL_RADIUS) / 60.0f; // RPM转m/s
//
//    state.speed = lpf_speed(instant_speed);
//
//    // 更新位置估计
//    static float filtered_pos = 0;
//    filtered_pos = filtered_pos * pos_lpf + state.speed * DT * (1 - pos_lpf);
//    state.position = filtered_pos;
//    //================ LQR计算 ================
//    float base_torque = lqr_control();
//
//    //================ 转向控制 ================
//    // 保留原有转向PID（建议先关闭转向调试直立）
//    static float steer_torque = 0;
//    if (++speed_count >= 2) {
//        PID_Adjust(&TurnPID, (Target_Steer * -55), Now_Speed_Erorr);
//        steer_torque = TurnPID.PID_Out;
//        speed_count = 0;
//    }
//
//
//    //================ 电机输出 ================
//    // 扭矩分配
//    Target_torque_A = base_torque + steer_torque;
//    Target_torque_B = -base_torque + steer_torque; // 注意电机方向
//
//    // 扭矩限幅（重要！）
//    const float torque_limit = 2.0f; // 根据电机承受能力调整
//
//    Target_torque_A = constrain(Target_torque_A, -torque_limit, torque_limit);
//    Target_torque_B = constrain(Target_torque_B, -torque_limit, torque_limit);
//
//    motor_A.loopFOC();
//    motor_B.loopFOC();
//
//    // 电机执行
//    motor_A.move(Target_torque_A);
//    motor_B.move(Target_torque_B);
//
//    // 原有保护机制
//    AbnormalSpinDetect();
//    LandingDetect();
//}
//

