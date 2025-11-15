#include "control.h"

#include "foc/foc_drive.h"
#include "handle/xbox_controls.h"
#include "pid/pid_adjust.h"


float K[4] = {-12, 1.45, -0, 50.0};


//卡尔曼滤波        e_mea: 测量不确定性   e_est: 估计不确定性 q: 过程噪声
SimpleKalmanFilter KalmanFilter_mpu(1.0, 0.3, 1.0);
//滤波
LowPassFilter lpf_speed = LowPassFilter(0.02f);
LowPassFilter lpf_speed_error = LowPassFilter(0.02f);
LowPassFilter lpf_gyro_x = LowPassFilter(0.020f);
LowPassFilter lpf_control_steer = LowPassFilter(0.02f);
LowPassFilter lpf_control_speed = LowPassFilter(0.03f);


Car_t Car_Info;


unsigned long StandUpTimerStart = 0;
bool StandUpTimerActive = false;

unsigned long kiTimerStart = 0;
bool kiTimerActive = false;


bool Car_Control_init() {
    Car_Info.Status = Open_Output;
    //灵敏度
    Car_Info.SpeedSensitivity = 40;
    Car_Info.SteerSensitivity = 40;
    //当前实际参数
    Car_Info.NowSpeed = 0;
    Car_Info.NowPitch = 0;
    Car_Info.NowSpeed_Erorr = 0;
    Car_Info.Angle_Deviation = Mechanical_Zero_Point;
    //目标速度
    Car_Info.TargetSpeed = 0;
    Car_Info.TargetSteer = 0;
    Car_Info.BoostSpeed = 0;
    Car_Info.AutoStandUp = false;
    //控制输出
    Car_Info.ControlOut_A = 0;
    Car_Info.ControlOut_B = 0;
    //加速度提取保护开关
    Car_Info.Acc_Protect = true;
    //连接状态
    Car_Info.Control_Connected = 0;
    return true;
}


bool RunAutoStandUp() {

    //不满足
    if (!Car_Info.AutoStandUp) {
        return false;
    }

    //到达45角就结束
    if (abs(Car_Info.NowPitch) <= 45.0f) {
        Car_Info.AutoStandUp = false;
        return false;
    }

    //执行立起来的任务
    if (Car_Info.NowPitch > 0.0f && Car_Info.NowPitch < 175.0f) {
        Car_Info.ControlOut_A = -50.0f;
        Car_Info.ControlOut_B = -50.0f;
    } else if (Car_Info.NowPitch < 0.0f && Car_Info.NowPitch > -175.0f) {
        Car_Info.ControlOut_A = 50.0f;
        Car_Info.ControlOut_B = 50.0f;
    }
        // 如果小车是倒立水平放置，则加大电机速度提高反扭力保证小车能顺利起立
    else {
        Car_Info.ControlOut_A = 100.0f;
        Car_Info.ControlOut_B = 100.0f;
    }


    //第一次进入
    if (!StandUpTimerActive) {
        MotorOpen();
        //进入计时器
        StandUpTimerStart = millis();
        StandUpTimerActive = true;
    } else {
        if (millis() - StandUpTimerStart >= 3000) {
            // 时间限制
            Car_Info.AutoStandUp = false;
        }
    }

    return true;
}

void AbnormalSpinDetect() {
    static unsigned short count = 0;

    if (abs(Car_Info.NowSpeed) > MAX_PROTECT_SPEED && Car_Info.Status == Open_Output) {
        Serial.println("[系统-动作]:速度过大,关闭输出");
        MotorClose();
    }


    if (abs(Car_Info.NowPitch) > 65 && Car_Info.Status == Open_Output) {
        Serial.println("[系统-动作]:小车倾倒,关闭输出");
        MotorClose();
    }

    if (Car_Info.Acc_Protect) {
        if (mpu6050.absAccZ > 2.0f && Car_Info.Status == Open_Output &&
            abs(Car_Info.NowSpeed) < 8 && abs(Car_Info.NowPitch) < 5) {
            Serial.println("[系统-动作]:小车提起(加速度),关闭输出");
            MotorClose();
        }
    }

    // 左右电机转速大于30、方向相同、持续时间超过250ms，且车身角度不超过30度，则判断为悬空空转
    if (Car_Info.Status == Open_Output) {
        if ((abs(Car_Info.NowSpeed) > 70 && abs(Car_Info.NowPitch) < 10)) {
            if (++count > 20) {
                count = 0;
                MotorClose();
                Serial.println("[系统-动作]:小车提起(速度),关闭输出");
            }
        } else {
            count = 0;
        }
    } else {
        count = 0;
    }

}

void LandingDetect() {
    static float lastCarAngle = 0;
    static unsigned short count = 0, count1 = 0;
    if (Car_Info.Status == Disable_Output) {
        // 小车角度5°~-5°启动检测
        if (abs(Car_Info.NowPitch) <= 5) {
            count1++;
            if (count1 >= 70) {//每隔250ms判断一次小车角度变化量，变化量小于0.8°或大于-0.8°判断为小车静止
                count1 = 0;
                if (abs(Car_Info.NowPitch - lastCarAngle) < 0.8) {
                    count++;
                    if (count >= 4) {
                        count = 0;
                        count1 = 0;
                        MotorOpen();
                        Serial.println("[系统-动作]:小车着地,恢复动力");
                    }
                } else {
                    count = 0;
                }
                lastCarAngle = Car_Info.NowPitch;
            }
        } else {
            count1 = 0;
            count = 0;
        }
    }
}


//控制接口范围是-0.5~0.5  dead_zone是死区,可以默认0.1
bool Control_interface(float steer, float speed, float dead_zone, float Speed_Sensitivity, float Steer_Sensitivity) {
    //灵敏度
    Car_Info.SpeedSensitivity = Speed_Sensitivity;
    Car_Info.SteerSensitivity = Steer_Sensitivity;

    // 限制速度范围-0.5~0.5
    if (speed < -MAX_C_SPEED) {
        speed = -MAX_C_SPEED;
    }
    if (speed > MAX_C_SPEED) {
        speed = MAX_C_SPEED;
    }

    // 限制转向范围-0.5~0.5
    if (steer < -MAX_C_STEER) {
        steer = -MAX_C_STEER;
    }
    if (steer > MAX_C_STEER) {
        steer = MAX_C_STEER;
    }

    //滤波
    Car_Info.TargetSteer = lpf_control_steer(steer); // 进行滤波处理
    Car_Info.TargetSpeed = lpf_control_speed(speed); // 进行滤波处理

    //添加死区
    if (fabs(Car_Info.TargetSpeed) <= dead_zone) {
        Car_Info.TargetSpeed = 0.0; // 如果在死区内，设置为零
    }
    if (fabs(Car_Info.TargetSteer) <= dead_zone) {
        Car_Info.TargetSteer = 0.0; // 如果在死区内，设置为零
    }

    return true;
}


//速度环计数
int speed_count = 5;



void Control_Loop() {
    //如果foc没有初始化成功，不控制
    if (motor_A.motor_status != motor_ready || motor_B.motor_status != motor_ready) {
        return;
    }

    //===============================FOC控制==============================
    motor_A.loopFOC();
    motor_B.loopFOC();

    motor_A.move(Car_Info.ControlOut_A);
    motor_B.move(Car_Info.ControlOut_B * (-1));
    //===============================传感器==============================

    // 读取并更新MPU6050传感器数据
    mpu6050.update();

    //当前俯仰角
    Car_Info.NowPitch = mpu6050.getAngleX();

    //卡尔曼滤波
    Car_Info.NowPitch = KalmanFilter_mpu.updateEstimate(Car_Info.NowPitch);

    //小车当前速度
    Car_Info.NowSpeed = lpf_speed((motor_A.shaft_velocity + motor_B.shaft_velocity * (-1)) / 2);

//    //自动起来
//    if (RunAutoStandUp()) return;


    //===============================PID==============================
    //PID速度环
    if (++speed_count > 3) {

        speed_count = 0;

        // 小车速度环
        PID_Adjust(
                &SpeedPID,
                0.0f,
                (Car_Info.NowSpeed + Car_Info.TargetSpeed * Car_Info.SpeedSensitivity)
        );


        // 小车旋转环
        PID_Adjust_T(&TurnPID,
                     (Car_Info.TargetSteer * Car_Info.SteerSensitivity) * (-1),
                     0.0f,
                     mpu6050.getGyroZ()
        );


        if (Car_Info.Acc_Protect) {
            //计算绝对方向的加速度
            mpu6050.calculateAbsoluteAcceleration();
        }

    }


    //旋转积分处理
    if (abs(Car_Info.TargetSteer) >= 0.02) {
        TurnPID.Ki_Out = 0;
    }

    //速度积分处理
    if (abs(Car_Info.TargetSpeed) >= 0.02 || Car_Info.AutoStandUp) {
        //遥控器正在遥控,解开小车位置
        SpeedPID.Ki = 0;
        SpeedPID.Ki_Out = 0;
        kiTimerActive = false;
    } else if (Car_Info.Control_Connected) {
        //遥控器没有遥控，并且连接
        //第一次进入
        if (!kiTimerActive) {
            //进入计时器
            kiTimerStart = millis();
            kiTimerActive = true;
        } else {
            // 不是第一次进入就持续检查时间差,这个时间是在等待小车稳定,确保速度环已经把速度降下来
            if (millis() - kiTimerStart >= 2500) {
                // 激活积分,固定小车位置
                SpeedPID.Ki = SpeedPID.tempKi;
            }
        }
    }


    //小车直立环
    PID_Adjust_T(
            &UprightPID,
            0.0f,
            (Car_Info.NowPitch + Car_Info.Angle_Deviation + SpeedPID.PID_Out + Car_Info.BoostSpeed),
            lpf_gyro_x(mpu6050.getGyroXFV())
    );


    // 输出电机速度控制量
    // if (TurnPID.PID_Out > 0) {
        Car_Info.ControlOut_A = UprightPID.PID_Out + TurnPID.PID_Out;
        Car_Info.ControlOut_B = UprightPID.PID_Out - TurnPID.PID_Out;
    // } else {
    //     Car_Info.ControlOut_A = UprightPID.PID_Out + TurnPID.PID_Out;
    //     Car_Info.ControlOut_B = UprightPID.PID_Out - TurnPID.PID_Out;
    // }


    //============================保护=================================

    //小车离地检查
    AbnormalSpinDetect();

    //小车着地检查
    LandingDetect();


}


