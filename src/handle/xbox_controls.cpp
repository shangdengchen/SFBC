#include "xbox_controls.h"


#include "control/control.h"
#include "foc/foc_drive.h"
#include "buzzer/BuzzerSound.h"
#include "power/BatteryAndButton.h"
#include "Init/init_System.h"

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

bool xbox_Handle_control_switch = false;

void Xbox_Handle_control_On() {
    if (xbox_Handle_control_switch) return;
#ifdef LED_1_GPIO
    B_LED.blink(50);
#endif
    xbox_Handle_control_switch = true;
}

[[noreturn]] void Handle_control_tasks(void *pvParameters) {
    Serial.println("[Xbox]:手柄控制已经开启");

    xboxController.begin();

    uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;

    while (true) {
        xboxController.onLoop();

        //延时2TICKS 防止独占cpu
        vTaskDelay(pdTICKS_TO_MS(2));

        if (xboxController.isConnected()) {

            if (!xboxController.isWaitingForFirstNotification()) {

                //方向控制接口
                Control_interface(
                        ((float) xboxController.xboxNotif.joyRHori / (float) joystickMax) - 0.5f,
                        ((float) xboxController.xboxNotif.joyLVert / (float) joystickMax) - 0.5f,
                        0.05,
                        40,
                        60
                );


                //进入关机
                if ((xboxController.xboxNotif.btnLB || xboxController.xboxNotif.btnRB)) {
                    balanceCarPowerOff();
                }


                //额外加速
                if (abs(((float) xboxController.xboxNotif.joyRHori / (float) joystickMax) - 0.5f) < 0.1 &&
                    (((float) xboxController.xboxNotif.joyLVert / (float) joystickMax) - 0.5f) < -0.03) {
                    Car_Info.BoostSpeed = ((float) xboxController.xboxNotif.trigLT / 100.0f) * 1;
                }

                //自动立
//                if (System_Status == Disable_Output&&AutoStandUp== false) {
//                    if (xboxController.xboxNotif.btnA) {
//                        Car_Info.AutoStandUp = true;
//                    }
//                }


                //进入保护模式
                if ((xboxController.xboxNotif.btnX ||
                     xboxController.xboxNotif.btnB ||
                     xboxController.xboxNotif.btnA ||
                     xboxController.xboxNotif.btnY) &&
                    Car_Info.Status == Open_Output) {
                    MotorClose();
                    buzzer.play(S_DISCONNECTION);
                }

                //退出保护模式
                if ((xboxController.xboxNotif.btnDirUp ||
                     xboxController.xboxNotif.btnDirLeft ||
                     xboxController.xboxNotif.btnDirRight ||
                     xboxController.xboxNotif.btnDirDown
                    ) &&
                    Car_Info.Status == Disable_Output) {
                    MotorOpen();
                    buzzer.play(S_JUMP);
                }


            }

            Car_Info.Acc_Protect = false;
            //连接了外部控制
            Car_Info.Control_Connected = 1;


#ifdef LED_1_GPIO
            B_LED.on();
#endif
        } else {

            //没有连接手柄
            Car_Info.Control_Connected = 0;
            Car_Info.Acc_Protect = true;


#ifdef LED_1_GPIO
            if (B_LED._mode != LED::Mode::Blink) {
                B_LED.blink(50);
            }
#endif
        }
    }
}