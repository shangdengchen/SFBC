//
// Created by MGJ on 2025/4/23.
//

#ifndef FOC_CODE_XBOX_CONTROLS_H
#define FOC_CODE_XBOX_CONTROLS_H

#include "include/precompiled.h"

extern XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
extern bool xbox_Handle_control_switch;



[[noreturn]] void Handle_control_tasks(void *pvParameters);
void Xbox_Handle_control_On();

#endif //FOC_CODE_XBOX_CONTROLS_H
