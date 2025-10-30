//
// Created by MGJ on 2025/5/1.
//

#include "web.h"
#include "control/control.h"
#include "handle/xbox_controls.h"
#include "pid/pid_adjust.h"
#include "foc/foc_drive.h"
#include "power/BatteryAndButton.h"

#include <ESPAsyncWebServer.h>

#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <DNSServer.h>


// AP配置
const char *ssid = "MyCar";
const char *password = "";

DNSServer dnsServer;                       //创建dnsServer实例
const int DNS_PORT = 53;                   //设置DNS端口号


AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

IPAddress apIP(192, 168, 1, 1);            //设置AP的IP地址

void initDNS() {
    if (dnsServer.start(DNS_PORT, "*", apIP)) { //判断将所有地址映射到esp32的ip上是否成功
        Serial.println("[DNS]:start dnsserver success.");
    } else {
        Serial.println("[DNS]:start dnsserver failed.");
    }
}


void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("[WEB]:WebSocket客户端 #%u 连接成功\n", client->id());
            Car_Info.Acc_Protect = false;
            Car_Info.Control_Connected=1;
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("[WEB]:WebSocket客户端 #%u 断开连接\n", client->id());
            Car_Info.Acc_Protect = true;
            //连接了外部控制
            Car_Info.Control_Connected=0;
            break;
        case WS_EVT_DATA: {
            AwsFrameInfo *info = (AwsFrameInfo *) arg;
            if (!info || !client) return; // 空指针检查

            if (info->final && info->index == 0 && info->len == len) {
                if (info->opcode == WS_TEXT) {
                    // 关键修改点1：使用安全解析方式
                    JsonDocument doc;
                    DeserializationError error = deserializeJson(doc, data, len); // 增加长度参数

                    if (error) {
                        Serial.print("[WEB]:JSON解析失败: ");
                        Serial.println(error.c_str());
                        return;
                    }

                    // 处理摇杆数据
                    if (doc["x"].is<float>() && doc["y"].is<float>()) {
                        float x = doc["x"];
                        float y = doc["y"];
                        Control_interface(
                                x / 2,
                                y / 2,
                                0.055,
                                30,
                                50)
                                ;
                    }

                        // 处理ping请求
                    else if (doc["type"] == "ping") {
                        JsonDocument resDoc;
                        resDoc["type"] = "pong";
                        resDoc["clientTime"] = doc["clientTime"];
                        if (client->status() == WS_CONNECTED) {
                            String response;
                            serializeJson(resDoc, response);
                            client->text(response);
                        }
                    }

                        // 统一处理PID消息类型
                    else if (doc["type"] == "pid") { // 确保类型匹配
                        Serial.println("[WEB]:收到PID参数更新");

                        // Speed PID
                        if (doc["speed"].is<JsonObject>()) {
                            JsonObject speed = doc["speed"];
                            SpeedPID.Kp = speed["Kp"] | SpeedPID.Kp;
                            SpeedPID.Ki = speed["Ki"] | SpeedPID.Ki;
                            SpeedPID.tempKi= speed["Ki"] | SpeedPID.tempKi;
                            SpeedPID.Kd = speed["Kd"] | SpeedPID.Kd;
                        }

                        // Upright PID
                        if (doc["upright"].is<JsonObject>()) {
                            JsonObject upright = doc["upright"];
                            UprightPID.Kp = upright["Kp"] | UprightPID.Kp;
                            UprightPID.Ki = upright["Ki"] | UprightPID.Ki;
                            UprightPID.Kd = upright["Kd"] | UprightPID.Kd;
                        }

                        // Turn PID
                        if (doc["turn"].is<JsonObject>()) {
                            JsonObject turn = doc["turn"];
                            TurnPID.Kp = turn["Kp"] | TurnPID.Kp;
                            TurnPID.Ki = turn["Ki"] | TurnPID.Ki;
                            TurnPID.Kd = turn["Kd"] | TurnPID.Kd;
                        }

                        if (doc["mpu"].is<JsonObject>()) {
                            JsonObject mpu = doc["mpu"];
                            Car_Info.Angle_Deviation = mpu["offset"] | Car_Info.Angle_Deviation;
                        }
                        // 处理电机PID
                        if (doc["motor_pid"].is<JsonObject>()) {
                            JsonObject motor_pid = doc["motor_pid"];
                            motor_A.PID_current_d.P = motor_pid["CS_P"] | motor_A.PID_current_d.P;
                            motor_A.PID_current_d.I = motor_pid["CS_I"] | motor_A.PID_current_d.I;

                            motor_B.PID_current_d.P = motor_pid["CS_P"] | motor_B.PID_current_d.P;
                            motor_B.PID_current_d.I = motor_pid["CS_I"] | motor_B.PID_current_d.I;

                        }

                        // 处理滤波器参数
                        if (doc["lpf"].is<JsonObject>()) {
                            JsonObject lpf = doc["lpf"];
                            motor_A.LPF_velocity.Tf = lpf["velocity_Tf"] | motor_A.LPF_velocity.Tf;
                            motor_A.LPF_current_d.Tf = lpf["current_d_Tf"] | motor_A.LPF_current_d.Tf;
                            motor_A.LPF_current_q.Tf = lpf["current_q_Tf"] | motor_A.LPF_current_q.Tf;
                            motor_A.LPF_angle.Tf = lpf["angle_Tf"] | motor_A.LPF_angle.Tf;

                            motor_B.LPF_velocity.Tf = lpf["velocity_Tf"] | motor_B.LPF_velocity.Tf;
                            motor_B.LPF_current_d.Tf = lpf["current_d_Tf"] | motor_B.LPF_current_d.Tf;
                            motor_B.LPF_current_q.Tf = lpf["current_q_Tf"] | motor_B.LPF_current_q.Tf;
                            motor_B.LPF_angle.Tf = lpf["angle_Tf"] | motor_B.LPF_angle.Tf;


                            lpf_speed.Tf = lpf["speed"] | lpf_speed.Tf;
                            lpf_speed_error.Tf = lpf["speed_error"] | lpf_speed_error.Tf;
                            lpf_gyro_x.Tf = lpf["gyro_x"] | lpf_gyro_x.Tf;
                            lpf_control_steer.Tf = lpf["control_steer"] | lpf_control_steer.Tf;
                            lpf_control_speed.Tf = lpf["control_speed"] | lpf_control_speed.Tf;


                        }

                        // 处理卡尔曼滤波器
                        if (doc["kalman_filter"].is<JsonObject>()) {
                            JsonObject kf = doc["kalman_filter"];
                            KalmanFilter_mpu._err_measure = kf["mea_e"] | KalmanFilter_mpu._err_measure;
                            KalmanFilter_mpu._err_estimate = kf["est_e"] | KalmanFilter_mpu._err_estimate;
                            KalmanFilter_mpu._q = kf["q"] | KalmanFilter_mpu._q;
                        }
                        if (doc["K"].is<JsonObject>()) {
                            JsonObject mpu = doc["K"];
                            K[0] = mpu["K1"] | K[0];
                            K[1] = mpu["K2"] | K[1];
                            K[2] = mpu["K3"] | K[2];
                            K[3] = mpu["K4"] | K[3];
                        }
                    }


                        // 处理PID请求
                    else if (doc["type"] == "pid_request") {
                        Serial.println("[WEB]:收到PID参数请求");
                        JsonDocument resDoc;
                        resDoc["type"] = "pid_data";

                        // 修改点：强制以小数格式输出数值
                        JsonObject speed = resDoc["speed"].to<JsonObject>();
                        speed["Kp"] = String(SpeedPID.Kp, 6).toFloat(); // 保留5位小数
                        speed["Ki"] = String(SpeedPID.Ki, 6).toFloat(); // 保留7位小数
                        speed["Kd"] = String(SpeedPID.Kd, 6).toFloat();

                        JsonObject upright = resDoc["upright"].to<JsonObject>();
                        upright["Kp"] = String(UprightPID.Kp, 6).toFloat();
                        upright["Ki"] = String(UprightPID.Ki, 6).toFloat();
                        upright["Kd"] = String(UprightPID.Kd, 6).toFloat();

                        JsonObject turn = resDoc["turn"].to<JsonObject>();
                        turn["Kp"] = String(TurnPID.Kp, 6).toFloat();
                        turn["Ki"] = String(TurnPID.Ki, 6).toFloat();
                        turn["Kd"] = String(TurnPID.Kd, 6).toFloat();

                        JsonObject motor_pid = resDoc["motor_pid"].to<JsonObject>();
                        motor_pid["CS_P"] = String(motor_A.PID_current_d.P, 6).toFloat();
                        motor_pid["CS_I"] = String(motor_A.PID_current_d.I, 6).toFloat();

// 滤波器参数
                        JsonObject lpf = resDoc["lpf"].to<JsonObject>();
                        lpf["velocity_Tf"] = String(motor_A.LPF_velocity.Tf, 6).toFloat();
                        lpf["current_d_Tf"] = String(motor_A.LPF_current_d.Tf, 6).toFloat();
                        lpf["current_q_Tf"] = String(motor_A.LPF_current_q.Tf, 6).toFloat();
                        lpf["angle_Tf"] = String(motor_A.LPF_angle.Tf, 6).toFloat();

// 卡尔曼滤波器
                        JsonObject kalman_filter = resDoc["kalman_filter"].to<JsonObject>();
                        kalman_filter["mea_e"] = String(KalmanFilter_mpu._err_measure, 6).toFloat();
                        kalman_filter["est_e"] = String(KalmanFilter_mpu._err_estimate, 6).toFloat();
                        kalman_filter["q"] = String(KalmanFilter_mpu._q, 6).toFloat();


                        lpf["speed"] = String(lpf_speed.Tf, 6).toFloat();
                        lpf["speed_error"] = String(lpf_speed_error.Tf, 6).toFloat();
                        lpf["gyro_x"] = String(lpf_gyro_x.Tf, 6).toFloat();
                        lpf["control_steer"] = String(lpf_control_steer.Tf, 6).toFloat();
                        lpf["control_speed"] = String(lpf_control_speed.Tf, 6).toFloat();


                        JsonObject mpu = resDoc["mpu"].to<JsonObject>();
                        mpu["offset"] = String(Car_Info.Angle_Deviation, 6).toFloat();

                        JsonObject K_data = resDoc["K"].to<JsonObject>();
                        K_data["K1"] = String(K[0], 6).toFloat();
                        K_data["K2"] = String(K[1], 6).toFloat();
                        K_data["K3"] = String(K[2], 6).toFloat();
                        K_data["K4"] = String(K[3], 6).toFloat();

                        if (client->status() == WS_CONNECTED) {
                            String response;
                            serializeJson(resDoc, response);
                            client->text(response);
                        }
                    }


                        // 在WS_EVT_DATA的JSON解析部分添加：
                    else if (doc["type"] == "sensor_request") {
                        Serial.println("[WEB]:收到传感器数据请求");

                        JsonDocument resDoc;
                        resDoc["type"] = "sensor_data";
                        resDoc["battery"] = battery_percent;
                        resDoc["temperature"] = mpu6050.temperature;

                        if (client->status() == WS_CONNECTED) {
                            String response;
                            serializeJson(resDoc, response);
                            client->text(response);
                        }

                    }
                }
            }
            break;
        }
        default:
            break;
    }
}

void wifi_init() {
    //跳过
    if (xbox_Handle_control_switch)
    {
        Serial.println("[WIFI]:跳过WIFI启动");
        return;
    }

    // 创建AP
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));   //设置AP热点IP和子网掩码
    Serial.println("[WIFI]:WIFI已启动");
    Serial.print("[WEB]:IP地址: ");
    Serial.println(WiFi.softAPIP());

    initDNS();

    // 配置WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // 配置路由
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", IndexhtmlContent);
    });

    server.on("/setting", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", pidHtmlContent);
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->redirect("http://192.168.1.1");
    });


    // 启动服务器
    server.begin();
}

void Web_loop(void *pvParameters) {
    while(true)
    {
        vTaskDelay(10);
        ws.cleanupClients();
        dnsServer.processNextRequest();   //检查客户端DNS请求
    }
}