#pragma once

#include <NimBLEDevice.h>
#include <XboxControllerNotificationParser.h>

#include <XboxSeriesXHIDReportBuilder_asukiaaa.hpp>
#include <freertos/FreeRTOS.h>

// #define XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL Serial
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
const unsigned long printInterval = 100UL;
#endif

namespace XboxSeriesXControllerESP32_asukiaaa {

// 定义通用属性服务的 UUID
    static NimBLEUUID uuidServiceGeneral("1801");
// 定义电池服务的 UUID
    static NimBLEUUID uuidServiceBattery("180f");
// 定义 HID（人机接口设备）服务的 UUID
    static NimBLEUUID uuidServiceHid("1812");
// 定义 HID 报告特征的 UUID
    static NimBLEUUID uuidCharaReport("2a4d");
// 定义 PnP（即插即用）信息特征的 UUID
    static NimBLEUUID uuidCharaPnp("2a50");
// 定义 HID 信息特征的 UUID
    static NimBLEUUID uuidCharaHidInformation("2a4a");
// 定义手柄外设外观特征的 UUID
    static NimBLEUUID uuidCharaPeripheralAppearance("2a01");
// 定义外设控制参数特征的 UUID
    static NimBLEUUID uuidCharaPeripheralControlParameters("2a04");




// 当前扫描到的广告设备（静态变量）
    static const NimBLEAdvertisedDevice *advDevice;

// 当前连接的客户端（静态变量）
    static NimBLEClient *pConnectedClient = nullptr;

// 控制器的外观代码
    static const uint16_t controllerAppearance = 964;

// 控制器的正常状态制造商数据（十六进制字符串）
    static const String controllerManufacturerDataNormal = "060000";

// 控制器的搜索状态制造商数据（十六进制字符串）
    static const String controllerManufacturerDataSearching = "0600030080";

// 定义连接状态枚举类
    enum class ConnectionState : uint8_t {
        Connected = 0, // 已连接
        WaitingForFirstNotification = 1, // 等待第一个通知
        Found = 2, // 已找到设备
        Scanning = 3, // 正在扫描
    };

    class ClientCallbacks : public NimBLEClientCallbacks {
    public:
        // 指向连接状态的指针
        ConnectionState *pConnectionState;

        // 构造函数，初始化连接状态指针
        ClientCallbacks(ConnectionState *pConnectionState) {
            this->pConnectionState = pConnectionState;
        }

        // 当客户端连接成功时调用
        void onConnect(NimBLEClient *pClient) override {
            // 将连接状态设置为“等待第一个通知”
            *pConnectionState = ConnectionState::WaitingForFirstNotification;
            // // 可选：更新连接参数（注释掉了）
            // pClient->updateConnParams(120, 120, 0, 60);
        }

        // 当客户端断开连接时调用
        void onDisconnect(NimBLEClient *pClient, int reason) override {
            // 将连接状态设置为“正在扫描”
            *pConnectionState = ConnectionState::Scanning;
            // 清空已连接的客户端指针
            pConnectedClient = nullptr;
        }

        // 当需要输入配对密码时调用
        void onPassKeyEntry(NimBLEConnInfo &connInfo) override {
            // 自动注入配对密码（这里设置为 0）
            NimBLEDevice::injectPassKey(connInfo, 0);
        }

        // 当需要确认配对密码时调用
        void onConfirmPasskey(NimBLEConnInfo &connInfo, uint32_t pass_key) override {
            // // 如果定义了调试串口，打印配对密码
            // Serial.print("The passkey YES/NO number: ");
            // Serial.println(pass_key);
            // 自动确认配对密码
            NimBLEDevice::injectConfirmPasskey(connInfo, true);
        }

        // 当配对过程完成时调用
        void onAuthenticationComplete(NimBLEConnInfo &connInfo) override {
            // 如果连接未加密，则断开连接
            if (!connInfo.isEncrypted()) {
                // 找到对应连接句柄的客户端并断开连接
                NimBLEDevice::getClientByHandle(connInfo.getConnHandle())->disconnect();
                return;
            }
        }
    };


/** 定义一个类，用于处理接收到广告包时的回调 */
    class ScanCallbacks : public NimBLEScanCallbacks {
    public:
        // 构造函数
        ScanCallbacks(String strTargetDeviceAddress, ConnectionState *pConnectionState) {
            // 如果指定了目标设备地址
            if (strTargetDeviceAddress != "") {
                // 创建目标设备地址对象
                this->targetDeviceAddress = new NimBLEAddress(strTargetDeviceAddress.c_str(), 0);
            }
            // 保存连接状态指针
            this->pConnectionState = pConnectionState;
        }

    private:
        // 目标设备地址（如果指定了目标设备）
        NimBLEAddress *targetDeviceAddress = nullptr;

        // 指向连接状态的指针
        ConnectionState *pConnectionState;

        // 当扫描到设备时调用
        void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override {
            // 获取设备的制造商数据并转换为十六进制字符串
            auto pHex = NimBLEUtils::dataToHexString(
                    (uint8_t *) advertisedDevice->getManufacturerData().data(),
                    advertisedDevice->getManufacturerData().length());

            //===============================修改的关键=========================================
            // 判断是否为目标设备
            if (
                    (targetDeviceAddress != nullptr &&
                     advertisedDevice->getAddress().equals(*targetDeviceAddress)
                    )

                    || // 如果指定了目标设备地址，并且扫描到的设备匹配
                    (targetDeviceAddress == nullptr &&
                     advertisedDevice->getAppearance() == controllerAppearance

                     && // 如果未指定目标设备地址，检查设备外观
                     (strcmp(pHex.c_str(), controllerManufacturerDataNormal.c_str()) == 0
                      || // 检查制造商数据是否匹配
                      strcmp(pHex.c_str(), controllerManufacturerDataSearching.c_str()) == 0)
                     &&

                     //并且控制器支持手柄数据检查是否支持 HID 服务
                     advertisedDevice->getServiceUUID().equals(uuidServiceHid)
                    )

                    ) {
                // 停止扫描（注释掉了）
                // NimBLEDevice::getScan()->stop();

                // 更新连接状态为“已找到设备”
                *pConnectionState = ConnectionState::Found;

                // 保存广告设备的引用，供客户端使用
                advDevice = advertisedDevice;
            }
        }

        // 当扫描结束时调用
        void onScanEnd(const NimBLEScanResults &scanResults, int reason) override {
            // 如果定义了调试串口，打印扫描结束信息
            //XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Scan Ended");

        }
    };


    class Core {
    public:
        // 构造函数，初始化 Core 对象
        Core(String targetDeviceAddress = "") {
            // 创建扫描回调对象
            this->scanCBs = new ScanCallbacks(targetDeviceAddress, &connectionState);
            // 创建客户端回调对象
            this->clientCBs = new ClientCallbacks(&connectionState);
        }

        // 扫描回调对象
        ScanCallbacks *scanCBs;

        // 客户端回调对象
        ClientCallbacks *clientCBs;

        // 电池电量（默认值为 0）
        uint8_t battery = 0;

        // 设备地址的长度（蓝牙地址通常为 6 字节）
        static const int deviceAddressLen = 6;

        // 用于存储设备地址的数组
        uint8_t deviceAddressArr[deviceAddressLen];

        void begin() {
            // 初始化 BLE 设备
            NimBLEDevice::init("");

            // 设置设备的地址类型为公共地址
            NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);

            // 设置安全认证参数
            // 参数含义：启用配对、不启用绑定、不启用 MITM（中间人攻击防护）
            NimBLEDevice::setSecurityAuth(true, false, false);

            // 设置 BLE 设备的发射功率为 +9dB
            NimBLEDevice::setPower(ESP_PWR_LVL_P9); /* +9db */
        }


        void writeHIDReport(uint8_t *dataArr, size_t dataLen) {
            // 如果没有连接的客户端，则直接返回
            if (pConnectedClient == nullptr) {
                return;
            }

            // 获取已连接的客户端
            NimBLEClient *pClient = pConnectedClient;

            // 获取客户端的 HID 服务
            auto pService = pClient->getService(uuidServiceHid);

            // 遍历 HID 服务中的所有特征
            for (auto pChara: pService->getCharacteristics()) {
                // 检查特征是否支持写操作
                if (pChara->canWrite()) {
                    // 向特征写入数据
                    pChara->writeValue(dataArr, dataLen, false);
                }
            }
        }

        void writeHIDReport(
                const XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase &repo) {
            writeHIDReport((uint8_t *) repo.arr8t, repo.arr8tLen);
        }

        void writeHIDReport(
                const XboxSeriesXHIDReportBuilder_asukiaaa::ReportBeforeUnion &
                repoBeforeUnion) {
            XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
            repo.v = repoBeforeUnion;
            writeHIDReport((uint8_t *) repo.arr8t, repo.arr8tLen);
        }


        void onLoop() {
            // 检查是否已连接
            if (!isConnected()) {
                // 如果有广告设备
                if (advDevice != nullptr) {
                    // 已经配对，尝试连接到服务
                    auto connectionResult = connectToServer(advDevice);
                    // 如果连接失败或仍然未连接
                    if (!connectionResult || !isConnected()) {
                        // 删除与该设备的绑定
                        NimBLEDevice::deleteBond(advDevice->getAddress());
                        // 增加连接失败次数
                        ++countFailedConnection;
                        // 可选：重置设备（注释掉了）
                        // reset();
                        // 设置连接状态为扫描
                        connectionState = ConnectionState::Scanning;
                    } else {
                        // 如果连接成功，重置失败次数
                        countFailedConnection = 0;
                    }
                    // 清空广告设备指针
                    advDevice = nullptr;
                } else if (!isScanning()) {
                    // 如果当前没有扫描且没有广告设备
                    // 可选：重置设备（注释掉了）
                    // reset();
                    // 开始扫描
                    startScan();
                }
            }
        }

        String buildDeviceAddressStr() {
            char buffer[18];
            auto addr = deviceAddressArr;
            snprintf(buffer, sizeof(buffer), "%02x:%02x:%02x:%02x:%02x:%02x", addr[5],
                     addr[4], addr[3], addr[2], addr[1], addr[0]);
            return String(buffer);
        }

        void startScan() {
            // 设置连接状态为扫描
            connectionState = ConnectionState::Scanning;

            // 获取扫描实例
            auto pScan = NimBLEDevice::getScan();

            // 禁用重复过滤器（允许接收所有广告包，包括重复的）
            pScan->setDuplicateFilter(false);

            // 设置扫描回调函数(关键这里搜索手柄)
            pScan->setScanCallbacks(scanCBs);

            // 设置扫描间隔（单位：ms）
            pScan->setInterval(97);

            // 设置扫描窗口（单位：ms）
            pScan->setWindow(97);

            // 开始扫描，扫描持续时间为 msScanTime（单位：ms）
            pScan->start(msScanTime);
        }

        XboxControllerNotificationParser xboxNotif;

        bool isWaitingForFirstNotification() {
            return connectionState == ConnectionState::WaitingForFirstNotification;
        }

        bool isConnected() {
            // 判断设备是否处于已连接状态
            return connectionState == ConnectionState::WaitingForFirstNotification || // 等待第一个通知
                   connectionState == ConnectionState::Connected;                     // 已连接
        }

        unsigned long getReceiveNotificationAt() { return receivedNotificationAt; }

        uint8_t getCountFailedConnection() { return countFailedConnection; }


    private:
        // 当前连接状态，默认为“正在扫描”
        ConnectionState connectionState = ConnectionState::Scanning;
        // 上次接收到通知的时间戳
        unsigned long receivedNotificationAt = 0;
        // 扫描持续时间（单位：毫秒），0 表示无限期扫描
        uint32_t msScanTime = 4000;
        // 连接失败的次数
        uint8_t countFailedConnection = 0;
        // 每次连接中尝试重试的次数
        uint8_t retryCountInOneConnection = 3;
        // 重试间隔时间（单位：毫秒）
        unsigned long retryIntervalMs = 100;
        // 当前连接的客户端指针
        NimBLEClient *pClient = nullptr;

        // 静态函数，用于读取并打印特征的值
        static void readAndPrint(NimBLERemoteCharacteristic *pChara) {
            // 读取特征的值
            auto str = pChara->readValue();

            // 如果读取到的值为空，则再次尝试读取
            if (str.size() == 0) {
                str = pChara->readValue();
            }
        }

        // 判断是否正在扫描
        bool isScanning() {
            return NimBLEDevice::getScan()->isScanning();
        }


        /** 处理客户端的创建，并连接到服务器 */
        bool connectToServer(const NimBLEAdvertisedDevice *advDevice) {
            NimBLEClient *pClient = nullptr;

            /** 检查是否有可以重用的客户端 */
            if (NimBLEDevice::getCreatedClientCount()) {
                // 尝试通过设备地址获取已创建的客户端
                pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
                if (pClient) {
                    // 如果找到客户端，则尝试连接
                    pClient->connect();
                }
            }

            /** 如果没有可以重用的客户端，则创建一个新的 */
            if (!pClient) {
                // 检查是否已经达到最大连接数
                if (NimBLEDevice::getCreatedClientCount() >= NIMBLE_MAX_CONNECTIONS) {
                    return false; // 如果达到最大连接数，返回失败
                }
                // 创建一个新的客户端
                pClient = NimBLEDevice::createClient();
                // 设置客户端回调函数
                pClient->setClientCallbacks(clientCBs, true);
                // 尝试连接到广告设备
                pClient->connect(advDevice, true);
            }

            // 初始化重试次数
            int retryCount = retryCountInOneConnection;
            // 检查是否成功连接
            while (!pClient->isConnected()) {
                if (retryCount <= 0) {
                    return false; // 如果重试次数耗尽，返回失败
                }
                // 等待一段时间后重试（使用 FreeRTOS 的 vTaskDelay 替代 delay）
                vTaskDelay(retryIntervalMs);

                // 尝试重新连接
                pClient->connect(true);
                --retryCount;
            }

            // 连接成功后，调用 afterConnect 函数进行后续处理
            bool result = afterConnect(pClient);
            if (!result) {
                return false; // 如果 afterConnect 返回失败，则返回 false
            }
            // 保存已连接的客户端
            pConnectedClient = pClient;
            return true; // 返回连接成功
        }



        // 连接成功后的处理函数
        bool afterConnect(NimBLEClient *pClient) {
            // 将连接设备的地址复制到设备地址数组
            memcpy(deviceAddressArr, pClient->getPeerAddress().getBase(), deviceAddressLen);

            // 遍历客户端的所有服务
            for (auto pService : pClient->getServices(true)) {
                // 获取服务的 UUID
                auto sUuid = pService->getUUID();

                // 如果服务的 UUID 不是 HID 服务或电池服务，则跳过
                if (!sUuid.equals(uuidServiceHid) && !sUuid.equals(uuidServiceBattery)) {
                    continue;
                }

                // 遍历服务中的所有特征
                for (auto pChara : pService->getCharacteristics(true)) {
                    // 处理特征
                    charaHandle(pChara);

                    // 订阅通知（例如按键监控）
                    charaSubscribeNotification(pChara);
                }
            }

            // 返回成功
            return true;
        }

        // 特征处理函数
        void charaHandle(NimBLERemoteCharacteristic *pChara) {
            // 如果特征支持写操作
            if (pChara->canWrite()) {
                // 这里可以添加写操作的逻辑
            }

            // 如果特征支持读操作
            if (pChara->canRead()) {
                // 读取特征的值（某些设备需要先读取值才能订阅通知）
                auto str = pChara->readValue();

                // 如果读取到的值为空，再次尝试读取
                if (str.size() == 0) {
                    str = pChara->readValue();
                }
            }
        }

// 特征通知订阅函数
        void charaSubscribeNotification(NimBLERemoteCharacteristic *pChara) {
            // 如果特征支持通知
            if (pChara->canNotify()) {
                // 订阅通知
                if (pChara->subscribe(
                        true, // 启用通知
                        std::bind(&Core::notifyCB, this, std::placeholders::_1, // 绑定通知回调函数
                                  std::placeholders::_2, std::placeholders::_3,
                                  std::placeholders::_4),
                        true)) {
                    // 订阅成功后的逻辑（如果需要）
                }
            }
        }

        void notifyCB(NimBLERemoteCharacteristic *pRemoteCharacteristic,
                      uint8_t *pData, size_t length, bool isNotify) {
            auto sUuid = pRemoteCharacteristic->getRemoteService()->getUUID();
            if (connectionState != ConnectionState::Connected) {
                connectionState = ConnectionState::Connected;
            }
            if (sUuid.equals(uuidServiceHid)) {
                xboxNotif.update(pData, length);
                receivedNotificationAt = millis();
            } else {
                if (sUuid.equals(uuidServiceBattery)) {
                    battery = pData[0];
                }
            }
        }



    };

};