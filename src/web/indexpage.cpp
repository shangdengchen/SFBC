//
// Created by MGJ on 2025/5/2.
//
#include "web.h"


// HTML页面内容
const char *IndexhtmlContent = R"rawliteral(
<!DOCTYPE html>
<html lang="cn">
<head>
    <title>智能遥控器</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        :root {
            --joystick-size: 250px;
            --card-width: 200px;
        }

        body {
            margin: 0;
            padding: 20px;
            height: 80vh;
            display: flex;
            flex-direction: column;
            background: #f8f9fa;
            font-family: 'Segoe UI', system-ui;
        }

        .header-bar {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 20px;
        }

        .nav-button {
            padding: 12px 24px;
            background: #4CAF50;
            color: white;
            text-decoration: none;
            border-radius: 8px;
            font-size: 14px;
            transition: background 0.2s;
        }

        .nav-button:hover {
            background: #45a049;
        }

        .dashboard {
            flex: 1;
            display: grid;
            gap: 85px;
        }

        /* 手机竖屏布局 */
        @media (orientation: portrait) {
            .dashboard {
                grid-template-rows: auto var(--joystick-size);
                align-items: center;
                align-content: space-evenly;
            }

            .status-cards {
                order: 1;
                grid-template-columns: repeat(2, 1fr);
            }

            .card {
                padding: 14px;
            }

            .joystick-section {
                order: 2;
                position: relative;
                height: var(--joystick-size);
            }


            #joystick-container {
                margin-top: 60px;
                margin-bottom: 50px;
            }
        }

        /* 平板/横屏布局 */
        @media (orientation: landscape) {
            .dashboard {
                grid-template-columns: 1fr var(--joystick-size);
                align-content: space-evenly;
            }
        }

        .status-cards {
            display: grid;
            gap: 20px;
            grid-template-columns: repeat(auto-fit, minmax(var(--card-width), 1fr));
        }

        .card {
            background: white;
            border-radius: 16px;
            padding: 24px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.05);
            transition: transform 0.2s;
        }

        .card:hover {
            transform: translateY(-2px);
        }

        .card h3 {
            margin: 0 0 12px 0;
            font-size: 14px;
            color: #666;
            font-weight: 500;
        }

        .card p {
            margin: 0;
            font-size: 28px;
            font-weight: 600;
        }

        #latency {
            color: #4CAF50;
        }

        #temperature {
            color: #FF9800;
        }

        .joystick-section {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
        }

        .battery-bar {
            background: #eee;
            border-radius: 8px;
            height: 28px;
            overflow: hidden;
            position: relative;
        }

        .battery-level {
            height: 100%;
            width: 0%;
            background: #4CAF50;
            transition: all 0.3s ease;
            position: relative;
        }

        .battery-percent {
            position: absolute;
            right: 8px;
            top: 50%;
            transform: translateY(-50%);
            color: white;
            font-size: 14px;
            font-weight: bold;
            text-shadow: 0 1px 2px rgba(0, 0, 0, 0.3);
        }

        #joystick-container {
            background: white;
            border-radius: 50%;
            position: relative;
            touch-action: none;
            box-shadow: 0 8px 24px rgba(0, 0, 0, 0.1);
            aspect-ratio: 1; /* 强制宽高比1:1 */
            width: min(80vw, 250px); /* 自适应屏幕宽度 */
            height: auto !important; /* 自动计算高度 */
        }

        #joystick {
            background: #4CAF50;
            border-radius: 50%;
            position: absolute;
            transition: transform 0.1s cubic-bezier(0.18, 0.89, 0.32, 1.28);
            width: 35% !important;
            height: 35% !important; /* 固定宽高比例 */
            left: 50% !important;
            top: 50% !important;
            transform: translate(-50%, -50%); /* 精准居中定位 */
        }

        #data {
            margin-top: 20px;
            font-size: 14px;
            color: #666;
            text-align: center;
        }

        /* 小屏幕手机优化 */
        @media (max-width: 480px) {
            :root {
                --joystick-size: 200px;
                --card-width: 1fr;
            }

            .status-cards {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
<div class="header-bar">
    <a href="/setting" class="nav-button">高级设置</a>
</div>

<div class="dashboard">
    <div class="status-cards">
        <div class="card">
            <h3>控制延迟</h3>
            <p id="latency">-- ms</p>
        </div>
        <div class="card">
            <h3>电池电量</h3>
            <div class="battery-bar">
                <div class="battery-level" id="battery-level">
                    <span class="battery-percent" id="battery-percent">--%</span>
                </div>
            </div>
        </div>
        <div class="card">
            <h3>设备温度</h3>
            <p id="temperature">-- °C</p>
        </div>
    </div>

    <div class="joystick-section">
        <div id="joystick-container">
            <div id="joystick"></div>
        </div>
        <div id="data">X: 0.00, Y: 0.00</div>
    </div>
</div>

<script>
    const joystick = document.getElementById('joystick');
    const container = document.getElementById('joystick-container');
    const dataDiv = document.getElementById('data');
    const latencyDiv = document.getElementById('latency');
    let ws;
    let pingInterval;
    let lastPingTime;

    function updateConnectionStatus(status) {
        latencyDiv.textContent = `${status}`;
    }

    function connectWebSocket() {
        ws = new WebSocket('ws://' + window.location.hostname + '/ws');

        ws.onopen = () => {
            console.log('WebSocket连接已建立');
            pingInterval = setInterval(sendPing, 1000);

        };

        ws.onerror = (error) => {
            console.log('WebSocket错误:', error);
        };

        ws.onclose = () => {
            console.log('WebSocket断开连接');
            clearInterval(pingInterval);
            setTimeout(connectWebSocket, 4000);
            latencyDiv.textContent = `断开连接`;
            latencyDiv.style.color = "#ff4444";
        };

        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'pong') {
                const latency = Date.now() - data.clientTime;
                latencyDiv.textContent = ` ${latency} ms`;
                latencyDiv.style.color = "#4CAF50";
            }
            // 在ws.onmessage事件处理器添加：
            if (data.type === 'sensor_data') {
                // 更新电池显示
                const batteryLevel = data.battery;
                const batteryElement = document.getElementById('battery-level');
                const batteryPercent = document.getElementById('battery-percent');

                batteryElement.style.width = `${batteryLevel}%`;
                batteryPercent.textContent = `${batteryLevel.toFixed(0)}%`;

                // 根据电量改变颜色
                batteryElement.style.background =
                    batteryLevel > 60 ? '#4CAF50' :
                        batteryLevel > 30 ? '#FFC107' : '#F44336';

                // 更新温度显示
                document.getElementById('temperature').textContent =
                    `${data.temperature.toFixed(1)}°C`; // 保留一位小数
            }
        };
    }

    function sendPing() {
        if (ws.readyState === WebSocket.OPEN) {
            const pingData = {
                type: 'ping',
                clientTime: Date.now()
            };
            ws.send(JSON.stringify(pingData));
        }
    }


    function initJoystick() {
        let isDragging = false;
        const rect = container.getBoundingClientRect();
        const centerX = rect.width / 2;
        const centerY = rect.height / 2;
        const maxDistance = 100;
        let currentX = 0;
        let currentY = 0;
        let animationFrameId = null;

        function updatePosition(clientX, clientY) {
            const rect = container.getBoundingClientRect();
            const x = clientX - rect.left - centerX;
            const y = clientY - rect.top - centerY;

            const distance = Math.min(maxDistance, Math.sqrt(x * x + y * y));
            const angle = Math.atan2(y, x);

            const normalizedX = (distance * Math.cos(angle)) / maxDistance;
            const normalizedY = (distance * Math.sin(angle)) / maxDistance;

            currentX = normalizedX;
            currentY = normalizedY;

            joystick.style.transform = `translate(
            calc(-50% + ${normalizedX * maxDistance}px),
            calc(-50% + ${normalizedY * maxDistance}px)
            )`;

            dataDiv.textContent = `X: ${normalizedX.toFixed(2)}, Y: ${normalizedY.toFixed(2)}`;

            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    x: normalizedX,
                    y: normalizedY
                }));
            }
        }

        function animateToCenter() {
            const startTime = Date.now();
            const duration = 200; // 动画持续时间300ms
            const startX = currentX;
            const startY = currentY;
            // 新增强制完成标记
            let forceComplete = false;
            const threshold = 0.035; // 设定归零阈值（2%的误差范围）
            function animate() {
                if (forceComplete) return;

                const elapsed = Date.now() - startTime;
                const progress = Math.min(1, elapsed / duration);

                // 使用ease-out缓动函数
                const easedProgress = 1 - Math.pow(1 - progress, 2);

                let currentPosX = startX * (1 - easedProgress);
                let currentPosY = startY * (1 - easedProgress);

                // 根据当前位置判断是否需要强制归零
                if (Math.abs(currentPosX) < threshold && Math.abs(currentPosY) < threshold) {
                    currentPosX = 0;
                    currentPosY = 0;
                    forceComplete = true;
                }

                joystick.style.transform = `translate(
                    calc(-50% + ${currentPosX * maxDistance}px),
                    calc(-50% + ${currentPosY * maxDistance}px)
                )`;

                dataDiv.textContent = `X: ${currentPosX.toFixed(2)}, Y: ${currentPosY.toFixed(2)}`;

                // 仅在动画结束时发送精确的0
                if (forceComplete) {
                    currentX = 0;
                    currentY = 0;
                    joystick.style.transform = 'translate(-50%, -50%)';
                    dataDiv.textContent = 'X: 0.00, Y: 0.00';
                    if (ws.readyState === WebSocket.OPEN) {
                        ws.send(JSON.stringify({ x: 0, y: 0 }));
                        ws.send(JSON.stringify({ x: 0, y: 0 }));
                    }
                } else {
                    // 动画过程中发送实际值
                    if (ws.readyState === WebSocket.OPEN) {
                        ws.send(JSON.stringify({
                            x: +currentPosX.toFixed(2),
                            y: +currentPosY.toFixed(2)
                        }));
                    }
                    animationFrameId = requestAnimationFrame(animate);
                }
            }

            animationFrameId = requestAnimationFrame(animate);
        }

        container.addEventListener('touchstart', (e) => {
            if (animationFrameId) {
                cancelAnimationFrame(animationFrameId);
                animationFrameId = null;
            }
            isDragging = true;
            updatePosition(e.touches[0].clientX, e.touches[0].clientY);
            e.preventDefault();
        });

        container.addEventListener('touchmove', (e) => {
            if (isDragging) {
                updatePosition(e.touches[0].clientX, e.touches[0].clientY);
                e.preventDefault();
            }
        });
        container.addEventListener('touchend', () => {
            if (isDragging) {
                isDragging = false;
                animateToCenter();
            }
        });
        container.addEventListener('mousedown', (e) => {
            if (animationFrameId) {
                cancelAnimationFrame(animationFrameId);
                animationFrameId = null;
            }
            isDragging = true;
            updatePosition(e.clientX, e.clientY);
        });

        document.addEventListener('mousemove', (e) => {
            if (isDragging) {
                updatePosition(e.clientX, e.clientY);
            }
        });
        document.addEventListener('mouseup', () => {
            if (isDragging) {
                isDragging = false;
                animateToCenter();
            }
        });
    }
    const updateSensors = () => {

        if (ws.readyState === WebSocket.OPEN) {
            const requestData = {
                type: "sensor_request",
                clientTime: Date.now()
            };
            ws.send(JSON.stringify(requestData));
        }
    };

    // 初始化
    window.addEventListener('DOMContentLoaded', () => {
        connectWebSocket();
        initJoystick();
        setInterval(updateSensors, 2000);
    });
</script>
</body>
</html>
)rawliteral";