//
// Created by MGJ on 2025/5/2.
//
#include "web.h"

// 在web.cpp中添加PID设置页面HTML
const char *pidHtmlContent = R"rawliteral(
<!DOCTYPE html>
<html lang="cn">
<head>
  <title>PID参数设置</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    :root {
      --primary-color: #4CAF50;
      --error-color: #f44336;
      --background-color: #f5f5f5;
      --card-bg: #ffffff;
    }

    body {
      font-family: 'Segoe UI', sans-serif;
      margin: 0;
      padding: 20px;
      background: var(--background-color);
    }

    .container {
      max-width: 1200px;
      margin: 0 auto;
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
      border: none;
      font-size: 14px;
      transition: background 0.2s;
    }

    .nav-button:hover {
      background: #45a049;
    }



    .pid-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
      gap: 25px;
      margin-top: 30px;
    }

    .pid-card {
      background: var(--card-bg);
      border-radius: 12px;
      padding: 25px;
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
    }

    .param-group {
      margin-bottom: 20px;
    }

    .input-group {
      display: flex;
      align-items: center;
      gap: 15px;
    }

    input[type="range"] {
      flex: 1;
      height: 8px;
      background: #ddd;
      border-radius: 4px;
      -webkit-appearance: none;
    }

    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 20px;
      height: 20px;
      background: var(--primary-color);
      border-radius: 50%;
      cursor: pointer;
      transition: transform 0.2s;
    }

    input[type="number"] {
      width: 120px;
      padding: 10px 15px;
      border: 2px solid #ddd;
      border-radius: 8px;
      font-size: 16px;
      text-align: center;
      transition: border-color 0.3s;
    }

    .status-bar {
      position: fixed;
      bottom: 20px;
      left: 50%;
      transform: translateX(-50%);
      color: white;
      padding: 12px 30px;
      border-radius: 8px;
      display: none;
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
      opacity: 0;
      transition: opacity 0.3s;
    }

    .status-success {
      background: var(--primary-color);
    }

    .status-error {
      background: var(--error-color);
    }
  </style>
</head>
<body>
<div class="container">
  <div class="header-bar">
    <a href="/" class="nav-button">← 返回控制页</a>
    <button class="nav-button" id="syncButton">更新参数</button>
  </div>

  <div class="pid-grid">
    <!-- 三个PID卡片保持不变，仅修改输入框step属性 -->
    <!-- Speed PID -->
    <div class="pid-card">
      <div class="card-title">速度PID</div>
      <div class="param-group">
        <label class="param-label">Kp (-5.00~5.00)</label>
        <div class="input-group">
          <input type="range" id="speedKp" min="-5" max="5" step="0.00001" value="0">
          <input type="number" id="speedKpVal" min="-5" max="5" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">Ki (-5.00~5.00)</label>
        <div class="input-group">
          <input type="range" id="speedKi" min="-5" max="5" step="0.00001" value="0">
          <input type="number" id="speedKiVal" min="-5" max="5" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">Kd (-5.00~5.00)</label>
        <div class="input-group">
          <input type="range" id="speedKd" min="-5" max="5" step="0.00001" value="0">
          <input type="number" id="speedKdVal" min="-5" max="5" step="0.00001" value="0">
        </div>
      </div>
    </div>

    <!-- Upright PID -->
    <div class="pid-card">
      <div class="card-title">直立PID</div>
      <div class="param-group">
        <label class="param-label">Kp (-5.00~5.00)</label>
        <div class="input-group">
          <input type="range" id="uprightKp" min="-5" max="5" step="0.00001" value="0">
          <input type="number" id="uprightKpVal" min="-5" max="5" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">Ki (-5.00~5.00)</label>
        <div class="input-group">
          <input type="range" id="uprightKi" min="-5" max="5" step="0.00001" value="0">
          <input type="number" id="uprightKiVal" min="-5" max="5" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">Kd (-5.00~5.00)</label>
        <div class="input-group">
          <input type="range" id="uprightKd" min="-5" max="5" step="0.00001" value="0">
          <input type="number" id="uprightKdVal" min="-5" max="5" step="0.00001" value="0">
        </div>
      </div>
    </div>

    <!-- Turn PID -->
    <div class="pid-card">
      <div class="card-title">转向PID</div>
      <div class="param-group">
        <label class="param-label">Kp (-5.00~5.00)</label>
        <div class="input-group">
          <input type="range" id="turnKp" min="-5" max="5" step="0.00001" value="0">
          <input type="number" id="turnKpVal" min="-5" max="5" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">Ki (-5.00~5.00)</label>
        <div class="input-group">
          <input type="range" id="turnKi" min="-5" max="5" step="0.00001" value="0">
          <input type="number" id="turnKiVal" min="-5" max="5" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">Kd (-5.00~5.00)</label>
        <div class="input-group">
          <input type="range" id="turnKd" min="-5" max="5" step="0.00001" value="0">
          <input type="number" id="turnKdVal" min="-5" max="5" step="0.00001" value="0">
        </div>
      </div>
    </div>
    <div class="pid-card">
      <div class="card-title">电机PID(⛔小心设置,目前调整不生效)</div>
      <div class="param-group">
        <label class="param-label">CS_P (0~2)</label>
        <div class="input-group">
          <input type="range" id="motorCsP" min="0" max="2" step="0.00001" value="0">
          <input type="number" id="motorCsPVal" min="0" max="2" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">CS_I (0~2)</label>
        <div class="input-group">
          <input type="range" id="motorCsI" min="0" max="2" step="0.00001" value="0">
          <input type="number" id="motorCsIVal" min="0" max="2" step="0.00001" value="0">
        </div>
      </div>
    </div>

    <!-- 滤波器参数 -->
    <div class="pid-card">
      <div class="card-title">电机滤波(⛔小心设置)</div>
      <div class="param-group">
        <label class="param-label">Velocity Tf</label>
        <div class="input-group">
          <input type="range" id="lpfVelocityTf" min="0" max="5" step="0.00001" value="0">
          <input type="number" id="lpfVelocityTfVal" min="0" max="5" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">Current d Tf</label>
        <div class="input-group">
          <input type="range" id="lpfCurrentDTf" min="0" max="5" step="0.00001" value="0">
          <input type="number" id="lpfCurrentDTfVal" min="0" max="5" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">Current q Tf</label>
        <div class="input-group">
          <input type="range" id="lpfCurrentQTf" min="0" max="5" step="0.00001" value="0">
          <input type="number" id="lpfCurrentQTfVal" min="0" max="5" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">Angle Tf</label>
        <div class="input-group">
          <input type="range" id="lpfAngleTf" min="0" max="5" step="0.00001" value="0">
          <input type="number" id="lpfAngleTfVal" min="0" max="5" step="0.00001" value="0">
        </div>
      </div>

    </div>




    <div class="pid-card">
      <div class="card-title">MPU调整</div>

      <div class="param-group">
        <label class="param-label">offset (-10.00~10.00)</label>
        <div class="input-group">
          <input type="range" id="mpu_offset" min="-10" max="10" step="0.00001" value="0">
          <input type="number" id="mpu_offsetVal" min="-10" max="10" step="0.00001" value="0">
        </div>
      </div>

    </div>

    <!-- 卡尔曼滤波器 -->
    <div class="pid-card">
      <div class="card-title">MPU卡尔曼滤波器</div>
      <div class="param-group">
        <label class="param-label">mea_e</label>
        <div class="input-group">
          <input type="range" id="kfMeaE" min="0" max="5" step="0.00001" value="0">
          <input type="number" id="kfMeaEVal" min="0" max="5" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">est_e</label>
        <div class="input-group">
          <input type="range" id="kfEstE" min="0" max="5" step="0.00001" value="0">
          <input type="number" id="kfEstEVal" min="0" max="5" step="0.00001" value="0">
        </div>
      </div>
      <div class="param-group">
        <label class="param-label">q</label>
        <div class="input-group">
          <input type="range" id="kfQ" min="0" max="5" step="0.00001" value="0">
          <input type="number" id="kfQVal" min="0" max="5" step="0.00001" value="0">
        </div>
      </div>
    </div>

    <div class="pid-card">
      <div class="card-title">传感器滤波</div>
      <div class="param-group">
        <label class="param-label">Speed (0~2)</label>
        <div class="input-group">
          <input type="range" id="lpfSpeed" min="0" max="2" step="0.00001" value="0">
          <input type="number" id="lpfSpeedVal" min="0" max="2" step="0.00001" value="0">
        </div>
      </div>

      <div class="param-group">
        <label class="param-label">Speed Error (0~2)</label>
        <div class="input-group">
          <input type="range" id="lpfSpeedError" min="0" max="2" step="0.00001" value="0">
          <input type="number" id="lpfSpeedErrorVal" min="0" max="2" step="0.00001" value="0">
        </div>
      </div>

      <div class="param-group">
        <label class="param-label">Gyro X (0~2)</label>
        <div class="input-group">
          <input type="range" id="lpfGyroX" min="0" max="2" step="0.00001" value="0">
          <input type="number" id="lpfGyroXVal" min="0" max="2" step="0.00001" value="0">
        </div>
      </div>


    </div>

    <div class="pid-card">
      <div class="card-title">控制滤波</div>

      <div class="param-group">
        <label class="param-label">Control Steer (0~2)</label>
        <div class="input-group">
          <input type="range" id="lpfControlSteer" min="0" max="2" step="0.00001" value="0">
          <input type="number" id="lpfControlSteerVal" min="0" max="2" step="0.00001" value="0">
        </div>
      </div>

      <div class="param-group">
        <label class="param-label">Control Speed (0~2)</label>
        <div class="input-group">
          <input type="range" id="lpfControlSpeed" min="0" max="2" step="0.00001" value="0">
          <input type="number" id="lpfControlSpeedVal" min="0" max="2" step="0.00001" value="0">
        </div>
      </div>

    </div>

    <div class="pid-card">
      <div class="card-title">K(LQR控制变量,不生效)</div>

      <div class="param-group">
        <label class="param-label">K1 (50)</label>
        <div class="input-group">
          <input type="range" id="K1" min="-100" max="100" step="0.001" value="0">
          <input type="number" id="K1Val" min="-100" max="100" step="0.001" value="0">
        </div>
      </div>

      <div class="param-group">
        <label class="param-label">K2 (50)</label>
        <div class="input-group">
          <input type="range" id="K2" min="-100" max="100" step="0.001" value="0">
          <input type="number" id="K2Val" min="-100" max="100" step="0.001" value="0">
        </div>
      </div>

      <div class="param-group">
        <label class="param-label">K3 (50)</label>
        <div class="input-group">
          <input type="range" id="K3" min="-100" max="100" step="0.001" value="0">
          <input type="number" id="K3Val" min="-100" max="100" step="0.001" value="0">
        </div>
      </div>

      <div class="param-group">
        <label class="param-label">K4 (50)</label>
        <div class="input-group">
          <input type="range" id="K4" min="-100" max="100" step="0.001" value="0">
          <input type="number" id="K4Val" min="-100" max="100" step="0.001" value="0">
        </div>
      </div>

    </div>

    </div>
  </div>
</div>

<div class="status-bar" id="statusBar">参数已保存</div>

<script>
  const ws = new WebSocket('ws://' + window.location.hostname + '/ws');
  let isUpdating = false;

  // 初始化同步逻辑
  function initSync(rangeId, numberId) {
    const range = document.getElementById(rangeId);
    const number = document.getElementById(numberId);

    const syncToNumber = () => {
      if(isUpdating) return;
      number.value = parseFloat(range.value).toFixed(5);
      debounceUpdate();
    };

    const syncToRange = () => {
      if(isUpdating) return;
      let val = Math.min(500, Math.max(-500, parseFloat(number.value) || 0));
      range.value = val;
      debounceUpdate();
    };

    range.addEventListener('input', syncToNumber);
    number.addEventListener('change', syncToRange);
    number.addEventListener('input', () => {
      if(isUpdating) return;
      debounceUpdate();
    });
  }

  // 获取当前PID参数
  function fetchCurrentParams() {
    // 判断 WebSocket 是否处于连接状态
    if (ws.readyState === WebSocket.OPEN) {
      try {
        ws.send(JSON.stringify({ type: "pid_request" }));
        showStatus("正在同步参数...", "status-success");
      } catch (e) {
        showStatus("同步失败: " + e.message, "status-error");
      }
    } else {
      // 如果 WebSocket 没有连接，显示错误信息
      showStatus("未连接", "status-error");
    }
  }
  function updateControls(data) {
    // 更新Speed PID控件
    updateControl('speedKp', data.speed.Kp);
    updateControl('speedKi', data.speed.Ki);
    updateControl('speedKd', data.speed.Kd);

    // 更新Upright PID控件
    updateControl('uprightKp', data.upright.Kp);
    updateControl('uprightKi', data.upright.Ki);
    updateControl('uprightKd', data.upright.Kd);

    // 更新Turn PID控件
    updateControl('turnKp', data.turn.Kp);
    updateControl('turnKi', data.turn.Ki);
    updateControl('turnKd', data.turn.Kd);
    // 修改后
    updateControl('mpu_offset', data.mpu.offset);
    // 电机PID
    updateControl('motorCsP', data.motor_pid?.CS_P);
    updateControl('motorCsI', data.motor_pid?.CS_I);

    // 低通滤波器
    updateControl('lpfVelocityTf', data.lpf?.velocity_Tf);
    updateControl('lpfCurrentDTf', data.lpf?.current_d_Tf);
    updateControl('lpfCurrentQTf', data.lpf?.current_q_Tf);
    updateControl('lpfAngleTf', data.lpf?.angle_Tf);

    // 卡尔曼滤波器
    updateControl('kfMeaE', data.kalman_filter?.mea_e);
    updateControl('kfEstE', data.kalman_filter?.est_e);
    updateControl('kfQ', data.kalman_filter?.q);

    updateControl('lpfSpeed', data.lpf?.speed);
    updateControl('lpfSpeedError', data.lpf?.speed_error);
    updateControl('lpfGyroX', data.lpf?.gyro_x);
    updateControl('lpfControlSteer', data.lpf?.control_steer);
    updateControl('lpfControlSpeed', data.lpf?.control_speed);

    updateControl('K1', data.K.K1);
    updateControl('K2', data.K.K2);
    updateControl('K3', data.K.K3);
    updateControl('K4', data.K.K4);

  }
  // 更新控件
  // 修改点1：增强数值转换逻辑
  function updateControl(id, value) {
    const range = document.getElementById(id);
    const number = document.getElementById(id + 'Val');
    if(range && number) {
      isUpdating = true;

      // 新增科学计数法转换逻辑
      const numValue = typeof value === 'string' ?
              Number(value.replace(/e/g, 'e')) :  // 显式处理科学计数法
              Number(value);

      if(isNaN(numValue)) {
        console.error('Invalid value for', id, ':', value);
        return;
      }

      range.value = numValue;
      number.value = numValue.toFixed(5); // 保证显示为小数格式
      setTimeout(() => isUpdating = false, 50);
    }
  }

  // 显示状态提示
  function showStatus(message, type = "status-success") {
    const statusBar = document.getElementById('statusBar');
    statusBar.textContent = message;
    statusBar.className = `status-bar ${type}`;
    statusBar.style.display = 'block';
    statusBar.style.opacity = '1';

    setTimeout(() => {
      statusBar.style.opacity = '0';
      setTimeout(() => statusBar.style.display = 'none', 300);
    }, 2000);
  }

  // 防抖更新
  let debounceTimer;
  function debounceUpdate() {
    clearTimeout(debounceTimer);
    debounceTimer = setTimeout(sendUpdate, 300);
  }

  // 发送更新
  function sendUpdate() {
    try {
      const params = {
        type: "pid", // 原为"pid_update"，改为与后端匹配的"pid"
        speed: {
          Kp: parseFloat(document.getElementById('speedKp').value),
          Ki: parseFloat(document.getElementById('speedKi').value),
          Kd: parseFloat(document.getElementById('speedKd').value)
        },
        upright: {
          Kp: parseFloat(document.getElementById('uprightKp').value),
          Ki: parseFloat(document.getElementById('uprightKi').value),
          Kd: parseFloat(document.getElementById('uprightKd').value)
        },
        turn: {
          Kp: parseFloat(document.getElementById('turnKp').value),
          Ki: parseFloat(document.getElementById('turnKi').value),
          Kd: parseFloat(document.getElementById('turnKd').value)
        },
        mpu: {
          offset: parseFloat(document.getElementById('mpu_offset').value)
        },
        motor_pid: {
          CS_P: parseFloat(document.getElementById('motorCsP').value),
          CS_I: parseFloat(document.getElementById('motorCsI').value)
        },
        lpf: {
          velocity_Tf: parseFloat(document.getElementById('lpfVelocityTf').value),
          current_d_Tf: parseFloat(document.getElementById('lpfCurrentDTf').value),
          current_q_Tf: parseFloat(document.getElementById('lpfCurrentQTf').value),
          angle_Tf: parseFloat(document.getElementById('lpfAngleTf').value),
          speed: parseFloat(document.getElementById('lpfSpeed').value),
          speed_error: parseFloat(document.getElementById('lpfSpeedError').value),
          gyro_x: parseFloat(document.getElementById('lpfGyroX').value),
          control_steer: parseFloat(document.getElementById('lpfControlSteer').value),
          control_speed: parseFloat(document.getElementById('lpfControlSpeed').value)
        },
        kalman_filter: {
          mea_e: parseFloat(document.getElementById('kfMeaE').value),
          est_e: parseFloat(document.getElementById('kfEstE').value),
          q:     parseFloat(document.getElementById('kfQ').value)
        },
        K:{
          K1:parseFloat(document.getElementById('K1').value),
          K2:parseFloat(document.getElementById('K2').value),
          K3:parseFloat(document.getElementById('K3').value),
          K4:parseFloat(document.getElementById('K4').value)
        }
      };
      console.log("Sending PID Update:", params); // 调试日志
      // 判断 WebSocket 是否处于连接状态
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(params));
        showStatus("参数已保存", "status-success");
      } else {
        // 如果 WebSocket 没有连接，显示错误信息
        showStatus("未连接", "status-error");
      }

    } catch (e) {
      showStatus("保存失败: " + e.message, "status-error");
    }
  }

  // 新增：数值类型验证

  // WebSocket消息处理
  ws.onmessage = (event) => {
    try {
      const rawData = event.data;
      console.log('原始数据:', rawData); // 调试输出原始数据

      const data = JSON.parse(rawData);
      console.log('解析后数据:', data); // 调试输出解析结果

      if(data.type === "pid_data") {
        // 新增字段完整性检查
        if(!data.speed || !data.upright || !data.turn) {
          throw new Error("PID数据结构不完整");
        }

        updateControls(data);
        showStatus("获取数据成功", "status-success");
      }
    } catch (e) {
      console.error('完整错误信息:', e);
      showStatus(`数据解析失败: ${e.message}`, "status-error");
    }
  };

  // 初始化
  window.onload = () => {
    // 初始化所有控件同步
    initSync('speedKp', 'speedKpVal');
    initSync('speedKi', 'speedKiVal');
    initSync('speedKd', 'speedKdVal');
    initSync('uprightKp', 'uprightKpVal');
    initSync('uprightKi', 'uprightKiVal');
    initSync('uprightKd', 'uprightKdVal');
    initSync('turnKp', 'turnKpVal');
    initSync('turnKi', 'turnKiVal');
    initSync('turnKd', 'turnKdVal');
    initSync('mpu_offset', 'mpu_offsetVal');
    initSync('motorCsP', 'motorCsPVal');
    initSync('motorCsI', 'motorCsIVal');
    initSync('lpfVelocityTf', 'lpfVelocityTfVal');
    initSync('lpfCurrentDTf', 'lpfCurrentDTfVal');
    initSync('lpfCurrentQTf', 'lpfCurrentQTfVal');
    initSync('lpfAngleTf', 'lpfAngleTfVal');
    initSync('kfMeaE', 'kfMeaEVal');
    initSync('kfEstE', 'kfEstEVal');
    initSync('kfQ', 'kfQVal');
    initSync('lpfSpeed', 'lpfSpeedVal');
    initSync('lpfSpeedError', 'lpfSpeedErrorVal');
    initSync('lpfGyroX', 'lpfGyroXVal');
    initSync('lpfControlSteer', 'lpfControlSteerVal');
    initSync('lpfControlSpeed', 'lpfControlSpeedVal');
    initSync('K1', 'K1Val');
    initSync('K2', 'K2Val');
    initSync('K3', 'K3Val');
    initSync('K4', 'K4Val');

    // 同步按钮事件
    document.getElementById('syncButton').addEventListener('click', fetchCurrentParams);

    // WebSocket连接
    ws.onopen = () => {
      fetchCurrentParams();
      showStatus("连接成功", "status-success");
    };

    ws.onerror = (error) => {
      showStatus("连接错误", "status-error");
    };

    ws.onclose = () => {
      showStatus("连接已断开", "status-error");
    };
  };
</script>
</body>
</html>
)rawliteral";


