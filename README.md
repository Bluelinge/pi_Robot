# pi_robot

运行在 `Raspberry Pi OS (64-bit)` 上的树莓派主控服务。  
它负责把 `D435i` 相机、网页控制台、地图/标定能力和 `ESP32` 小车底盘控制整合到一起。

## 当前功能

- 通过 `UART` 控制 ESP32 小车
- 接入 `Intel RealSense D435i`
- 提供彩色流、深度伪彩色流、点击测距
- 网页手动控制四驱、电机速度、单舵机、舵机组
- 标定、地图加载与基础导航数据结构
- 目标跟随相关实验代码保留在仓库中

另外还提供一个独立相机页：

```bash
python -m pi_robot.camera_viewer
```

默认端口：

```text
http://<raspberry-pi-ip>:8001
```

## 目录结构

- `pi_robot/`: Python 主程序
- `data/`: 地图与电机参数
- `deploy/`: systemd 服务文件
- `scripts/`: 安装脚本
- `tests/`: 基础测试

## 启动主控

### 本地环境

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
python -m pi_robot.main
```

默认网页：

```text
http://<raspberry-pi-ip>:8000
```

## 主要环境变量

复制 `.env.example` 为 `.env` 后，重点确认：

- `PI_ROBOT_HOST`
- `PI_ROBOT_PORT`
- `PI_ROBOT_SERIAL_PORT`
- `PI_ROBOT_SERIAL_BAUD`
- `PI_ROBOT_COLOR_WIDTH`
- `PI_ROBOT_COLOR_HEIGHT`
- `PI_ROBOT_DEPTH_WIDTH`
- `PI_ROBOT_DEPTH_HEIGHT`
- `PI_ROBOT_CAMERA_FPS`
- `PI_ROBOT_REALSENSE_REQUIRE_IMU`

当前默认串口主链路为：

```env
PI_ROBOT_SERIAL_PORT=/dev/serial0
PI_ROBOT_SERIAL_BAUD=115200
```

## 部署到树莓派

### 1. 同步项目

```powershell
scp -r "E:\AI\Codex\RoBot\Project_Develop\pi_robot" admin@<树莓派IP>:/home/admin/
```

### 2. 登录并安装依赖

```bash
ssh admin@<树莓派IP>
source ~/rsenv312/bin/activate
cd ~/pi_robot
python -m pip install -r requirements.txt
```

### 3. 启动

```bash
python -m pi_robot.main
```

## 与 ESP32 的连接方式

当前推荐通过树莓派硬件串口接 ESP32：

- 树莓派 `GPIO14 TXD`
- 树莓派 `GPIO15 RXD`
- 树莓派 `GND`

接到 ESP32：

- `GPIO27 RX2`
- `GPIO19 TX2`
- `GND`

接线关系：

- 树莓派 `TXD` -> ESP32 `RX2`
- 树莓派 `RXD` -> ESP32 `TX2`
- `GND` 共地

## 关键源码

- 主入口：[pi_robot/main.py](/E:/AI/Codex/RoBot/Project_Develop/pi_robot/pi_robot/main.py)
- Web 主服务：[pi_robot/app.py](/E:/AI/Codex/RoBot/Project_Develop/pi_robot/pi_robot/app.py)
- 串口适配器：[pi_robot/control/car0513_adapter.py](/E:/AI/Codex/RoBot/Project_Develop/pi_robot/pi_robot/control/car0513_adapter.py)
- 相机适配：[pi_robot/camera/realsense.py](/E:/AI/Codex/RoBot/Project_Develop/pi_robot/pi_robot/camera/realsense.py)

