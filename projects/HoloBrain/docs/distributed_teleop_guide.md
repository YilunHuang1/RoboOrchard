# 分布式遥操作部署指南（PC + 机器狗 S100）

## 架构概览

```
PC (Docker内)                         机器狗 S100 (宿主机)
┌──────────────────────┐   WiFi/LAN   ┌──────────────────┐
│ master_controller    │              │ left_controller   │
│   (读主臂 CAN)       │              │   (控从臂 CAN)    │
│   pub: /master/joint │   ROS2 DDS   │   sub: joint_cmd  │
│                      │ ──────────→  │                   │
│ takeover_muxer       │              │                   │
│   pub: /joint_cmd    │              │                   │
└──────────────────────┘              └──────────────────┘
```

## 前置条件

- PC 和机器狗在**同一局域网**
- PC：已按原 README 完成 Docker + RoboOrchard 安装
- 机器狗 S100：Ubuntu 22.04 + ROS2 Humble (arm64)

---

## 第一步：ROS2 多机通信配置

PC 和机器狗需设置相同的 `ROS_DOMAIN_ID`，确保 DDS 能互相发现。

### 两端都执行：

```bash
export ROS_DOMAIN_ID=42   # 选一个 0-232 之间的数字，两边一致即可
```

### PC Docker 内额外设置

Docker 默认网络隔离会阻止 DDS 发现。启动 docker 时需使用 `--network host`：

```bash
# 在你的 launch/docker.sh 中确认有 --network host 参数
docker run --network host ...
```

> 如果已经在用 `--network host`，则无需修改。

### 验证连通性

在 PC Docker 内：
```bash
export ROS_DOMAIN_ID=42
ros2 topic pub /test std_msgs/msg/String "data: hello" --once
```

在机器狗上：
```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /test
```

如果能收到消息，说明多机通信已就绪。

---

## 第二步：机器狗 S100 环境准备

### 2.1 安装 piper_sdk

```bash
pip install piper_sdk
```

> 如果 pip 安装 arm64 版本失败，需从源码编译：
> ```bash
> git clone https://github.com/agilexrobotics/piper_sdk.git
> cd piper_sdk
> pip install .
> ```

### 2.2 安装 RoboOrchard ROS2 包（最小集）

机器狗上只需要以下 5 个 ROS2 包（从臂控制所需的最小依赖）：

```
robo_orchard_piper_msg_ros2   ← 自定义消息定义（cmake包，必须先构建）
robo_orchard_piper_ros2       ← single_ctrl 可执行文件（从臂控制器）
robo_orchard_pico_msg_ros2    ← teleop 的依赖消息
robo_orchard_teleop_msg_ros2  ← teleop 的依赖消息
robo_orchard_teleop_ros2      ← slave launch 文件
```

**将 ros2_package 目录同步到机器狗（只同步这 5 个包）：**

```bash
# 在 PC 上执行，将所需包 rsync 到机器狗
rsync -av --progress \
    /path/to/RoboOrchard/ros2_package/robo_orchard_piper_msg_ros2 \
    /path/to/RoboOrchard/ros2_package/robo_orchard_piper_ros2 \
    /path/to/RoboOrchard/ros2_package/robo_orchard_pico_msg_ros2 \
    /path/to/RoboOrchard/ros2_package/robo_orchard_teleop_msg_ros2 \
    /path/to/RoboOrchard/ros2_package/robo_orchard_teleop_ros2 \
    user@<robot_dog_ip>:/home/user/robo_orchard_slave/
```

**在机器狗上构建：**

```bash
# SSH 到机器狗
ssh user@<robot_dog_ip>

source /opt/ros/humble/setup.bash
cd /home/user/robo_orchard_slave

# 安装 Python 依赖
pip install piper_sdk scipy numpy

# 构建（只构建这 5 个包，colcon 会自动按依赖顺序构建）
colcon build --packages-select \
    robo_orchard_piper_msg_ros2 \
    robo_orchard_pico_msg_ros2 \
    robo_orchard_teleop_msg_ros2 \
    robo_orchard_piper_ros2 \
    robo_orchard_teleop_ros2

# 验证构建成功
source install/setup.bash
ros2 pkg list | grep robo_orchard
# 应看到上述 5 个包
```

> **注意**：每次 PC 侧更新了代码，在机器狗上重新 rsync + colcon build 即可，不需要完整重装。

### 2.3 配置从臂 CAN

在机器狗上，将从臂的 CAN-USB 适配器插入 USB 口，然后：

```bash
# 查看 CAN 设备
ip link show type can

# 如果需要重命名 CAN 接口（根据实际情况修改）
sudo ip link set can0 down
sudo ip link set can0 name can_left
sudo ip link set can_left type can bitrate 1000000
sudo ip link set can_left up
```

或者直接使用默认的 can0，启动时指定端口即可。

---

## 第三步：启动

### 3.1 机器狗端（先启动）

```bash
# SSH 到机器狗
ssh user@<robot_dog_ip>

# 环境配置
source /opt/ros/humble/setup.bash
source /home/user/robo_orchard_slave/install/setup.bash
export ROS_DOMAIN_ID=42

# 确认从臂 CAN 接口已 up
ip link show type can   # 查看实际名称，如 can0

# 启动从臂控制器（根据实际 CAN 端口名修改 left_slave_can_port）
ros2 launch robo_orchard_teleop_ros2 piper_dagger_slave.launch.py \
    left_slave_can_port:=can0
```

或者直接用脚本（先把 dagger_slave.sh 拷贝到机器狗）：

```bash
# 把启动脚本也同步过去
rsync -av /path/to/RoboOrchard/projects/HoloBrain/teleop/dagger_slave.sh \
    user@<robot_dog_ip>:/home/user/

# 在机器狗上
bash dagger_slave.sh
```

### 3.2 PC 端

**方式 A：使用分布式 launch.yaml（一键启动）**

```bash
# 1. 拷贝分布式配置模板
cp launch/templates/launch_distributed.yaml launch/launch.yaml

# 2. 修改 DOCKER_ROBO_ORCHARD_PATH 等变量

# 3. 确保 Docker 启动时设置了 ROS_DOMAIN_ID=42

# 4. 启动
./launch/start.sh
```

**方式 B：手动启动（调试用）**

```bash
# Docker 内
export ROS_DOMAIN_ID=42
source /path/to/venv/bin/activate
source /path/to/RoboOrchard/ros2_package/install/setup.bash

bash teleop/rename-can.sh
bash teleop/dagger_master.sh
```

---

## 故障排查

### Topic 不通
```bash
# 两端分别检查
ros2 topic list
ros2 topic echo /robot/left/joint_cmd  # 机器狗端应能看到数据
```

### DDS 发现问题

如果默认的 Simple Discovery 不工作（跨子网等），可以用 FastDDS Discovery Server：

**PC 端（作为 Discovery Server）：**
```bash
fastdds discovery -i 0 -l <PC_IP> -p 11811
export ROS_DISCOVERY_SERVER=<PC_IP>:11811
```

**机器狗端：**
```bash
export ROS_DISCOVERY_SERVER=<PC_IP>:11811
```

### 延迟优化

当前 sync 频率 200Hz，跨网络后如遇延迟问题：

1. 确认使用**有线连接**（WiFi 延迟不稳定）
2. 可降低 sync_frequency 至 100Hz
3. 考虑使用 Zenoh 替代 DDS（更适合不稳定网络）：
   ```bash
   # 两端都安装
   apt install ros-humble-rmw-zenoh-cpp
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   ```

---

## 回退到本地模式

如需回退到两个臂都接 PC 的模式，只需恢复原来的 `launch.yaml`：

```bash
# 重新拷贝原始模板
cp launch/templates/launch.yaml launch/launch.yaml
# 修改后启动即可，原 dagger.sh 不受影响
```
