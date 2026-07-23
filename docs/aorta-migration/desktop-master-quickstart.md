# Aorta 主机端快速使用指南

本文说明如何在 PC 主机上部署并运行 Aorta 左臂主从控制。S100 从机需已运行
`robo_orchard_slave.service`。

## 1. 前置条件

- PC 架构为 `x86_64`，已连接左主臂 USB-CAN。
- 可以通过 `ssh sh-106-s100` 登录 S100。
- S100 与 PC 使用相同版本的 Aorta SDK 和消息定义。
- 已有 RoboOrchard Python 环境：
  `docker_env/venv/robot-venv/bin/python`。
- 不要同时运行 ROS 主臂控制脚本和本 Aorta 控制栈。

## 2. 首次部署

准备以下两个 wheel：

```text
aorta_sdk-2026.7.22-py3-none-linux_x86_64.whl
aorta_msgs-2026.7.22-py3-none-any.whl
```

假设 wheel 位于 `~/Downloads`，执行：

```bash
cd /home/vita-4090/project_yilun_copy

docker_env/venv/robot-venv/bin/python -m pip install \
  --target docker_env/aorta-runtime \
  ~/Downloads/aorta_sdk-2026.7.22-py3-none-linux_x86_64.whl \
  ~/Downloads/aorta_msgs-2026.7.22-py3-none-any.whl
```

验证安装：

```bash
PYTHONPATH=$PWD/docker_env/aorta-runtime \
  docker_env/venv/robot-venv/bin/python -c \
  "import aorta, arm_joint_state_schema_meta, arm_trigger_schema_meta; print('Aorta import OK')"
```

## 3. 启动

确认主臂已上电、急停可用，然后执行：

```bash
cd /home/vita-4090/project_yilun_copy/RoboOrchard/projects/HoloBrain
bash teleop/start_aorta_master.sh
```

该脚本会自动：

1. 检查并初始化 `can_left_mst`。
2. 从 S100 获取 Zenoh 配置并建立 SSH 隧道。
3. 启动接管 muxer 和主臂 Aorta 节点。
4. 保持安全的 `AUTONOMOUS` 模式，不会立即联动从臂。

## 4. 开始联动

手动将主臂与从臂调整到接近的位置，然后执行：

```bash
bash teleop/trigger_aorta_takeover.sh
```

脚本会检查六个关节的位置差异。最大差异超过 **5°** 时会拒绝接管；检查通过后
切换为 `OVERRIDE`，从臂开始跟随主臂。

## 5. 停止

正常停止使用：

```bash
bash teleop/stop_aorta_master.sh
```

脚本会先请求 muxer 进入 `STOP`，再停止主机端主臂节点、muxer 和 SSH 隧道。
S100 上的从臂服务保持运行，但不会再收到主机命令。

紧急情况下应先使用硬件急停，再执行停止脚本。

## 6. 日志与常见故障

日志目录：

```bash
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/robo-orchard-aorta-${UID}"
ls -l "$RUNTIME_DIR"
```

主要日志：

- `master.log`：主臂 CAN 与 Aorta 节点。
- `muxer.log`：接管状态和命令转发。
- `tunnel.log`：PC 到 S100 的 SSH/Zenoh 隧道。
- `stop.log`：停止 RPC。

常见检查：

```bash
# S100 是否可登录
ssh sh-106-s100 true

# 主臂 CAN 是否存在且为 UP
ip link show can_left_mst

# S100 从臂服务状态
ssh sh-106-s100 systemctl status robo_orchard_slave.service
```

若提示端口 `17447` 被占用或控制栈已经运行，先执行
`bash teleop/stop_aorta_master.sh`，确认旧进程退出后再启动。
