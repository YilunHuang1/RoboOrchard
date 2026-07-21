# RoboOrchard 机械臂主从：ROS2 → Aorta 迁移方案

> 状态：草案 v1（2026-07-21）· 覆盖仓库：`RoboOrchard`（master 侧，桌面 PC）与 `VitaDynamics/vita-robot@feature/slave-arm-deploy`（slave 侧，S100）

---

## 0. 一页速览（TL;DR）

- **两个仓库的关系**：`vita-robot` 的 `feature/slave-arm-deploy` 分支把 RoboOrchard 的 `ros2_ws` 子集（`piper` + `teleop` + `pico` 三组包）**vendored** 到 `src/application/robo_orchard/ros2_ws/`，并用 systemd 在 **S100 上跑 slave 臂**；**master 臂在桌面 PC 上跑 RoboOrchard 原仓库**。两侧通过 **ROS2 DDS**（实为 `rmw_zenoh`）通信。
- **主从之间真正跨主机的只有一个话题**：`/robot/left/joint_cmd`（`sensor_msgs/JointState`，master→slave）。其余 `/master/*`、`/puppet/*` 状态话题基本是本机可观测量。
- **传输层已经是 Zenoh**：ROS2 用 `rmw_zenoh_cpp`，和 Aorta 共用同一个 `zenohd` router 进程，仅靠 keyspace 隔离（`@ros2_lv/**` vs `aorta/**`）。**云端 / 跨主机 ACL 明确 `deny @ros2_lv/**`、`allow aorta/**`** —— 这是迁移到 Aorta 最强的业务动机：ROS2 流量出不了本机网段，Aorta 才是一等公民。
- **Aorta 有 Python SDK**（`VitaDynamics/aorta` 的 `python/aorta/`，纯 ctypes over `libaorta_core`，底层 Zenoh），与 rclpy 能力基本对齐：Node / Pub-Sub（原始字节 + FlatBuffers typed）/ Service-Client / Timer / Executor / ExecutionGroup / ContextStore / Action。**RoboOrchard 全是 Python，因此可平滑迁移，无需换语言。**
- **迁移本质**：把 rclpy 的 `create_publisher/subscription/service/timer` 换成 aorta 等价物，把 `.msg/.srv` 换成 **FlatBuffers `.fbs`**；CAN（`piper_sdk`）、HTTP、VR SDK、键盘、MCAP 等非 ROS 传输**完全不动**。
- **已决定的路径（one-shot）**：范围只限**主从臂 demo**，直接重写成 **aorta-only（不做双栈/桥）**，一次 schema release + 一个代码 PR 切换，保留**唯一一个真机验证闸门**，通过后再删 ROS2 代码（可回滚）。详见 §4。

---

## 1. 当前架构全景

### 1.1 仓库与角色

| 仓库 | 语言/构建 | 角色 | 运行位置 |
|---|---|---|---|
| `RoboOrchard`（本仓库，`YilunHuang1/RoboOrchard`） | Python / colcon，ROS2 Humble | **master 臂 + teleop + 推理部署 + 数据录制**框架 | 桌面 PC |
| `vita-robot@feature/slave-arm-deploy` | Bazel monorepo（C++/Rust + Python） | vendored `ros2_ws` 子集 + **slave 臂** systemd 部署；**aorta 中间件所在仓库** | S100（机器狗算力板，ARM64） |

vendored 子集（只搬了主从臂控制相关，**没有** deploy/data/handeye）：
`robo_orchard_pico_msg_ros2`、`robo_orchard_piper_msg_ros2`、`robo_orchard_piper_ros2`、`robo_orchard_teleop_msg_ros2`、`robo_orchard_teleop_ros2`。
导入基线：RoboOrchard commit `4d10424`。

### 1.2 主从拓扑（分布式 teleop / DAgger）

```
桌面 PC（RoboOrchard 原仓库）                     S100（vita-robot vendored）
┌───────────────────────────────────────┐        ┌──────────────────────────────────┐
│  master 臂 (CAN can_left_mst)          │        │  slave 臂 (CAN can0/can_left)      │
│    │ piper_sdk 读关节                   │        │            ▲ piper_sdk 写关节      │
│    ▼                                    │        │            │                       │
│  single_ctrl (single.py)               │        │  single_ctrl (single.py)          │
│    ns=/robot/left_master               │        │    ns=/robot/left                 │
│    P: /master/joint_left  (JointState) │        │    node=robot_left_controller     │
│    P: /master/status_left (PiperStatus)│        │    S: /robot/left/joint_cmd  ◀──┐  │
│    P: /master/end_pose_left(PoseStamped)        │    P: /puppet/joint_left        │  │
│    │                                    │        │    P: /puppet/status_left       │  │
│    ▼ override_topic=/master/joint_left │        │    P: /puppet/end_pose_left     │  │
│  take_over (muxer)                      │        │    Svc: enable_ctrl/reset_ctrl  │  │
│    ns=/robot/left/takeover_muxer       │        └─────────────────────────────────┼──┘
│    S: /left_algo_cmd  (autonomous 输入)│                                          │
│    S: /master/joint_left (override)    │        跨主机唯一链路（现 ROS2 DDS）      │
│    P: output=/robot/left/joint_cmd ────┼════════ /robot/left/joint_cmd ══════════┘
│    P: control_mode / events            │        （sensor_msgs/JointState）
│    Svc: trigger_takeover/release/stop  │
└───────────────────────────────────────┘

muxer 三态：AUTONOMOUS（转发 /left_algo_cmd 模型指令）
           OVERRIDE  （转发 /master/joint_left 遥操主臂）→ 触发时回放 replay_time_s 前的历史
           STOP      （保持）
```

`single.py` 是**同一份代码**，靠 launch 的 remap/参数区分 master / slave 两种角色——这意味着**迁移这一个文件就同时迁移了主从两端**。

### 1.3 完整 ROS 端点清单（launch remap 后的全局名）

**话题**

| 全局话题 | 类型 | 发布 / 订阅 | 是否跨主机 |
|---|---|---|---|
| `/robot/left/joint_cmd`（+right） | `JointState` | muxer(P) / slave single_ctrl(S)、master replay(S)、VR teleop(P) | **是（唯一）** |
| `/master/joint_left`（+right） | `JointState` | master single_ctrl / aloha / aloha_raw(P)；muxer override(S) | 否（本机） |
| `/master/status_left`、`/master/end_pose_left` | `PiperStatusMsg` / `PoseStamped` | master(P) | 否 |
| `/puppet/joint_left`、`/puppet/status_left`、`/puppet/end_pose_left` | `JointState` / `PiperStatusMsg` / `PoseStamped` | slave(P) | 否（若集中录制则需） |
| `/left_algo_cmd`（+right） | `JointState` | deploy ActionExecutor(P) / muxer algo(S) | 否 |
| `.../control_mode` | `ControlMode` | muxer(P) | 否 |
| `.../events` | `TakeOverEvent` | muxer(P) | 否 |
| `/pico_bridge/vr_state` | `VRState` | pico_bridge(P) / trigger、VR teleop(S) | 否 |
| `robot/{l,r}/ee_pose_target` | `PoseStamped` | VR teleop(P) | 否 |
| deploy 观测（color/depth/intrinsic/arm_state） | Image/Image/CameraInfo/JointState | deploy obs(S，动态) | 否 |

**服务**（除注明外均 `std_srvs/Trigger`）

| 全局服务 | Server | Client |
|---|---|---|
| `/robot/{l,r}/enable_ctrl` | single_ctrl | — |
| `/robot/{l,r}/reset_ctrl` | single_ctrl | deploy 配置引用 |
| `.../takeover_muxer/trigger_takeover` | muxer | pico_vr_trigger、keyboard_trigger |
| `.../takeover_muxer/release_control` | muxer | 同上 |
| `.../takeover_muxer/stop` | muxer | keyboard_trigger |
| `/robot/inference_service/enable`、`/disable` | deploy 节点 | — |
| `mcap_recorder_service/start_recording` | mcap 录制 | 外部（`StartRecording.srv`） |
| `mcap_recorder_service/stop_recording` | mcap 录制 | 外部 |

**Action**：全仓库**没有** ROS2 `.action` 接口。deploy 里的“action”是模型指令载荷，不是 ROS action。

**QoS 现状**：除 mcap 录制外**没有任何自定义 QoS**——控制/状态话题一律 `depth=1 KEEP_LAST`（latest-wins），`vr_state`/`control_mode`/`events`/图像用 `depth=10`。默认 `RELIABLE / VOLATILE`。**没有 `transient_local` 锁存**。

**非 ROS 传输（迁移不触碰，保持原样）**：
1. **CAN** — `piper_sdk.C_PiperInterface`（所有臂 I/O，`ros_bridge.py`）
2. **HTTP/REST** — `requests.post` 到模型 server（`deploy/model_request.py`）
3. **Pico VR SDK** — `xrobotoolkit_sdk`（`bridge/pico/node.py`）
4. **键盘** — `sshkeyboard`（`take_over/trigger/keyboard.py`）
5. **MCAP 文件** — `rosbag2_py`（`data_ros2/mcap`）

**rclpy 细节（迁移需还原语义）**：
- Executor：仅 deploy 节点用 `MultiThreadedExecutor`；其余全是单线程 `rclpy.spin`。
- Callback group：`MutuallyExclusive`（deploy 推理/动作分离）、`Reentrant`（mcap）。
- Timer：遍地都是（piper 200Hz 控制+发布；VR fps；IK 10Hz；deploy 推理/控制频率；1Hz 发现）。
- 动态消息类型：muxer 用 `rosidl_runtime_py.get_message`、deploy/mcap 用 `__import__` 按运行时话题名解析类型——**Aorta 需要静态 schema，动态类型必须替换成显式 schema**。
- `message_filters.ApproximateTimeSynchronizer`：deploy obs 与 handeye 用到——**Aorta 无等价物，需自实现按时间戳对齐**（仅影响 deploy/handeye，不在主从核心路径）。

---

## 2. Aorta 能力与关键差异

Aorta Python SDK（`aorta` 包）与 rclpy 的对照：

| 概念 | rclpy | aorta（Python） |
|---|---|---|
| 节点 | `rclpy.node.Node` | `aorta.Node(name, group)`（**`group` 参数当前被忽略，用 `AORTA_GROUP` 环境变量**） |
| 发布 | `create_publisher(T, topic, depth)` | `node.create_publisher_typed(schema, topic, qos=...)`，`pub.publish_typed(fill)` / `publish_bytes` |
| 订阅（回调） | `create_subscription(T, topic, cb, depth)` | `node.create_subscriber_typed(decoder, topic, cb)`（回调跑在 **Zenoh worker 线程**） |
| 订阅（latest-wins） | `depth=1` keep-last | **`node.create_subscriber_pull(topic, depth)`** → 自己 `recv()`/`try_recv()` 拉取，不走 executor 线程 |
| 服务 | `create_service(Srv, name, cb)` | `node.create_service_typed_view(decoder, name, cb)` |
| 客户端 | `create_client(Srv, name)` | `node.create_client_typed(name, decoder)`，`client.call(fill)` |
| 定时器 | `create_timer(period, cb)` | `node.create_timer(period, cb)` |
| 执行器 | `MultiThreadedExecutor` + callback group | `aorta.Executor(MULTI_THREADED)` + `node.create_execution_group(SERIALIZED/CONCURRENT)` |
| 时钟 | `get_clock().now()` | `node.now()`（纳秒） |
| Header/stamp | `std_msgs/Header` | **typed publish 自动注入 `AortaHeader`**（每 publisher 单调序号 + 节点名 + Core 时间戳） |
| tf2 | `TransformBroadcaster` | 用 foxglove `FrameTransform`/`FrameTransforms` schema 发普通话题 |
| 长任务 | （本仓库无 action） | `Action`（有 goal/feedback/result/cancel，本次可不用） |

**必须警惕的语义差异**：

1. **没有 durability / `transient_local` / `history_depth`**（都被移除）。“给晚加入的订阅者最后一个值”改用 **`ContextStore`**（声明式锁存状态）。→ 本仓库控制路径无锁存需求；`control_mode` 是 1Hz 周期发布，也不需要锁存。**唯一要留意**：若希望 slave/UI 一连上就拿到当前 mode，用 ContextStore。
2. **QoS 简化为 4 个预设**：`realtime_control()`（电机指令，本项目 `joint_cmd` 用它）、`sensor_data()`（可丢弃）、`state_update()`（状态机/配置）、`bulk_transfer()`（日志/大文件）。可选 `CongestionControl.BLOCK`（不静默丢，背压）vs `DROP`（低延迟有损）。
3. **typed 路径基于 FlatBuffers**：build 用 `fill(builder,...)` 回调、read 用 `root_decoder`，没有可变消息对象。标准 ROS 类型（`JointState`/`PoseStamped`/`Header`）都要重新定义为 `.fbs`。
4. **回调线程模型**：subscriber/service 回调在 Zenoh worker 线程执行。**CAN 写入不是线程安全的**——见 §5 风险。
5. **schema 分层**：Tier 1（aorta 仓库，跨仓库/跨主机契约，需 PR→release→在 vita-robot `MODULE.bazel` pin）；Tier 2（模块本地 `aorta_fbs_library`）。

**Aorta 已有、可复用的 schema**：foxglove 的 `Pose/Quaternion/Point3/Vector3/Time/FrameTransform`（用于 `ee_pose`）、`lowlevel/low_cmd/low_state`（人形关节，可参考但字段不完全匹配机械臂）。**没有**现成的机械臂 `JointState`/`PiperStatus`，需自定义。

---

## 3. Schema 策略（FlatBuffers）

因为 master（RoboOrchard 仓库）与 slave（vita-robot 仓库）**跨仓库 + 跨主机**共享同一契约，按 Aorta 规则，跨界 schema 必须放 **Tier 1（aorta 仓库）**，两侧 pin 同一 release，消费同一份规范定义。

建议在 `aorta/message/schemas/topic/arm/` 新增：

| `.fbs` | 对应 ROS 类型 | 字段要点 |
|---|---|---|
| `arm_joint_command.fbs` | `sensor_msgs/JointState`（joint_cmd） | `stamp`、`name[]`、`position[]`、`velocity[]`、`effort[]`、`gripper` |
| `arm_joint_state.fbs` | `sensor_msgs/JointState`（joint_state/master/puppet） | 同上 |
| `arm_ee_pose.fbs` | `geometry_msgs/PoseStamped` | `stamp` + 复用 foxglove `Pose` |
| `piper_status.fbs` | `PiperStatusMsg` | 20 字段照搬（`ubyte ctrl_mode/arm_status/...`、`long err_code`、各关节 limit/comm `bool`）|
| `control_mode.fbs` | `ControlMode` | `stamp` + `string data`（或改 enum） |
| `takeover_event.fbs` | `TakeOverEvent` | `stamp` + `event_type` + `details`（常量改 enum）|
| `vr_state.fbs`（后期） | `VRState`/`Head`/`*Controller` | 供 VR teleop 路径 |

服务用 `aorta/message/schemas/service/arm/`：`enable.fbs`、`reset.fbs`、`takeover_trigger.fbs`（对应 `std_srvs/Trigger`：空请求 / `bool success + string message` 响应，复用 `shared/service_common/status.fbs`）。

> `PosCmd.msg` 和 `Enable.srv` 经确认**已定义但代码未使用**（实际用 `std_srvs/Trigger`），迁移时直接弃用，不必建对应 `.fbs`。

**话题命名**：统一加 `aorta/` 前缀命名空间以通过云端 ACL，例如 `aorta/robot/left/joint_cmd`、`aorta/robot/left/joint_state`。master/slave 需在同一 `AORTA_GROUP`。

### 3.1 单帧大小基准（flatc 实测，含 AortaHeader）
`AortaHeader` 本身 72 B（`ulong stamp` + `ulong seq` + `string publisher_node`；节点名 19 字符时）。各消息实测（`docs/aorta-migration/reference-impl/validate_fill.py`）：

| 消息 | 内容 | 含 header |
|---|---|---|
| ArmJointState（`joint_cmd`/state） | name[7]+position[7]+velocity[6]+effort[7] | **416 B** |
| ArmJointState（精简命令） | 仅 position[7] | ~168 B |
| ArmEePose | Vec3 + Quat | 160 B |
| PiperStatus | 19 字段 | 136 B |
| ArmTriggerResponse | status + message | 120 B |

200 Hz × 416 B ≈ **83 KB/s/臂**，双臂 ~166 KB/s——带宽无压力。网线上另加 Zenoh/TCP 封装 ~60~90 B/帧（与 schema 无关）。

---

## 4. 迁移方案（one-shot，aorta-only）

**决定**：范围 = 主从臂 demo；不做双栈/桥，直接把相关节点重写成 **aorta-only**；一次 schema release + 一个代码 PR 切换；保留**唯一一个真机验证闸门**；验证通过后再删 ROS2 代码（`git revert` 即回滚）。

### 4.0 冻结的迁移表面（就这些，别扩散）
| 文件 | 角色 | 迁移动作 | 保持不变 |
|---|---|---|---|
| `piper_ros2/single.py` | master + slave 单臂控制器（同一份） | pub/sub/service/timer → aorta | `ros_bridge.py`（CAN，`piper_sdk`）全不动 |
| `teleop_ros2/take_over/node.py` | 接管 muxer | pub/sub/service → aorta；状态机、`deque` 回放不动 | 三态逻辑、`replay_time_s` |
| `teleop_ros2/take_over/trigger/keyboard.py` | 接管触发前端 | `Trigger` client → aorta client | `sshkeyboard` 输入源不动 |
| `start_robo_orchard_slave.sh` + `robo_orchard_slave.service` | S100 部署 | 去 `--ros-args` remap，改 aorta env/参数 | CAN 依赖、persistent piper_sdk 逻辑 |

> **不在本刀内**（保持 ROS2 或后续单独处理）：VR 触发 `pico_vr.py` + `pico_bridge`（若 demo 用 VR 接管才纳入）、`aloha*.py`（本机双 CAN，不跨主机）、`deploy`/`data`/`handeye`。

### 4.1 前置 spike（半天，代码不动）
先验证两个最大未知，通过才写业务代码：
1. **PC 入网**：桌面 PC 配 aorta session（`connect tcp/192.168.127.2:7447` 或共享 router）+ `AORTA_GROUP`，确认 ACL 放行 `aorta/**`。
2. **通路**：PC↔S100 用 aorta CLI（`aorta`/`aorta-tui`）跑一条 `aorta/robot/test` pub-sub，量端到端时延——必须与现 rmw_zenoh 基线相当。

### 4.2 PR-A（aorta 仓库）— 一次性提交全部 arm schema
`message/schemas/topic/arm/`：`arm_joint_command.fbs`、`arm_joint_state.fbs`、`arm_ee_pose.fbs`、`piper_status.fbs`、`control_mode.fbs`、`takeover_event.fbs`；`message/schemas/service/arm/`：`enable.fbs`、`reset.fbs`、`takeover.fbs`。全部注册进 `registry.bzl`、`cpp=True, cpp_wrapper=True` + python 绑定 → 评审 → 打 release tag。（本地联调期用 `bazel build --override_module=aorta=/path/to/local/aorta` 免等 release。）

### 4.3 PR-B（RoboOrchard + vita-robot）— 代码切 aorta-only
> **草稿已就绪**：`docs/aorta-migration/reference-impl/`（`arm_bridge.py` = 传输无关 CAN 层；`single_aorta.py` = aorta-only 节点；`validate_fill.py` = fill/decode 已实测通过）。以下映射即该草稿的实现。

- **`single.py`**（端点映射见下表）：`rclpy.spin` → `aorta.Executor(SINGLE_THREADED).spin`；`get_clock().now()` → `node.now()`；**topic 名从「launch remap」改为「节点参数」**（aorta 无 remap 层，slave 用 `/puppet/*`、master 用 `/master/*` 由参数注入）。

  | ROS2（现状） | aorta（目标） | QoS |
  |---|---|---|
  | `create_publisher(JointState,"joint_state",1)` | `create_publisher_typed(ArmJointState, <joint_state_topic>)` | `realtime_control()` |
  | `create_publisher(PiperStatusMsg,"status",1)` | `create_publisher_typed(PiperStatus, <status_topic>)` | `state_update()` |
  | `create_publisher(PoseStamped,"ee_pose",1)` | `create_publisher_typed(ArmEePose, <ee_pose_topic>)` | `realtime_control()` |
  | `create_subscription(JointState,"joint_cmd",cb,1)` | **`create_subscriber_pull(<joint_cmd_topic>, 1)`**，在 200Hz timer 里 `try_recv()` | — |
  | `create_service(Trigger,"enable_ctrl",cb)` | `create_service_typed_view(dec, <enable_topic>, cb)` | — |
  | `create_service(Trigger,"reset_ctrl",cb)` | `create_service_typed_view(dec, <reset_topic>, cb)` | — |
  | `create_timer(1/200, publish_callback)` | `create_timer(1/200, publish_callback)` | — |

- **`take_over/node.py`**（草稿 `reference-impl/take_over_aorta.py` 已就绪，fill 已实测）：muxer 是纯**路由器**，命令流用**原始字节透传**（`publish_bytes`，不解码 JointState，保留 master 原始 header，200Hz 零分配）；`control_mode`/`events` 用 `create_publisher_typed`；`algo`/`override` 用**回调订阅**并全部绑到 `ExecutionGroup(SERIALIZED)`（复现单线程 spin，`_current_mode`/`_history` 无竞争，且转发零延迟）；`trigger_takeover`/`release_control`/`stop` → `create_service_typed_view`。动态 `message_type` 参数删除，固定 `ArmJointState`。Time/Duration → `node.now()`（int ns）。行为逐条保留（含 STOP 实际不发零命令的原始怪癖）。
- **`keyboard.py`**：per-service `Trigger` client → `create_client_typed`。
- **systemd**：`start_robo_orchard_slave.sh` 去掉 `-r /robot/left/status:=/puppet/status_left` 等 remap，改成 `--joint-state-topic=aorta/puppet/joint_left` 之类的参数 + `AORTA_GROUP` / `AORTA_CORE_FFI_LIB` / session env；`sd_notify(READY=1)` 在 CAN 与 aorta 都就绪后再发。vita-robot `MODULE.bazel` pin PR-A 的 release。
- **ROS2 代码此刻先不删**（`*_msg_ros2`、旧 launch 留在树里作回滚路径）。

### 4.4 验证闸门（真机，唯一 gate）
真臂上电，逐条过：① master 遥操 → slave 跟随，端到端时延/抖动 ≈ ROS2 基线；② `enable_ctrl`/`reset_ctrl`（3s 归零插值）正常；③ 键盘 `trigger_takeover`/`release_control`/`stop` 三态切换正常；④ 断连/重连行为可接受；⑤ 连续跑 N 分钟无 CAN 竞争/丢帧。

### 4.5 PR-C（gate 通过后）— 删 ROS2
删 vendored `*_msg_ros2` 接口包、旧 rclpy 后端、旧 `--ros-args` launch；arm 路径移除 `rmw_zenoh` 依赖；`ros2_ws` 就此清空。arm 流量全走 `aorta/**` keyspace，天然获得跨主机 / 上云能力。

---

## 5. 风险与注意事项

| # | 风险 | 影响 | 缓解 |
|---|---|---|---|
| 1 | **回调线程 vs CAN 非线程安全** | aorta 订阅回调在 Zenoh worker 线程，若直接在回调里写 CAN，会与 200Hz timer 的 CAN 读/写竞争 | **slave 的 `joint_cmd` 用 pull-subscriber**，在 timer 线程内拉取；或对 `piper` 句柄加 mutex / 用 `ExecutionGroup(SERIALIZED)` |
| 2 | **无 transient_local 锁存** | 晚加入者拿不到最后值（如 mode 初值） | 控制路径无需；`control_mode` 周期发布已足够；确需锁存改 `ContextStore` |
| 3 | **FlatBuffers 代码生成** | 增加构建步骤；标准 ROS 类型需重定义 | 复用 foxglove `Pose` 等；Tier1 一次定义两仓库共用 |
| 4 | **跨仓库 schema 协同** | 首次改契约要 aorta PR→release→pin，链路长 | 先冻结 `.fbs`，本地用 `--override_module=aorta=/path` 联调 |
| 5 | **桌面 PC 入网 aorta/zenoh** | PC 不在 S100/X5 既有拓扑内，需显式配 session + group + ACL | Phase 0 先用 aorta CLI 验证 PC↔S100 `aorta/**` 通路 |
| 6 | **跨主机时钟一致性** | JointState/Header 时间戳跨机对齐（回放 `replay_time_s` 依赖时间） | 确认 PC 与 S100 NTP/PTP 同步；统一用 `node.now()` Core time |
| 7 | **动态消息类型解析** | muxer/deploy/mcap 靠运行时话题名解析类型，aorta 要静态 schema | 主从路径固定为机械臂 schema；deploy/mcap 后置处理 |
| 8 | **message_filters 无等价物** | deploy obs / handeye 时间同步 | 仅影响后置阶段；自实现 stamp 缓冲对齐 |
| 9 | **`AORTA_GROUP` 生效方式** | 构造函数 `group` 被忽略，配错会静默不通 | 每进程显式设 `AORTA_GROUP` env，master/slave 必须一致 |

---

## 6. 执行顺序与回滚

```
spike(§4.1) ── 通 ──► PR-A schema(§4.2) ──► PR-B 代码切 aorta-only(§4.3) ──► 真机 gate(§4.4) ──► PR-C 删 ROS2(§4.5)
     │ 不通                                                                    │ 不过
     ▼                                                                        ▼
  先解决 PC 入网/ACL/时延，不写业务代码                                    git revert PR-B（ROS2 仍在树，可回滚）
```

**为什么这样就是"一步到位"**：真正的跨主机风险（PC 入网 + 时延）在 spike 一次性排掉；schema 一个 PR 全给；代码一个 PR 全切；只有一个真机 gate；ROS2 代码留到 gate 通过才删，保证可回滚。没有双栈期、没有逐话题灰度——但也没有"盲切物理臂"。
