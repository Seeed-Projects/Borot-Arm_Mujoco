# reBot Arm MuJoCo DM

面向 reBot Arm B601-DM（达妙电机版）的 ROS 2 + MuJoCo 仿真与网页控制项目。

本仓库覆盖完整软件链路：机械臂 ROS 2 接口、真机驱动、Fake Driver、MuJoCo 动力学与抓取、real2sim 同步、力矩对比、虚拟相机、颜色检测、Three.js 网页控制台以及 LLM/MCP 控制。

## 硬件要求

- **机械臂**: reBot Arm B601-DM
- **电机**: 达妙 4340P（joint1-3）+ 4310（joint4-6 + 夹爪）
- **通信**: USB-CAN（`/dev/ttyACM0`）
- **主机**: Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12

## 软件前置条件

| 组件 | 版本 | 安装方式 |
| --- | --- | --- |
| Ubuntu | 24.04 | 系统 |
| ROS 2 | Jazzy | `apt install ros-jazzy-desktop` |
| Python | 3.12 | 系统自带 |
| Node.js | ≥ 18 | `apt install nodejs` |
| MuJoCo | 3.10+ | `pip install mujoco` |
| Pinocchio | 4.1+ | `pip install pin` |
| MotorBridge | 0.5+ | `pip install motorbridge` |

## 依赖总览

### 工作空间内（本仓库自带）

| 内容 | 位置 | 说明 |
| --- | --- | --- |
| ROS 2 包（7 个） | `reBotArmController_ROS2-main/src/` | msgs、controller、bringup、mujoco、agent |
| URDF + STL 网格 | `rebotarm_bringup/description/` | 机械臂模型文件 |
| MuJoCo 模型 | `rebotarm_mujoco/models/` | STL/运动学 XML |
| 网页前端 | `reBotArm_simulator-DM/` | Three.js + HTML/CSS/JS |
| 启动脚本 | `reBotArmController_ROS2-main/scripts/` | 一键启动 |

### 外部依赖（需单独安装）

| 依赖 | 来源 | 用途 | 安装方式 |
| --- | --- | --- | --- |
| ROS 2 Jazzy | 系统 | rclpy、消息类型、rosbridge | `apt install ros-jazzy-desktop ros-jazzy-rosbridge-suite` |
| reBotArm_control_py SDK | GitHub | RebotArm、IK、动力学、重力补偿 | 见下方安装步骤 |
| motorbridge | pip | 达妙电机 CAN 通信 | `pip install motorbridge` |
| pinocchio (pin) | pip (cmeel) | 刚体动力学模型 | `pip install pin` |
| mujoco | pip | 物理仿真引擎 | `pip install mujoco` |
| numpy | pip | 数值计算 | `pip install numpy` |
| pyyaml | pip | YAML 配置解析 | `pip install pyyaml` |
| transforms3d | pip | 坐标变换 | `pip install transforms3d` |
| tf_transformations | ROS .deb | 四元数/欧拉角转换 | 见下方安装步骤 |
| Node.js | 系统 | 网页服务器 | `apt install nodejs` |

## 安装

### 1. 系统前置

```bash
# ROS 2 Jazzy（如未安装）
sudo apt update && sudo apt install -y ros-jazzy-desktop ros-jazzy-rosbridge-suite

# Node.js（网页控制台）
sudo apt install -y nodejs

# 验证
ros2 --version    # Jazzy
node --version    # >= 18
```

### 2. 安装 reBotArm_control_py SDK

SDK 提供真机驱动、逆运动学、动力学计算和重力补偿，是核心外部依赖。

```bash
cd ~
git clone https://github.com/Seeed-Projects/reBotArm_control_py.git
cd reBotArm_control_py
pip install -e .    # 可编辑安装，或直接用 sys.path 引用
```

安装后目录结构：
```text
~/reBotArm_control_py/
├─ reBotArm_control_py/
│  ├─ actuator/          RebotArm 类、JointGroup、电机控制
│  ├─ controllers/       RebotArmEndPose（轨迹、IK、重力补偿）
│  ├─ kinematics/        正逆运动学、load_robot_model、pad_q_for_model
│  └─ dynamics/          compute_generalized_gravity 等动力学函数
├─ config/
│  └─ rebotarm_dm.yaml   DM 版电机配置（ID、波特率、限位、PID）
├─ urdf/                 Pinocchio 动力学模型 URDF
└─ pyproject.toml
```

> **注意**：SDK 的 `pyproject.toml` 声明 `requires-python >=3.10,<3.12`，但本项目通过 `sys.path` 引用而非 pip 安装，在 Python 3.12 下可正常工作。如果 pip 安装报版本冲突，跳过 `pip install -e .`，确保目录在 `~/reBotArm_control_py/` 即可（代码会自动搜索此路径）。

### 3. 创建 Python venv 并安装依赖

```bash
cd ~/reBot_Arm_Mujoco-DM/reBotArmController_ROS2-main

# 创建 venv（必须启用系统站点包，否则 rosbridge 的 tornado/psutil 等不可见）
python3 -m venv .venv --system-site-packages
source .venv/bin/activate

# 安装 Python 依赖
pip install mujoco pin motorbridge numpy pyyaml transforms3d tornado psutil
```

### 4. 安装 tf_transformations

`tf_transformations` 不在 pip 上，需从 ROS .deb 提取：

```bash
# 下载并提取到 venv
cd /tmp
apt download ros-jazzy-tf-transformations
dpkg-deb -x ros-jazzy-tf-transformations*.deb tf_transformations_extract
cp -r tf_transformations_extract/opt/ros/jazzy/lib/python3.12/site-packages/tf_transformations \
      ~/reBot_Arm_Mujoco-DM/reBotArmController_ROS2-main/.venv/lib/python3.12/site-packages/
```

### 5. 验证 venv 配置

```bash
# 确认系统站点包已启用
grep "include-system-site-packages" .venv/pyvenv.cfg
# 应输出: include-system-site-packages = true

# 如果是 false，修改：
sed -i 's/include-system-site-packages = false/include-system-site-packages = true/' .venv/pyvenv.cfg

# 验证所有关键包可导入
.venv/bin/python -c "
import mujoco; print('mujoco', mujoco.__version__)
import pinocchio as pin; print('pinocchio', pin.__version__)
import motorbridge; print('motorbridge OK')
import tornado; print('tornado OK')
import psutil; print('psutil OK')
import argcomplete; print('argcomplete OK')
import bson; print('bson OK')
import tf_transformations; print('tf_transformations OK')
import sys; sys.path.insert(0, '/home/robot/reBotArm_control_py')
from reBotArm_control_py.actuator import RebotArm; print('SDK OK')
"
```

### 6. 编译 ROS 2 工作空间

```bash
cd ~/reBot_Arm_Mujoco-DM/reBotArmController_ROS2-main
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --symlink-install
```


## 项目结构

```text
.
├─ PROJECT_ARCHITECTURE_ZH.md       整体架构、仿真原理和防抖说明
├─ setup.sh                         可重复执行的一键安装与版本检查
├─ rebotarm                         统一启动、停止、状态和诊断入口
├─ requirements.txt                 Python 依赖兼容版本范围
├─ reBotArmController_ROS2-main/    ROS 2 工作空间
│  ├─ scripts/                      一键启动脚本
│  ├─ third_party/                  新安装时的 reBotArm_control_py SDK
│  ├─ .venv/                        项目 Python 虚拟环境（由 setup.sh 创建）
│  └─ src/
│     ├─ rebotarm_msgs/             自定义 msg/srv/action
│     ├─ rebotarmcontroller/        真机驱动、Fake Driver、硬件管理
│     ├─ rebotarm_bringup/          URDF、STL、launch、电机配置
│     ├─ rebotarm_mujoco/           MuJoCo 仿真、IK、相机、视觉
│     └─ rebotarm_agent/            MCP Server 与文本 Agent
└─ reBotArm_simulator-DM/           Node.js + Three.js 网页控制台
   ├─ public/                       页面、样式、前端逻辑
   └─ split_meshes/grouped_gripper/ 网页夹爪网格
```

## 仿真模式

| 模式 | 动力学步进 | 用途 |
| --- | --- | --- |
| Three.js 网页模拟器 | 无 | 浏览器显示、交互、示教、ROS 控制面板 |
| MuJoCo real2sim | `mj_forward` | 真机关节角实时映射到 MuJoCo 模型 |
| MuJoCo physics grasp | `mj_step` | 重力、惯性、碰撞、摩擦、物理抓取 |
| MuJoCo torque control | `mj_step` | `tau_g + PD` 闭环、重力模型对比 |

## 快速启动

### 克隆后一键安装（推荐）

```bash
git clone <本仓库地址>
cd reBot_Arm_Mujoco-DM
./setup.sh
./rebotarm doctor
```

安装器可重复运行：已有且满足要求的组件会跳过，不会删除现有 SDK、虚拟环境或网页 `.env`；缺失项才会安装。结束时会分别汇总已安装、已跳过、版本不匹配和失败项。只检查、不修改系统：

```bash
./setup.sh --check
```

统一启动入口：

```bash
./rebotarm start web   # rosbridge + 网页
./rebotarm start dm    # DM 真机（单独终端）
./rebotarm start sim   # MuJoCo 仿真；不要与 DM 真机同时启动
./rebotarm status
```

所有命令前先 source 环境：

```bash
cd ~/reBot_Arm_Mujoco-DM/reBotArmController_ROS2-main
source scripts/source_rebotarm_env.sh
```

### 1. Fake Driver（纯仿真，不接真机）

```bash
ros2 launch rebotarm_bringup fake_bringup.launch.py
```

验证：`ros2 topic echo /rebotarm/joint_states --once` 应返回非零角度。

### 2. 真机控制

```bash
# 确认设备节点并赋予权限
ls /dev/ttyACM0
sudo chmod 666 /dev/ttyACM0 

# 启动真机驱动
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

验证：
```bash
ros2 topic echo /rebotarm/joint_states --once   # 应显示真机关节角度
ros2 service call /rebotarm/enable std_srvs/srv/Trigger   # 使能
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger   # 安全回零
```

### 3. 网页控制台

**Terminal 2 — rosbridge：**

```bash
cd ~/reBot_Arm_Mujoco-DM/reBotArmController_ROS2-main
source scripts/source_rebotarm_env.sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

**Terminal 3 — Web 服务器：**

```bash
cd ~/reBot_Arm_Mujoco-DM/reBotArm_simulator-DM
cp .env.example .env   # 首次使用：复制环境变量模板
# 编辑 .env，把 localhost 改成虚拟机 IP（如 ws://192.168.x.x:9090）
node server.js
```

**浏览器：**

1. 打开 `http://localhost:3001`
2. 在 WebSocket 输入框填入 rosbridge 地址（如 `ws://localhost:9090`），地址会自动记忆
3. 点击「连接 ROS」
4. 3D 模型应显示真机当前姿态

**控制真机（两步解锁）：**

1. 勾选「允许网页向真实机械臂发控制」（连接真机控制器时会弹出二次确认）
2. 点击「使能」按钮

三步完成后，拖动关节滑块或点击夹爪按钮即可控制真机。

### 4. Real2Sim 同步

真机控制器运行中，新开终端：

```bash
source scripts/source_rebotarm_env.sh
ros2 launch rebotarm_mujoco real2sim.launch.py
```

预期：MuJoCo viewer 窗口打开，3D 模型实时跟随真机运动（含夹爪）。

### 5. 力矩控制对比

真机控制器运行中，新开终端：

```bash
source scripts/source_rebotarm_env.sh
scripts/start_mujoco_torque_control.sh
```

预期：MuJoCo viewer 打开，控制台每 0.5 秒打印力矩对比：

```text
tau_g compare: max_abs=0.5080 Nm at joint3, mujoco=-7.4782, sdk=-6.9702, rms=0.2478
```

- `mujoco`：MuJoCo 物理引擎计算的重力力矩
- `sdk`：Pinocchio 计算的重力力矩
- `diff`：两者差值
- RMS < 0.3 Nm 表示动力学模型一致性良好

### 6. 完整 MuJoCo 仿真栈

```bash
source scripts/source_rebotarm_env.sh
scripts/start_rebot_mujoco_all.sh
```

一键启动：Fake Driver + MuJoCo 物理抓取 + 任务服务器 + RGB 相机 + 颜色检测 + rosbridge。

## ROS 2 接口

### Topics

| Topic | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/rebotarm/joint_states` | `sensor_msgs/JointState` | 发布 | 6 关节 + finger_left 位置/速度/力矩 |
| `/rebotarm/arm_status` | `rebotarm_msgs/ArmStatus` | 发布 | 模式、使能状态、状态机、错误码 |
| `/rebotarm/gripper/state` | `rebotarm_msgs/JointMotorState` | 发布 | 夹爪位置（米）/速度/力矩 |
| `/rebotarm/joints/jointN/cmd` | `rebotarm_msgs/JointMotorCmd` | 订阅 | 单关节位置/速度/MIT 命令 |
| `/rebotarm/gripper/cmd` | `rebotarm_msgs/JointMotorCmd` | 订阅 | 夹爪命令（pos 为米） |

### Services

| Service | 类型 | 说明 |
| --- | --- | --- |
| `/rebotarm/enable` | `std_srvs/Trigger` | 使能机械臂 |
| `/rebotarm/disable` | `std_srvs/Trigger` | 失能机械臂 |
| `/rebotarm/safe_home` | `std_srvs/Trigger` | 安全回零 |
| `/rebotarm/gripper/set` | `rebotarm_msgs/SetGripper` | 设置夹爪（position 米, max_effort） |
| `/rebotarm/gravity_compensation/start` | `std_srvs/Trigger` | 启动重力补偿 |
| `/rebotarm/gravity_compensation/stop` | `std_srvs/Trigger` | 停止重力补偿 |
| `/rebotarm/gravity_compensation/status` | `std_srvs/Trigger` | 查询重力补偿状态 |
| `/rebotarm/set_mode` | `rebotarm_msgs/SetMode` | 切换控制模式 |
| `/rebotarm/move_to_pose_ik` | `rebotarm_msgs/MoveToPoseIK` | IK 运动到目标位姿 |

## 配置

### 电机配置

SDK 配置文件：`~/reBotArm_control_py/config/rebotarm_dm.yaml`

包含电机 ID、波特率、关节限位、PID 参数等。修改后需重启控制器。

#### 电机一览

| 关节 | 电机 ID | 反馈 ID | 型号 | 用途 |
|------|---------|---------|------|------|
| joint1 | 0x01 | 0x11 | 4340P | 底座旋转 |
| joint2 | 0x02 | 0x12 | 4340P | 肩部俯仰 |
| joint3 | 0x03 | 0x13 | 4340P | 肘部俯仰 |
| joint4 | 0x04 | 0x14 | 4310  | 腕部旋转 |
| joint5 | 0x05 | 0x15 | 4310  | 腕部俯仰 |
| joint6 | 0x06 | 0x16 | 4310  | 腕部旋转 |
| gripper | 0x07 | 0x17 | 4310  | 夹爪 |

#### 控制架构：手臂关节与夹爪统一走 POS_VEL

所有 7 个电机（joint1–joint6 + gripper）均使用 **POS_VEL 模式**，由电机固件内部 PID 控制器闭环，上位机不做外部 PD 运算。这避免了双重 PD 叠加导致的振荡问题。

**数据流：**

```
上位机 (ROS 2 / 网页)
  │
  ├─ 手臂 joint1-6:  SDK 控制循环 500 Hz → arm.send_pos_vel(q_target, vlim)
  │                   → 电机固件内部 PID (pos_kp/pos_ki + vel_kp/vel_ki) 闭环
  │
  └─ 夹爪 gripper:    set_gripper_target() → gripper.send_pos_vel(target, vlim)
                       → 电机固件内部 PID (pos_kp/pos_ki + vel_kp/vel_ki) 闭环
```

**关键设计：**

- 手臂关节由 SDK 的 `RebotArmEndPose` 控制循环（`_loop_cb`）以 500 Hz 调用 `arm.send_pos_vel(q_target, vlim)`，目标位置写入 `_q_target` 数组
- 夹爪由 `HardwareManager.set_gripper_target()` 直接调用 `send_pos_vel(target, vlim)`，无需控制循环——POS_VEL 模式下电机会自动保持目标位置
- 夹爪反馈轮询线程以 50 Hz 调用 `request_feedback()` + `poll_feedback_once()`，仅读取状态（位置/速度/力矩），不发控制命令
- 单关节命令（`/rebotarm/joints/jointN/cmd`）在 mode=1 时也走 `send_pos_vel`，同时同步 `_q_target[idx]` 防止控制循环覆盖

#### POS_VEL PID 参数

| 关节 | pos_kp | pos_ki | vel_kp | vel_ki | vlim (rad/s) |
|------|--------|--------|--------|--------|--------------|
| joint1–3 (4340P) | 150.0 | 0.5 | 0.0125 | 0.004 | 5.0 |
| joint4–6 (4310)  | 50.0  | 1.0 | 0.0008 | 0.002 | 3.0 |
| gripper (4310)   | 50.0  | 1.0 | 0.0008 | 0.002 | 3.0 |

这些参数存储在电机固件寄存器中，由 SDK 在 `ensure_mode(Mode.POS_VEL)` 时写入。修改 PID 需编辑 `rebotarm_dm.yaml` 对应关节的 `POS_VEL` 段后重启控制器。

#### 夹爪单位换算

网页和 ROS 接口使用**米**（0.0 = 完全闭合，0.09 = 完全张开），电机固件使用**弧度**（0.0 = 闭合，−5.0 = 张开）。换算在 `HardwareManager` 中完成：

```
弧度 = (距离_m / 0.09) × (−5.0)
距离_m = (弧度 / −5.0) × 0.09
```

> **历史问题**：早期版本夹爪使用 MIT 模式 + 500 Hz 外部 PD 环（`_gripper_safe_mit`），与电机内部 MIT PD 叠加形成双重 PD，导致闭合时持续抖动。已改为 POS_VEL 模式，与 motorbridge studio 行为一致，问题消除。



### 网页 rosbridge 地址

rosbridge WebSocket 地址由用户在网页「ROS2 桥接」面板手动输入，默认不硬编码。`rebot-ros-ui.js` 会从 `localStorage` 读取上次保存的地址。首次连接时填入实际地址即可，例如 `ws://<Ubuntu IP>:9090`。

## 环境说明

`scripts/source_rebotarm_env.sh` 按顺序加载：

1. ROS 2 Jazzy（`/opt/ros/jazzy/setup.bash`）
2. Python venv（`.venv/bin/activate`）
3. cmeel.prefix 路径（Pinocchio 的 C 扩展和共享库）
4. 工作空间（`install/setup.bash`）

venv 必须启用 `include-system-site-packages = true`（`.venv/pyvenv.cfg`），否则 rosbridge 的 `tornado`、`psutil`、`argcomplete`、`bson` 等系统包不可见。

## 故障排查

### `ModuleNotFoundError: No module named 'pinocchio'`

安装 `pin`（不是 `pinocchio`）：
```bash
pip install pin
```

### `ModuleNotFoundError: No module named 'tornado'/'psutil'/'argcomplete'/'bson'`

venv 未启用系统站点包：
```bash
sed -i 's/include-system-site-packages = false/include-system-site-packages = true/' .venv/pyvenv.cfg
```

### `write_register_f32 failed: register 25 write ack not received within 50ms`

电机总线通信超时。检查：
- USB-CAN 线缆是否连接
- `/dev/ttyACM0` 是否存在
- 电机电源是否开启
- 是否有其他进程占用总线

### 网页控制不了真机

确认三步解锁：
1. 在「ROS2 桥接」面板连接 ROS（WebSocket 连接到真机控制器的 rosbridge）
2. 勾选「允许控制」→ 确认对话框点「确定」
3. 点「使能」按钮

### 夹爪不同步到网页

`/rebotarm/gripper/state` 的 position 必须是米（0~0.09），不是弧度。如果不同步，检查 `ros_publishers.py` 是否使用 `gripper_position_m()`。

### 夹爪闭合抖动

降低夹爪目标力矩 `_G_DEFAULT_FORCE`（如 0.30→0.15），或增大夹爪到位容差 `_G_ARRIVE_TOL`。

### 重力补偿停止后机械臂异常

`stop_gravity_compensation` 必须调用 `self._arm.arm.mode_pos_vel()`（不是 `self._arm.mode_pos_vel()`），否则电机无法切回 POS_VEL 模式。

## 已知修复（DM 适配）

| 问题 | 根因 | 修复 |
| --- | --- | --- |
| SDK API 不兼容 | 仓库使用旧 API（`RobotArm`），DM SDK 用新 API（`RebotArm`） | 重写 `hardware_manager.py` 21 处 |
| Pinocchio 导入失败 | pip 包名是 `pin` 不是 `pinocchio` | 安装 `pin` |
| rosbridge 缺依赖 | venv 隔离了系统包 | 启用 `include-system-site-packages` |
| 夹爪不同步 | 发布原始弧度，网页期望米 | `ros_publishers.py` 改用 `gripper_position_m()` |
| 关节滑块控不了真机 | pos_vel 循环覆盖单关节命令 | `send_joint_motor_cmd` 同步 `_q_target` |
| 夹爪 mode 1 不工作 | 直接 `send_pos_vel` 传米给弧度电机 | 改走 `set_gripper_target` |
| 重力补偿停止异常 | `self._arm.mode_pos_vel()` 应为 `self._arm.arm.mode_pos_vel()` | 修正调用路径 |
| 控制锁不生效 | 仿真模式跳过控制锁检查 | `controlAllowed` 所有模式统一检查 |
| 夹爪闭合抖动 | kp=5.0 过高、kd=1.0 不足 | 降至 kp=2.0、kd=2.0 |

## 文档

- [项目架构、MuJoCo 与网页说明](./PROJECT_ARCHITECTURE_ZH.md)
- [DM 真机数据链路与数据流向](./DATA_FLOW_ZH.md)
- [B601-DM 用户使用手册](./USER_MANUAL_ZH.md)
- [ROS 2 工作空间说明](./reBotArmController_ROS2-main/README_zh.md)
- [MuJoCo 包说明](./reBotArmController_ROS2-main/src/rebotarm_mujoco/README.md)
- [网页控制台说明](./reBotArm_simulator-DM/README.md)
- [AI Agent/MCP 说明](./reBotArmController_ROS2-main/src/rebotarm_agent/README.md)

## License

软件代码沿用仓库中的 Apache-2.0 许可说明。模型和资产的使用条件请同时参考原始 reBot-DevArm 项目。
