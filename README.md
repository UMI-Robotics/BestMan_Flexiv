<img src="docs/BestMan_logo.png" alt="BestMan Logo" width="700"/>

# BestMan Flexiv — Rizon 4 / RDK 1.9.4.1

Python 非实时控制封装，默认机械臂为 **Rizon 4（7 轴，无外部轴）**。
包含关节/笛卡尔运动、混合运动力控、主动力控/被动力控、笛卡尔/关节阻抗、
外力读取、传感器归零、工具参数配置和 Robotiq 串口夹爪。

## SDK 版本选择

截至 2026-11-16，Rizon 4 使用 **`flexivrdk==1.9.4.1`**，对应控制器软件
**v3.11.3 / v3.11.4**。PyPI 的 2.1.0 面向 **Enlight 系列**，不是 Rizon 4 的升级目标；
2.2 发布页仍标为 WIP。安装时必须固定版本。

- [RDK 1.9.4.1 发布说明](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v1.9.4.1)
- [RDK 2.1 的 Enlight 兼容范围](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v2.1)
- [完整迁移说明和力控接口](docs/SDK_MIGRATION.md)
- [验证结果及仿真边界](docs/VALIDATION.md)

## 安装

推荐 Ubuntu 22.04+ / Python 3.12；官方也提供 Python 3.10 的 Linux、Windows 和
macOS arm64 wheel。Python 3.8 不再支持。

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install '.[dev]'
# 使用串口夹爪时：python -m pip install '.[gripper]'
# 使用 RealSense 时：python -m pip install '.[camera]'
python -m pytest -q
```

无需 ROS、`roscore` 或手工添加 SDK 路径。之前使用过旧安装脚本的机器，应删除 shell 配置里
指向 `Install/flexiv_rdk/lib_py` 的 `PYTHONPATH`。旧 SDK 二进制及 Python 缓存已从本分支移除。
详细步骤见 [Install/install.md](Install/install.md)。

## 连接与运动

新版使用机身/控制器上的**序列号**，而非旧的 `robot_ip, local_ip` 两个参数。
网络接口限制使用本机 NIC 的 IPv4 地址。构造对象只连接并读取信息，不自动使能或运动。

```python
from Robotics_API import Bestman_Real_Flexiv, Pose

arm = Bestman_Real_Flexiv("Rizon4-123456", config="config/rizon4.json")
try:
    arm.initialize_robot()
    pose = arm.get_current_eef_pose()
    # Pose: [x,y,z] 米；四元数 [qx,qy,qz,qw]，与 SciPy 一致
    goal = Pose([pose.x, pose.y, pose.z + 0.01], pose.orientation)
    arm.move_eef_to_goal_pose(goal)
    if not arm.wait_for_eef(goal):
        raise TimeoutError("未到达目标，机器人已停止")
finally:
    arm.stop()
```

`Pose.from_rdk([x,y,z,qw,qx,qy,qz])` / `pose.to_rdk()` 用于 SDK 格式转换。
旧脚本中从 `tcpPose` 复制来的四元数必须显式转换，不可直接作为 xyzw 输入。

## 力控

先配置实际工具质量、质心、惯量和 TCP；在机械臂静止且无接触时归零。
`config/rizon4.json` 的 `flange_to_tcp` 只用于离线 FK/IK，不会改写控制器工具配置。
控制器工具可通过 `configure_tool()` 设置，参数必须来自实际工具测量。

```python
arm.initialize_robot()
try:
    # 静止且未接触环境时才执行：arm.zero_force_torque_sensors()
    hold = arm.get_current_eef_pose()
    arm.configure_force_control(
        [False, False, True, False, False, False],
        frame="WORLD", passive=False,
        max_linear_velocity=[0.02, 0.02, 0.02],
    )
    # 目标是 TCP 感受到的力；WORLD +Fz 表示向世界 -Z 方向压。
    arm.move_eef_to_goal_pose_wrench(hold, [0, 0, 3, 0, 0, 0])
    # 此命令持续生效；应用必须监测接触、行程和测量力，并明确停止。
finally:
    arm.stop()
```

`configure_force_control()` 自动先停止至 IDLE，再设置主动力控/被动力控、坐标系和轴。
`SetMaxContactWrench` **仅限制运动控制部分**，不等同于全局碰撞检测；开启转动力矩控制时，
SDK 不允许同时开启该调节，需明确传入 `[float("inf")]*6`。
被动力控是开环力前馈，SDK 的主动力控轴速度保护在此模式不生效。
Python 的轮询限值也不是控制器级实时保护。

## Examples

以下命令从仓库根目录运行，将示例序列号替换为实际值。默认 20 Hz，可用 `--frequency` 设置 1–100 Hz；
各机器人示例均支持 `--config config/rizon4.json` 和可重复的 `--interface 192.168.2.108`。

```bash
# 只读状态，不使能机械臂
python -m Examples.display_states Rizon4-123456 --duration 5
# 回到控制器 Home
python -m Examples.move_arm_to_home Rizon4-123456
# 无接触、静止条件下归零（不会自动回 Home）
python -m Examples.zero_force_torque_sensors Rizon4-123456
# 从当前姿态开始 Z 轴 3N 主动力控，5秒结束，最大位移3cm
python -m Examples.force_control Rizon4-123456 --duration 5
# 使用TCP坐标系：力的方向必须结合实际工具朝向确定
python -m Examples.force_control Rizon4-123456 --frame TCP --wrench 0 0 -3 0 0 0
# 当前姿态柔顺保持，刚度为控制器标称值的30%
python -m Examples.cartesian_impedance Rizon4-123456 --stiffness-scale 0.3
python -m Examples.joint_impedance Rizon4-123456 --stiffness-scale 0.3
# 示教记录为xyzw JSON；使用轨迹示例独立回放
python -m Examples.teach_by_demonstration Rizon4-123456 --output taught_poses.json
python -m Examples.move_eef_follow_trajectory Rizon4-123456 --trajectory taught_poses.json
# 夹爪位置示例；单位是原始0–255指令，并非毫米
python -m Examples.open_gripper_width Rizon4-123456 --port /dev/ttyUSB0 --position 100
```

力控/阻抗示例有时长、TCP 位移和测量力/力矩限值；没有接触搜索或自动归零，
应从已经检查过的工作位姿运行。所有运动示例在正常结束、异常、Ctrl+C 时调用 `Stop()`。
关节目标、笛卡尔目标及轨迹由命令行明确提供，不再使用作者实验台的硬编码路径。
各参数见 `python -m Examples.<name> --help`。

`Demos/` 和 `TODO/` 保留为历史实验资料，依赖旧 API/ROS，未作为新版可运行入口；见
[Demos/README.md](Demos/README.md)。`Sensor/Camera.py` 是原有相机模块，未做相机标定或实机验证。

## 验证与范围

测试使用真实 RDK wheel 核对符号/签名，并用严格命令记录器检查调用参数、顺序和异常路径。
它不会伪造物理力控结果。5090-2 的 DJ-Sim 当前使用 Isaac/PhysX 控制接口，缺少 Flexiv RDK
网络服务和闭环力控实现，不能证明真实控制器的力跟踪性能。

Python 仅提供 NRT 控制；1000 Hz 实时关节力矩/实时笛卡尔控制应使用官方 C++ API。
Rizon 4 的离线 URDF 是运动学资源，含模糊动力学参数，不用于力矩控制或重力补偿。

## 联系与参考

- dingyan at pjlab.org.cn
- zhaxizhuoma at pjlab.org.cn
- [Flexiv RDK](https://www.flexiv.com/software/rdk)
- [IKPy](https://ikpy.readthedocs.io/en/latest/)
