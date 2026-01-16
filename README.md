# Mujoco ROS 2 MoveIt 2 FR3 Simulation

本项目提供了一个基于 **MuJoCo** 物理引擎的 Franka Emika FR3 机器人仿真环境。它通过 `mujoco_ros2_control` 插件将 ROS 2 的控制能力与 MuJoCo 的高精度物理仿真相结合，并集成了手动配置的 **MoveIt 2** 规划框架。

## 📂 项目架构 (File Tree)
```text
fr3_ws/src/
├── fr3_sim/                        # 仿真核心包
│   ├── config/
│   │   ├── fr3_mujoco.xml          # MuJoCo 动力学模型、执行器(Motor)及场景定义
│   │   └── ros2_controllers.yaml    # 手臂与夹爪的控制器接口参数配置
│   ├── launch/
│   │   └── fr3_sim.launch.py       # 一键启动仿真、加载控制器与广播员的脚本
│   └── urdf/
│       └── fr3.urdf.xacro          # 桥接硬件插件 mujoco_ros2_control 的核心描述
├── fr3_moveit_config/              # 手动创建的 MoveIt 2 配置包
│   ├── config/
│   │   ├── fr3.srdf                # 定义规划组、从动关节(Passive)及预设姿态
│   │   └── ompl_planning.yaml      # 运动规划算法参数配置
│   └── launch/
│       └── moveit.launch.py        # 启动 Rviz2 交互界面与 MoveIt 核心节点
├── franka_description/             # Franka 官方机器人描述文件 (Mesh、运动学参数)
└── mujoco_ros2_control/            # MuJoCo 与 ros2_control 的硬件接口中间件
```
## 🚀 快速开始

### 1. 安装依赖
确保系统已安装 ROS 2 Humble 和 MuJoCo。在工作空间根目录下运行：
```bash
rosdep install -i --from-path src --rosdistro humble -y
```
### 2.编译工作空间
```bash
colcon build --symlink-install
source install/setup.bash
```
### 3.启动仿真环境
```bash
ros2 launch fr3_sim fr3_sim.launch.py
```
### 4.运行 MoveIt 2 规划
```bash
ros2 launch fr3_moveit_config moveit.launch.py
```