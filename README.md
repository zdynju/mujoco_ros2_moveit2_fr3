# Mujoco ROS 2 MoveIt 2 FR3 Simulation

本项目提供了一个基于 **MuJoCo** 物理引擎的 Franka Emika FR3 机器人仿真环境。它通过 [`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control) 插件将 ROS 2 的控制能力与 MuJoCo 的高精度物理仿真相结合，并集成了手动配置的 **MoveIt 2** 规划框架。
## 📂 项目架构 (File Tree)
```text
fsrc/
├── fr3_application/                # [核心逻辑] 用户自定义的 MoveItPy 控制脚本
│   ├── config/
│   │   └── moveit_cpp.yaml         # MoveItPy 的参数配置
│   ├── fr3_application/
│   │   └── grasp.py                # 🐍 主程序：抓取规划与执行脚本
│   └── launch/
│       └── grasp.launch.py         # 🚀 启动文件 (MoveItPy + RViz)
│
├── fr3_moveit_config/              # [规划配置] MoveIt Setup Assistant 生成的包
│   ├── config/
│   │   ├── fr3.srdf                # 语义描述 (关节组、预设姿态、碰撞矩阵)
│   │   ├── fr3.urdf                # 机器人模型 (从 xacro 生成)
│   │   ├── moveit_controllers.yaml # MoveIt 控制器接口配置
│   │   └── ompl_planning.yaml      # OMPL 规划算法参数
│   └── launch/
│       └── moveit.launch.py        # 仅启动 MoveIt 节点的入口
│
├── fr3_sim/                        # [仿真环境] MuJoCo + ROS 2 Control 集成
│   ├── config/
│   │   ├── fr3_mujoco.xml          # ⚛️ MuJoCo 物理场景 (MJCF)
│   │   ├── ros2_controllers.yaml   # ros2_control 控制器参数 (PID, 关节名)
│   ├── fr3_sim/
│   │   └── mujoco_obstacle_pub.py  # (可选) 动态发布障碍物到规划场景
│   ├── launch/
│   │   └── fr3_sim.launch.py       # 🚀 启动仿真器、控制器管理器
│   └── urdf/
│       └── fr3.urdf.xacro          # 包含 MuJoCo 硬件插件的机器人描述
│
├── franka_description/             # [官方资源] Franka 机器人的 Mesh 和基础 URDF
│   ├── meshes/                     # 3D 模型文件 (Visual & Collision)
│   └── robots/
│       └── fr3/                    # FR3 原始 xacro 文件
│
└── mujoco_ros2_control/            # [硬件接口] MuJoCo 与 ROS 2 的通信桥梁
    └── mujoco/                     # MuJoCo 引擎库文件
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
### 5.实现简单的避障规划
```bash
ros2 launch fr3_application grasp.launch.py
```