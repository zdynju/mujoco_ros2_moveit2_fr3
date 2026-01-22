# 🤖 Mujoco ROS 2 MoveIt 2 FR3 Simulation

<!-- [![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-34aec5.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![MuJoCo](https://img.shields.io/badge/Physics-MuJoCo-orange.svg)](https://mujoco.org/)
[![MoveIt 2](https://img.shields.io/badge/Motion_Planning-MoveIt_2-blue.svg)](https://moveit.picknik.ai/)
[![Status](https://img.shields.io/badge/Status-Active_Development-yellow.svg)]() -->

> **基于 MuJoCo 物理引擎的 Franka Emika FR3 机器人仿真与控制平台。**

本项目旨在为 Franka FR3 机械臂搭建一个了 **高精度物理仿真Mujoco**、**ROS 2 Control 硬件接口** 以及 **MoveIt 2 运动规划** 的开发环境。目前，**仿真底层** 与 **抓取控制接口** 已基本完善，视觉感知与环境建图模块正在积极完善中。

---

## ✨ 已完善功能 (Completed Features)

* **高保真物理仿真 (Stable)**：
    * 利用 MuJoCo 引擎实现了稳定的机器人动力学仿真。
    * 集成 [`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control) 插件，实现了标准的 `JointTrajectoryController` 接口，仿真与控制通信稳定。
* **灵活的抓取接口 (Ready)**：
    * 基于 `moveit_py` 开发了 Python 端的 `fr3grasp` 接口。
    * 封装了 `move_ptp`,`move_lin`, `grapper_open`, `grapper_close` 等高层指令，支持复杂的动作序列编排，逻辑处理已比较完善。
* **模块化启动**：
    * 实现了仿真、规划、应用层的解耦启动，便于独立调试各个模块。

---

## 开发中与已知问题 (WIP & Known Issues)

**当前项目的主要挑战集中在感知与建图模块：**

* **OctoMap 建图严重不稳定**：
    * 目前在将仿真点云集成到 MoveIt 的 OctoMap 时存在显著问题。
    * 表现为：动态障碍物清除不及时、体素更新存在严重抖动/残影，导致规划场景不可靠。
* **Perception 模块尚不完善**：
    * `fr3_perception` 包虽然实现了基础的深度图转点云功能，但相机参数、噪声模型以及 TF 同步仍需进一步调优。
    * 目前视觉避障功能仅处于实验阶段。

---

## 🛠️ 环境要求 (Prerequisites)

* **Operating System**: Ubuntu 22.04 LTS
* **ROS 2**: Humble Hawksbill
* **Physics Engine**: MuJoCo 3.3.4
---

## 🚀 快速开始 (Quick Start)

### 1. 编译工作空间
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```
### 2. 运行仿真与抓取演示

####  步骤 I: 启动仿真环境 (Stable)
启动 MuJoCo 模拟器及底层控制器。这是目前最稳定的部分。
```bash
ros2 launch fr3_sim fr3_sim.launch.py use_sim_time:=true
```
####  步骤 II: 运行抓取应用 (Stable)

启动基于 `moveit__py`实现`fr3grasp`仿真接口的抓取任务，并在 RViz 中查看规划结果：
```bash
ros2 launch fr3_application grasp.launch.py
```
#### (option)
```bash
ros2 launch fr3_moveit_config moveit.launch.py
```
###  参考引用 (References)

本项目使用了以下开源项目作为基础组件，特此致谢：

- [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)  
  本项目集成了该仓库代码以实现 MuJoCo 与 ROS 2 Control 的通信。
