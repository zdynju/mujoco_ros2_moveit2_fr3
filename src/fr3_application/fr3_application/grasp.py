#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder # 关键依赖
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import math
import time
from ament_index_python.packages import get_package_share_directory
import os
def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    execute=True,
):
    """
    辅助函数：规划并执行
    """
    logger.info("开始规划路径...")
    
    # 执行规划
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(single_plan_parameters=single_plan_parameters)
    else:
        plan_result = planning_component.plan()

    # 检查规划结果
    if plan_result:
        logger.info("✅ 规划成功！")
        
        if execute:
            logger.info("正在执行轨迹...")
            # MoveItPy 的 execute 需要传入 trajectory 对象
            robot.execute(plan_result.trajectory, controllers=[])
            logger.info("执行完成")
        return True
    else:
        logger.error("❌ 规划失败")
        return False


def find_safe_pre_grasp(arm, target_pose, logger, radius=0.1, heights=[0.1, 0.15, 0.2]):
    """
    自动生成避障的 Pre-grasp 位姿
    """
    # 搜索 8 个方向
    angles = [0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi, -3*math.pi/4, -math.pi/2, -math.pi/4]

    logger.info("正在搜索可行的 Pre-grasp 位姿...")

    for h in heights:
        for angle in angles:
            pre_pose = deepcopy(target_pose)
            
            # 1. 位置偏移：在上方 h 处，并在水平方向偏移 radius
            pre_pose.pose.position.z += h
            pre_pose.pose.position.x += radius * math.cos(angle)
            pre_pose.pose.position.y += radius * math.sin(angle)
            
            # 2. 设置目标
            arm.set_start_state_to_current_state()
            arm.set_pose_target(pre_pose)
            
            # 3. 尝试规划（只规划不执行）
            plan_result = arm.plan()
            
            # 4. 如果规划成功，说明这个点是安全的
            if plan_result:
                logger.info(f"✅ 找到安全点: 高度+{h}, 角度{int(math.degrees(angle))}度")
                return pre_pose
                
    return None


def grasp_sequence(moveit_py_instance, target_pose):
    """
    鲁棒三段式抓取序列
    """
    # 获取 logger
    logger = rclpy.logging.get_logger("grasp_sequence")
    
    # 获取规划组 (确保名字和你的 SRDF 一致，可能是 fr3_arm 或 panda_arm)
    arm = moveit_py_instance.get_planning_component("fr3_arm")
    gripper = moveit_py_instance.get_planning_component("fr3_hand") # 如果有手爪

    # --------------------------------------------------------
    # 阶段 1: Pre-grasp (自动搜索安全区)
    # --------------------------------------------------------
    logger.info("--- Stage 1: Pre-grasp ---")
    
    # 修正目标姿态：让手爪朝下 (旋转 180 度绕 X 轴)
    # 默认 w=1 通常是朝天的，MuJoCo里的 FR3 需要特定朝向
    # 这里给一个常见的朝下四元数 (0, 1, 0, 0) 或者你保持原样如果你的TF树不同
    # target_pose.pose.orientation.x = 1.0 
    # target_pose.pose.orientation.y = 0.0
    # target_pose.pose.orientation.z = 0.0
    # target_pose.pose.orientation.w = 0.0

    pre_grasp_pose = find_safe_pre_grasp(arm, target_pose, logger)
    
    if pre_grasp_pose is None:
        logger.error("❌ 找不到安全的 Pre-grasp 位姿，抓取中止。")
        return

    # 执行移动到 Pre-grasp
    arm.set_start_state_to_current_state()
    arm.set_pose_target(pre_grasp_pose)
    if not plan_and_execute(moveit_py_instance, arm, logger):
        return

    time.sleep(1.0) # 停顿一下

    # --------------------------------------------------------
    # 阶段 2: Approach (直线下降)
    # --------------------------------------------------------
    logger.info("--- Stage 2: Approach (Cartesian) ---")
    
    # 这里简化为直接规划到目标（实际建议用 compute_cartesian_path）
    arm.set_start_state_to_current_state()
    arm.set_pose_target(target_pose)
    
    success = plan_and_execute(moveit_py_instance, arm, logger)
    if not success:
        logger.warn("Approach 失败，保持在 Pre-grasp 位置")
        return

    # --------------------------------------------------------
    # 阶段 3: Grasp (闭合夹爪)
    # --------------------------------------------------------
    logger.info("--- Stage 3: Grasp ---")
    # 如果你的 SRDF 里定义了 "close" 状态
    # gripper.set_goal_state(configuration_name="close")
    # plan_and_execute(moveit_py_instance, gripper, logger)
    logger.info("✊ 假装抓住了物体")

    logger.info("🎉 抓取流程结束")


def main():
    rclpy.init()

    # 假设你的 MoveIt 配置包叫 fr3_moveit_config
    try:
        # 2. 使用自动加载，移除冗余路径
        # 只要你的包结构是标准的 (config/ 文件夹下有对应文件)，这样写就够了
        moveit_config = MoveItConfigsBuilder("fr3", package_name="fr3_moveit_config") \
            .robot_description(file_path="config/fr3.urdf.xacro") \
            .robot_description_semantic(file_path="config/fr3.srdf") \
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner"], # 建议加上 pilz，处理直线运动很方便
                default_planning_pipeline="ompl"
            ) \
            .to_moveit_configs()

        # 检查是否成功加载 (可选，打印调试)
        print(moveit_config.to_dict()) 
        
    except Exception as e:
        print(f"❌ 配置文件加载失败: {e}")
        return
    # # 传入 config_dict
    fr3 = MoveItPy(node_name="moveit_py", config_dict=moveit_config.to_dict())

    # ---------------------------------------------------------
    # 2. 设置目标点 (来自 MuJoCo)
    # ---------------------------------------------------------
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base" # 建议用 world，对应我们之前修好的 TF 树
    target_pose.pose.position.x = 0.125    # 这里的坐标最好稍微改一下，MuJoCo的 0.125 可能在底座里面
    target_pose.pose.position.y = -0.25
    target_pose.pose.position.z = 0.2     # 抬高一点测试
    
    # 姿态：这取决于你的夹爪坐标系定义。
    # 通常 (1, 0, 0, 0) 是绕 X 轴 180 度，即指尖向下
    target_pose.pose.orientation.x = 1.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.0

    # ---------------------------------------------------------
    # 3. 执行
    # ---------------------------------------------------------
    grasp_sequence(fr3, target_pose)

    rclpy.shutdown()


if __name__ == "__main__":
    main()