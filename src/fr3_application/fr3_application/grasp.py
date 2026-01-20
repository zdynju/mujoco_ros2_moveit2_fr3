#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit.core.collision_detection import AllowedCollisionMatrix
from tf_transformations import quaternion_from_euler
import math
def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    execute=True,
    description="任务"
):
    """
    辅助函数：规划并执行
    """
    logger.info(f"--- 开始: {description} ---")
    
    # 执行规划
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(multi_plan_parameters)
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(single_plan_parameters)
    else:
        plan_result = planning_component.plan()

    if plan_result:
        logger.info(f"规划成功！")
        
        if execute:
            logger.info(f"正在执行轨迹...")
            group_name = planning_component.planning_group_name
            robot.execute(group_name, plan_result.trajectory, blocking=True)
            logger.info(f"执行完成")
        return True
    else:
        logger.error(f"规划失败")
        return False


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py")

    
    fr3 = MoveItPy(node_name="moveit_py")
    logger.info("MoveItPy 初始化完成")

    # 获取规划组件
    arm = fr3.get_planning_component("fr3_arm")
    hand = fr3.get_planning_component("fr3_hand")
    hand.set_start_state_to_current_state()

    arm.set_start_state_to_current_state()
    time.sleep(5.0) # 等待场景更新

    # ------------------------------------------------------------------
    # 3. 移动到预备点 (PTP)
    # ------------------------------------------------------------------
    roll = 0.0
    pitch = math.radians(90)  # 把角度转为弧度
    yaw = 0.0

    q = quaternion_from_euler(roll, pitch, yaw)
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base"
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    pose_goal.pose.position.x = 0.5
    pose_goal.pose.position.y = 0.0
    pose_goal.pose.position.z = 0.5

    arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="fr3_link8")
    
    pilz_params = PlanRequestParameters(fr3) 
    pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
    pilz_params.planner_id = "PTP"
    
    plan_and_execute(fr3, arm, logger, single_plan_parameters=pilz_params, description="移动到预备点")
    time.sleep(2.0)
    # # ------------------------------------------------------------------
    # # 4. 张开手
    # # ------------------------------------------------------------------
    # hand.set_start_state_to_current_state()
    # hand.set_goal_state(configuration_name="open")
    # plan_and_execute(fr3, hand, logger, description="张开手爪")

    # time.sleep(1.0)

    # # ------------------------------------------------------------------
    # # 5. 直线接近 (LIN)
    # # ------------------------------------------------------------------
    # pilz_pose_goal = PoseStamped()
    # pilz_pose_goal.header.frame_id = "base"
    # pilz_pose_goal.pose.orientation.x = q[0]
    # pilz_pose_goal.pose.orientation.y = q[1]
    # pilz_pose_goal.pose.orientation.z = q[2]
    # pilz_pose_goal.pose.orientation.w = q[3]
    # pilz_pose_goal.pose.position.x = 0.1
    # pilz_pose_goal.pose.position.y = -0.25
    # pilz_pose_goal.pose.position.z = 0.3

    # arm.set_start_state_to_current_state()
    # arm.set_goal_state(pose_stamped_msg=pilz_pose_goal, pose_link="fr3_link8") #
    
    # pilz_lin_params = PlanRequestParameters(fr3)
    # pilz_lin_params.planning_pipeline = "pilz_industrial_motion_planner"
    # pilz_lin_params.planner_id = "LIN"
    
    # if plan_and_execute(fr3, arm, logger, single_plan_parameters=pilz_lin_params, description="直线接近"):
        
    #     logger.info("正在将物体附着到手部...")
        
    #     # 使用 read_write() 上下文管理器，这是 MoveItPy 修改场景的正确方式
    #     with fr3.get_planning_scene_monitor().read_write() as scene:
    #         attached_object = AttachedCollisionObject()
    #         attached_object.link_name = "fr3_hand"   
    #         attached_object.object.id = "hoop_object"  
    #         attached_object.object.operation = CollisionObject.ADD

    #         attached_object.touch_links = ["fr3_hand", "fr3_leftfinger", "fr3_rightfinger"]


    #     logger.info("物体已附着！")
        
    #     # ------------------------------------------------------------------
    #     # 7. 闭合夹取
    #     # ------------------------------------------------------------------
    #     hand.set_start_state_to_current_state()
    #     hand.set_goal_state(configuration_name="close")
    #     if plan_and_execute(fr3, hand, logger, description="夹取物体"):
    #         pilz_pose_goal = PoseStamped()
    #         pilz_pose_goal.header.frame_id = "base"
    #         pilz_pose_goal.pose.orientation.x = 1.0
    #         pilz_pose_goal.pose.orientation.w = 0.0
    #         pilz_pose_goal.pose.position.x = 0.0
    #         pilz_pose_goal.pose.position.y = -0.25
    #         pilz_pose_goal.pose.position.z = 0.7 # 下降 10cm

    #         arm.set_start_state_to_current_state()
    #         arm.set_goal_state(pose_stamped_msg=pilz_pose_goal, pose_link="fr3_link8") 
            
    #         pilz_lin_params = PlanRequestParameters(fr3)
    #         pilz_lin_params.planning_pipeline = "pilz_industrial_motion_planner"
    #         pilz_lin_params.planner_id = "LIN"
            
    #         plan_and_execute(fr3, arm, logger, single_plan_parameters=pilz_lin_params, description="直线离开")
            
    #         # ------------------------------------------------------------------
    #         # 放到目标点
    #         # ------------------------------------------------------------------
    #         pose_goal = PoseStamped()
    #         pose_goal.header.frame_id = "base"
    #         pose_goal.pose.orientation.x = 1.0 # 假设这是垂直向下的姿态
    #         pose_goal.pose.orientation.w = 0.0
    #         pose_goal.pose.position.x = -0.25
    #         pose_goal.pose.position.y = -0.25
    #         pose_goal.pose.position.z = 0.25

    #         arm.set_start_state_to_current_state()
    #         arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="fr3_link8")
            
    #         pilz_params = PlanRequestParameters(fr3) 
    #         pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
    #         pilz_params.planner_id = "PTP"
            
    #         plan_and_execute(fr3, arm, logger, single_plan_parameters=pilz_params, description="移动到目标点")
    #         # 结束
    #         time.sleep(2)
    #     else:
    #         logger.error("抓取失败")

    # else:
    #     logger.error("接近失败，取消抓取")

    try:
        # 进入一个循环，只要 ROS 还是 OK 的，就一直挂起
        while rclpy.ok():
            # 这里可以处理回调，或者单纯休眠
            # 对于 MoveItPy，通常不需要显式 spin，因为它内部有线程
            time.sleep(1.0) 
    except KeyboardInterrupt:
        logger.info("检测到退出信号，正在关闭...")
    finally:
        # 只有当用户按 Ctrl+C 跳出循环后，才执行关闭
        rclpy.shutdown()

if __name__ == "__main__":
    main()