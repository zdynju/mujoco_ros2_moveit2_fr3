#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.logging import get_logger
from moveit.planning import MoveItPy, PlanRequestParameters
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler

# --- 配置参数 ---
PLANNING_GROUP_ARM = "fr3_arm"
PLANNING_GROUP_HAND = "fr3_hand"
LINK_EE = "fr3_link8"
FRAME_BASE = "base"

PIPELINE_PILZ = "pilz_industrial_motion_planner"

class Fr3Grasper:
    def __init__(self, node_name="moveit_py_node"):
        # 初始化 MoveItPy
        self.fr3 = MoveItPy(node_name=node_name)
        rc = rclpy.logging.get_logger("moveit_ros.perception").set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.logger = get_logger("fr3_grasper")
        
        # 获取规划组件
        self.arm = self.fr3.get_planning_component(PLANNING_GROUP_ARM)
        self.hand = self.fr3.get_planning_component(PLANNING_GROUP_HAND)
        
        self.logger.info("MoveItPy 组件初始化完成")

    def plan_and_execute(
        self,
        planning_component,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        description="未知任务"
    ):
        """通用的规划与执行函数 (修复版)"""
        self.logger.info(f"--- 正在规划: {description} ---")
        
        # 1. 执行规划
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(multi_plan_parameters)
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(single_plan_parameters)
        else:
            plan_result = planning_component.plan()

        # 2. 检查结果并执行
        if plan_result:
            self.logger.info(f"[{description}] 规划成功，准备执行...")
            robot_trajectory = plan_result.trajectory
            
            # --- 关键修正 ---
            # 获取当前组件的组名 (如 "fr3_arm")
            group_name = planning_component.planning_group_name
            
            # 调用 execute (注意参数顺序：组名, 轨迹, 阻塞模式)
            self.fr3.execute(group_name, robot_trajectory, blocking=True)
            self.logger.info(f"[{description}] 执行完毕")
            return True
        else:
            self.logger.error(f"[{description}] 规划失败")
            return False
 
    def move_ptp(self, position, rpy, description="PTP运动"):
        """点到点运动 (使用 Pilz PTP)"""
        self.arm.set_start_state_to_current_state()
        
        # 设置目标
        pose_goal = self._create_pose_stamped(position, rpy)
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=LINK_EE)

        # 配置参数
        params = PlanRequestParameters(self.fr3) # 显式指定 Pilz 管道
        params.planner_id = "PTP"
        params.max_velocity_scaling_factor = 0.4
        params.max_acceleration_scaling_factor = 0.4

        return self.plan_and_execute(self.arm, single_plan_parameters=params, description=description)

    def move_lin(self, position, rpy, description="LIN运动"):
        """直线运动 (使用 Pilz LIN)"""
        self.arm.set_start_state_to_current_state()
        
        pose_goal = self._create_pose_stamped(position, rpy)
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=LINK_EE)

        # 配置参数
        params = PlanRequestParameters(self.fr3)# 显式指定 Pilz 管道
        params.planner_id = "LIN"
        params.max_velocity_scaling_factor = 0.2  # 直线运动建议慢一点

        return self.plan_and_execute(self.arm, single_plan_parameters=params, description=description)

    def control_gripper(self, command="open"):
        """控制手爪 (基于 SRDF 定义的 named state)"""
        self.hand.set_start_state_to_current_state()
        if command not in ["open", "close"]:
            self.logger.warn(f"未知的夹爪指令: {command}")
            return False
            
        self.hand.set_goal_state(configuration_name=command)
        return self.plan_and_execute(self.hand, description=f"手爪-{command}")


    def attach_object(self, object_id):
        """将物体附着到手爪"""
        self.logger.info(f"附着物体: {object_id}")
        with self.fr3.get_planning_scene_monitor().read_write() as scene:
            aco = AttachedCollisionObject()
            aco.link_name = LINK_EE 
            aco.object.id = object_id
            aco.object.operation = CollisionObject.ADD
            # 允许 gripper 手指接触该物体，忽略碰撞
            aco.touch_links = [LINK_EE, "fr3_leftfinger", "fr3_rightfinger", "fr3_hand"]
            scene.apply_attached_collision_object(aco)

    def detach_object(self, object_id):
        """分离物体"""
        self.logger.info(f"分离物体: {object_id}")
        with self.fr3.get_planning_scene_monitor().read_write() as scene:
            aco = AttachedCollisionObject()
            aco.link_name = LINK_EE
            aco.object.id = object_id
            aco.object.operation = CollisionObject.REMOVE
            scene.apply_attached_collision_object(aco)

    def _create_pose_stamped(self, position, rpy):
        q = quaternion_from_euler(*rpy)
        pose = PoseStamped()
        pose.header.frame_id = FRAME_BASE
        pose.header.stamp = rclpy.clock.Clock().now().to_msg()
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

def main():
    rclpy.init()

    bot = Fr3Grasper()
    time.sleep(10.0) # 等待初始化

    try:

        # 1. 移动到预备点

        ready_rpy = [math.radians(90), math.radians(90),0.0]  
        ready_pos = [0, -0.3, 0.5]
        
        if not bot.move_ptp(ready_pos, ready_rpy, "移动到预备点"):
            return
        time.sleep(3)
        # 2. 张开
        bot.control_gripper("open")
        time.sleep(3)
        # # 3. 直线向前抓取
        grasp_pos = [0.0, -0.4, 0.5] # 略高于物体中心
        if not bot.move_lin(grasp_pos, ready_rpy, "直线接近"):
            return

        # # 4. 逻辑附着 (MoveIt) + 物理抓取
        # bot.control_gripper("close")

        # # 5. 提起
        # lift_pos = [0.5, 0.0, 0.5]
        # if not bot.move_lin(lift_pos, ready_rpy, "提起物体"):
        #     return

        # # 6. 搬运到侧面
        # place_pos = [0.0, -0.5, 0.5]
        # if not bot.move_ptp(place_pos, ready_rpy, "移动到放置点"):
        #     return
            
        # # 7. 放下
        # place_down_pos = [0.0, -0.5, 0.12]
        # bot.move_lin(place_down_pos, ready_rpy, "直线放置下降")
        
        # bot.control_gripper("open")
        
        # # 8. 撤离
        # bot.move_lin(place_pos, ready_rpy, "撤离")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        bot.logger.error(f"发生未捕获异常: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()