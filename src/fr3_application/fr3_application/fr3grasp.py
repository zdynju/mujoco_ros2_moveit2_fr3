#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.logging import get_logger
from moveit.planning import MoveItPy, PlanRequestParameters
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from tf_transformations import quaternion_from_euler

# 定义常用参数
PLANNING_GROUP_ARM = "fr3_arm"
PLANNING_GROUP_HAND = "fr3_hand"
LINK_EE = "fr3_link8"
FRAME_BASE = "base"

class Fr3Grasper:
    def __init__(self, node_name="moveit_py_node"):
        # MoveItPy 会自动初始化节点，无需再次调用 rclpy.create_node (除非有特殊需求)
        self.fr3 = MoveItPy(node_name=node_name)
        self.logger = get_logger("fr3_grasper")
        
        # 获取规划组件
        self.arm = self.fr3.get_planning_component(PLANNING_GROUP_ARM)
        self.hand = self.fr3.get_planning_component(PLANNING_GROUP_HAND)
        
        self.logger.info("MoveItPy 组件初始化完成")

    def plan_and_execute(self, component, single_plan_params=None, description="未知任务"):
        """通用的规划与执行函数"""
        self.logger.info(f"--- 开始任务: {description} ---")
        
        # 1. 规划
        if single_plan_params:
            plan_result = component.plan(single_plan_params)
        else:
            plan_result = component.plan()

        # 2. 检查规划结果
        if plan_result:
            self.logger.info(f"[{description}] 规划成功，开始执行...")
            # 3. 执行
            # 注意：execute 默认是阻塞的 (blocking=True)
            success = self.fr3.execute(plan_result.trajectory, controllers=[])
            if success:
                self.logger.info(f"[{description}] 执行完成")
                return True
            else:
                self.logger.error(f"[{description}] 执行失败 (控制器报错)")
                return False
        else:
            self.logger.error(f"[{description}] 规划失败")
            return False

    def move_ptp(self, position, rpy, description="PTP运动"):
        """点到点运动 (使用 Pilz PTP 或 OMPL)"""
        self.arm.set_start_state_to_current_state()
        
        # 设置目标位姿
        pose_goal = self._create_pose_stamped(position, rpy)
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=LINK_EE)

        # 配置参数
        params = PlanRequestParameters(self.fr3, "pilz_industrial_motion_planner")
        params.planner_id = "PTP"
        params.max_velocity_scaling_factor = 0.5  # 速度限制
        params.max_acceleration_scaling_factor = 0.5

        return self.plan_and_execute(self.arm, params, description)

    def move_lin(self, position, rpy, description="LIN运动"):
        """直线运动 (使用 Pilz LIN)"""
        self.arm.set_start_state_to_current_state()
        
        pose_goal = self._create_pose_stamped(position, rpy)
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=LINK_EE)

        params = PlanRequestParameters(self.fr3, "pilz_industrial_motion_planner")
        params.planner_id = "LIN"
        params.max_velocity_scaling_factor = 0.3 # 直线运动通常慢一点

        return self.plan_and_execute(self.arm, params, description)

    def control_gripper(self, command="open"):
        """控制手爪"""
        self.hand.set_start_state_to_current_state()
        if command not in ["open", "close"]:
            self.logger.warn(f"未知的夹爪指令: {command}")
            return False
            
        # 假设 SRDF 中定义了 'open' 和 'close' 的 group state
        self.hand.set_goal_state(configuration_name=command)
        return self.plan_and_execute(self.hand, description=f"手爪动作-{command}")

    def attach_object(self, object_id):
        """将物体附着到手爪 (用于 MoveIt 碰撞检测)"""
        self.logger.info(f"尝试附着物体: {object_id}")
        with self.fr3.get_planning_scene_monitor().read_write() as scene:
            attached_object = AttachedCollisionObject()
            attached_object.link_name = LINK_EE 
            attached_object.object.id = object_id
            attached_object.object.operation = CollisionObject.ADD
            # 设置允许发生碰撞的连杆，防止附着后报错
            attached_object.touch_links = [LINK_EE, "fr3_leftfinger", "fr3_rightfinger"]
            scene.apply_attached_collision_object(attached_object)

    def detach_object(self, object_id):
        """分离物体"""
        self.logger.info(f"尝试分离物体: {object_id}")
        with self.fr3.get_planning_scene_monitor().read_write() as scene:
            attached_object = AttachedCollisionObject()
            attached_object.link_name = LINK_EE
            attached_object.object.id = object_id
            attached_object.object.operation = CollisionObject.REMOVE
            scene.apply_attached_collision_object(attached_object)

    def _create_pose_stamped(self, position, rpy):
        """辅助函数：创建 PoseStamped 消息"""
        q = quaternion_from_euler(*rpy)
        pose = PoseStamped()
        pose.header.frame_id = FRAME_BASE
        pose.header.stamp = rclpy.clock.Clock().now().to_msg()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

def main():
    rclpy.init()
    
    # 1. 初始化控制类
    bot = Fr3Grasper()
    
    # 等待系统稳定
    time.sleep(2.0)

    try:
        # --- 任务序列 ---
        
        # 1. 移动到预备点 (上方)
        ready_pos = [0.5, 0.0, 0.5]
        ready_rpy = [0.0, math.radians(180), math.radians(45)] # 爪子垂直向下
        if not bot.move_ptp(ready_pos, ready_rpy, "移动到预备点"):
            return

        # 2. 张开手爪
        bot.control_gripper("open")

        # 3. 直线接近物体 (假设物体在正下方)
        grasp_pos = [0.5, 0.0, 0.2] # Z轴下降
        if not bot.move_lin(grasp_pos, ready_rpy, "直线接近"):
            return

        # 4. 附着物体逻辑 (逻辑上的，告诉MoveIt现在这是机器人的一部分)
        # 注意：这里的 object_id 必须与 mujoco_obstacle_pub 发布的一致
        bot.attach_object("box_1") 

        # 5. 闭合手爪
        bot.control_gripper("close")

        # 6. 提起物体
        lift_pos = [0.5, 0.0, 0.5]
        if not bot.move_lin(lift_pos, ready_rpy, "提起物体"):
            return

        # 7. 移动到放置点
        place_pos = [0.0, -0.5, 0.5]
        if not bot.move_ptp(place_pos, ready_rpy, "移动到放置点"):
            return
            
        # 8. 放下
        place_down_pos = [0.0, -0.5, 0.2]
        bot.move_lin(place_down_pos, ready_rpy, "直线下降")
        bot.control_gripper("open")
        bot.detach_object("box_1")
        
        # 9. 回到安全高度
        bot.move_lin(place_pos, ready_rpy, "离开")

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()