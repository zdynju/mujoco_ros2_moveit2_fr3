#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from moveit.planning import MoveItPy, PlanRequestParameters
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler
from .gripperController import GripperController

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
        self.logger = get_logger("fr3_grasper")
        self.gripper_node = Node("gripper_node")
        self.aco_pub = self.gripper_node.create_publisher(AttachedCollisionObject, 
            "/attached_collision_object", 
            10)
        # 获取规划组件
        self.arm = self.fr3.get_planning_component(PLANNING_GROUP_ARM)
        self.hand = GripperController(self.gripper_node)

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
    def gripper_open(self):
        return self.hand.open()
    
    def gripper_close(self):
        return self.hand.close()
            
    def move_ptp(self, position, rpy, description="PTP运动"):
        """点到点运动 (使用 Pilz PTP)"""
        self.arm.set_start_state_to_current_state()
        
        # 设置目标
        pose_goal = self._create_pose_stamped(position, rpy)
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=LINK_EE)

        # 配置参数
        params = PlanRequestParameters(self.fr3) # 显式指定 Pilz 管道
        params.planner_id = "PTP"
        params.planning_pipeline = PIPELINE_PILZ
        params.max_velocity_scaling_factor = 0.3
        params.max_acceleration_scaling_factor = 0.3

        return self.plan_and_execute(self.arm, single_plan_parameters=params, description=description)

    def move_lin(self, position, rpy, description="LIN运动"):
        """直线运动 (使用 Pilz LIN)"""
        self.arm.set_start_state_to_current_state()
        
        pose_goal = self._create_pose_stamped(position, rpy)
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=LINK_EE)

        # 配置参数
        params = PlanRequestParameters(self.fr3)# 显式指定 Pilz 管道
        params.planner_id = "LIN"
        params.planning_pipeline = PIPELINE_PILZ
        params.max_velocity_scaling_factor = 0.2  # 直线运动建议慢一点

        return self.plan_and_execute(self.arm, single_plan_parameters=params, description=description)
    

    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

    def attach_object(self, object_id):
        """将物体附着到手爪（MoveItPy 正确方式）"""
        self.logger.info(f"附着物体: {object_id}")

        # 1. 构造 AttachedCollisionObject
        aco = AttachedCollisionObject()
        aco.link_name = LINK_EE
        aco.object.id = object_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = [
            LINK_EE,
            "fr3_leftfinger",
            "fr3_rightfinger",
            "fr3_hand",
        ]

        # 2. 直接修改 MoveItPy 使用的 PlanningScene（关键）
        psm = self.fr3.get_planning_scene_monitor()
        with psm.read_write() as scene:
            scene.process_attached_collision_object(aco)

            self.logger.info(f"物体 {object_id} 已附着到 {LINK_EE}")
            acm = scene.allowed_collision_matrix

            for link in [
                "fr3_leftfinger",
                "fr3_rightfinger",
                "fr3_hand",
            ]:
                acm.set_entry(object_id, link, True)
        
        time.sleep(0.5)

        self.logger.info(f"{object_id} 已附着并允许与手指接触")

            

    def detach_object(self, object_id):
        """从手爪分离物体（MoveItPy 正确方式）"""
        self.logger.info(f"分离物体: {object_id}")

        # 1. 构造 AttachedCollisionObject
        aco = AttachedCollisionObject()
        aco.link_name = LINK_EE
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE

        # 2. 修改 PlanningScene
        psm = self.fr3.get_planning_scene_monitor()
        with psm.read_write() as scene:
            scene.process_attached_collision_object(aco)

            self.logger.info(f"物体 {object_id} 已从 {LINK_EE} 分离")
            
            acm = scene.allowed_collision_matrix
            for link in [
                "fr3_leftfinger",
                "fr3_rightfinger",
                "fr3_hand",
            ]:
                acm.remove_entry(object_id, link)
            print("acm",acm)


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
    time.sleep(5.0) # 等待初始化

    try:
       
        # 1. 移动到预备点

        ready_rpy = [0.0, math.pi,0.0]  
        ready_pos = [0.0, -0.5, 0.5]
        
        if not bot.move_ptp(ready_pos, ready_rpy, "移动到预备点"):
            return
        time.sleep(1)
        # 2. 张开
        bot.gripper_open()

        # # 3. 直线向前抓取
        grasp_pos = [0, -0.5, 0.4] # 略高于物体中心
        if not bot.move_lin(grasp_pos, ready_rpy, "直线接近"):
            return
        #4. grasp
        bot.gripper_close()
        #
        bot.attach_object("target1")
        # 5. 提起
        lift_pos = [0.0, -0.5, 0.5]
        if not bot.move_lin(lift_pos, ready_rpy, "提起物体"):
            return
        
        put_pos = [0.4, -0.2, 0.4]
        if not bot.move_ptp(put_pos, ready_rpy, "put物体"):
            return
        
        # down_pos = [0.3, -0.2, 0.25]
        # if not bot.move_ptp(down_pos, ready_rpy, "down物体"):
        #     return
        
        bot.detach_object("target1")
        bot.gripper_open()

        lift_pos = [0.4, -0.2, 0.6]
        if not bot.move_ptp(lift_pos, ready_rpy, "提起物体"):
            return
        bot.gripper_close()
        time.sleep(3)
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