#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class GripperController:
    def __init__(self,node):
        self.gripper_client = ActionClient(node, GripperCommand, '/fr3_hand_controller/gripper_cmd')
        self.node = node
        self.node.get_logger().info("正在连接夹爪 Action Server...")
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("连接超时！请检查 fr3_hand_controller 是否启动。")
        else:
            self.node.get_logger().info("夹爪 Action Server 连接成功！")

    def _send_command(self, position: float, max_effort: float = 100.0):
        """发送指令（异步），返回 goal_future"""
        goal_msg = GripperCommand.Goal()
        # 注意嵌套结构
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.node.get_logger().info(f"发送指令 -> Pos: {position:.4f}, Force: {max_effort}")
        return self.gripper_client.send_goal_async(goal_msg)

    def open(self):
        """完全张开 (异步)"""
        return self._execute_blocking(command_type="open")

    def close(self):
        """完全闭合 (异步)"""
        return self._execute_blocking(command_type="close")

    def _execute_blocking(self, command_type="close"):
        """
        同步执行方法：发送指令并阻塞等待，直到动作完成
        :param command_type: "open" 或 "close"
        :return: True=成功(到位或抓住了), False=失败
        """
        # 1. 发送指令
        if command_type == "open":
            future_goal = self._send_command(0.08, 100.0)
        else:
            future_goal = self._send_command(0.0, 100.0)

        # 2. 等待服务器确认接收 (Wait for Accept)
        rclpy.spin_until_future_complete(self.node, future_goal)
        goal_handle = future_goal.result()

        if not goal_handle.accepted:
            self.node.get_logger().error("指令被服务器拒绝！")
            return False

        # 3. 等待动作执行完成 (Wait for Result)
        future_result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, future_result)

        # 4. 获取最终结果
        result = future_result.result().result
        status = future_result.result().status
        
        # 只要到位(reached) 或者 堵转(stalled, 即抓住了) 都算成功
        if result.stalled or result.reached_goal:
            self.node.get_logger().info(f"✅ 动作完成! (Stalled: {result.stalled})")
            return True
        else:
            self.node.get_logger().warn("⚠️ 动作未完全成功")
            return False