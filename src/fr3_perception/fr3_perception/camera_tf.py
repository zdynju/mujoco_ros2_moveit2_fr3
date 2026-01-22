import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time

class CameraTFBroadcaster(Node):
    def __init__(self):
        super().__init__('camera_tf_broadcaster')
        
        # 使用静态广播器
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 发布静态 TF
        self.publish_static_tf()
        
        self.get_logger().info("Static Camera TF Broadcaster Started (base -> camera1_mujoco_frame)")

    def publish_static_tf(self):
        t = TransformStamped()
        
        # --- 坐标系 ---
        t.header.frame_id = 'base'
        t.child_frame_id = 'camera1_mujoco_frame'

        # --- 位移参数 ---
        t.transform.translation.x = -2.8
        t.transform.translation.y = -2.8
        t.transform.translation.z = 2.1

        # --- 四元数参数 ---
        t.transform.rotation.x = 0.8205
        t.transform.rotation.y = -0.3399
        t.transform.rotation.z = 0.1759
        t.transform.rotation.w = -0.4247

        # 使用 0 时间戳，保证 static TF 锁存且 late subscribers 可以拿到
        t.header.stamp = rclpy.time.Time().to_msg()

        # 发送多次，确保所有订阅者收到
        for _ in range(3):
            self.tf_broadcaster.sendTransform(t)
            time.sleep(0.05)  # 小延时，避免发送过快被丢掉

        # 可选：稍微等待，让点云 / octomap 节点拿到 transform
        time.sleep(0.2)

def main():
    rclpy.init()
    node = CameraTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
