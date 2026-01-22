import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class CameraTFBroadcaster(Node):
    def __init__(self):
        super().__init__('camera_tf_broadcaster')
        
        # 使用静态广播器
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 发布一次即可，静态 TF 会被锁存
        self.publish_static_tf()
        
        self.get_logger().info("Static Camera TF Broadcaster Started (base -> camera1_mujoco_frame)")

    def publish_static_tf(self):
        t = TransformStamped()
        
        # 静态 TF 的时间戳通常设为 0 或者当前时间，tf2 会特殊处理
        t.header.stamp = self.get_clock().now().to_msg()
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

        # 发送变换
        self.tf_broadcaster.sendTransform(t)

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


# -2.800, -2.800, 2.100 -2.186, 0.000, -0.785