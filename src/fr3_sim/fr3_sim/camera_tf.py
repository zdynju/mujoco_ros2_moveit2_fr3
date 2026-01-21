import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.exceptions import ParameterAlreadyDeclaredException # 引入异常类

class CameraTFBroadcaster(Node):
    def __init__(self):
        super().__init__('camera_tf_broadcaster')
        
        try:
            self.declare_parameter('use_sim_time', False) 
        except ParameterAlreadyDeclaredException:
            # 如果参数已经存在（比如从 launch 文件传进来的），就什么都不做
            pass
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 频率建议设为 30Hz 或 50Hz，略高于点云频率即可
        self.timer = self.create_timer(1.0/50.0, self.broadcast_timer_callback)
        
        self.get_logger().info("Dynamic Camera TF Broadcaster Started (base -> camera1_mujoco_frame)")

    def broadcast_timer_callback(self):
        t = TransformStamped()
        print("update_camera_tf")
        # 【核心】使用当前节点时间（仿真时间）作为时间戳
        t.header.stamp = self.get_clock().now().to_msg()
        
        # 对应你原来 static_transform_publisher 的 --frame-id
        t.header.frame_id = 'base'              
        
        # 对应你原来 static_transform_publisher 的 --child-frame-id
        t.child_frame_id = 'camera1_mujoco_frame' 

        # --- 填入你提供的位移参数 ---
        t.transform.translation.x = -2.8
        t.transform.translation.y = -2.8
        t.transform.translation.z = 2.1
        
        # --- 填入你提供的四元数参数 ---
        t.transform.rotation.x = 0.8205
        t.transform.rotation.y = -0.3399
        t.transform.rotation.z = 0.1759
        t.transform.rotation.w = -0.4247

        # 发布消息
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