import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudGenerator(Node):
    def __init__(self):
        super().__init__('pc_generator')
        self.bridge = CvBridge()
        self.depth_image = None
        self.cam_info = None

        # 订阅对齐后的深度图和相机内参
        self.create_subscription(Image, '/camera1/aligned_depth_to_color/image_raw',
                                 self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera1/color/camera_info',
                                 self.cam_info_callback, 10)

        # 发布点云话题
        self.pc_pub = self.create_publisher(PointCloud2, '/camera1/pointcloud', 10)
        self.get_logger().info("点云生成器已启动...")

    def cam_info_callback(self, msg):
        # 只需要获取一次内参即可
        if self.cam_info is None:
            self.cam_info = msg
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f"相机内参已加载: fx={self.fx}, fy={self.fy}")

    def depth_callback(self, msg):
        if self.cam_info is None:
            return
        
        # 将消息转换为 OpenCV 格式
        # 注意：使用 passthrough 以保留原始深度单位（通常为米）
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        msg.header.frame_id = "camera1_mujoco_frame"
        # 将当前消息的 header 传给生成函数，确保时间戳完全同步
        self.generate_pc(msg.header)

    def generate_pc(self, header):
        depth = self.depth_image.astype(np.float32)
        h, w = depth.shape

        # 创建像素网格
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        
        # 过滤无效深度点（0表示无效，3.0米以上太远）
        z = depth
        mask = (z > 0.0) & (z < 6.0)
        
        # 只对有效点进行反投影计算，大幅提升速度
        z_valid = z[mask]
        u_valid = u[mask]
        v_valid = v[mask]

        # 计算 3D 坐标
        x = (u_valid - self.cx) * z_valid / self.fx
        y = (v_valid - self.cy) * z_valid / self.fy
        
        # 组合成 (N, 3) 的数组
        points = np.stack((x, y, z_valid), axis=-1).astype(np.float32)

        # 直接使用 numpy 数组创建 PointCloud2，避免使用 .tolist() 导致的 CPU 剧烈震荡
        pc_msg = pc2.create_cloud_xyz32(header, points)
        self.get_logger().info(f"Published pointcloud, num points: {points.shape[0]}")

        # 发布
        self.pc_pub.publish(pc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()