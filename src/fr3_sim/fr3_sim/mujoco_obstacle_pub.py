import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import mujoco
import numpy as np
from rcl_interfaces.msg import ParameterType

class MujocoObstaclesPublisher(Node):
    def __init__(self):
        super().__init__('mujoco_obstacles_pub')
        self.pub_count = 0
        self.max_pub_count = 10
        self.declare_parameter('mujoco_model_path', '')
        # 1. 声明 obstacles 参数，明确告诉它是字符串数组
        self.declare_parameter(
            'obstacles', 
            rclpy.Parameter.Type.STRING_ARRAY
        )

        # 2. 声明 targets 参数，同样明确类型
        self.declare_parameter(
            'targets', 
            rclpy.Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter('frame_id', 'world')
        # 频率：0.5 Hz
        self.declare_parameter('update_rate', 1.0) 

        # 2. 读取参数
        model_path = self.get_parameter('mujoco_model_path').get_parameter_value().string_value
        self.obstacle_names = self.get_parameter('obstacles').get_parameter_value().string_array_value
        self.target_names = self.get_parameter('targets').get_parameter_value().string_array_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        if not model_path:
            self.get_logger().error("Parameter 'mujoco_model_path' is empty!")
            return

        # 3. 初始化 MoveIt 碰撞对象发布者
        # MoveIt 默认监听 /collision_object 话题
        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(CollisionObject, '/collision_object', qos)

        # 4. 加载 MuJoCo 模型
        try:
            self.get_logger().info(f"Loading MuJoCo model from: {model_path}")
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            
            mujoco.mj_forward(self.model, self.data)

        except Exception as e:
            self.get_logger().error(f"Failed to load MuJoCo model: {e}")
            return

        # 5. 定时器
        if update_rate > 0:
            self.timer = self.create_timer(1.0 / update_rate, self.publish_scene_objects)
        else:
            self.publish_scene_objects()

    def publish_scene_objects(self):
        """统一处理障碍物和目标物"""
        if self.pub_count >= self.max_pub_count:
            self.get_logger().info("已达到发布次数限制，停止发布。")
            self.timer.destroy()  # 销毁定时器，不再执行
            return
        # 发布障碍物
        self._process_list(self.obstacle_names, is_target=False)
        # 发布目标物
        self._process_list(self.target_names, is_target=True)
        self.pub_count += 1
    def _process_list(self, body_names, is_target):
        for name in body_names:
            msg = self._create_collision_object_from_body(name)
            if msg:
                self.pub.publish(msg)
                role = "TARGET" if is_target else "OBSTACLE"
                # debug 日志，防止刷屏
                self.get_logger().debug(f"Published {role}: {name}")

    def _create_collision_object_from_body(self, body_name):
        try:
            # 1. 获取 Body ID
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        except ValueError:
            self.get_logger().warn(f"Body '{body_name}' not found!")
            return None

        msg = CollisionObject()
        msg.id = body_name # 一个 Body 对应一个 MoveIt ID
        msg.header.frame_id = self.frame_id
        msg.operation = CollisionObject.ADD

        # 2. 遍历属于该 Body 的所有 Geom
        # geomadr 是该 Body 第一个 geom 的索引，geomnb 是 geom 的数量
        first_gid = self.model.body_geomadr[bid]
        geom_count = self.model.body_geomnum[bid]

        for i in range(geom_count):
            gid = first_gid + i
            
            # 获取该 Geom 相对于 Body 的位置和旋转 (或使用全局坐标)
            # 建议使用全局坐标 geom_xpos，因为 MoveIt 的 CollisionObject 默认相对于 frame_id
            pos = self.data.geom_xpos[gid]
            mat = self.data.geom_xmat[gid].reshape(3, 3)
            quat = self.rotmat_to_quaternion(mat)

            g_type = self.model.geom_type[gid]
            g_size = self.model.geom_size[gid]

            primitive = SolidPrimitive()
        
            #  类型与尺寸自动转换
            if g_type == 6: # BOX
                primitive.type = SolidPrimitive.BOX
                # MuJoCo size 是半长 (half-extents)，ROS 需要全长
                primitive.dimensions = [g_size[0]*2, g_size[1]*2, g_size[2]*2]
            
            elif g_type == 2: # SPHERE
                primitive.type = SolidPrimitive.SPHERE
                primitive.dimensions = [g_size[0]] # radius
            
            elif g_type == 5: # CYLINDER
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = [g_size[1]*2, g_size[0]] # [height, radius] 注意顺序！
                
            else:
                self.get_logger().warn(f"Geom {body_name} type {g_type} not supported yet (Mesh/Plane/Capsule).")
                return None

            msg.primitives.append(primitive)

            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            msg.primitive_poses.append(pose)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"OBJECT CHECK: name={body_name}, bid={bid}, pos={pos}")
        return msg

    @staticmethod
    def rotmat_to_quaternion(mat):
        """
        手动将 3x3 旋转矩阵转换为四元数 [x, y, z, w]
        """
        m = mat
        tr = m[0, 0] + m[1, 1] + m[2, 2]

        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            w = 0.25 * S
            x = (m[2, 1] - m[1, 2]) / S
            y = (m[0, 2] - m[2, 0]) / S
            z = (m[1, 0] - m[0, 1]) / S
        elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            w = (m[2, 1] - m[1, 2]) / S
            x = 0.25 * S
            y = (m[0, 1] + m[1, 0]) / S
            z = (m[0, 2] + m[2, 0]) / S
        elif m[1, 1] > m[2, 2]:
            S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            w = (m[0, 2] - m[2, 0]) / S
            x = (m[0, 1] + m[1, 0]) / S
            y = 0.25 * S
            z = (m[1, 2] + m[2, 1]) / S
        else:
            S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            w = (m[1, 0] - m[0, 1]) / S
            x = (m[0, 2] + m[2, 0]) / S
            y = (m[1, 2] + m[2, 1]) / S
            z = 0.25 * S
            
        return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)
    node = MujocoObstaclesPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()