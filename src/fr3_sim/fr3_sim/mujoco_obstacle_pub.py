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
            
            # 执行一次正向运动学计算，确保 xpos 和 xmat 是基于 qpos0 (默认位置) 计算的
            mujoco.mj_kinematics(self.model, self.data)
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
        # 发布障碍物
        self._process_list(self.obstacle_names, is_target=False)
        # 发布目标物
        self._process_list(self.target_names, is_target=True)

    def _process_list(self, geom_names, is_target):
        for name in geom_names:
            msg = self._create_collision_object(name)
            if msg:
                self.pub.publish(msg)
                role = "TARGET" if is_target else "OBSTACLE"
                # debug 日志，防止刷屏
                self.get_logger().debug(f"Published {role}: {name}")

    def _create_collision_object(self, geom_name):
        try:
            # 获取 geom 的 ID
            # 参数顺序为：模型对象，对象类型枚举，对象名字
            # 核心修正：使用 mjOBJ_GEOM 而不是 mjtGEOM
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        except ValueError:
            self.get_logger().warn(f"Geom '{geom_name}' not found in MuJoCo model!")
            return None

        # 获取位置和旋转
        # 注意：如果仿真是在运行的，这里应该连接仿真器的 pose 话题，
        # 但如果是静态场景加载，读取 self.data 就足够了 (基于 mj_kinematics)
        pos = self.data.geom_xpos[gid]
        mat = self.data.geom_xmat[gid].reshape(3, 3)
        quat = self.rotmat_to_quaternion(mat)

        # 获取类型和尺寸
        g_type = self.model.geom_type[gid] # 2: sphere, 3: capsule, 5: cylinder, 6: box
        g_size = self.model.geom_size[gid] # 根据类型含义不同

        # 构建 ROS 消息
        msg = CollisionObject()
        msg.id = geom_name  
        msg.header.frame_id = self.frame_id
        msg.operation = CollisionObject.ADD

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
            self.get_logger().warn(f"Geom {geom_name} type {g_type} not supported yet (Mesh/Plane/Capsule).")
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
        # 在 _create_collision_object 函数里
        self.get_logger().info(f"OBJECT CHECK: name={geom_name}, gid={gid}, pos={pos}")
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