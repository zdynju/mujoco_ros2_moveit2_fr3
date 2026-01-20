from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 静态 TF 发布者：告诉 ROS "world_camera_optical_frame" 在哪
    # 这里的数值是你运行第一步脚本得到的 (假设值如下，请用你算出来的替换)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_camera_tf',
        arguments=['--x', '-2.8', '--y', '-2.8', '--z', '2.1',
            '--qx', '0.8205', '--qy', '-0.3399', '--qz', '0.1759', '--qw', '-0.4247',
            '--frame-id', 'base', '--child-frame-id', 'camera1_mujoco_frame']
) 

    # 2. 你的点云生成节点
    pc_node = Node(
        package='fr3_application',
        executable='pointcloud', # 确保 setup.py 里配了这个 entry point
        name='pc_generator',
        output='screen',
        parameters=[{"use_sim_time":True}]
    )

    return LaunchDescription([
        static_tf,
        pc_node
    ])