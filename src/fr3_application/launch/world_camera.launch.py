from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    

    # 2. 你的点云生成节点
    pc_node = Node(
        package='fr3_application',
        executable='pointcloud', # 确保 setup.py 里配了这个 entry point
        name='pc_generator',
        output='screen',
        parameters=[{"use_sim_time":True}]
    )

    return LaunchDescription([
        # # world_base_static_tf,
        # static_tf,
        pc_node
    ])