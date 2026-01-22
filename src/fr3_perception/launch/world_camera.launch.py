from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    dynamic_tf_node = Node(
        package='fr3_perception',
        executable='camera_tf', 
        name='world_camera_tf_dynamic',
        # 这里把 launch 里的 use_sim_time=true 传给了 Python 节点
        parameters=[{'use_sim_time': True}],   
        output='screen'
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
        dynamic_tf_node,
        pc_node
    ])