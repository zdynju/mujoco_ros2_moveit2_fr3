import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    package_name = 'fr3_sim'
    
    # 1. 获取参数配置
    # 这一步将 use_sim_time 变成了一个可以在后续传给 Node 的变量
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 2. 声明启动参数 (给 launch 命令行用的)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo/MuJoCo) clock if true'
    )

    # 3. 路径配置
    # 模型文件 (URDF/Xacro)
    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'urdf', 'fr3.urdf.xacro'
    ])

    # 控制器配置文件 (YAML)
    controller_config = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', 'ros2_controllers.yaml'
    ])

    # 4. 解析 Xacro 得到 Robot Description
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ==========================================================
    # 5. 节点定义
    # ==========================================================

    # 【关键】动态 TF 发布节点
    # 注意：executable='camera_tf' 必须和你 setup.py 里 entry_points 的名字一致
    dynamic_tf_node = Node(
        package='fr3_sim',
        executable='camera_tf', 
        name='world_camera_tf_dynamic',
        # 这里把 launch 里的 use_sim_time=true 传给了 Python 节点
        parameters=[{'use_sim_time': use_sim_time}],   
        output='screen'
    )

    # A. Robot State Publisher (负责发布静态 TF 和 URDF)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # B. MuJoCo ROS2 Control Node (核心仿真节点)
    node_mujoco = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
            controller_config
        ]
    )

    # C. 启动 Joint State Broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # D. 启动手臂控制器
    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # E. 启动夹爪控制器
    spawn_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_hand_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # F. 事件处理器：等 joint_state_broadcaster 启动后再启动控制器
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_arm_controller, spawn_hand_controller],
        )
    )

    # ==========================================================
    # 6. 返回 Launch 描述
    # ==========================================================
    return LaunchDescription([
        declare_use_sim_time,          # 1. 先声明参数
        node_robot_state_publisher,    # 2. 发布机器人模型
        node_mujoco,                   # 3. 启动物理引擎
        dynamic_tf_node,               # 4. 启动动态相机 TF (解决 filtered_points 问题)
        spawn_joint_state_broadcaster, # 5. 启动关节状态广播
        delay_arm_controller_spawner,  # 6. 启动控制器
    ])