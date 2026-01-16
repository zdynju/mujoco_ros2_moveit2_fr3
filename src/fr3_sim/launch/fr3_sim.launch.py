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
    # 1. 定义一些方便的变量
    package_name = 'fr3_sim'
    
    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 2. 声明路径
    # 模型文件 (URDF/Xacro)
    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'urdf', 'fr3.urdf.xacro'
    ])
    
    # MuJoCo 场景文件 (XML)
    mujoco_scene_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', 'fr3_mujoco.xml'
    ])

    # 控制器配置文件 (YAML)
    controller_config = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', 'ros2_controllers.yaml'
    ])

    # RViz 配置文件 (如果有的话，没有就用默认空配置)
    rviz_config = PathJoinSubstitution([FindPackageShare(package_name), 'rviz', 'sim.rviz'])

    # 3. 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo/MuJoCo) clock if true'
    )

    # 4. 解析 Xacro 得到 Robot Description
    # 注意：这里假设你的 fr3.urdf.xacro 能够通过 xacro 命令直接编译
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content,value_type=str)}

    # 5. 节点定义
    
    # A. Robot State Publisher (负责发布静态 TF)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # B. MuJoCo ROS2 Control Node (核心仿真节点)
    # 这个节点既跑物理引擎，又作为 Controller Manager
    node_mujoco = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
            {'mujoco_model_path': mujoco_scene_file}, # 传入 scene.xml 路径
            controller_config  # 传入控制器参数
        ]
    )

    # C. 启动 Joint State Broadcaster (负责发布 /joint_states)
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # D. 启动手臂控制器 (注意：名字必须和你的 yaml 文件里的一致！)
    # 假设你的 yaml 里叫 "fr3_arm_controller" 或 "joint_trajectory_controller"
    # 请根据你的 ros2_controllers.yaml 修改下面的名字
    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_hand_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    # E. RViz2
    # node_rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config], # 如果有配置文件，取消这行注释
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # 6. 设置启动顺序
    # 只有当 joint_state_broadcaster 启动成功后，才启动 arm_controller
    # (这是一种保护机制，防止控制器找不到状态)
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_arm_controller,spawn_hand_controller],
        )
    )


    auto_home_command = TimerAction(
    period=3.0, # 等待 3 秒，确保控制器已经启动
    actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once',
                '/fr3_arm_controller/joint_trajectory',
                'trajectory_msgs/msg/JointTrajectory',
                '{header: {frame_id: world}, joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], points: [{positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785], time_from_start: {sec: 2, nanosec: 0}}]}'
            ],
            output='screen'
        )
    ]
)
    return LaunchDescription([
        declare_use_sim_time,
        node_robot_state_publisher,
        node_mujoco,
        spawn_joint_state_broadcaster,
        delay_arm_controller_spawner,
        auto_home_command
        # node_rviz
    ])