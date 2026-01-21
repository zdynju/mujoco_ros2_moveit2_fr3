from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    moveit_pkg_name = "fr3_moveit_config"

    world_base_static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="world_base_static_transform_publisher",
    output="log",
    arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
    )

    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name=moveit_pkg_name)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("fr3_application")+"/config/moveit_cpp.yaml"
        )
        .sensors_3d(file_path ="config/sensors_3d.yaml")
        .to_moveit_configs()
    )


    # ------------------------------------------------------------s
    # 2. 启动你的 MoveItPy 节点
    # ------------------------------------------------------------
    grasp_node = Node(
        package="fr3_application",
        executable="fr3grasp",          # setup.py 里的 entry point
        name="moveit_py",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}  # ⭐ 关键：参数在 launch 阶段注入
        ],
    )


    run_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            moveit_config.package_path, "rviz", "rviz.rviz")],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}, # 同上，保持时间同步
        ],
    )
    return LaunchDescription([
        world_base_static_tf,
        grasp_node,
        # run_move_group_node,
        run_rviz_node
    ])
