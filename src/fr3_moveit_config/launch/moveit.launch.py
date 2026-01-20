import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_pkg_name = "fr3_moveit_config" 

    urdf_file_path = os.path.join(
        get_package_share_directory("franka_description"),
        "robots", "fr3", "fr3.urdf.xacro"
    )
    print("hello")
    moveit_config = MoveItConfigsBuilder("fr3", package_name=moveit_pkg_name) \
        .robot_description(
            file_path=urdf_file_path
        ) \
        .robot_description_semantic(file_path="config/fr3.srdf") \
        .trajectory_execution(file_path="config/moveit_controllers.yaml") \
        .joint_limits(file_path="config/joint_limits.yaml") \
        .robot_description_kinematics(file_path="config/kinematics.yaml") \
        .planning_pipelines(
            pipelines=["ompl"] 
        ) \
        .to_moveit_configs()


    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(), 
            
            {"use_sim_time": True},  
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
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}, 
        ],
    )

    return LaunchDescription([
        run_move_group_node,
        run_rviz_node,
    ])