import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

'''
作者：Daohui Liu
邮箱：daohui.liu@mail.utoronto.ca
@2025-07-22

本launch文件用于启动moveit核心节点，伺服节点以及rviz可视化
'''

def generate_launch_description():
    # ========= MoveIt Config =========
    moveit_config = (
        MoveItConfigsBuilder("ur5", package_name="ur5_gripper_moveit_config")
        .robot_description(file_path="config/ur5.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # ========= RViz Config =========
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur5_gripper_moveit_config"), "config", "moveit.rviz"]
    )

    # ========= MoveGroup Node =========
    move_group_params = [
        moveit_config.to_dict(),
        {
            "publish_robot_description_semantic": True,
            "publish_robot_description": False,
            "allow_trajectory_execution": True,
            "capabilities": "",
            "disable_capabilities": "",
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
        },
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    # ========= Servo Node =========
    servo_params = {
        "moveit_servo": ParameterBuilder("ur5_gripper_moveit_config")
        .yaml("config/ur_real_servo_config.yaml")
        .to_dict()
    }

    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "manipulator"}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # ========= Robot State Publisher =========
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="screen",
    #     parameters=[moveit_config.robot_description],
    # )

    # ========= RViz Node =========
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=rviz_parameters,
    )

    # ========= Launch Description =========
    ld = LaunchDescription()

    ld.add_action(move_group_node)
    ld.add_action(servo_node)
    # ld.add_action(robot_state_publisher)
    ld.add_action(rviz_node)

    return ld
