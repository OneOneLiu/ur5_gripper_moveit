import os
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from launch.actions import TimerAction

'''
作者：Daohui Liu
邮箱：daohui.liu@mail.utoronto.ca
@2025-07-22

本launch文件用于启动moveit核心节点，伺服节点以及rviz可视化

它并不启动robot_state_publisher节点，这个是属于对机器人的控制，而不是moveit相关的东西，所以做了隔离。robot_state_publisher节点由跟（仿真）硬件通信的launch文件启动
'''

def generate_launch_description():
    # ========= MoveIt Config =========
    moveit_config = (
        MoveItConfigsBuilder("ur5", package_name="ur5_gripper_moveit_config")
        .robot_description(file_path="config/ur5.urdf_isaac.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers_isaac.yaml") # 使用这个配置文件，而不是默认的moveit_controllers.yaml
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
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

    # ========= Servo Container Node =========
    servo_params = {
        "moveit_servo": ParameterBuilder("ur5_gripper_moveit_config")
        .yaml("config/ur_isaac_servo_config.yaml")
        .to_dict()
    }

    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "manipulator"}

    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    acceleration_filter_update_period,
                    planning_group_name,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            ),
        ],
        output="screen",
    )

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

    # ========= Ignore Certain Collisions Node =========
    ignore_certain_collisions_node = Node(
        package="ur5_gripper_moveit_config",
        executable="ignore_certain_collisions",
        name="ignore_certain_collisions_node",
        output="screen",
    )

    # ========= Launch Description =========
    ld = LaunchDescription()

    ld.add_action(move_group_node)
    ld.add_action(container)
    ld.add_action(rviz_node)

    # 添加一个延迟，让机器人先启动，然后忽略某些碰撞, 否则无法生效
    ld.add_action(
        TimerAction(
            period=10.0,
            actions=[ignore_certain_collisions_node]
        )
    )

    return ld
