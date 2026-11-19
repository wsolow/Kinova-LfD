"""
gen3.launch.py.

Main launch file for Kinova Gen3 Arm with Robotiq 2F 85 gripper

Written by Will Solow, 2025. IRAS Lab.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Packages to load
    pkg_kortex_vision = get_package_share_directory("kinova_vision")
    pkg_kinova_lfd = get_package_share_directory("kinova_lfd_ros")
    pkg_kortex_bringup = get_package_share_directory("kortex_bringup")

    # Variables
    use_fake_hardware = LaunchConfiguration("use_fake_hardware", default="true")
    use_wrist_camera = LaunchConfiguration("use_wrist_camera", default="true")
    robot_ip = LaunchConfiguration("robot_ip", default="192.168.1.10")
    use_table_camera = LaunchConfiguration("use_table_camera", default="false")
    use_servo_controller = LaunchConfiguration("use_servo_controller", default="true")

    # Kinova Arm Launch Description
    kinova_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_kortex_bringup, "launch", "gen3.launch.py"])),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "robot_ip": robot_ip,
            "gripper": "robotiq_2f_85",
            "vision": use_wrist_camera,
        }.items(),
    )

    kinova_tool_pose_b_launch = Node(
        package="kinova_lfd_ros",
        executable="tool_publisher",
        name="tcp_pose_b_pub",
        output="screen",
    )

    # Load the twist controller by default for teleop
    switch_controllers = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "until ros2 control list_controllers | grep -q joint_trajectory_controller "
            "&& ros2 control list_controllers | grep -q twist_controller; "
            "do sleep 1; done; "
            "ros2 control switch_controllers --activate twist_controller "
            "--deactivate joint_trajectory_controller",
        ],
        output="screen",
        condition=IfCondition(use_servo_controller),
    )

    kinova_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_kortex_vision, "launch", "kinova_vision.launch.py"])),
        launch_arguments={
            "device": robot_ip,
        }.items(),
        condition=IfCondition(use_wrist_camera),
    )

    table_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_kinova_lfd, "launch", "tabletop_camera.py"])),
        condition=IfCondition(use_table_camera),
    )
    return [kinova_arm_launch, kinova_vision_launch, table_camera_launch, switch_controllers, kinova_tool_pose_b_launch]


def generate_launch_description():
    declared_arguments = []

    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="If to only launch the fake hardware",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10",
            description="ip of robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_wrist_camera",
            default_value="true",
            description="If to load kinova wrist vision topics",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_table_camera",
            default_value="false",
            description="If to load realsense tabletop camera",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_servo_controller",
            default_value="true",
            description="If to use the servo controller on the arm",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
