#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    delay_arg = DeclareLaunchArgument(
        "start_delay",
        default_value="8.0",
        description="Seconds to wait before running the Cartesian demo node.",
    )

    use_real_hardware_arg = DeclareLaunchArgument(
        "use_real_hardware",
        default_value="false",
        description="Set to 'true' when running on a real robot (starts myco_gui); "
                    "leave 'false' for Gazebo simulation (starts myco_gui with use_fake_robot=true).",
    )

    use_real_hardware = LaunchConfiguration("use_real_hardware")

    # Simulation stack (Gazebo + MoveIt2 + RViz)
    sim_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("myco_10_1000mm_ros2_moveit2"),
                    "launch",
                    "myco_10_1000mm.launch.py",
                )
            ]
        ),
        condition=UnlessCondition(use_real_hardware),
    )

    # Real-robot stack (MoveIt2 + RViz, no Gazebo)
    real_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("myco_10_1000mm_ros2_moveit2"),
                    "launch",
                    "myco_10_1000mm_moveit_rviz.launch.py",
                )
            ]
        ),
        condition=IfCondition(use_real_hardware),
    )

    # Basic API launcher for both simulation and real hardware
    basic_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("myco_10_1000mm_ros2_moveit2"),
                    "launch",
                    "myco_10_1000mm_basic_api.launch.py",
                )
            ]
        ),
    )

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("myco_10_1000mm_ros2_gazebo"),
            "urdf",
            "MyCo-10-1.00.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "myco_10_1000mm_ros2_moveit2", "config/MyCo-10-1.00.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml("myco_10_1000mm_ros2_moveit2", "config/kinematics.yaml")

    # GUI – simulation: use_fake_robot=True / use_sim_time=True
    gui_sim = Node(
        package="myco_basic_api",
        executable="myco_gui.py",
        name="myco_gui_node",
        output="screen",
        parameters=[
            {"use_fake_robot": True},
            {"use_sim_time": True},
        ],
        condition=UnlessCondition(use_real_hardware),
    )

    # GUI – real robot: use_fake_robot=False / use_sim_time=False
    gui_real = Node(
        package="myco_basic_api",
        executable="myco_gui.py",
        name="myco_gui_node",
        output="screen",
        parameters=[
            {"use_fake_robot": False},
            {"use_sim_time": False},
        ],
        condition=IfCondition(use_real_hardware),
    )

    # Demo node – use_sim_time only in simulation
    cartesian_demo_node_sim = Node(
        package="myco_10_1000mm_cartesian_demo",
        executable="cartesian_3_positions_node",
        name="myco_10_1000mm_cartesian_demo",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": True},
        ],
        condition=UnlessCondition(use_real_hardware),
    )

    cartesian_demo_node_real = Node(
        package="myco_10_1000mm_cartesian_demo",
        executable="cartesian_3_positions_node",
        name="myco_10_1000mm_cartesian_demo",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": False},
        ],
        condition=IfCondition(use_real_hardware),
    )

    delayed_demo_node = TimerAction(
        period=LaunchConfiguration("start_delay"),
        actions=[cartesian_demo_node_sim, cartesian_demo_node_real],
    )

    return LaunchDescription(
        [
            delay_arg,
            use_real_hardware_arg,
            sim_stack_launch,
            real_stack_launch,
            basic_api_launch,
            gui_sim,
            gui_real,
            delayed_demo_node,
        ]
    )
