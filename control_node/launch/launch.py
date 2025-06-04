from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch SLAM Toolbox, Nav2, and custom control/teleop nodes in one file."""

    # ───────────────────── Config paths ──────────────────────
    config_dir = os.path.join(
        get_package_share_directory("control_node"),
        "config",
    )

    slam_params = os.path.join(config_dir, "slam_params_base_link.yaml")
    nav2_params = os.path.join(config_dir, "nav2_params.yaml")

    # ───────────────────── User nodes ────────────────────────
    control_node = Node(
        package="control_node",
        executable="control_node",
        name="control_node",
        output="screen",
        prefix="xterm -hold -e",
    )

    keyboard_node = Node(
        package="control_node",
        executable="keyboard",
        name="keyboard_teleop",
        output="screen",
        prefix="gnome-terminal --",
    )

    explorer_node = Node(
        package="control_node",
        executable="explorer",
        name="explorer",
        output="screen",
        prefix="xterm -hold -e",
    )

    # Static TF between base_link and laser frame
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_static_tf",
        arguments=[
            "0", "0", "0",  # x  y  z
            "0", "0", "0",  # roll  pitch  yaw
            "base_link", "laser",
        ],
        output="screen",
    )

    # ───────────────────── SLAM Toolbox ──────────────────────
    slam_toolbox_launch = ExecuteProcess(
        cmd=[
            "xterm", "-hold", "-e",
            "ros2", "launch", "slam_toolbox", "online_async_launch.py",
            f"slam_params_file:={slam_params}",
            "scan_topic:=scan",
        ],
        name="slam_toolbox",
        output="screen"
    )

    # ───────────────────── Nav2 Bring-up  ─────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "slam": "False",          # already started SLAM Toolbox above
            "use_sim_time": "False",
            "params_file": nav2_params,
            "autostart": "True",
        }.items(),
    )

    # ─────────────────── LaunchDescription ───────────────────
    return LaunchDescription(
        [
            control_node,
            static_tf,
            slam_toolbox_launch,   
            nav2_launch,
            keyboard_node,
            explorer_node,
        ]
    )
