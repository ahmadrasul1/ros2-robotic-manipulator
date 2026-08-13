"""Home the robot and keep the PDO/ros2_control stack alive for Task 1.

Run this launch in its own terminal and leave it running. It homes all configured
axes, moves to the configured pick-ready pose, enters CSP, starts ros2_control and
the position controller, and then holds the robot. It does not generate or play a
Task 1 trajectory.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = "rascl_description"
    hardware = "rascl_hardware_interface"

    default_pdo = PathJoinSubstitution(
        [FindPackageShare(hardware), "config", "ethercat_pdo_task1.yaml"]
    )
    hardware_launch = PathJoinSubstitution(
        [FindPackageShare(description), "launch", "ros2_control.launch.py"]
    )

    pdo_config_arg = DeclareLaunchArgument(
        "pdo_config",
        default_value=default_pdo,
        description=(
            "PDO configuration used for the persistent homed Task 1 hardware "
            "session. The default homes all axes, moves to pick-ready, and "
            "permits gated trajectory commands."
        ),
    )

    hardware_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_launch),
        launch_arguments={"pdo_config": LaunchConfiguration("pdo_config")}.items(),
    )

    return LaunchDescription(
        [
            pdo_config_arg,
            LogInfo(
                msg=(
                    "Starting homing and the persistent PDO/controller stack only. "
                    "Leave this launch running, then start Task 1 from another terminal."
                )
            ),
            hardware_stack,
        ]
    )
