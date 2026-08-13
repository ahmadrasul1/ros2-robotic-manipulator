"""Run the Task 2 online planner against an already homed hardware session.

Start wp3_homing_hardware.launch.py first and leave it running. This launch does
not touch homing, EtherCAT configuration, the URDF, or the frozen Task 1 files.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wp3_package = "rascl_wp3_ss26_group11"
    description_package = "rascl_description"

    config_file = PathJoinSubstitution(
        [FindPackageShare(wp3_package), "config", "task2_online_planning.yaml"]
    )
    limits_file = PathJoinSubstitution(
        [FindPackageShare(wp3_package), "config", "robot_limits.yaml"]
    )
    urdf_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", "rascl.urdf.xacro"]
    )

    task2_config = DeclareLaunchArgument(
        "task2_config",
        default_value=config_file,
        description=(
            "Task 2 online-planning YAML. Hardware execution remains blocked "
            "while workspace.values_are_placeholders=true."
        ),
    )

    return LaunchDescription(
        [
            task2_config,
            LogInfo(
                msg=(
                    "Starting only wp3_tsk2. The persistent homed PDO/controller "
                    "stack must already be running in another terminal."
                )
            ),
            Node(
                package=wp3_package,
                executable="wp3_tsk2",
                name="wp3_tsk2",
                output="screen",
                parameters=[
                    {
                        "config_file": LaunchConfiguration("task2_config"),
                        "robot_urdf_file": urdf_file,
                        "robot_limits_file": limits_file,
                        "execution_mode": "controller",
                        "gripper_mode": "hardware",
                        "controller_topic": "/joint_position_controller/commands",
                        "joint_states_topic": "/joint_states",
                        "status_topic": "/wp3_tsk2/status",
                    }
                ],
            ),
        ]
    )
