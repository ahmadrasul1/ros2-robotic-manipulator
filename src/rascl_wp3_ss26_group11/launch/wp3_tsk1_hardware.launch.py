"""Execute a pre-generated Task 1 trajectory on an already homed hardware stack.

This launch never regenerates a trajectory, opens EtherCAT, or runs homing. Start
``wp3_homing_hardware.launch.py`` first and leave it running. The task node remains
gated until the existing PDO marker proves that motion is enabled, all references
are valid, the position controller is active, and the measured startup pose agrees
with sample zero of the selected CSV.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


JOINT_NAMES = [
    "shoulder_joint",
    "upperarm_joint",
    "lowerarm_joint",
    "end_effector_joint",
]


def generate_launch_description():
    wp3 = "rascl_wp3_ss26_group11"
    description = "rascl_description"

    default_csv = PathJoinSubstitution(
        [FindPackageShare(wp3), "trajectories", "task1", "task1_full_hardware.csv"]
    )
    default_poses = PathJoinSubstitution(
        [FindPackageShare(wp3), "config", "task1_cube_poses.yaml"]
    )
    default_urdf = PathJoinSubstitution(
        [FindPackageShare(description), "urdf", "rascl.urdf.xacro"]
    )

    arguments = [
        DeclareLaunchArgument(
            "hardware_trajectory_file",
            default_value=default_csv,
            description=(
                "Previously generated and validated hardware CSV. Run "
                "wp3_prepare_task1.launch.py whenever poses, URDF, limits, or "
                "gripper calibration change."
            ),
        ),
        DeclareLaunchArgument(
            "poses_file",
            default_value=default_poses,
            description="Current Cartesian Task 1 pose source used for freshness checks.",
        ),
        DeclareLaunchArgument(
            "robot_urdf_file",
            default_value=default_urdf,
            description="Current URDF source used for trajectory freshness checks.",
        ),
        DeclareLaunchArgument(
            "hardware_ready_file",
            default_value="/tmp/rascl_pdo_ready",
            description="Ready marker owned by the still-running homing/PDO launch.",
        ),
    ]

    task = Node(
        package=wp3,
        executable="wp3_tsk1",
        name="wp3_tsk1",
        output="screen",
        parameters=[
            {
                "trajectory_file": LaunchConfiguration("hardware_trajectory_file"),
                "poses_file": LaunchConfiguration("poses_file"),
                "robot_urdf_file": LaunchConfiguration("robot_urdf_file"),
                "require_fresh_trajectory": True,
                "trajectory_gripper_mode": "hardware",
                "execution_mode": "controller",
                "controller_topic": "/joint_position_controller/commands",
                "publish_rate_hz": 100.0,
                "joint_names": JOINT_NAMES,
                "auto_start": True,
                "hold_last_sample": True,
                "hardware_ready_file": LaunchConfiguration("hardware_ready_file"),
                "require_arm_reference": True,
                "require_gripper_reference": True,
                "start_pose_tolerance_rad": [0.03, 0.03, 0.03, 0.05],
            }
        ],
    )

    return LaunchDescription(
        [
            *arguments,
            LogInfo(
                msg=(
                    "Starting the Task 1 player only. Homing and the PDO/controller "
                    "stack must already be running in another terminal."
                )
            ),
            task,
        ]
    )
