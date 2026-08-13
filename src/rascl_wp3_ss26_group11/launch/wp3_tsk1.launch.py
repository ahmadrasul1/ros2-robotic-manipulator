from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


JOINT_NAMES = [
    "shoulder_joint",
    "upperarm_joint",
    "lowerarm_joint",
    "end_effector_joint",
]


def _task_node(
    wp3_pkg: str,
    trajectory_file,
    trajectory_gripper_mode,
    condition=None,
) -> Node:
    return Node(
        package=wp3_pkg,
        executable="wp3_tsk1",
        name="wp3_tsk1",
        output="screen",
        condition=condition,
        parameters=[
            {
                "trajectory_file": trajectory_file,
                "trajectory_gripper_mode": trajectory_gripper_mode,
                "execution_mode": LaunchConfiguration("execution_mode"),
                "controller_topic": "/joint_position_controller/commands",
                "joint_states_topic": "/joint_states",
                "publish_rate_hz": 100.0,
                "joint_names": JOINT_NAMES,
                "auto_start": True,
                "hold_last_sample": True,
                "hardware_ready_file": "/tmp/rascl_pdo_ready",
                "require_arm_reference": True,
                "require_gripper_reference": ParameterValue(
                    LaunchConfiguration("require_gripper_reference"),
                    value_type=bool,
                ),
                "start_pose_tolerance_rad": [0.03, 0.03, 0.03, 0.05],
            }
        ],
    )


def generate_launch_description():
    """Regenerate a selected trajectory atomically, validate it, then execute it."""

    wp3_pkg = "rascl_wp3_ss26_group11"
    description_pkg = "rascl_description"

    default_trajectory = PathJoinSubstitution(
        [
            FindPackageShare(wp3_pkg),
            "trajectories",
            "task1",
            "task1_full_simulation_ik.csv",
        ]
    )
    default_poses = PathJoinSubstitution(
        [FindPackageShare(wp3_pkg), "config", "task1_cube_poses.yaml"]
    )
    default_waypoints = PathJoinSubstitution(
        [
            FindPackageShare(wp3_pkg),
            "trajectories",
            "task1",
            "input_waypoints_ik_sim.yaml",
        ]
    )
    default_limits = PathJoinSubstitution(
        [FindPackageShare(wp3_pkg), "config", "robot_limits.yaml"]
    )
    default_urdf = PathJoinSubstitution(
        [FindPackageShare(description_pkg), "urdf", "rascl.urdf.xacro"]
    )
    regeneration_script = PathJoinSubstitution(
        [FindPackageShare(wp3_pkg), "scripts", "regenerate_task1_trajectory.py"]
    )

    launch_arguments = [
        DeclareLaunchArgument(
            "trajectory_file",
            default_value=default_trajectory,
            description="Pre-generated/custom CSV used when regeneration is disabled.",
        ),
        DeclareLaunchArgument(
            "trajectory_gripper_mode",
            default_value="simulation",
            description=(
                "Metadata for a pre-generated CSV: simulation or hardware. "
                "Controller mode rejects simulation."
            ),
        ),
        DeclareLaunchArgument(
            "generated_trajectory_file",
            default_value=default_trajectory,
            description="CSV output generated and then executed when regeneration=true.",
        ),
        DeclareLaunchArgument(
            "regenerate_trajectory",
            default_value="true",
            description="Run IK, minimum-jerk sampling and validation before execution.",
        ),
        DeclareLaunchArgument("poses_file", default_value=default_poses),
        DeclareLaunchArgument("ik_waypoints_file", default_value=default_waypoints),
        DeclareLaunchArgument("robot_urdf_file", default_value=default_urdf),
        DeclareLaunchArgument("robot_limits_file", default_value=default_limits),
        DeclareLaunchArgument(
            "gripper_mode",
            default_value="simulation",
            description=(
                "Mode used for newly generated waypoints. Hardware mode refuses "
                "an unset hardware_hold_rad."
            ),
        ),
        DeclareLaunchArgument(
            "trajectory_name",
            default_value="task1_full",
            description="Trajectory key produced by the IK generator and sampled.",
        ),
        DeclareLaunchArgument(
            "execution_mode",
            default_value="joint_states",
            description="joint_states for RViz, controller for ros2_control/PDO.",
        ),
        DeclareLaunchArgument(
            "require_gripper_reference",
            default_value="true",
            description="Require the PDO marker to confirm end-effector reference.",
        ),
    ]

    regenerate = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            regeneration_script,
            "--poses",
            LaunchConfiguration("poses_file"),
            "--urdf",
            LaunchConfiguration("robot_urdf_file"),
            "--waypoints-output",
            LaunchConfiguration("ik_waypoints_file"),
            "--trajectory-name",
            LaunchConfiguration("trajectory_name"),
            "--csv-output",
            LaunchConfiguration("generated_trajectory_file"),
            "--limits",
            LaunchConfiguration("robot_limits_file"),
            "--gripper-mode",
            LaunchConfiguration("gripper_mode"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("regenerate_trajectory")),
    )

    task_without_regeneration = _task_node(
        wp3_pkg,
        LaunchConfiguration("trajectory_file"),
        LaunchConfiguration("trajectory_gripper_mode"),
        condition=UnlessCondition(LaunchConfiguration("regenerate_trajectory")),
    )
    task_after_regeneration = _task_node(
        wp3_pkg,
        LaunchConfiguration("generated_trajectory_file"),
        LaunchConfiguration("gripper_mode"),
    )

    def generation_finished(event, _context):
        if event.returncode != 0:
            return [
                EmitEvent(
                    event=Shutdown(reason="Task 1 trajectory generation failed")
                )
            ]
        return [
            LogInfo(msg="Task 1 trajectory regenerated and validated; starting player."),
            task_after_regeneration,
        ]

    return LaunchDescription(
        [
            *launch_arguments,
            RegisterEventHandler(
                OnProcessExit(target_action=regenerate, on_exit=generation_finished)
            ),
            regenerate,
            task_without_regeneration,
        ]
    )
