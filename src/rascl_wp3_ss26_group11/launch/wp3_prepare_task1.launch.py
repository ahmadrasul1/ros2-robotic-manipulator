"""Regenerate and validate both Task 1 trajectories without starting hardware.

The simulation and hardware files are generated from the same corrected URDF,
Cartesian targets, IK solver, and Cartesian-line descend/lift logic. Only the
configured gripper hold value may differ. No EtherCAT or homing process starts.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wp3 = "rascl_wp3_ss26_group11"
    description = "rascl_description"

    default_hardware_csv = PathJoinSubstitution(
        [FindPackageShare(wp3), "trajectories", "task1", "task1_full_hardware.csv"]
    )
    default_hardware_waypoints = PathJoinSubstitution(
        [FindPackageShare(wp3), "trajectories", "task1", "input_waypoints_ik_hardware.yaml"]
    )
    default_simulation_csv = PathJoinSubstitution(
        [FindPackageShare(wp3), "trajectories", "task1", "task1_full_simulation_ik.csv"]
    )
    default_simulation_waypoints = PathJoinSubstitution(
        [FindPackageShare(wp3), "trajectories", "task1", "input_waypoints_ik_sim.yaml"]
    )
    default_poses = PathJoinSubstitution(
        [FindPackageShare(wp3), "config", "task1_cube_poses.yaml"]
    )
    default_limits = PathJoinSubstitution(
        [FindPackageShare(wp3), "config", "robot_limits.yaml"]
    )
    default_urdf = PathJoinSubstitution(
        [FindPackageShare(description), "urdf", "rascl.urdf.xacro"]
    )
    regeneration_script = PathJoinSubstitution(
        [FindPackageShare(wp3), "scripts", "regenerate_task1_trajectory.py"]
    )

    arguments = [
        DeclareLaunchArgument("hardware_trajectory_file", default_value=default_hardware_csv),
        DeclareLaunchArgument("hardware_waypoints_file", default_value=default_hardware_waypoints),
        DeclareLaunchArgument("simulation_trajectory_file", default_value=default_simulation_csv),
        DeclareLaunchArgument("simulation_waypoints_file", default_value=default_simulation_waypoints),
        DeclareLaunchArgument("poses_file", default_value=default_poses),
        DeclareLaunchArgument("robot_limits_file", default_value=default_limits),
        DeclareLaunchArgument("robot_urdf_file", default_value=default_urdf),
        DeclareLaunchArgument("trajectory_name", default_value="task1_full"),
    ]

    def regeneration(output_waypoints, output_csv, gripper_mode):
        return ExecuteProcess(
            cmd=[
                FindExecutable(name="python3"),
                regeneration_script,
                "--poses",
                LaunchConfiguration("poses_file"),
                "--urdf",
                LaunchConfiguration("robot_urdf_file"),
                "--waypoints-output",
                output_waypoints,
                "--trajectory-name",
                LaunchConfiguration("trajectory_name"),
                "--csv-output",
                output_csv,
                "--limits",
                LaunchConfiguration("robot_limits_file"),
                "--gripper-mode",
                gripper_mode,
            ],
            output="screen",
        )

    hardware = regeneration(
        LaunchConfiguration("hardware_waypoints_file"),
        LaunchConfiguration("hardware_trajectory_file"),
        "hardware",
    )
    simulation = regeneration(
        LaunchConfiguration("simulation_waypoints_file"),
        LaunchConfiguration("simulation_trajectory_file"),
        "simulation",
    )

    def after_hardware(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Hardware trajectory generation failed"))]
        return [LogInfo(msg="Hardware trajectory passed; generating simulation trajectory."), simulation]

    def after_simulation(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Simulation trajectory generation failed"))]
        return [
            LogInfo(
                msg=(
                    "Task 1 simulation and hardware trajectories were regenerated "
                    "from the same corrected kinematic model. No hardware was started."
                )
            ),
            EmitEvent(event=Shutdown(reason="Task 1 preparation completed successfully")),
        ]

    return LaunchDescription(
        [
            *arguments,
            RegisterEventHandler(OnProcessExit(target_action=hardware, on_exit=after_hardware)),
            RegisterEventHandler(OnProcessExit(target_action=simulation, on_exit=after_simulation)),
            hardware,
        ]
    )
