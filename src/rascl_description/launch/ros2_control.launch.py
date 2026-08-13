import uuid

import launch.logging
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start EtherCAT startup/PDO first, then ros2_control after verified ready."""

    logger = launch.logging.get_logger("rascl_hardware_launch")
    description_package = "rascl_description"
    hardware_package = "rascl_hardware_interface"
    bridge_run_id = uuid.uuid4().hex

    robot_description_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", "rascl.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "controllers.yaml"]
    )
    safe_pdo_config = PathJoinSubstitution(
        [
            FindPackageShare(hardware_package),
            "config",
            "ethercat_pdo.yaml",
        ]
    )

    hardware_bridge_script = PathJoinSubstitution(
        [
            FindPackageShare(hardware_package),
            "..",
            "..",
            "lib",
            hardware_package,
            "hardware_bridge.py",
        ]
    )
    ready_waiter_script = PathJoinSubstitution(
        [
            FindPackageShare(hardware_package),
            "..",
            "..",
            "lib",
            hardware_package,
            "wait_for_bridge_ready.py",
        ]
    )

    pdo_config_arg = DeclareLaunchArgument(
        "pdo_config",
        default_value=safe_pdo_config,
        description=(
            "PDO YAML to use. The default is hold-only commissioning. Select "
            "ethercat_pdo_task1.yaml explicitly for motion tests."
        ),
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", robot_description_file]),
            value_type=str,
        )
    }

    hardware_bridge = ExecuteProcess(
        cmd=[FindExecutable(name="python3"), hardware_bridge_script],
        additional_env={
            "RASCL_BRIDGE_RUN_ID": bridge_run_id,
            "RASCL_PDO_CONFIG": LaunchConfiguration("pdo_config"),
        },
        output="screen",
    )

    bridge_ready_waiter = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            ready_waiter_script,
            "--path",
            "/tmp/rascl_pdo_ready",
            "--run-id",
            bridge_run_id,
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_file,
            "--controller-manager-timeout",
            "30",
            "--service-call-timeout",
            "30",
        ],
        output="screen",
    )

    joint_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_file,
            "--controller-manager-timeout",
            "30",
            "--service-call-timeout",
            "30",
        ],
        output="screen",
    )

    def start_ros_after_ready_exit(event, _context):
        if event.returncode != 0:
            logger.error("PDO readiness waiter failed; ros2_control will not start")
            return [EmitEvent(event=Shutdown(reason="PDO bridge did not become ready"))]
        return [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
        ]

    def start_position_controller_after_spawner_exit(event, _context):
        if event.returncode != 0:
            logger.error("joint_state_broadcaster failed; position controller blocked")
            return [EmitEvent(event=Shutdown(reason="Controller startup failed"))]
        return [joint_position_controller_spawner]

    return LaunchDescription(
        [
            pdo_config_arg,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=bridge_ready_waiter,
                    on_exit=start_ros_after_ready_exit,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=start_position_controller_after_spawner_exit,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=hardware_bridge,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="EtherCAT PDO bridge exited"))
                    ],
                )
            ),
            hardware_bridge,
            bridge_ready_waiter,
        ]
    )
