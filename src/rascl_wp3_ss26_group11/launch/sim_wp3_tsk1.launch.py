from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def generate_launch_description():
    """Visualization-only launch for the first WP3 checkpoint.

    It publishes the sampled trajectory directly to /joint_states, so it does not
    require EtherCAT, ros2_control, or the Faulhaber controllers.
    """

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

    robot_description_file = PathJoinSubstitution(
        [FindPackageShare(description_pkg), "urdf", "rascl.urdf.xacro"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_pkg), "rviz", "urdf.rviz"]
    )

    trajectory_arg = DeclareLaunchArgument(
        "trajectory_file",
        default_value=default_trajectory,
        description="CSV trajectory to visualize.",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz together with the trajectory player.",
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", robot_description_file]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    wp3_tsk1 = Node(
        package=wp3_pkg,
        executable="wp3_tsk1",
        name="wp3_tsk1",
        output="screen",
        parameters=[
            {
                "trajectory_file": LaunchConfiguration("trajectory_file"),
                "execution_mode": "joint_states",
                "trajectory_gripper_mode": "simulation",
                "joint_states_topic": "/joint_states",
                "publish_rate_hz": 100.0,
                # Keep this order identical to controllers.yaml and generated CSV files.
                "joint_names": [
                    "shoulder_joint",
                    "upperarm_joint",
                    "lowerarm_joint",
                    "end_effector_joint",
                ],
                "auto_start": True,
                "hold_last_sample": True,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            trajectory_arg,
            use_rviz_arg,
            robot_state_publisher,
            wp3_tsk1,
            rviz,
        ]
    )
