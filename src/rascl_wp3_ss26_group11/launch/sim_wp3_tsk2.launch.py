from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    rviz_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "urdf.rviz"]
    )

    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz with the Task 2 online planner.",
    )
    task2_config = DeclareLaunchArgument(
        "task2_config",
        default_value=config_file,
        description="Task 2 online-planning YAML.",
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", urdf_file]),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            use_rviz,
            task2_config,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
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
                        "execution_mode": "joint_states",
                        "gripper_mode": "simulation",
                        "joint_states_topic": "/joint_states",
                        "status_topic": "/wp3_tsk2/status",
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_file],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
