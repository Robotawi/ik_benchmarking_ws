from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import sys


def generate_launch_description():

    # Declare launch arguments
    move_group_arg = DeclareLaunchArgument(
        "move_group", default_value="iiwa_arm", description="Move group name required for run_ik_benchmarks node"
    )

    moveit_config_pkg_arg = DeclareLaunchArgument(
        'moveit_config_pkg', default_value="iiwa_moveit_config", description="Moveit config package to load robot description"
    )

    # Get parameters from launch arguments
    move_group = LaunchConfiguration("move_group")

    # Extract the robot name if the moveit_config_pkg arg is provided
    # It is usually in the form of <robot_name>_moveit_config
    robot_name = "iiwa" #default
    for arg in sys.argv:
        if arg.startswith("moveit_config_pkg:="):
            arg_value = arg.split(":=")[1]
            robot_name = arg_value.split("_")[0]

    # Build moveit_config using the robot name
    moveit_config = MoveItConfigsBuilder(robot_name).to_moveit_configs()

    # Start benchmarking node with required robot description and move_group parameters
    benchmarks_node = Node(
        package="ik_benchmarks",
        executable="run_ik_benchmarks",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"move_group": move_group},
        ],
    )

    return LaunchDescription([move_group_arg, moveit_config_pkg_arg, benchmarks_node])
