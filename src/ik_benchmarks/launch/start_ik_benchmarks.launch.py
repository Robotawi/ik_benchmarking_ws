from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    #TODO(Mohamed) do not hard code the moveit config package name
    moveit_config = MoveItConfigsBuilder("iiwa").to_moveit_configs()

    benchmarks_node = Node(
        package="ik_benchmarks",
        executable="run_ik_benchmarks",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([benchmarks_node])