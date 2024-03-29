from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fanuc_cr7ia").to_moveit_configs()

    move_group_demo = Node(
        name="kinematics_cost_fn_tutorial",
        package="week4",
        executable="kinematics_cost_function_tutorial",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([move_group_demo])