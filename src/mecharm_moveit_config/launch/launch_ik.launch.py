import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "firefighter", package_name="mecharm_moveit_config"
        )
        .robot_description(file_path="config/firefighter.urdf.xacro")
        .robot_description_semantic(file_path="config/firefighter.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["stomp", "pilz_industrial_motion_planner"]
        )
        # Add kinematics configuration
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # Explicitly load joint limits
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Convert the moveit_config to a dictionary
    moveit_config_dict = moveit_config.to_dict()

    # # Explicitly ensure kinematics yaml is loaded properly
    # # This ensures the kinematics solver configuration is properly passed to move_group
    # if "robot_description_kinematics" not in moveit_config_dict:
    #     moveit_config_dict["robot_description_kinematics"] = {
    #         "mecharm": {
    #             "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
    #             "kinematics_solver_search_resolution": 0.005,
    #             "kinematics_solver_timeout": 0.005
    #         }
    #     }
    
    # Add planning parameters with cartesian limits for Pilz planner
    planning_params = {
        "robot_description_planning": {
            "joint_limits": moveit_config.joint_limits,
            "planning_pipelines": moveit_config.planning_pipelines,
            "cartesian_limits": {
                "max_trans_vel": 1.0,
                "max_trans_acc": 0.5,
                "max_trans_dec": 0.5,
                "max_rot_vel": 1.0,
                "max_rot_acc": 0.5,
                "max_rot_dec": 0.5
            }
        }
    }
    
    # Combine all parameters
    all_params = {}
    all_params.update(moveit_config_dict)
    all_params.update(planning_params)

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[all_params],
    )

    # Get the path to the RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="config/moveit.rviz",
        description="RViz configuration file",
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mecharm_moveit_config"), "config", "moveit.rviz"]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[all_params],  # Use all parameters for RViz as well
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("mecharm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path, {"robot_description": moveit_config.robot_description["robot_description"]}],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecharm_controller", "-c", "/controller_manager"],
    )

    # Add the target marker node
    target_marker_node = Node(
        package="utils",
        executable="target_marker",
        name="target_marker_node",
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            target_marker_node,  # Add the target marker node to the launch description
        ]
    )