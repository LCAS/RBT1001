from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="mecharm_moveit_config").to_moveit_configs()
    
    # Ensure kinematics.yaml is loaded properly
    moveit_config.robot_description_kinematics = {"mecharm": {
        "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
        "kinematics_solver_search_resolution": 0.005,
        "kinematics_solver_timeout": 0.005
    }}

    return generate_demo_launch(moveit_config)
