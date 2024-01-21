import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="rg2_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/name.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )


    # Pipeline Parameters for Constrained Planning with Database
    '''
    moveit_config.planning_pipelines["ompl"]["ur_manipulator"][
        "enforce_constrained_state_space"
    ] = True
    
    moveit_config.planning_pipelines["ompl"]["ur_manipulator"][
        "projection_evaluation"
    ] = "joints(should_pan_joint,shoulder_lift_joint,elbow_joint)"
    
    moveit_config.planning_pipelines["ompl"]["manipulator_gripper"][
        "enforce_constrained_state_space"
    ] = True
    
    moveit_config.planning_pipelines["ompl"]["manipulator_gripper"][
        "projection_evaluation"
    ] = "joints(should_pan_joint,shoulder_lift_joint,elbow_joint)"
    '''
    mtc_sim = Node(
        package="ur_mtc",
        executable="mtc_sim",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([mtc_sim])