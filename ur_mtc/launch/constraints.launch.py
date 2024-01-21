from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Constraints Database Generator ARGS use_current_scene default=false, planning_group, constraints_file
    #onstraint_params = {
        #"planning_group": "ur_manipulator",
        #"constraints.type": "orientation",
        #"constraints.frame_id": "base_link",
        #"constraints.link_name": "tool0",
        #"constraints.orientation": [0, 0, 0],
        #"constraints.tolerances": [0.65, 0.65, 3.15],
        #"constraints.weight": 1.0,
    #}
    constraint_config = "/home/koala/ros2_ws/src/final_project/ur_mtc/config/constraints.yaml"
    
    constraints_dbg = Node(
        package="moveit_planners_ompl",
        executable="generate_state_database",
        name="generate_state_database",
        output="screen",
        parameters=[
            #constraint_params,
            constraint_config,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([constraints_dbg,])