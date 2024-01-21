## UR3e Arm Coffee Pick and Place
1. Ensure xserver is enabled for with(Docker Only):
    xhost +localhost:root
2. Launch Gazebo with the cafe world file:
    ros2 launch ur_mtc world.launch.py
3. Spawn the arm:
    ros2 launch ur_mtc spawn_arm.launch.py
4. Launch the controllers:
    ros2 launch ur_mtc controllers.launch.py
5. Launch the movegroup node and RVIZ:
    ros2 launch ur_mtc mtc_setup2.launch.py
6. Run the task node:
    ros2 launch ur_mtc mtc_sim2.launch.py
7. After task planning is complete select the lowest cost option and execute via RVIZ in the MoveIt Task Constructor Panel. 

### Notes
The task node may need to be relaunched if octomap is not cleared in time(planning will fail at the gasp/close gripper).

### Docker
Run the container with: 
    docker compose up -d
            OR
    docker-compose up -d
Acccess the container:
    docker exec -it arm_sim bash