FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
gazebo \
ros-dev-tools \
ros-humble-gazebo-ros-pkgs \
ros-humble-gazebo-ros2-control \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-joint-state-publisher \
ros-humble-joint-state-broadcaster \
ros-humble-robot-state-publisher \
ros-humble-robot-localization \
ros-humble-xacro \
ros-humble-tf2-ros \
ros-humble-tf2-tools \
ros-humble-pcl-ros \
ros-humble-pcl-msgs \
ros-humble-pcl-conversions \
ros-humble-gazebo-ros \
ros-humble-gripper-controllers \
git \
python3-colcon-common-extensions \
python3-rosdep \
python3-vcstool \
&& rm -rf /var/lib/apt/lists/*

WORKDIR /
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

COPY ./final_project/rg2_moveit_config /ros2_ws/src/rg2_moveit_config
COPY ./universal_robot_ros2 /ros2_ws/src/universal_robot_ros2

RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --executor sequential"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /ros2_ws
 
CMD ["/bin/bash"]

# Project Directory, Moveit and MTC Added Via Commit
# Moveit/MTC Added Via Instructions at https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html
# Project Directory Added Via $docker cp ./ur_mtc arm_sim:/ros2_ws/src/ur_mtc
# Image Updated Via $docker commit arm_sim arm_sim:latest