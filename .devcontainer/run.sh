#!/bin/bash

add_source_bashrc () {
    if ! grep -q "source $1" ~/.bashrc; then 
        echo "source $1" >> ~/.bashrc
    fi 
}

set -e

export WORKSPACE="`pwd`"
sudo chown -R lcas /home/lcas/ws
sudo chown -R lcas /workspaces

source /opt/ros/lcas/install/setup.bash
sudo apt update


rosdep --rosdistro=humble update 


# sudo rm -rf /opt/ros/lcas
# sudo mkdir -p /opt/ros/lcas/src
# sudo chown -R lcas /opt/ros/lcas
# cd /opt/ros/lcas/src
# vcs import < $WORKSPACE/.devcontainer/lcas.repos
# rosdep install --from-paths . -i -y
# cd /opt/ros/lcas
# colcon build


cd /home/lcas/ws/src
git clone -b ros2 https://github.com/PickNikRobotics/bio_ik.git
git clone -b humble https://github.com/ros-planning/moveit2_tutorials.git
git clone -b humble https://github.com/ros-planning/moveit_task_constructor.git
git clone -b humble https://github.com/ros-planning/moveit_resources.git
git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
git clone -b ros2 https://github.com/tylerjw/serial.git
# vcs import < moveit2_tutorials/moveit2_tutorials.repos

# instal moveit
sudo apt install ros-humble-moveit -y
# sudo apt install ros-humble-moveit-resources -y
sudo apt install ros-humble-pick-ik -y
# sudo apt install python3-pykdl

cd /home/lcas/ws

rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

colcon build 
add_source_bashrc "/opt/ros/lcas/install/setup.bash"
add_source_bashrc "/home/lcas/ws/install/setup.bash"
echo "function rebuild { cd /home/lcas/ws; colcon build; source install/setup.bash; cd -; }" >> ~/.bashrc
source /home/lcas/ws/install/setup.bash

