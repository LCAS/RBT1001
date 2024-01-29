#!/bin/bash

add_source_bashrc () {
    if ! grep -q "source $1" ~/.bashrc; then 
        echo "source $1" >> ~/.bashrc
    fi 
}

set -e

export WORKSPACE="`pwd`"

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


cd /home/lcas/ws
colcon build 
add_source_bashrc "/opt/ros/lcas/install/setup.bash"
add_source_bashrc "/home/lcas/ws/install/setup.bash"
echo "function rebuild { cd /home/lcas/ws; colcon build; cd -; }" >> ~/.bashrc
source /home/lcas/ws/install/setup.bash

