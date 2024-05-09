#!/bin/bash

set -e

sudo chown -R lcas /home/lcas
sudo chown -R lcas /workspaces
sudo chown -R lcas /etc/hosts

sudo apt update

sudo apt install -y ros-foxy-rviz2 joint-state-publisher-gui \
    ros-foxy-joint-state-broadcaster ros-foxy-joint-state-controller \
    ros-foxy-geometry-msgs ros-foxy-tf2-geometry-msgs ros-foxy-rqt* \
    ros-foxy-joint-state-publisher-gui sshpass iproute2 iputils-ping \
    wget python3-rosinstall
    # ros-noetic-plotjuggler-ros

mkdir ~/tiago_public_ws
cd ~/tiago_public_ws

wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/noetic-devel/tiago_public-noetic.rosinstall
rosinstall src /opt/ros/noetic tiago_public-noetic.rosinstall
# sudo rosdep init
rosdep update
source /opt/ros/noetic/setup.bash
catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc` / 2) tiago_description

echo "sr1 () { source /home/lcas/tiago_public_ws/devel/setup.bash &> /dev/null ; }" >> ~/.bashrc
echo "sr2 () { source /opt/ros/foxy/setup.bash &> /dev/null ; }" >> ~/.bashrc
echo "sr2" >> ~/.bashrc