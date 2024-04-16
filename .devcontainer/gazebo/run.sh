#!/bin/bash

set -e

sudo chown -R lcas /home/lcas
sudo chown -R lcas /workspaces
sudo chown -R lcas /etc/hosts

sudo apt update

# sudo apt install ros-humble-tiago-simulation

# sudo apt install -y ros-foxy-rviz2 joint-state-publisher-gui \
#     ros-foxy-joint-state-broadcaster ros-foxy-joint-state-controller \
#     ros-foxy-geometry-msgs ros-foxy-tf2-geometry-msgs ros-foxy-rqt* \
#     ros-foxy-joint-state-publisher-gui sshpass iproute2 iputils-ping \
#     ros-noetic-plotjuggler-ros

# echo "sr1 () { source /opt/ros/noetic/setup.bash &> /dev/null ; }" >> ~/.bashrc
echo "sr2 () { source /opt/ros/humble/setup.bash &> /dev/null ; }" >> ~/.bashrc
echo "sr2" >> ~/.bashrc