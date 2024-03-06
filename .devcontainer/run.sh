#!/bin/bash

add_source_bashrc () {
    if ! grep -q "source $1" ~/.bashrc; then 
        echo "source $1" >> ~/.bashrc
    fi 
}

set -e

sudo apt update

sudo apt install -y ros-foxy-rviz2 joint-state-publisher-gui \
    ros-foxy-joint-state-publisher-gui sshpass iproute2

add_source_bashrc "/opt/ros/noetic/setup.bash > /dev/null"
add_source_bashrc "/opt/ros/foxy/setup.bash > /dev/null"
