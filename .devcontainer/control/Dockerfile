FROM ros:foxy-ros1-bridge

RUN useradd -rm -d /home/lcas -s /bin/bash -g root -G sudo -u 1001 lcas
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers


RUN DEBIAN_FRONTEND=noninteractive apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y \
    #  ros-foxy-tiago-simulation \
    ros-foxy-plotjuggler-ros \
    python3-matplotlib ros-foxy-rqt* \
    python3-sympy ros-noetic-rviz \
    python3-catkin-tools ros-noetic-xacro