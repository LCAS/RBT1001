FROM lcas.lincoln.ac.uk/lcas/devcontainer/ros2-teaching:2324-devel

# Install additional ROS 2 packages
RUN DEBIAN_FRONTEND=noninteractive apt update && \
    apt install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    ros-humble-joint-state-publisher-gui \
    ros-humble-moveit

RUN DEBIAN_FRONTEND=noninteractive pip install pymycobot --upgrade

# Install mycobot_ros2
RUN . /opt/ros/humble/setup.sh && \
    cd /opt/ros/lcas/src/ && \
    git clone --depth 1 https://github.com/elephantrobotics/mycobot_ros2.git && \
    cd .. && colcon build

WORKDIR /home/lcas/ws