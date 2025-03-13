FROM lcas.lincoln.ac.uk/lcas/devcontainer/ros2-teaching:2324-devel

# Install additional ROS 2 packages
RUN DEBIAN_FRONTEND=noninteractive apt update && \
    apt upgrade -y && \
    apt install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-controllers

# Install moveit2 from source in order to have moveit_py API which is not directly integrated into humble distribution
RUN . /opt/ros/humble/setup.sh && \
    cd /opt/ros/lcas/src/ && \
    git clone https://github.com/moveit/moveit2.git -b main && \
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_humble.repos"; test -r $f && echo $f); do vcs import < "$repo"; done && \
    rosdep install -r --from-paths . --ignore-src --rosdistro humble -y 

RUN DEBIAN_FRONTEND=noninteractive pip install pymycobot --upgrade

# Install mycobot_ros2
RUN . /opt/ros/humble/setup.sh && \
    cd /opt/ros/lcas/src/ && \
    git clone --depth 1 https://github.com/elephantrobotics/mycobot_ros2.git && \
    cd .. && \
    colcon build

    # Install additional ROS 2 packages
RUN DEBIAN_FRONTEND=noninteractive apt update && \
    apt upgrade -y && \
    apt install -y \
    ros-humble-tf-transformations
# RUN DEBIAN_FRONTEND=noninteractive apt update && \
#     apt upgrade -y && \
#     apt install -y && \
#     ros-humble-ros2-control \
#     ros-humble-controller-manager \


WORKDIR /home/lcas/ws