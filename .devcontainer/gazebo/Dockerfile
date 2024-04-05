FROM ros:humble

RUN useradd -rm -d /home/lcas -s /bin/bash -g root -G sudo -u 1001 lcas
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN DEBIAN_FRONTEND=noninteractive apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-tiago-simulation ros-humble-plotjuggler-ros \
    python3-matplotlib