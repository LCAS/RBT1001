# RBT1001
Introduction to Robotics workshops materials

## Syllabus & weekly instructions
Go to the [wiki page](https://github.com/francescodelduchetto/RBT1001/wiki)

## Usage

### Setup your environment

1. Make sure you have VSCode installed: https://code.visualstudio.com/download
2. Make sure you have the `Docker` and the `Dev Containers` extension in VSCode installed and working: https://code.visualstudio.com/docs/containers/overview and https://code.visualstudio.com/docs/devcontainers/containers
    * ensure docker is working, i.e. try `docker run --rm hello-world` and check it succeeds for your user
3. The docker image used to provide the Development Container is provided by the [L-CAS](https://lcas.lincoln.ac.uk) Container Registry. You must log in to use it. For simple read access, the username and password is public and is username `lcas`, password: `lincoln`. So, to log in do `docker login -u lcas -p lincoln lcas.lincoln.ac.uk` (you should only have to do this once, as the credentials should be cached unless your home directory is wiped).

### Open in VSCode

1. Open this repository in VSCode: https://code.visualstudio.com/docs/sourcecontrol/intro-to-git (or any other way you prefer)
2. VSCode should prompt you that there is a devcontainer configured and ask if you want to reopen in container. Re-open in the container

### Access the embedded lite Desktop

1. Click on the VNC extension in VSCode:
 
   <img width="59" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/8f41fd58-c41d-440a-afb9-099504369be4">

2. Add a VNC server, as `localhost:5901` if it's not there yet

   <img width="353" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/a6e83e80-f981-42bb-80bd-21aca6f53bde">

3. Click Connect, and when prompted for a password, enter `vscode` (VNC insists on a password)

   <img width="358" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/89657b1c-bb67-4731-8747-ed5ba9a9ebb2">


