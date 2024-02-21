cmake_minimum_required(VERSION 3.22)
project(fanuc_cr7ia_moveit_config)

find_package(ament_cmake REQUIRED)

# catkin_package()

install(DIRECTORY launch DESTINATION share/fanuc_cr7ia_moveit_config)
install(DIRECTORY config DESTINATION share/fanuc_cr7ia_moveit_config)

# install(FILES readme.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

ament_package()