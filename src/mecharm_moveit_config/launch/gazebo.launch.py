# Software License Agreement (BSD License 2.0)
#
# Copyright (c) 2023, Metro Robots
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Metro Robots nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    # these are the arguments you can pass this launch file, for example paused:=true
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )
    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='mecharm_moveit_config')
    model_arg = DeclareLaunchArgument('urdf_package_path',
                                      description='The path to the robot description relative to the package root',
                                      default_value='config/firefighter.urdf.xacro')

    # empty_world_launch = IncludeLaunchDescription(
    #     PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
    #     launch_arguments={
    #         'gui': LaunchConfiguration('gui'),
    #         'pause': 'true',
    #     }.items(),
    # )

    # description_launch_py = IncludeLaunchDescription(
    #     PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
    #     launch_arguments={
    #         'urdf_package': LaunchConfiguration('urdf_package'),
    #         'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
    # )



    package_dir = FindPackageShare('mecharm_moveit_config')
    urdf_path = PathJoinSubstitution([package_dir, 'config/mecharm_270_m5.urdf'])


    print('urdf_path', dir(urdf_path), urdf_path.describe())
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    print(robot_description_content, type(robot_description_content))

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    parameters=[{
                                        'robot_description': robot_description_content,
                                    }])

    # push robot_description to factory and spawn robot in gazebo
    urdf_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'robot2', '-z', '1.5'],
        output='screen',
    )

    return LaunchDescription([
        gui_arg,
        package_arg,
        model_arg,
        # empty_world_launch,
        robot_state_publisher_node,
        urdf_spawner_node,
    ])