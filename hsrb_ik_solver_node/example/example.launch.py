#!/usr/bin/env python3
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

from tmc_launch_ros_utils.tmc_launch_ros_utils import (
    load_collision_description,
    load_robot_description,
)


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('description_package', default_value='hsrb_description',
                                                    description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(DeclareLaunchArgument('description_file', default_value='hsrb4s.urdf.xacro',
                                                    description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(DeclareLaunchArgument('collision_file', default_value='collision_pair_hsrb.xml',
                                                    description='Collision config with the robot.'))

    declared_arguments.append(DeclareLaunchArgument('runtime_config_package', default_value='hsrb_rviz_simulator',
                                                    description="Package with the controller's configuration."))
    declared_arguments.append(DeclareLaunchArgument('controllers_file', default_value='controllers.yaml',
                                                    description='YAML file with the controllers configuration.'))

    return declared_arguments


def generate_launch_description():
    robot_description = load_robot_description()
    collision_description = load_collision_description()

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 parameters=[{'source_list': ['ik_result']}])
    robot_state_pub_node = Node(package='robot_state_publisher',
                                executable='robot_state_publisher',
                                parameters=[robot_description])

    rviz_config = os.path.join(get_package_share_directory('hsrb_ik_solver_node'), 'example/display.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config],
                     parameters=[robot_description])

    ik_solver = Node(package='tmc_ik_solver_node',
                     executable='ik_solver_node',
                     parameters=[robot_description, collision_description,
                                 {'ik_plugin_type': 'hsrb_ik_solver_node::HsrbIkSolverPluginRobustToBasePositionError',
                                  'map_convolution_type': 'tmc_ik_solver_node::EuclideanDistanceMapConvolution',
                                  'convolution': {'grid_distance_threhsold': 1.5}}])

    return LaunchDescription(declare_arguments() + [joint_state_publisher, robot_state_pub_node, rviz_node, ik_solver])
