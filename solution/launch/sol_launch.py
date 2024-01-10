#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('assessment'), 'launch')
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # config_dir = os.path.join(get_package_share_directory('course'),'config')
    # launch_dir = os.path.join(get_package_share_directory('course'),'launch')
    # map_dir = os.path.join(get_package_share_directory('course'),'map')
    param_dir = os.path.join(get_package_share_directory('solution'),'param')
    rviz_dir = os.path.join(get_package_share_directory('solution'),'rviz')
    # maze_path = os.path.join(get_package_share_directory('course'),'world','maze','model.sdf')
    
    # map_file = os.path.join(map_dir,'maze.yaml')
    params_file = os.path.join(param_dir,'sol.yml')
    rviz_config= os.path.join(rviz_dir,'namespaced_nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    assessment_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'assessment_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'visualise_sensors': 'false',
            'obstacles': 'true',
            'num_robots': '3',
            'use_nav2': 'true',
            'rviz_config': '/home/aei/auro/src/assessment/rviz/namespaced_nav2.rviz',
            'params_file': '/home/aei/auro/src/solution/params/sol.yml',
            }.items()
    )
    
    x_pose = LaunchConfiguration('x_pose', default='-3.5')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    neg_y_pose = LaunchConfiguration('y_pose', default='-1.0')
    
    y1_pose = LaunchConfiguration('y_pose', default='2.0')
    y2_pose = LaunchConfiguration('y_pose', default='0.0')
    y3_pose = LaunchConfiguration('y_pose', default='-2.0')
    
    handbreak=Node(
        package='solution',
        output='screen',
        executable='handbreak',
        name='handbreak',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    fusion=Node(
        package='solution',
        output='screen',
        executable='fusion',
        name='fusion',
        namespace='robot1',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    cloud=Node(
        package='solution',
        output='screen',
        executable='cloud',
        name='cloud',
        namespace='robot1',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    vis_lidar=Node(
        package='solution',
        output='screen',
        executable='vis_lidar',
        name='vis_lidar',
        namespace='robot1',
    )
    
    marker=Node(
        package='solution',
        output='screen',
        executable='marker',
        name='marker',
        namespace='robot1',
    )
    
    sol=Node(
        package='solution',
        output='screen',
        executable='sol',
        name='sol',
        namespace='robot1',
        parameters=[{'use_sim_time': use_sim_time, 'x_pose': x_pose, 'y_pose': y_pose}],
    )
    
    fusion2=Node(
        package='solution',
        output='screen',
        executable='fusion',
        name='fusion2',
        namespace='robot2',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    cloud2=Node(
        package='solution',
        output='screen',
        executable='cloud',
        name='cloud2',
        namespace='robot2',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    vis_lidar2=Node(
        package='solution',
        output='screen',
        executable='vis_lidar',
        name='vis_lidar2',
        namespace='robot2',
    )
    
    marker2=Node(
        package='solution',
        output='screen',
        executable='marker',
        name='marker2',
        namespace='robot2',
    )
    
    sol2=Node(
        package='solution',
        output='screen',
        executable='sol',
        name='sol2',
        namespace='robot2',
        parameters=[{'use_sim_time': use_sim_time, 'x_pose': x_pose, 'y_pose': y2_pose}],
    )
    
    fusion3=Node(
        package='solution',
        output='screen',
        executable='fusion',
        name='fusion3',
        namespace='robot3',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    cloud3=Node(
        package='solution',
        output='screen',
        executable='cloud',
        name='cloud3',
        namespace='robot3',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    vis_lidar3=Node(
        package='solution',
        output='screen',
        executable='vis_lidar',
        name='vis_lidar3',
        namespace='robot3',
    )
    
    marker3=Node(
        package='solution',
        output='screen',
        executable='marker',
        name='marker3',
        namespace='robot3',
    )
    
    sol3=Node(
        package='solution',
        output='screen',
        executable='sol',
        name='sol3',
        namespace='robot3',
        parameters=[{'use_sim_time': use_sim_time, 'x_pose': x_pose, 'y_pose': y3_pose}],
    )
    
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(assessment_launch)
    
    ld.add_action(handbreak)
    
    ld.add_action(fusion)
    ld.add_action(cloud)
    ld.add_action(vis_lidar)
    ld.add_action(marker)
    ld.add_action(sol)
    
    # ld.add_action(fusion2)
    # ld.add_action(cloud2)
    # ld.add_action(vis_lidar2)
    # ld.add_action(marker2)
    # ld.add_action(sol2)
    
    # ld.add_action(fusion3)
    # ld.add_action(cloud3)
    # ld.add_action(vis_lidar3)
    # ld.add_action(marker3)
    # ld.add_action(sol3)
    

    return ld