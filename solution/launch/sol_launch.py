import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('assessment'), 'launch')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    num_robots = LaunchConfiguration('num_robots')
    
    declare_num_robots_cmd = DeclareLaunchArgument(
            'num_robots',
            default_value='1',
            description='Number of robots to spawn')

    assessment_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'assessment_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'visualise_sensors': 'false',
            'obstacles': 'true',
            'num_robots': num_robots,
            'use_nav2': 'true',
            'rviz_config': '/home/aei/auro/src/assessment/rviz/namespaced_nav2.rviz',
            'params_file': '/home/aei/auro/src/solution/params/sol.yml',
            }.items()
    )
    
    x_pose = LaunchConfiguration('x_pose', default='-3.5')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    traffic_manager=Node(
        package='solution',
        output='screen',
        executable='traffic_manager',
        name='traffic_manager',
        parameters=[{'use_sim_time': use_sim_time, 'num_robots': num_robots}],
    )

    sensor_fusion=Node(
        package='solution',
        output='screen',
        executable='sensor_fusion',
        name='sensor_fusion',
        namespace='robot1',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    item_cloud=Node(
        package='solution',
        output='screen',
        executable='item_cloud',
        name='item_cloud',
        namespace='robot1',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    lidar_cloud=Node(
        package='solution',
        output='screen',
        executable='lidar_cloud',
        name='lidar_cloud',
        namespace='robot1',
    )
    
    state_marker_transformer=Node(
        package='solution',
        output='screen',
        executable='state_marker_transformer',
        name='state_marker_transformer',
        namespace='robot1',
        parameters=[{'r': 1.0, 'g': 0.0, 'b': 0.0}],
    )
    
    sol=Node(
        package='solution',
        output='screen',
        executable='sol',
        name='sol',
        namespace='robot1',
        parameters=[{'use_sim_time': use_sim_time, 'x_pose': x_pose, 'y_pose': y_pose}],
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_num_robots_cmd)

    # Add the commands to the launch description
    ld.add_action(assessment_launch)
    
    ld.add_action(traffic_manager)
    
    ld.add_action(sensor_fusion)
    ld.add_action(item_cloud)
    ld.add_action(lidar_cloud)
    ld.add_action(state_marker_transformer)
    ld.add_action(sol)
    
    return ld