import os
import yaml


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap, SetParameter

def robot_controller_actions(context : LaunchContext):

    num_robots = int(context.launch_configurations['num_robots'])
        
    yaml_path = os.path.join(get_package_share_directory('assessment'), 'config', 'initial_poses.yaml')

    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f)

    initial_poses = configuration[num_robots]

    actions = []

    for robot_number in range(1, num_robots + 1):
        robot_name = 'robot' + str(robot_number)
        
        pos = initial_poses[robot_name]

        group = GroupAction([

            PushRosNamespace(robot_name),
            
            Node(
                package='solution',
                output='screen',
                executable='sensor_fusion',
                name='sensor_fusion',
            ),
            
            Node(
                package='solution',
                output='screen',
                executable='item_cloud',
                name='item_cloud',
            ),
            
            Node(
                package='solution',
                output='screen',
                executable='lidar_cloud',
                name='lidar_cloud',
            ),
            
            Node(
                package='solution',
                output='screen',
                executable='state_marker_transformer',
                name='state_marker_transformer',
            ),
            
            Node(
                package='solution',
                output='screen',
                executable='autonomous_navigation',
                name='autonomous_navigation',
                parameters=[{'x_pose': pos['x'], 'y_pose': pos['y'], 'yaw': pos['yaw']}]
            ),
        ])

        actions.append(group)

    return actions

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
    
    traffic_manager=Node(
        package='solution',
        output='screen',
        executable='traffic_manager',
        name='traffic_manager',
        parameters=[{'num_robots': num_robots}],
    )
    
    robot_controller_cmd = OpaqueFunction(function=robot_controller_actions)

    
    ld = LaunchDescription()
    
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    
    ld.add_action(traffic_manager)
    
    ld.add_action(declare_num_robots_cmd)

    # Add the commands to the launch description
    ld.add_action(assessment_launch)
    
    ld.add_action(robot_controller_cmd)
    
    return ld