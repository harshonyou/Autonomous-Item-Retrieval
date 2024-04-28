import os
import yaml


from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace, SetRemap, SetParameter, RosTimer
from launch.conditions import IfCondition

ASSESSMENT_PKG_NAME = 'assessment'
PACKAGE_NAME = 'solution'

def robot_controller_actions(context : LaunchContext):

    num_robots = int(context.launch_configurations['num_robots'])
    # use_old_fsm = context.launch_configurations['use_old_fsm']
    # use_ekf = context.launch_configurations['use_ekf']
        
    yaml_path = os.path.join(get_package_share_directory(ASSESSMENT_PKG_NAME), 'config', 'initial_poses.yaml')

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
                package=PACKAGE_NAME,
                output='screen',
                executable='sensor_fusion',
                name='sensor_fusion',
            ),
            
            Node(
                package=PACKAGE_NAME,
                output='screen',
                executable='item_cloud',
                name='item_cloud',
            ),
            
            Node(
                package=PACKAGE_NAME,
                output='screen',
                executable='lidar_cloud',
                name='lidar_cloud',
            ),
            
            Node(
                package=PACKAGE_NAME,
                output='screen',
                executable='state_marker_transformer',
                name='state_marker_transformer',
            ),
            
            Node(
                package=PACKAGE_NAME,
                output='screen',
                executable='transformer',
                name='transformer',
            ),
            
            # Node(
            #     condition=IfCondition(use_ekf),
            #     package='robot_localization',
            #     output='screen',
            #     executable='ekf_node',
            #     name='ekf_filter_node',
            #     parameters=['/home/aei/kmy/src/solution/param/ekf.yaml'],
            #     remappings=[('odometry/filtered', 'odometry/local')],
            # ),
            
            # Node(
            #     condition=IfCondition(use_old_fsm),
            #     package='solution',
            #     output='screen',
            #     executable='fsm_old_implementation',
            #     name='fsm_old_implementation',
            #     parameters=[{'x_pose': pos['x'], 'y_pose': pos['y'], 'yaw': pos['yaw']}]
            # ),
            
            Node(
                # condition=IfCondition(PythonExpression([use_old_fsm, '==', 'False'])),
                package=PACKAGE_NAME,
                output='screen',
                executable='autonomous_navigation',
                name='autonomous_navigation',
                parameters=[{'x_pose': pos['x'], 'y_pose': pos['y'], 'yaw': pos['yaw']}]
            ),
        ])

        actions.append(group)

    return actions

def generate_launch_description():
    
    assessmenet_launch_file_dir = os.path.join(get_package_share_directory(ASSESSMENT_PKG_NAME), 'launch')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    num_robots = LaunchConfiguration('num_robots')
    random_seed = LaunchConfiguration('random_seed')
    experiment_duration = LaunchConfiguration('experiment_duration')
    data_log_path = LaunchConfiguration('data_log_path')
    data_log_run_name = LaunchConfiguration('data_log_run_name')
    # use_old_fsm = LaunchConfiguration('use_old_fsm')
    # use_ekf = LaunchConfiguration('use_ekf')
    
    declare_num_robots_cmd = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn'
    )
    
    declare_random_seed_cmd = DeclareLaunchArgument(
        'random_seed',
        default_value='0',
        description='Random number seed for item manager'
    )
    
    declare_experiment_duration_cmd = DeclareLaunchArgument(
        'experiment_duration',
        default_value='300.0',
        description='Experiment duration in seconds'
    )
    
    declare_data_log_path_cmd = DeclareLaunchArgument(
        'data_log_path',
        default_value = os.path.join(get_package_prefix(PACKAGE_NAME), '../../'),
        description='Full path to directory where data logs will be saved'
    )
    
    declare_data_log_run_name_cmd = DeclareLaunchArgument(
        'data_log_run_name',
        default_value='data_log',
        description='Run name to use for data logs'
    )
    
    # declare_use_old_fsm_cmd = DeclareLaunchArgument(
    #     'use_old_fsm',
    #     default_value='false',
    #     description='Use old FSM implementation'
    # )
    
    # declare_use_ekf_cmd = DeclareLaunchArgument(
    #     'use_ekf',
    #     default_value='false',
    #     description='Use EKF for odometry'
    # )
    
    rviz_config = PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'rviz', 'custom_namespaced_nav2.rviz'])
    rviz_windows = PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'config', 'custom_rviz_windows.yaml'])
    map = PathJoinSubstitution([FindPackageShare(ASSESSMENT_PKG_NAME), 'maps', 'assessment_world.yaml'])
    params = PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'params', 'custom_nav2_params_namespaced.yml'])
    

    assessment_launch = GroupAction([
        # SetRemap('/robot1/cmd_vel', '/robot1/mock/cmd_vel'),
        
        LogInfo(msg=["Launching assessment"]),
        
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(assessmenet_launch_file_dir, 'assessment_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'num_robots': num_robots,
            'rviz_config': rviz_config,
            'rviz_windows': rviz_windows,
            'random_seed': random_seed,
            'use_nav2': 'True',
            'map': map,
            'params_file': params,
            'wait_for_items': 'true',
            }.items()
        # launch_arguments={
        #     'use_sim_time': use_sim_time,
        #     'visualise_sensors': 'false',
        #     'num_robots': num_robots,
        #     'use_nav2': 'true',
        #     'rviz_config': '/home/aei/kmy/src/solution/rviz/namespaced_nav2.rviz',
        #     'params_file': '/home/aei/kmy/src/solution/params/sol.yml',
        #     'rviz_windows': '/home/aei/kmy/src/solution/config/custom_rviz_windows.yaml',
        #     }.items()
    )
    ])

    traffic_manager=Node(
        package=PACKAGE_NAME,
        output='screen',
        executable='traffic_manager',
        name='traffic_manager',
        parameters=[{'num_robots': num_robots}],
    )
    
    data_logger_cmd = Node(
        package=PACKAGE_NAME,
        executable='data_logger',
        output='screen',
        arguments=['--path', data_log_path,
                   '--run_name', data_log_run_name,
                   '--num_robots', num_robots]
    )
    
    timeout_cmd = RosTimer(                                         
            period = experiment_duration,
            actions = [                                                       
                Shutdown(reason="Experiment timeout reached")     
            ],
    )
    
    robot_controller_cmd = OpaqueFunction(function=robot_controller_actions)

    
    ld = LaunchDescription()
    
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    
    ld.add_action(declare_num_robots_cmd)
    ld.add_action(declare_random_seed_cmd)
    ld.add_action(declare_experiment_duration_cmd)
    ld.add_action(declare_data_log_path_cmd)
    ld.add_action(declare_data_log_run_name_cmd)
    # ld.add_action(declare_use_old_fsm_cmd)
    # ld.add_action(declare_use_ekf_cmd)
    
    ld.add_action(traffic_manager)
    ld.add_action(data_logger_cmd)
    ld.add_action(timeout_cmd)

    ld.add_action(assessment_launch)
    
    ld.add_action(robot_controller_cmd)
    
    return ld