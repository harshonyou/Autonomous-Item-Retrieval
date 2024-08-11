# Autonomous Mobile Robot Item Retrieval System

This repository contains the implementation of an autonomous mobile robot system designed for item retrieval tasks within a simulated environment. Built with ROS 2 Humble Hawksbill, Gazebo Classic 11, and utilizing the TurtleBot3 Waffle Pi robots, this project showcases an integrated approach to exploration, item collection, and navigation in an obstacle-laden terrain.

## System Overview

The system leverages ROS 2's modularity and real-time capabilities to orchestrate a fleet of TurtleBot3 Waffle Pi robots equipped with LiDAR, cameras, and odometry sensors. The implementation focuses on autonomous exploration, dynamic obstacle avoidance, item detection and retrieval, and efficient path planning back to a designated home zone.

## Prerequisites

- Ubuntu 20.04 LTS
- ROS 2 Humble Hawksbill
- Gazebo Classic 11
- TurtleBot3 packages for ROS 2

Ensure ROS 2 and Gazebo are properly installed on your system. For installation instructions, refer to the official ROS 2 documentation and the Gazebo installation guide.

## Installation

Clone the repository and build the ROS 2 workspace:

```bash
# Remove pre-eixsting build
rm -rf build/ log install/

# Build the workspace
colcon build --symlink-install

```

## Configuration

Before running the simulation, configure the TurtleBot3 model by setting the `TURTLEBOT3_MODEL` environment variable:

```bash
export TURTLEBOT3_MODEL=waffle_pi

```

Make sure you source the workspace:

```bash
source /home/user/workshop-dir/install/setup.bash

```


## Running the Simulation

Launch the simulation environment with the TurtleBot3 robots in Gazebo:

```bash
ros2 launch solution sol_launch.py num_robots:=3 2>&1 | tee results.log
```

Available arguments:

```bash
Arguments (pass arguments as '<name>:=<value>'):

    'num_robots':
        Number of robots to spawn
        (default: '1')

    'random_seed':
        Random number seed for item manager
        (default: '0')

    'experiment_duration':
        Experiment duration in seconds
        (default: '300.0')

    'data_log_path':
        Full path to directory where data logs will be saved
        (default: '/home/user/workshop-dir/install/solution/logs/')

    'data_log_run_name':
        Run name to use for data logs
        (default: 'data_log')

    'visualise_sensors':
        Whether to visualise sensors in Gazebo
        (default: 'false')

    'odometry_source':
        Odometry source - ENCODER or WORLD
        (default: 'ENCODER')

    'sensor_noise':
        Whether to enable sensor noise (applies to camera, LiDAR, and IMU)
        (default: 'false')

    'use_rviz':
        Whether to start RViz
        (default: 'True')

    'rviz_config':
        Full path to the RViz config file to use
        (default: LocalVar('FindPackageShare(pkg='assessment') + 'rviz' + 'namespaced.rviz''))

    'rviz_windows':
        Full path to the RViz windows YAML file to use
        (default: LocalVar('FindPackageShare(pkg='assessment') + 'config' + 'rviz_windows.yaml''))

    'obstacles':
        Whether the world contains obstacles
        (default: 'true')

    'item_manager':
        Whether to start the item manager
        (default: 'true')

    'use_nav2':
        Whether to use the navigation stack (Nav2)
        (default: 'False')

    'map':
        Full path to map file to load
        (default: '/home/user/workshop-dir/install/assessment/share/assessment/maps/assessment_world.yaml')

    'params_file':
        Full path to the ROS2 parameters file to use for all launched nodes
        (default: '/home/user/workshop-dir/install/assessment/share/assessment/params/nav2_params_namespaced.yaml')

    'headless':
        Whether to run the Gazebo GUI
        (default: 'False')

    'limit_real_time_factor':
        Whether to limit the Gazebo real-time factor to 1.0
        (default: 'True')

    'wait_for_items':
        Whether to wait for every item to spawn before spawning any robots
        (default: 'False')

    'gazebo_verbose':
        Enable/disable verbose output for gzserver and gzclient
        (default: 'False')

    'version':
        Set "true" to output version information
        (default: 'false')

    'verbose':
        Set "true" to increase messages written to terminal
        (default: 'false')

    'help':
        Set "true" to produce gzclient help message
        (default: 'false')

    'extra_gazebo_args':
        Extra arguments to be passed to Gazebo
        (default: '')

    'gdb':
        Set "true" to run gzserver with gdb
        (default: 'false')

    'valgrind':
        Set "true" to run gzserver with valgrind
        (default: 'false')

    'gui_required':
        Set "true" to shut down launch script when GUI is terminated
        (default: 'false')
```

## Nodes

- **Autonomous Navigation:** A ROS 2 node designed for autonomous navigation with complex behaviors based on environmental feedback and sensor data. It uses behavior trees to manage navigation tasks, obstacle avoidance, and item interaction strategies.

- **Data Logger:** A ROS 2 node for logging data related to robot navigation and item interaction in a CSV format.

- **FSM Old Implementation:** Old Implementation of the autonomous navigation using solely State Machines.

- **Item Cloud:** A ROS 2 node that converts items detected in 2D space into a 3D point cloud representation. Additionally, it processes these items to calculate their 3D coordinates based on their 2D positions and publishes this processed item list.

- **LiDAR Cloud:**     A ROS 2 node that converts 2D laser scan data into a 3D point cloud by extruding the 2D scan points into the vertical dimension. This is particularly useful for simulating vertical structures like walls in environments where only 2D laser scans are available.

- **Sensor Fusion:** A ROS 2 node for fusing camera and laser scan data to enhance obstacle detection. This node subscribes to a topic publishing items detected by a camera and a topic for laser scan data, merges these data sources, and republishes the enhanced laser scan data.

- **State Marker Transformer:** A ROS 2 node that subscribes to StateMarker messages, transforms their positions from the 'odom' coordinate frame to the 'map' coordinate frame, and publishes the transformed state as a Marker message for visualization purposes.

- **Traffic Manager:** A ROS 2 node responsible for managing the traffic of a fleet of robots by ensuring they maintain a safe distance from each other. It subscribes to the pose of each robot and their item holding status, calculates the proximity between robots, and publishes halt commands to prevent collisions. Additionally, it publishes peer lists and combined point clouds for visualization purposes.

- **Transformer:** A ROS 2 node for transforming the coordinates of peers and processed items from their original frames to the robot's base frame.


## Visualization

Use RViz for real-time visualization of the robot's sensor data and navigation paths:

```bash
ros2 launch solution sol_launch.py use_rviz:=True

```

## Analysis

Use the provided Python Notebook and update the log output dir path to get the up-to-date graphs and heatmaps.
