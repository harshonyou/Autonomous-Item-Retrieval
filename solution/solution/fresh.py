import math
import random
import sys
from enum import Enum
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Pose, Twist
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan, CameraInfo
from assessment_interfaces.msg import ItemList, Item, HomeZone, ItemHolders, ItemHolder
from rclpy.qos import QoSPresetProfiles
from nav_msgs.msg import Odometry
from image_geometry import PinholeCameraModel
from std_msgs.msg import Header, Bool
from rclpy.duration import Duration

import matplotlib.pyplot as plt
import numpy as np
import time

from solution_interfaces.msg import StateMarker, ProcessedItem, ProcessedItemList

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.trees import BehaviourTree
from py_trees import logging as log_tree
from py_trees.display import dot_tree, unicode_tree
from py_trees.common import ParallelPolicy

# Default Pose Orientation
DEFAULT_POSE_ORIENTATION_Z = 0.0
DEFAULT_POSE_ORIENTATION_W = 0.99

# Turning
TURN_LEFT = 1
TURN_RIGHT = -1

# Robot Movement
LINEAR_VELOCITY = 0.11 # 0.3
ANGULAR_VELOCITY = 0.5 # 0.5
DISTANCE_PROPRTIONAL = 0.5

# LiDAR Scan Segments
DEVIATION_THRESHOLD = 1
AVOIDANCE_THRESHOLD = 0.5
CONSTRAINT_THRESHOLD = 0.25
LETHAL_THRESHOLD = 0.225
SCAN_FRONT = 0
SCAN_FRONT_LEFT = 1
SCAN_FRONT_RIGHT = 2
SCAN_LEFT = 3
SCAN_RIGHT = 4
SCAN_BACK = 5

class State(Enum):
    AUTONOMOUS = 0
    AVOIDANCE = 1
    CONSTRAINT = 2
    LETHAL = 3
    HALT = 4

# Camera Properties
H_FOV = 1.085595
CAMERA_POS_GAZEBO = (0.076, 0.0, 0.093)
ACTUAL_RADIUS = 0.075
ACTUAL_DIAMETER = ACTUAL_RADIUS * 2

class LethalDetected(Behaviour):
    def __init__(self, name, robot_node):
        super(LethalDetected, self).__init__(name)
        self.robot_node = robot_node

    def update(self):
        if self.robot_node.state == State.LETHAL:
            return Status.FAILURE
        
        return Status.SUCCESS

class BallFound(Behaviour): # check if the robot already has a ball
    def __init__(self, name, robot_node):
        super(BallFound, self).__init__(name)
        self.robot_node = robot_node

    def update(self):
        if self.robot_node.grabbed_item.holding_item:
            return Status.SUCCESS
        
        if self.robot_node.ball_in_fov():
            self.logger.info(f"{len(self.robot_node.fov_items.data)} items found")
            return Status.SUCCESS

        return Status.FAILURE

class FindBall(Behaviour):
    def __init__(self, name, robot_node):
        super(FindBall, self).__init__(name)
        self.robot_node = robot_node

    def initialise(self):
        self.scout_direction = TURN_LEFT if random.random() > 0.5 else TURN_RIGHT

    def update(self):
        if self.robot_node.ball_in_fov():
            self.stop_rotating()
            self.logger.info(f"{len(self.robot_node.fov_items.data)} items found")
            return Status.SUCCESS
        
        # Continue rotating to find the ball
        self.rotate_to_find_ball()
        return Status.RUNNING

    def rotate_to_find_ball(self): # problomatic if rotation starts with COnstraint or Lethal state
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = ANGULAR_VELOCITY * self.scout_direction
        self.robot_node.cmd_vel_publisher.publish(cmd)
    
    def stop_rotating(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.robot_node.cmd_vel_publisher.publish(cmd)

class BallClose(Behaviour): # check if the ball robot has got is the highest value among the balls in the camera view
    def __init__(self, name, robot_node):
        super(BallClose, self).__init__(name)
        self.robot_node = robot_node
        
    def update(self):
        if not self.robot_node.ball_in_fov():
            return Status.FAILURE

        if self.robot_node.grabbed_item.holding_item:
            self.logger.info(f"Item {self.robot_node.grabbed_item.item_colour} is close: {self.robot_node.grabbed_item.item_value:.2f}")
            threshold_value = self.robot_node.grabbed_item.item_value
            fov_items = [item for item in self.robot_node.fov_items.data if item.value > threshold_value]
            
            if len(fov_items) == 0:
                return Status.SUCCESS
        
        item = max(self.robot_node.fov_items.data, key=self.robot_node.custom_key)
        Z_world = self.robot_node.calculate_distance_to_ball(item)

        if Z_world < 0.3:  # Ball is close
            self.logger.info(f"Item {item.colour} is close: {Z_world:.2f}")
            return Status.SUCCESS
        else:
            return Status.FAILURE

class ApproachBall(Behaviour):
    def __init__(self, name, robot_node):
        super(ApproachBall, self).__init__(name)
        self.robot_node = robot_node
        self.obstacle_distance_threshold = 0.5
    
    def update(self):
        if not self.robot_node.ball_in_fov():
            if self.robot_node.grabbed_item.holding_item:
                return Status.SUCCESS
            return Status.FAILURE

        item = max(self.robot_node.fov_items.data, key=self.robot_node.custom_key)
        Z_world = self.robot_node.calculate_distance_to_ball(item)

        if Z_world < 0.3:
            # Stop moving if close enough
            self.robot_node.stop_moving()
            self.logger.info(f"Item {item.colour} is close: {Z_world:.2f}")
            return Status.SUCCESS

        # is_obstacle_detected = any(distance < self.obstacle_distance_threshold for distance in self.robot_node.lidar_scan.ranges)
        # if is_obstacle_detected:
        #     # Stop moving if obstacle detected
        #     self.robot_node.stop_moving()
        #     self.logger.info(f"Obstacle detected: {min(self.robot_node.lidar_scan.ranges):.2f}")
        #     return Status.RUNNING
        
        # Approach the ball
        self.robot_node.approach_ball(item, Z_world)
        return Status.RUNNING

class BallGrasped(Behaviour): # check if ball is hoolding the highest value ball
    def __init__(self, name, robot_node):
        super(BallGrasped, self).__init__(name)
        self.robot_node = robot_node
        
    def update(self):
        # Check if the robot is holding an item
        if self.robot_node.grabbed_item.holding_item:
            # Further logic can be added to check for a ball of higher value
            # For example:
            # if self.robot_node.grabbed_item.item_value < some_value:
            #     return Status.FAILURE
            if self.robot_node.ball_in_fov():
                # fov_items = [item for item in self.robot_node.fov_items.data if item.value >= threshold_value]
                # if self.robot_node.grabbed_item.item_value < self.robot_node.fov_items.data[0].value:
                #     self.logger.info(f"Item {self.robot_node.grabbed_item.item_colour} is not the highest value")
                #     return Status.FAILURE
                for item in self.robot_node.fov_items.data:
                    if item.value > self.robot_node.grabbed_item.item_value:
                        self.logger.info(f"Item {self.robot_node.grabbed_item.item_colour} is not the highest value")
                        return Status.FAILURE
            
            return Status.SUCCESS
        else:
            return Status.FAILURE

class GraspBall(Behaviour):
    def __init__(self, name, robot_node, distance_to_move=0.2):
        super(GraspBall, self).__init__(name)
        self.robot_node = robot_node
        self.distance_to_move = distance_to_move  # Meters

    def initialise(self):
        self.moved = False
        self.pose = self.robot_node.pose

    def update(self):
        if not self.moved:
            # Move forward
            self.move_forward(self.distance_to_move)
            # self.moved = True
            return Status.RUNNING
        else:
            return Status.SUCCESS

    def move_forward(self, distance):
        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL
        self.robot_node.cmd_vel_publisher.publish(msg)
        
        difference_x = self.robot_node.pose.position.x - self.pose.position.x
        difference_y = self.robot_node.pose.position.y - self.pose.position.y
        
        distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)
        if distance_travelled >= distance:
            self.robot_node.stop_moving()
            self.moved = True

class HomeClose(Behaviour):
    def __init__(self, name, robot_node, threshold=0.9):
        super(HomeClose, self).__init__(name)
        self.robot_node = robot_node
        self.threshold = threshold

    def update(self):
        # Check if the home zone size in camera view is more than the threshold
        # if self.robot_node.home_zone_size_in_camera > self.threshold:
        #     return Status.SUCCESS
        # else:
        #     return Status.FAILURE
        
        # self.robot_node.logger.info(f"Home Zone: {self.robot_node.home_zone}")
        if self.robot_node.home_zone.size > self.threshold:
            if not self.robot_node.grabbed_item.holding_item:
                return Status.SUCCESS
        
        return Status.FAILURE

class ApproachHome(Behaviour): # enroute needs to check if the home zone is visible
    def __init__(self, name, robot_node, target_position, threshold=0.9):
        super(ApproachHome, self).__init__(name)
        self.robot_node = robot_node
        self.target_position = target_position
        self.threshold = threshold
    
    def initialise(self):
        self.enroute = False

    def update(self):
        # Command the robot to move to the target position
        # self.robot_node.navigator.go_to_pose(self.target_position)

        if not self.enroute:
            self.robot_node.go_to_pose(self.target_position)
            self.enroute = True
            return Status.RUNNING
        else:
            # if self.robot_node.state == State.CONSTRAINT:
            #     self.robot_node.stop_moving()
            #     self.logger.info(f"While enroute, constraint detected: {min(self.robot_node.scan[SCAN_LEFT], self.robot_node.scan[SCAN_RIGHT]):.3f}")
            #     return Status.RUNNING
            if self.robot_node.check_navigation_status():
                return Status.SUCCESS
            else:
                if self.robot_node.home_zone.size > self.threshold:
                    if not self.robot_node.grabbed_item.holding_item:
                        self.logger.info(f"Home Zone: {self.robot_node.home_zone}")
                        self.robot_node.stop_moving()
                        self.robot_node.navigator.cancelTask()
                        return Status.SUCCESS
                return Status.RUNNING

class BallPlaced(Behaviour):
    def __init__(self, name, robot_node):
        super(BallPlaced, self).__init__(name)
        self.robot_node = robot_node

    def update(self):
        if not self.robot_node.grabbed_item.holding_item:
            return Status.SUCCESS
        else:
            return Status.FAILURE

class PlaceBall(Behaviour):
    def __init__(self, name, robot_node):
        super(PlaceBall, self).__init__(name)
        self.robot_node = robot_node

    def update(self):
        sleep(1)
        return Status.SUCCESS


class AutonomousNavigation(Node):

    def __init__(self):
        super().__init__('autonomous_navigation', namespace='robot1')        
        self.robot_name = self.get_namespace().replace('/', '').strip()
        self.callback_group = ReentrantCallbackGroup()

        # Initial Pose
        self.declare_parameter('x_pose', 0.0)
        self.declare_parameter('y_pose', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x_pose').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y_pose').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        
        self.initial_pos = Pose()
        self.initial_pos.position.x = self.initial_x -3.5
        self.initial_pos.position.y = self.initial_y
        self.initial_pos.orientation.z = DEFAULT_POSE_ORIENTATION_Z
        self.initial_pos.orientation.w = DEFAULT_POSE_ORIENTATION_W
        
        # Logger
        self.logger = self.get_logger()
        self.logger.info(f"Initial Pose of Robot {self.robot_name}: ({self.initial_x:.2f}, {self.initial_y:.2f}), {self.initial_yaw:.2f}")
        
        # State
        self.state = State.AUTONOMOUS
        self.log_counter = 0
        
        # Camera Model
        self.camera_model:PinholeCameraModel = camera_model()
        
        # NAV2
        self.navigator = BasicNavigator(namespace=self.robot_name)
        
        # Odometry
        self.pose = Pose()
        
        # FOV Items
        self.fov_items = ItemList()
        
        # Lidar Scan
        self.lidar_scan = LaserScan()
        self.scan = [0.0] * 6
        # self.triggered_distance = [0.0] * 4
        # self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        # self.lidar_plot, = self.ax.plot([], [], 'b.')  # Initial empty plot
        # self.ax.set_theta_zero_location('N')  # Set 0 degrees to the top
        # plt.ion()  # Turn on interactive mode
        # plt.show()
        # self.last_update_time = time.time()
        
        # Home Zone
        self.home_zone = HomeZone()
        
        # Grabbed Item
        self.grabbed_item = ItemHolder()
        
        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.lidiar_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
            callback_group=self.callback_group
        )
        
        self.fov_items_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.fov_items_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.home_zone_subscriber = self.create_subscription(
            HomeZone,
            'home_zone',
            self.home_zone_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.garbbed_item_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.garbbed_item_callback,
            10,
            callback_group=self.callback_group
        )
        
        # log_tree.level = log_tree.Level.DEBUG
        log_tree.level = log_tree.Level.INFO
        self.tree = self.create_behavior_tree()
        dot_graph = dot_tree(self.tree.root)
        with open('tree.dot', 'w') as f:
            f.write(dot_graph.to_string())
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_publisher = self.create_publisher(StateMarker, 'raw_state_marker', 10)
        
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.temp_logger_counter = 0
        self.controller_timer = self.create_timer(self.timer_period, self.control_loop)
    
    def odom_callback(self, msg):
        self.pose = msg.pose.pose

        (roll, pitch, yaw) = get_euler_from_quaternion(self.pose.orientation)
        
        self.yaw = yaw
    
    def lidar_callback(self, msg):
        self.lidar_scan = msg
        
        # current_time = time.time()
        # if current_time - self.last_update_time >= 1:  # Update every second
        #     angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        #     ranges = np.array(msg.ranges)
        #     ranges[np.isinf(ranges)] = msg.range_max  # Replace inf with max range

        #     # Update the plot
        #     self.lidar_plot.set_data(angles, ranges)
        #     self.ax.relim()  # Recompute the ax.dataLim
        #     self.ax.autoscale_view()  # Update axes with new data
        #     plt.draw()
        #     plt.pause(0.001)

        #     self.last_update_time = current_time  # Update last update time


        
        front_left_ranges = msg.ranges[:45]
        front_right_ranges = msg.ranges[315:]
        front_ranges = front_left_ranges + front_right_ranges
        left_ranges = msg.ranges[45:135]
        right_ranges = msg.ranges[225:315]
        back_ranges = msg.ranges[135:225]
        
        self.scan[SCAN_FRONT] = min(front_ranges)
        self.scan[SCAN_FRONT_LEFT] = min(front_left_ranges)
        self.scan[SCAN_FRONT_RIGHT] = min(front_right_ranges)
        self.scan[SCAN_LEFT] = min(left_ranges)
        self.scan[SCAN_RIGHT] = min(right_ranges)
        self.scan[SCAN_BACK] = min(back_ranges)

    def fov_items_callback(self, msg):
        # self.logger.info(f"FOV Items: {msg}")
        self.fov_items = msg
        self.fov_items.data = [item for item in self.fov_items.data if item.y >= 2 and item.y <= 4]

    def home_zone_callback(self, msg):
        self.home_zone = msg
    
    def garbbed_item_callback(self, msg):
        for item_holder in msg.data:
            if item_holder.robot_id == self.robot_name:
                self.grabbed_item = item_holder
                break
    
    def create_behavior_tree(self):
        # # Create specific behaviors
        # ball_found = BallFound("Ball Found", self)
        # find_ball = FindBall("Find Ball", self)

        # # Build the tree structure
        # root = Selector(name="Root", memory=True)
        # main_sequence = Selector(name="Main Sequence", memory=True)
        # root.add_child(main_sequence)
        # main_sequence.add_child(ball_found)
        # main_sequence.add_child(find_ball)

        # return BehaviourTree(root)
        
        # Create specific behaviors
        lethal_detected = LethalDetected("Lethal Detected", self)
        ball_found = BallFound("Ball Found", self)
        find_ball = FindBall("Find Ball", self)
        ball_close = BallClose("Ball Close", self)
        approach_ball = ApproachBall("Approach Ball", self)
        ball_grasped = BallGrasped("Ball Grasped", self)
        grasp_ball = GraspBall("Grasp Ball", self)
        home_close = HomeClose("Home Close", self)
        approach_home = ApproachHome("Approach Home", self, self.initial_pos)
        ball_placed = BallPlaced("Ball Placed", self)
        place_ball = PlaceBall("Place Ball", self)

        # Build the tree structure
        root = Sequence(name="Root", memory=True)

        # Sequence for finding the ball
        find_sequence = Selector(name="Find Sequence", memory=True)
        find_sequence.add_child(ball_found)
        find_sequence.add_child(find_ball)

        # Sequence for approaching the ball
        approach_sequence = Selector(name="Approach Sequence", memory=True)
        approach_sequence.add_child(ball_close)
        approach_sequence.add_child(approach_ball)
        
        # Sequence for grasping the ball
        grasp_sequence = Selector(name="Grasp Sequence", memory=True)
        grasp_sequence.add_child(ball_grasped)
        grasp_sequence.add_child(grasp_ball)
        
        # Sequence for enrouting to home
        home_sequence = Selector(name="Home Sequence", memory=True)
        home_sequence.add_child(home_close)
        home_sequence.add_child(approach_home)
        # Sequence for placing the ball
        place_sequence = Selector(name="Place Sequence", memory=True)
        place_sequence.add_child(ball_placed)
        place_sequence.add_child(place_ball)

        # Add sequences to the root
        root.add_child(lethal_detected)
        root.add_child(find_sequence)
        root.add_child(approach_sequence)
        root.add_child(grasp_sequence)
        root.add_child(home_sequence)
        root.add_child(place_sequence)

        return BehaviourTree(root)
        
    def set_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose = self.initial_pos
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

    def post_tick_handler(self):
        # tree_ascii = unicode_tree(self.tree.root)
        # self.get_logger().info("\n" + tree_ascii)
        
        with open("behavior_tree.txt", "w") as file:
            tree_ascii = unicode_tree(self.tree.root)
            file.write(tree_ascii)

    # Potential States: Halt, Constraint, Avoidance, Autonomous, Lethal

    def state_machine(self):
        if self.state == State.LETHAL:
            return
        elif self.scan[SCAN_FRONT] < LETHAL_THRESHOLD:
            self.logger.info(f"Lethal Detected: {self.scan[SCAN_FRONT]:.3f}")
            self.stop_moving()
            self.navigator.cancelTask()
            self.navigator.backup(backup_dist=0.15, backup_speed=LINEAR_VELOCITY, time_allowance=10)
            self.log_counter = 0
            self.state = State.LETHAL
        elif self.scan[SCAN_LEFT] < CONSTRAINT_THRESHOLD or self.scan[SCAN_RIGHT] < CONSTRAINT_THRESHOLD:
            self.logger.info(f"Constraint Detected: {min(self.scan[SCAN_LEFT], self.scan[SCAN_RIGHT]):.3f}")
            self.state = State.CONSTRAINT
        else:
            self.state = State.AUTONOMOUS
    
    def post_state_machine(self):
        marker = StateMarker()
        marker.text = str(self.state)
        marker.color.a = 1.0
        marker.pose = self.pose
        
        match self.state:
            case State.AUTONOMOUS:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            case State.AVOIDANCE:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            case State.CONSTRAINT:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            case State.LETHAL:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            case State.HALT:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            case _:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0
        
        self.marker_publisher.publish(marker)
        
    
    def control_loop(self):
        # self.logger.info(f"Fresh running")    
        
        # if self.scan[SCAN_FRONT] < LETHAL_THRESHOLD:
        #     self.logger.info(f"Lethal Detected: {self.scan[SCAN_FRONT]:.3f}")
        
        # return
        
        self.state_machine()
        self.post_state_machine()
            
        # self.tree.tick()
        # self.post_tick_handler()
        
        # self.logger.info(f"State: {self.state}")
        
        self.temp_logger_counter += 1
        if self.temp_logger_counter % 20 == 0:
            self.logger.info(f"Front: {self.scan[SCAN_FRONT]:.3f}, Front Left: {self.scan[SCAN_FRONT_LEFT]:.3f}, Front Right: {self.scan[SCAN_FRONT_RIGHT]:.3f}, Left: {self.scan[SCAN_LEFT]:.3f}, Right: {self.scan[SCAN_RIGHT]:.3f}")
            self.temp_logger_counter = 0
        
        self.tree.tick()
        self.post_tick_handler()
        
        match self.state:
            # case State.AUTONOMOUS | State.CONSTRAINT:
            case State.LETHAL:
                if not self.navigator.isTaskComplete():
                    if self.scan[SCAN_BACK] < LETHAL_THRESHOLD:
                        self.logger.info(f"Lethal Detected: {self.scan[SCAN_BACK]:.3f} on backing up, canceling task")
                        self.navigator.cancelTask()
                    
                    feedback = self.navigator.getFeedback()
                    if feedback and self.log_counter % 10 == 0:
                        self.get_logger().info(f"Distance travelled: {feedback.distance_traveled:.2f} metres")
                else:
                    self.state = State.AUTONOMOUS
                    self.log_counter = 0
                    
                    result = self.navigator.getResult()
                    match result:
                        case TaskResult.SUCCEEDED:
                            self.get_logger().info("Backup succeeded!")
                        case TaskResult.CANCELED:
                            self.get_logger().info("Backup canceled!")
                        case TaskResult.FAILED:
                            self.get_logger().info("Backup failed!")
                        case _:
                            self.get_logger().info("Backup unknown result {result}!")

        
        
        # angular_z_correction = 0.0
        # Z_world = 1.0
        # if self.scan[SCAN_LEFT] < DEVIATION_THRESHOLD:
        #     angular_z_correction += ( TURN_RIGHT * ANGULAR_VELOCITY * DISTANCE_PROPRTIONAL) / Z_world
        # if self.scan[SCAN_RIGHT] < DEVIATION_THRESHOLD:
        #     angular_z_correction += (TURN_LEFT * ANGULAR_VELOCITY * DISTANCE_PROPRTIONAL) / Z_world
        
        # self.logger.info(f"Angular Z Correction: {angular_z_correction:.3f}")
    
    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        if self.state == State.CONSTRAINT:
            msg.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(msg)
    
    def ball_in_fov(self):
        return len(self.fov_items.data) != 0
    
    def custom_key(self, item):
        w_value = 1.0
        w_diameter = 0.5
        return w_value * item.value / 15.0 + w_diameter * item.diameter / 640.0
    
    def calculate_distance_to_ball(self, item):
        diameter_pixels = item.diameter
        scale_factor = ACTUAL_DIAMETER / diameter_pixels
        focal_length = self.camera_model.fx()
        
        Z_camera = focal_length * scale_factor
        Z_world = Z_camera + CAMERA_POS_GAZEBO[2]
        return Z_world
    
    def approach_ball(self, item, Z_world):
        msg = Twist()
        
        x_smoothness = Z_world
        y_smoothness = 1
        
        if Z_world < 0.5:
            x_smoothness = 0.5
            y_smoothness = Z_world
        
        
        angular_z_correction = 0.0
        try:
            if self.scan[SCAN_FRONT_LEFT] < DEVIATION_THRESHOLD:
                angular_z_correction += (TURN_RIGHT * ANGULAR_VELOCITY * DISTANCE_PROPRTIONAL * y_smoothness) / self.scan[SCAN_FRONT_LEFT]
            if self.scan[SCAN_FRONT_RIGHT] < DEVIATION_THRESHOLD:
                angular_z_correction += (TURN_LEFT * ANGULAR_VELOCITY * DISTANCE_PROPRTIONAL * y_smoothness) / self.scan[SCAN_FRONT_RIGHT]
        except ZeroDivisionError:
            pass
        
        if angular_z_correction != 0.0:
            self.logger.info(f"Deviation Angular Z Correction: {angular_z_correction:.3f}")
        
        msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL * x_smoothness
        msg.angular.z = (item.x / 320.0) + angular_z_correction

        self.publish_cmd_vel(msg.linear.x, msg.angular.z)
        # self.cmd_vel_publisher.publish(msg)
        
    def stop_moving(self):
        msg = Twist()
        self.publish_cmd_vel(msg.linear.x, msg.angular.z)
        # self.cmd_vel_publisher.publish(msg)
        
    def go_to_pose(self, pos):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose = pos
        
        self.navigator.goToPose(goal_pose)
        
    def check_navigation_status(self):
        try:
            msg = Twist()
            msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL
            if self.scan[SCAN_FRONT_LEFT] < AVOIDANCE_THRESHOLD:
                msg.angular.z += (TURN_RIGHT * ANGULAR_VELOCITY * DISTANCE_PROPRTIONAL) / self.scan[SCAN_FRONT_LEFT]
            if self.scan[SCAN_FRONT_RIGHT] < AVOIDANCE_THRESHOLD:
                msg.angular.z += (TURN_LEFT * ANGULAR_VELOCITY * DISTANCE_PROPRTIONAL) / self.scan[SCAN_FRONT_RIGHT]
            if msg.angular.z != 0.0:
                self.logger.info(f"Avoidance Angular Z Correction: {msg.angular.z:.3f}")
                self.publish_cmd_vel(msg.linear.x, msg.angular.z)
        except ZeroDivisionError:
            pass
        
        return self.navigator.isTaskComplete()

    def destroy_node(self):
        self.navigator.cancelTask()
        self.stop_moving()
        super().destroy_node()
    
def camera_model():
    camera_info = CameraInfo()

    camera_info.height = 480
    camera_info.width = 640

    camera_info.distortion_model = "plumb_bob"
    camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

    camera_info.k = [530.4669406576809, 0.0, 320.5, 0.0, 530.4669406576809, 240.5, 0.0, 0.0, 1.0]
    camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info.p = [530.4669406576809, 0.0, 320.5, -0.0, 0.0, 530.4669406576809, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]

    camera_info.binning_x = 0
    camera_info.binning_y = 0
    camera_info.roi.x_offset = 0
    camera_info.roi.y_offset = 0
    camera_info.roi.height = 0
    camera_info.roi.width = 0
    camera_info.roi.do_rectify = False
    
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)
    
    return camera_model

def get_euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = AutonomousNavigation()
    
    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # except ExternalShutdownException:
    #     sys.exit(1)
    # finally:
    #     node.destroy_node()
    #     rclpy.try_shutdown()


if __name__ == '__main__':
    main()