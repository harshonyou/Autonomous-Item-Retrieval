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

from solution_interfaces.msg import StateMarker, ProcessedItem, ProcessedItemList, Peers, PeersList, Halt

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.trees import BehaviourTree
from py_trees import logging as log_tree
from py_trees.display import dot_tree, unicode_tree
from py_trees.common import ParallelPolicy

from .pkgs.camera import camera_model
from .pkgs.tf import get_euler_from_quaternion

# Default Pose Orientation
DEFAULT_POSE_ORIENTATION_Z = 0.0
DEFAULT_POSE_ORIENTATION_W = 0.99

# Turning
TURN_LEFT = 1
TURN_RIGHT = -1

# Robot Movement
LINEAR_VELOCITY = 0.11 # 0.3
ANGULAR_VELOCITY = 0.5 # 0.5
LINEAR_DISTANCE_PROPRTIONAL = 0.75
ANGULAR_DISTANCE_PROPRTIONAL = 0.80
DEVIATION_SMOOTHNESS_THRESHOLD = 0.5
MIN_X_SMOOTHNESS = 0.5

# LiDAR Scan Segments
DEVIATION_THRESHOLD = 0.75
AVOIDANCE_THRESHOLD = 0.5
CONSTRAINT_THRESHOLD = 0.275
LETHAL_THRESHOLD = 0.25
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
    """
    A behavior to detect lethal obstacles based on the robot's current state.
    
    Attributes:
        robot_node (AutonomousNavigation): Reference to the main robot node handling navigation and sensor data.
    """
    def __init__(self, name, robot_node):
        super(LethalDetected, self).__init__(name)
        self.robot_node = robot_node

    def update(self):
        if self.robot_node.state == State.LETHAL:
            return Status.FAILURE
        
        return Status.SUCCESS

class BallFound(Behaviour):
    """
    Checks if the robot has detected a ball within its field of view (FOV) or if it already holds a ball.
    
    Attributes:
        robot_node (AutonomousNavigation): Reference to the main robot node.
    """

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
    """
    Rotates the robot to find a ball if not already detected in the FOV.
    
    Attributes:
        robot_node (AutonomousNavigation): Reference to the main robot node.
        scout_direction (int): Direction to rotate in search of a ball.
    """
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

    def rotate_to_find_ball(self): # problomatic if rotation starts with Constraint or Lethal state
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = ANGULAR_VELOCITY * self.scout_direction
        self.robot_node.cmd_vel_publisher.publish(cmd)
    
    def stop_rotating(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.robot_node.cmd_vel_publisher.publish(cmd)

class BallClose(Behaviour):
    """
    Determines if the detected ball is close enough to approach.
    
    Attributes:
        robot_node (AutonomousNavigation): Reference to the main robot node.
    """
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
    """
    Controls the robot to approach a detected ball, considering obstacles detected via lidar.
    
    Attributes:
        robot_node (AutonomousNavigation): Reference to the main robot node.
    """
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
        
        # Approach the ball
        self.robot_node.approach_ball(item, Z_world)
        return Status.RUNNING

class BallGrasped(Behaviour):
    """
    Checks if the robot has successfully grasped a ball.
    
    Attributes:
        robot_node (AutonomousNavigation): Reference to the main robot node.
    """
    def __init__(self, name, robot_node):
        super(BallGrasped, self).__init__(name)
        self.robot_node = robot_node
        
    def update(self):
        # Check if the robot is holding an item
        if self.robot_node.grabbed_item.holding_item:
            if self.robot_node.ball_in_fov():
                for item in self.robot_node.fov_items.data:
                    if item.value > self.robot_node.grabbed_item.item_value:
                        self.logger.info(f"Item {self.robot_node.grabbed_item.item_colour} is not the highest value")
                        return Status.FAILURE
            
            return Status.SUCCESS
        else:
            return Status.FAILURE

class GraspBall(Behaviour):
    """
    Activates the robot's mechanism to grasp a ball.
    
    Attributes:
        robot_node (AutonomousNavigation): Reference to the main robot node.
        distance_to_move (float): Distance for the robot to move forward to grasp the ball.
    """
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
        msg.linear.x = LINEAR_VELOCITY * LINEAR_DISTANCE_PROPRTIONAL
        self.robot_node.cmd_vel_publisher.publish(msg)
        
        difference_x = self.robot_node.pose.position.x - self.pose.position.x
        difference_y = self.robot_node.pose.position.y - self.pose.position.y
        
        distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)
        if distance_travelled >= distance:
            self.robot_node.stop_moving()
            self.moved = True

class HomeClose(Behaviour):
    """
    Determines if the robot is close to the home zone.
    
    Attributes:
        robot_node (AutonomousNavigation): Reference to the main robot node.
        threshold (float): Threshold distance to consider the robot as close to the home zone.
    """
    def __init__(self, name, robot_node, threshold=0.9):
        super(HomeClose, self).__init__(name)
        self.robot_node = robot_node
        self.threshold = threshold

    def update(self):
        # self.robot_node.logger.info(f"Home Zone: {self.robot_node.home_zone}")
        if self.robot_node.home_zone.size > self.threshold:
            if not self.robot_node.grabbed_item.holding_item:
                return Status.SUCCESS
        
        return Status.FAILURE

class ApproachHome(Behaviour):
    """
    Navigates the robot towards the home zone to deposit a grasped ball.
    
    Attributes:
        robot_node (AutonomousNavigation): Reference to the main robot node.
        target_position (Pose): Target pose within the home zone.
        threshold (float): Threshold distance to consider the robot as close to the home zone.
    """

    def __init__(self, name, robot_node, target_position, threshold=0.9):
        super(ApproachHome, self).__init__(name)
        self.robot_node = robot_node
        self.target_position = target_position
        self.threshold = threshold
    
    def initialise(self):
        self.enroute = False

    def update(self):
        if not self.enroute:
            self.robot_node.go_to_pose(self.target_position)
            self.enroute = True
            return Status.RUNNING
        else:
            if self.robot_node.check_navigation_status():
                return Status.SUCCESS
            else:
                if self.robot_node.grabbed_item.holding_item:
                    if self.robot_node.ball_in_fov():
                        for item in self.robot_node.fov_items.data:
                            if item.value > self.robot_node.grabbed_item.item_value:
                                self.logger.info(f"Item {self.robot_node.grabbed_item.item_colour} is not the highest value")
                                self.robot_node.stop_moving()
                                self.robot_node.navigator.cancelTask()
                                return Status.FAILURE
                
                if self.robot_node.home_zone.size > self.threshold:
                    if not self.robot_node.grabbed_item.holding_item:
                        self.logger.info(f"Home Zone: {self.robot_node.home_zone}")
                        self.robot_node.stop_moving()
                        self.robot_node.navigator.cancelTask()
                        return Status.SUCCESS
                return Status.RUNNING

class BallPlaced(Behaviour):
    """
    Checks if the robot has placed the ball in the home zone.
    
    Attributes:
        robot_node (AutonomousNavigation): Reference to the main robot node.
    """
    def __init__(self, name, robot_node):
        super(BallPlaced, self).__init__(name)
        self.robot_node = robot_node

    def update(self):
        if not self.robot_node.grabbed_item.holding_item:
            return Status.SUCCESS
        else:
            return Status.FAILURE

class PlaceBall(Behaviour):
    """
    Activates the robot's mechanism to place the ball in the home zone.
    
    Attributes:
        robot_node (AutonomousNavigation): Reference to the main robot node.
    """
    def __init__(self, name, robot_node):
        super(PlaceBall, self).__init__(name)
        self.robot_node = robot_node

    def update(self):
        sleep(1)
        return Status.SUCCESS


class AutonomousNavigation(Node):
    """
    A ROS 2 node designed for autonomous navigation with complex behaviors based on environmental feedback and sensor data. It uses behavior trees to manage navigation tasks, obstacle avoidance, and item interaction strategies.

    Attributes:
        robot_name (str): The name of the robot, derived from its namespace.
        navigator (BasicNavigator): A BasicNavigator object for handling navigation tasks.
        pose (Pose): The current pose of the robot.
        lidar_scan (LaserScan): The latest lidar scan data.
        fov_items (ItemList): Items currently visible in the field of view.
        home_zone (HomeZone): Information about the home zone.
        grabbed_item (ItemHolder): The item currently held by the robot, if any.
        peers (PeersList): List of peers and their positions.
        halted (bool): Indicates whether the robot is currently halted.
        state (State): The current state of the robot, based on defined behaviors.
    """
    
    def __init__(self):
        """
        Initializes the AutonomousNavigation node, setting up parameters, subscribers, publishers, and the behavior tree.
        """
        super().__init__('autonomous_navigation')
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
        self.initial_pos.position.x = self.initial_x
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
        self.navigator = BasicNavigator()
        self.set_initial_pose()
        
        # Odometry
        self.pose = Pose()
        
        # FOV Items
        self.fov_items = ItemList()
        
        # Lidar Scan
        self.lidar_scan = LaserScan()
        self.scan = [0.0] * 6
        
        # Home Zone
        self.home_zone = HomeZone()
        
        # Grabbed Item
        self.grabbed_item = ItemHolder()
        
        # Peers
        self.peers = PeersList()
        
        # Halt
        self.halted = False
        
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
            ProcessedItemList,
            'tf_processed_items',
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
        
        self.peers_subscriber = self.create_subscription(
            PeersList,
            'tf_peers',
            self.peers_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.halt_subscriber = self.create_subscription(
            Halt,
            'halt',
            self.halt_callback,
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
        """
        Callback for odometry data. Updates the robot's current pose.

        Args:
            msg (Odometry): The odometry message containing the robot's pose.
        """
        self.pose = msg.pose.pose

        (roll, pitch, yaw) = get_euler_from_quaternion(self.pose.orientation)
        
        self.yaw = yaw
    
    def lidar_callback(self, msg):
        """
        Callback for lidar scan data. Processes the scan data for use in navigation decisions.

        Args:
            msg (LaserScan): The lidar scan message.
        """
        self.lidar_scan = msg
        
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
        """
        Callback for processed items detected in the field of view.

        Args:
            msg (ProcessedItemList): The list of processed items.
        """
        self.fov_items = msg
        self.fov_items.data = [item for item in self.fov_items.data if item.y >= 2 and item.y <= 4]

    def home_zone_callback(self, msg):
        """
        Callback for home zone data. Updates the robot's knowledge of the home zone location and size.

        Args:
            msg (HomeZone): The home zone message.
        """
        self.home_zone = msg
    
    def garbbed_item_callback(self, msg):
        """
        Callback for the item currently held by the robot.

        Args:
            msg (ItemHolders): The item holders message.
        """
        for item_holder in msg.data:
            if item_holder.robot_id == self.robot_name:
                self.grabbed_item = item_holder
                break
    
    def peers_callback(self, msg:PeersList):
        """
        Callback for peers list. Updates the robot's knowledge of peer positions.

        Args:
            msg (PeersList): The peers list message.
        """
        self.peers = msg
        self.peers.data = msg.data
        
    def halt_callback(self, msg):
        """
        Callback for halt commands. Controls the robot's motion based on halt status.

        Args:
            msg (Halt): The halt message.
        """
        if msg.status:
            self.halted = True
        else:
            self.halted = False
    
    def create_behavior_tree(self):
        """
        Creates and initializes the behavior tree used for decision-making in autonomous navigation.

        Returns:
            BehaviourTree: The root of the behavior tree.
        """
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
        """
        Sets the robot's initial pose in the navigator for navigation tasks.
        """
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose = self.initial_pos
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

    def post_tick_handler(self):
        """
        Handles post-tick actions for the behavior tree, such as logging tree status.
        """
        # tree_ascii = unicode_tree(self.tree.root)
        # self.get_logger().info("\n" + tree_ascii)
        
        with open(f"behavior_tree_{self.robot_name}.txt", "w") as file:
            tree_ascii = unicode_tree(self.tree.root)
            file.write(tree_ascii)
        pass
        
    def state_machine(self):
        """
        State Machine Manager, responsible for managing the robot's state based on sensor data and environmental feedback.
        """
        if self.state == State.LETHAL:
            return
        elif self.halted:
            if self.state == State.HALT:
                return
            self.logger.info(f"Robot {self.robot_name} is halted")
            self.stop_moving()
            self.navigator.cancelTask()
            self.state = State.HALT
        elif self.scan[SCAN_FRONT] < LETHAL_THRESHOLD:
            self.logger.info(f"Lethal Detected: {self.scan[SCAN_FRONT]:.3f}")
            self.stop_moving()
            self.navigator.cancelTask()
            self.navigator.backup(backup_dist=0.15, backup_speed=LINEAR_VELOCITY, time_allowance=10)
            sleep(5)
            self.log_counter = 0
            self.state = State.LETHAL
        elif self.scan[SCAN_LEFT] < CONSTRAINT_THRESHOLD or self.scan[SCAN_RIGHT] < CONSTRAINT_THRESHOLD:
            # self.logger.info(f"Constraint Detected: {min(self.scan[SCAN_LEFT], self.scan[SCAN_RIGHT]):.3f}")
            self.state = State.CONSTRAINT
        else:
            self.state = State.AUTONOMOUS
    
    def post_state_machine(self):
        """
        Publishes the current state of the robot as a marker for visualization.
        """
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
        """
        Main control loop for the node, ticked at a regular interval. It evaluates the behavior tree and publishes necessary commands.
        """
        self.state_machine()
        self.post_state_machine()
        
        if self.state == State.HALT:
            sleep(1)
            return
            
        self.tree.tick()
        
        self.temp_logger_counter += 1
        if self.temp_logger_counter % 5 == 0:
            self.post_tick_handler()
            # self.logger.info(f"Front: {self.scan[SCAN_FRONT]:.3f}, Front Left: {self.scan[SCAN_FRONT_LEFT]:.3f}, Front Right: {self.scan[SCAN_FRONT_RIGHT]:.3f}, Left: {self.scan[SCAN_LEFT]:.3f}, Right: {self.scan[SCAN_RIGHT]:.3f}")
            self.temp_logger_counter = 0
        
        match self.state:
            case State.LETHAL:
                if not self.navigator.isTaskComplete():
                    if self.scan[SCAN_BACK] < LETHAL_THRESHOLD:
                        self.logger.info(f"Lethal Detected: {self.scan[SCAN_BACK]:.3f} on backing up, canceling task")
                        self.navigator.cancelTask()
                    
                    feedback = self.navigator.getFeedback()
                    try:
                        if feedback and self.log_counter % 10 == 0:
                            self.get_logger().info(f"Distance travelled: {feedback.distance_traveled:.2f} metres")
                    except AttributeError:
                        self.logger.info(f"Feedback: {feedback}")
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
    
    def publish_cmd_vel(self, linear_x, angular_z):
        """
        Publishes linear and angular velocity commands to the robot's cmd_vel topic.
         
        Args:
            linear_x (float): Linear velocity in the x-axis.
            angular_z (float): Angular velocity in the z-axis.
        """
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        if self.state == State.CONSTRAINT:
            msg.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(msg)
    
    def ball_in_fov(self):
        """
        Checks if there are any balls within the robot's field of view.
        
        Returns:
            bool: True if there are balls detected, False otherwise.
        """
        return len(self.fov_items.data) != 0
    
    def custom_key(self, item):
        """
        Custom key function for sorting items based on value, diameter, distance, and proximity to peers.
        
        Args:
            item (Item): The item to calculate the priority for.
            
        Returns:
            float: The calculated priority of the item.
        """
        w_value = 1.0
        w_diameter = 0.5
        w_distance = 0.2
        w_proximity_to_peers = 0.5
        
        # Normalize value and diameter as before
        normalized_value = w_value * item.value / 15.0
        normalized_diameter = w_diameter * item.diameter / 640.0
        
        # Normalize distance (assuming a maximum relevant distance, adjust as needed)
        max_distance = 8.5  # Diamter of 6x6 map (1x6 is safe zone)
        normalized_distance = (max_distance - item.distance) / max_distance

        peer_proximity_penalty = 0
        for peer in self.peers.data:
            distance_to_peer = self.calculate_distance(item.x_coord, item.y_coord, peer.x_coord, peer.y_coord)
            if distance_to_peer < 3.5:  # Example proximity threshold
                # Scale penalty based on distance (closer items receive a higher penalty)
                penalty = w_proximity_to_peers * (1 - distance_to_peer / 3.5)
                peer_proximity_penalty += penalty
        
        priority = normalized_value + normalized_diameter + (w_distance * normalized_distance) - peer_proximity_penalty
        return priority
    
    def calculate_distance(self, x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        
        Args:
            x1, y1: Coordinates of the first point.
            x2, y2: Coordinates of the second point.
            
        Returns:
            float: The distance between the points.
        """
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    
    def calculate_distance_to_ball(self, item):
        """
        Calculates the distance to a detected ball using camera model calculations.
        
        Args:
            item (Item): The detected item (ball).
            
        Returns:
            float: The distance to the ball in the world frame.
        """
        diameter_pixels = item.diameter
        scale_factor = ACTUAL_DIAMETER / diameter_pixels
        focal_length = self.camera_model.fx()
        
        Z_camera = focal_length * scale_factor
        Z_world = Z_camera + CAMERA_POS_GAZEBO[2]
        return Z_world
    
    def approach_ball(self, item, Z_world):
        """
        Commands the robot to approach a detected ball, adjusting for obstacles detected via lidar.
        
        Args:
            item (Item): The detected item (ball).
            Z_world (float): The distance to the ball in the world frame.
        """
        msg = Twist()
        
        x_smoothness = Z_world
        y_smoothness = 1
        
        if Z_world < DEVIATION_SMOOTHNESS_THRESHOLD:
            x_smoothness = MIN_X_SMOOTHNESS
            y_smoothness = Z_world
        
        
        angular_z_correction = 0.0
        # linear_x_correction = 1.0
        # scaling_factor = 1.0
        
        try:
            if self.scan[SCAN_FRONT_LEFT] < DEVIATION_THRESHOLD:
                angular_z_correction += (TURN_RIGHT * ANGULAR_VELOCITY * ANGULAR_DISTANCE_PROPRTIONAL * y_smoothness) / self.scan[SCAN_FRONT_LEFT]
                # distance_ratio_left = max(0, (DEVIATION_THRESHOLD - self.scan[SCAN_FRONT_LEFT]) / DEVIATION_THRESHOLD)
                # linear_x_correction = min(linear_x_correction, np.exp(-scaling_factor * distance_ratio_left))
            if self.scan[SCAN_FRONT_RIGHT] < DEVIATION_THRESHOLD:
                angular_z_correction += (TURN_LEFT * ANGULAR_VELOCITY * ANGULAR_DISTANCE_PROPRTIONAL * y_smoothness) / self.scan[SCAN_FRONT_RIGHT]
                # distance_ratio_right = max(0, (DEVIATION_THRESHOLD - self.scan[SCAN_FRONT_RIGHT]) / DEVIATION_THRESHOLD)
                # linear_x_correction = min(linear_x_correction, np.exp(-scaling_factor * distance_ratio_right))
        except ZeroDivisionError:
            pass
        
        if angular_z_correction != 0.0:
            self.logger.info(f"Deviation Angular Z Correction: {angular_z_correction:.3f}")
        
        # if linear_x_correction != 1.0:
        #     self.logger.info(f"Deviation Linear Speed Correction: {linear_x_correction:.3f}")
            
        msg.linear.x = LINEAR_VELOCITY * LINEAR_DISTANCE_PROPRTIONAL * x_smoothness
        msg.angular.z = (item.x / 320.0) + angular_z_correction

        self.publish_cmd_vel(msg.linear.x, msg.angular.z)
        # self.cmd_vel_publisher.publish(msg)
        
    def stop_moving(self):
        """
        Commands the robot to stop all linear and angular movements.
        """
        msg = Twist()
        self.publish_cmd_vel(msg.linear.x, msg.angular.z)
        # self.cmd_vel_publisher.publish(msg)
        
    def go_to_pose(self, pos):
        """
        Commands the robot to navigate to a specified pose.
        
        Args:
            pos (Pose): The target pose to navigate to.
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose = pos
        
        self.navigator.goToPose(goal_pose)
        
    def check_navigation_status(self):
        """
        Checks the status of the current navigation task and makes decisions based on the robot's current state.
        
        Returns:
            bool: True if the navigation task is complete, False otherwise.
        """
        try:
            msg = Twist()
            
            if self.scan[SCAN_FRONT_LEFT] < AVOIDANCE_THRESHOLD:
                msg.angular.z += (TURN_RIGHT * ANGULAR_VELOCITY * ANGULAR_DISTANCE_PROPRTIONAL) / self.scan[SCAN_FRONT_LEFT]
                # distance_ratio_left = max(0, (DEVIATION_THRESHOLD - self.scan[SCAN_FRONT_LEFT]) / DEVIATION_THRESHOLD)
                # linear_x_correction = min(linear_x_correction, np.exp(-scaling_factor * distance_ratio_left))
            if self.scan[SCAN_FRONT_RIGHT] < AVOIDANCE_THRESHOLD:
                msg.angular.z += (TURN_LEFT * ANGULAR_VELOCITY * ANGULAR_DISTANCE_PROPRTIONAL) / self.scan[SCAN_FRONT_RIGHT]
                # distance_ratio_right = max(0, (DEVIATION_THRESHOLD - self.scan[SCAN_FRONT_RIGHT]) / DEVIATION_THRESHOLD)
                # linear_x_correction = min(linear_x_correction, np.exp(-scaling_factor * distance_ratio_right))
            if msg.angular.z != 0.0:
                x_smoothness = MIN_X_SMOOTHNESS
                msg.linear.x = LINEAR_VELOCITY * LINEAR_DISTANCE_PROPRTIONAL * x_smoothness
                self.logger.info(f"Avoidance Angular Z Correction: {msg.angular.z:.3f} and Linear Speed Correction: {x_smoothness:.3f}")
                self.publish_cmd_vel(msg.linear.x, msg.angular.z)
        except ZeroDivisionError:
            pass
        
        return self.navigator.isTaskComplete()

    def destroy_node(self):
        """
        Clean up resources and stop the robot before shutting down the node.
        """
        self.navigator.cancelTask()
        self.stop_moving()
        super().destroy_node()

def main(args=None):
    """
    Main function for running the AutonomousNavigation node. Initializes the node, sets up a multi-threaded executor, and handles clean shutdown.
    """
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


if __name__ == '__main__':
    main()