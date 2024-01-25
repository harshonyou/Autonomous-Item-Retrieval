# enroute pauses when backing up
import math
import random
import sys
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
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


from solution_interfaces.msg import StateMarker

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector
from py_trees.trees import BehaviourTree
from py_trees import logging as log_tree
from py_trees.display import dot_tree, unicode_tree

# Default Pose Orientation
DEFAULT_POSE_ORIENTATION_Z = 0.0
DEFAULT_POSE_ORIENTATION_W = 0.99

# Turning
TURN_LEFT = -1
TURN_RIGHT = 1

# Robot Movement
LINEAR_VELOCITY = 0.11 # 0.3
ANGULAR_VELOCITY = 0.5 # 0.5
DISTANCE_PROPRTIONAL = 0.5

# Camera Properties
H_FOV = 1.085595
CAMERA_POS_GAZEBO = (0.076, 0.0, 0.093)
ACTUAL_RADIUS = 0.075
ACTUAL_DIAMETER = ACTUAL_RADIUS * 2

class BallFound(Behaviour):
    def __init__(self, name, robot_node):
        super(BallFound, self).__init__(name)
        self.robot_node = robot_node
        self.logger.debug(f"BallFound::setup {self.name}")

    def update(self):
        self.logger.debug(f"BallFound::update {self.name}")
        if self.robot_node.ball_in_fov():
            self.logger.info(f"{self.robot_node.fov_items.data} items found")
            return Status.SUCCESS
        else:
            return Status.FAILURE

class FindBall(Behaviour):
    def __init__(self, name, robot_node):
        super(FindBall, self).__init__(name)
        self.robot_node = robot_node
        self.logger.debug(f"FindBall::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"FindBall::initialise {self.name}")
        self.scout_direction = TURN_LEFT if random.random() > 0.5 else TURN_RIGHT

    def update(self):
        self.logger.debug(f"FindBall::update {self.name}")
        
        if self.robot_node.ball_in_fov():
            self.stop_rotating()
            return Status.SUCCESS
        
        # Continue rotating to find the ball
        self.rotate_to_find_ball()
        return Status.RUNNING

    def rotate_to_find_ball(self):
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
    def __init__(self, name, robot_node):
        super(BallClose, self).__init__(name)
        self.robot_node = robot_node
        self.logger.debug(f"BallClose::setup {self.name}")

    def update(self):
        self.logger.debug(f"BallClose::update {self.name}")

        if self.robot_node.ball_in_fov():
            return Status.FAILURE

        item = max(self.robot_node.fov_items.data, key=self.robot_node.custom_key)
        Z_world = self.robot_node.calculate_distance_to_ball(item)

        if Z_world < 0.3:  # Ball is close
            return Status.SUCCESS
        else:
            return Status.FAILURE

class ApproachBall(Behaviour):
    def __init__(self, name, robot_node):
        super(ApproachBall, self).__init__(name)
        self.robot_node = robot_node
        self.logger.debug(f"ApproachBall::setup {self.name}")

    def update(self):
        self.logger.debug(f"ApproachBall::update {self.name}")

        if not self.robot_node.ball_in_fov():
            return Status.FAILURE

        item = max(self.robot_node.fov_items.data, key=self.robot_node.custom_key)
        Z_world = self.robot_node.calculate_distance_to_ball(item)

        if Z_world < 0.3:
            # Stop moving if close enough
            self.robot_node.stop_moving()
            return Status.SUCCESS

        # Approach the ball
        self.robot_node.approach_ball(item, Z_world)
        return Status.RUNNING


class AutonomousNavigation(Node):

    def __init__(self):
        super().__init__('autonomous_navigation', namespace='robot1')        
        self.robot_name = self.get_namespace().replace('/', '').strip()

        # Initial Pose
        self.declare_parameter('x_pose', 0.0)
        self.declare_parameter('y_pose', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x_pose').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y_pose').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        
        # Logger
        self.logger = self.get_logger()
        self.logger.info(f"Initial Pose of Robot {self.robot_name}: ({self.initial_x:.2f}, {self.initial_y:.2f}), {self.initial_yaw:.2f}")
        
        # Camera Model
        self.camera_model:PinholeCameraModel = camera_model()
        
        # NAV2
        # self.navigator = BasicNavigator()
        # self.set_initial_pose()
        
        # FOV Items
        self.fov_items = ItemList()
        
        # Subscribers
        self.fov_items_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.fov_items_callback,
            10
        )
        
        log_tree.level = log_tree.Level.DEBUG
        self.tree = self.create_behavior_tree()
        dot_graph = dot_tree(self.tree.root)
        with open('tree.dot', 'w') as f:
            f.write(dot_graph.to_string())
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.controller_timer = self.create_timer(self.timer_period, self.control_loop)
        
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
        ball_found = BallFound("Ball Found", self)
        find_ball = FindBall("Find Ball", self)
        ball_close = BallClose("Ball Close", self)
        approach_ball = ApproachBall("Approach Ball", self)

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

        # Add sequences to the root
        root.add_child(find_sequence)
        root.add_child(approach_sequence)

        return BehaviourTree(root)
    
    def fov_items_callback(self, msg):
        # self.logger.info(f"FOV Items: {msg}")
        self.fov_items = msg
        self.fov_items.data = [item for item in self.fov_items.data if item.y >= 2 and item.y <= 4]
    
    def set_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.initial_x
        initial_pose.pose.position.y = self.initial_y
        initial_pose.pose.orientation.z = DEFAULT_POSE_ORIENTATION_Z
        initial_pose.pose.orientation.w = DEFAULT_POSE_ORIENTATION_W
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

    def post_tick_handler(self):
        # tree_ascii = unicode_tree(self.tree.root)
        # self.get_logger().info("\n" + tree_ascii)
        
        with open("behavior_tree.txt", "w") as file:
            tree_ascii = unicode_tree(self.tree.root)
            file.write(tree_ascii)

    def control_loop(self):
        # self.logger.info(f"Fresh running")        
        self.tree.tick()
        
        self.post_tick_handler()
        
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
        msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL * Z_world
        msg.angular.z = (item.x / 320.0)

        self.cmd_vel_publisher.publish(msg)
        
    def stop_moving(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)

    def destroy_node(self):
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



def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = AutonomousNavigation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()