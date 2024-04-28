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

import angles
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

# LiDAR Scan Segments
SCAN_THRESHOLD = 0.5
ENROUTE_HOME_SCAN_THRESHOLD = 0.45 # 0.35
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

# Backup
BACKWARD_THRESHOLD = 0.375 # 0.3

# Different potential states for the robot
class State(Enum):
    SCOUTING = 0
    COLLECTING = 1
    GRABBING = 2
    HOMING = 3
    TURNING = 4
    FORWARD = 5
    BACKWARD = 6
    
# Camera Properties
H_FOV = 1.085595
CAMERA_POS_GAZEBO = (0.076, 0.0, 0.093)
ACTUAL_RADIUS = 0.075
ACTUAL_DIAMETER = ACTUAL_RADIUS * 2

class AutonomousNavigation(Node):

    def __init__(self):
        super().__init__('autonomous_navigation')        
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
        self.navigator = BasicNavigator()
        self.set_initial_pose()
        self.log_counter = 0
        
        # Odom
        self.previous_pose = Pose()
        self.pose = Pose()
        self.yaw = 0.0
        
        # FOV Items
        self.fov_items = ItemList()
        
        # LiDAR
        self.scan_triggered = [False] * 4
        self.triggered_distance = [0.0] * 4
        
        # Scouting
        self.scout_ts = self.get_clock().now().nanoseconds / 1e9
        self.scout_direction = TURN_LEFT
        
        # Grabbed Item
        self.grabbed_item = ItemHolder()
        
        # Obstacle Avoidance
        self.turn_direction = TURN_LEFT
        self.enroute_home = False
        
        # Halt
        self.halted = False

        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.lidiar_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            QoSPresetProfiles.SENSOR_DATA.value)
        
        self.fov_items_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.fov_items_callback,
            10
        )
        
        self.halt_subscriber = self.create_subscription(
            Bool,
            'halt',
            self.halt_callback,
            10
        )
        
        self.garbbed_item_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.garbbed_item_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_publisher = self.create_publisher(StateMarker, 'raw_state_marker', 10)
        
        self.set_state(State.SCOUTING)
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.controller_timer = self.create_timer(self.timer_period, self.control_loop)
    
    def set_state(self, state):
        self.state = state
        
        match state:
            case State.SCOUTING:
                self.scout_ts = self.get_clock().now().nanoseconds / 1e9
                self.scout_direction = TURN_LEFT if random.random() > 0.5 else TURN_RIGHT
            case State.HOMING:
                self.enroute_home = True
                self.log_counter = 0
                self.go_to_pose(self.initial_x, self.initial_y)
            case State.GRABBING:
                self.previous_pose = self.pose
                self.goal_distance = 0.2
            case State.FORWARD:
                self.previous_pose = self.pose
            case State.BACKWARD:
                self.previous_pose = self.pose
                self.log_counter = 0
                self.navigator.backup(backup_dist=0.15, backup_speed=0.025, time_allowance=10)
    
    def odom_callback(self, msg):
        self.pose = msg.pose.pose

        (roll, pitch, yaw) = get_euler_from_quaternion(self.pose.orientation)
        
        self.yaw = yaw
    
    def fov_items_callback(self, msg):            
        self.fov_items = msg
        self.fov_items.data = [item for item in self.fov_items.data if item.y >= 2 and item.y <= 4]
    
    def garbbed_item_callback(self, msg):
        for item_holder in msg.data:
            if item_holder.robot_id == self.robot_name:
                self.grabbed_item = item_holder
                break
            
    def halt_callback(self, msg):
        if msg.data:
            self.halted = True
        else:
            self.halted = False
    
    def lidar_callback(self, msg):
        left_ranges  = msg.ranges[314:359]
        right_ranges = msg.ranges[0:45]
        
        threshold = SCAN_THRESHOLD
        if self.enroute_home:
            threshold = ENROUTE_HOME_SCAN_THRESHOLD
            
        self.triggered_distance[SCAN_LEFT] = min(left_ranges)
        self.triggered_distance[SCAN_RIGHT] = min(right_ranges)
        
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < threshold
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < threshold
    
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
        
    def go_to_pose(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = DEFAULT_POSE_ORIENTATION_Z
        goal_pose.pose.orientation.w = DEFAULT_POSE_ORIENTATION_W
        
        self.navigator.goToPose(goal_pose)

    def scout(self):
        if (self.get_clock().now().nanoseconds / 1e9) - self.scout_ts > 25:
            self.logger.info("Scouting for too long, returning to home")
            self.set_state(State.HOMING)
            return
        
        if len(self.fov_items.data) != 0:
            self.state = State.COLLECTING
        
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = ANGULAR_VELOCITY * self.scout_direction
        self.cmd_vel_publisher.publish(cmd)
    
    def collect(self):
        if len(self.fov_items.data) == 0:
            self.logger.info("No items in FOV, returning to scouting state")
            self.set_state(State.SCOUTING)
            return

        w_value = 1.0
        w_diameter = 0.5
        def custom_key(item):
            return w_value * item.value / 15.0 + w_diameter * item.diameter / 640.0
        
        item = max(self.fov_items.data, key=custom_key)
        
        diameter_pixels = item.diameter
        scale_factor = ACTUAL_DIAMETER / diameter_pixels
        focal_length = self.camera_model.fx()
        
        Z_camera = focal_length * scale_factor
        Z_world = Z_camera + CAMERA_POS_GAZEBO[2]
        
        if Z_world < 0.3:
            self.logger.info("Item is in the range of the robot, initiating grabbing sequence")
            self.set_state(State.GRABBING)
            return

        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL * Z_world
            
        # Calculate angular z based on item position and adjust for obstacles
        angular_z_correction = 0.0
        if self.triggered_distance[SCAN_LEFT] < 1:
            angular_z_correction = TURN_RIGHT * ANGULAR_VELOCITY * DISTANCE_PROPRTIONAL * Z_world
        if self.triggered_distance[SCAN_RIGHT] < 1:
            angular_z_correction += TURN_LEFT * ANGULAR_VELOCITY * DISTANCE_PROPRTIONAL * Z_world

        msg.angular.z = (item.x / 320.0) + angular_z_correction

        self.cmd_vel_publisher.publish(msg)
    
    def grab(self):
        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL
        self.cmd_vel_publisher.publish(msg)

        difference_x = self.pose.position.x - self.previous_pose.position.x
        difference_y = self.pose.position.y - self.previous_pose.position.y
        distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

        if distance_travelled >= self.goal_distance: 
            self.logger.info(f"Grabbed item, drived forward by {self.goal_distance:.2f} metres")
            if not self.grabbed_item.holding_item:
                self.logger.info(f"Robot {self.get_namespace()} has lost its item, returning to scouting state")
                self.set_state(State.SCOUTING)
            self.logger.info(f"Robot {self.get_namespace()} has grabbed its item, returning to home")
            self.set_state(State.HOMING) # State.HOMING
        
    def obstacle_detection(self):
        if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
            self.cmd_vel_publisher.publish(Twist())
            
            self.logger.info(f"Obstacle detected: {self.triggered_distance[SCAN_LEFT]} and {self.triggered_distance[SCAN_RIGHT]}")
            
            self.set_state(State.TURNING)

            if self.triggered_distance[SCAN_LEFT] < BACKWARD_THRESHOLD or self.triggered_distance[SCAN_RIGHT] < BACKWARD_THRESHOLD:
                self.logger.info("Backing up as obstacle is too close")
                self.set_state(State.BACKWARD)
            elif self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT]:
                self.logger.info("Dead end, backing up")
                self.set_state(State.BACKWARD)
            elif self.scan_triggered[SCAN_LEFT]:
                self.logger.info("Obstacle detected on the left, turning right")
                self.turn_direction = TURN_RIGHT
            else: # self.scan_triggered[SCAN_RIGHT]
                self.logger.info("Obstacle detected on the right, turning left")
                self.turn_direction = TURN_LEFT
            return True
        
        return False
    
    def turn(self):
        msg = Twist()
        msg.angular.z = self.turn_direction * ANGULAR_VELOCITY * DISTANCE_PROPRTIONAL
        self.cmd_vel_publisher.publish(msg)    
        
        self.goal_distance = random.uniform(0.075, 0.125)
        self.set_state(State.FORWARD)

    def forward(self):
        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL
        self.cmd_vel_publisher.publish(msg)
        
        difference_x = self.pose.position.x - self.previous_pose.position.x
        difference_y = self.pose.position.y - self.previous_pose.position.y
        distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)
        
        if distance_travelled >= self.goal_distance:
            self.logger.info(f"Finished driving forward by {self.goal_distance:.2f} metres")
            self.set_state(State.SCOUTING)
            if self.enroute_home:
                self.set_state(State.HOMING)

    def control_loop(self):
        self.marker_publisher.publish(self.process_marker())
        
        if self.halted:
            self.cmd_vel_publisher.publish(Twist())
            return
        
        match self.state:
            case State.SCOUTING:
                # self.obstacle_detection()
                self.scout()
            case State.COLLECTING:
                if self.obstacle_detection():
                    return
                self.collect()
            case State.GRABBING:
                self.grab()
            case State.HOMING:  # New case for MOVING_TO_GOAL state
                if self.navigator.isTaskComplete():
                    self.enroute_home = False
                    self.set_state(State.SCOUTING)
                    
                    result = self.navigator.getResult()
                    match result:
                        case TaskResult.SUCCEEDED:
                            self.logger.info('Reached home zone successfully!')
                        case TaskResult.CANCELED:
                            self.logger.info(f'Enroute home canceled!')
                        case TaskResult.FAILED:
                            self.logger.info(f'Enroute home failed!')
                        case _:
                            self.get_logger().info("Enroute home unknown result {result}!")
                else:
                    if not self.grabbed_item.holding_item:
                        self.navigator.cancelTask()
                        self.logger.info(f'Robot {self.get_namespace()} has lost its item, returning to scouting state')
                    elif self.obstacle_detection():
                        self.navigator.cancelTask()
                    
                    self.log_counter += 1
                    feedback = self.navigator.getFeedback()
                    if feedback and self.log_counter % 10 == 0:
                        self.logger.info(f"ETA: {(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9):.2f} seconds")
            case State.BACKWARD:
                if not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if feedback and self.log_counter % 10 == 0:
                        self.get_logger().info(f"Distance travelled: {feedback.distance_traveled:.2f} metres")
                else:
                    if self.previous_pose != self.pose:
                        self.set_state(self.previous_pose)
                    else:
                        self.set_state(State.SCOUTING)
                    
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
            case State.TURNING:
                self.turn()
            case State.FORWARD:
                self.forward()
                if self.obstacle_detection():
                    return
    
    def process_marker(self):
        marker = StateMarker()
        marker.text = str(self.state)
        marker.color.a = 1.0
        marker.pose = self.pose
        
        match self.state:
            case State.SCOUTING:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 255/255.0
            case State.COLLECTING, State.GRABBING:
                marker.color.r = 0.0
                marker.color.g = 255/255.0
                marker.color.b = 0.0
            case State.HOMING:
                marker.color.r = 167/255.0
                marker.color.g = 21/255.0
                marker.color.b = 252/255.0
            case State.TURNING, State.FORWARD:
                marker.color.r = 252/255.0
                marker.color.g = 159/255.0
                marker.color.b = 21/255.0
            case _:
                marker.color.r = 21/255.0
                marker.color.g = 252/255.0
                marker.color.b = 244/255.0
        
        if self.halted:
            marker.text = "Halted"
            marker.color.r = 255/255.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif self.enroute_home:
            marker.text = "Enroute-Home"
            
        return marker
        

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