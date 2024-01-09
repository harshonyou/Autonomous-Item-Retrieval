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


from tf_transformations import euler_from_quaternion

import angles

# Default Pose Orientation
DEFAULT_POSE_ORIENTATION_Z = 0.0
DEFAULT_POSE_ORIENTATION_W = 0.99

# Turning
TURN_LEFT = -1
TURN_RIGHT = 1

# Robot Movement
LINEAR_VELOCITY = 0.11 # 0.3
ANGULAR_VELOCITY = 0.75 # 0.5
DISTANCE_PROPRTIONAL = 0.5

# LiDAR Scan Segments
SCAN_THRESHOLD = 0.5
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

# Different potential states for the robot
class State(Enum):
    SCOUTING = 0
    COLLECTING = 1
    GRABBING = 2
    HOMING = 3
    MOVING_TO_GOAL = 4
    TURNING = 5
    FORWARD = 6
    
# Camera Properties
H_FOV = 1.085595
CAMERA_POS_GAZEBO = (0.076, 0.0, 0.093)
ACTUAL_RADIUS = 0.075
ACTUAL_DIAMETER = ACTUAL_RADIUS * 2

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller', namespace='robot1', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Initial Pose
        self.declare_parameter('x', -3.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        
        # Logger
        self.logger = self.get_logger()
        
        # Camera Model
        self.camera_model:PinholeCameraModel = camera_model()
        
        # NAV2
        self.navigator = BasicNavigator(namespace='robot1')
        self.set_initial_pose()
        self.fn = lambda: False
        
        # Odom
        self.previous_pose = Pose()
        self.previous_yaw = 0.0
        self.pose = Pose()
        self.yaw = 0.0
        
        # FOV Items
        self.fov_items = ItemList()
        
        # LiDAR
        self.scan_triggered = [False] * 4
        
        # Scouting
        self.scout_ts = self.get_clock().now()
        self.scout_direction = TURN_LEFT
        
        # Grabbed Item
        self.grabbed_item = ItemHolder()
        
        # Obstacle Avoidance
        self.turn_angle = 0.0
        self.turn_direction = TURN_LEFT
        self.enroute_home = False
        

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
        
        self.garbbed_item_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.garbbed_item_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.camdar_publisher = self.create_publisher(LaserScan, 'updated_scan', 10)
        
        self.set_state(State.SCOUTING)
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.controller_timer = self.create_timer(self.timer_period, self.control_loop)
        # self.camdar_timer = self.create_timer(self.timer_period, self.camdar_loop)
    
    def set_state(self, state):
        match state:
            case State.SCOUTING:
                self.state = State.SCOUTING
                self.scout_direction = TURN_LEFT if random.random() > 0.5 else TURN_RIGHT
    
    def odom_callback(self, msg):
        self.pose = msg.pose.pose

        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        
        self.yaw = yaw
    
    def fov_items_callback(self, msg):            
        self.fov_items = msg
    
    def garbbed_item_callback(self, msg):
        for item_holder in msg.data:
            if item_holder.robot_id == "robot1":
                self.grabbed_item = item_holder
                break

    
    def lidar_callback(self, msg):
        left_ranges  = msg.ranges[314:359]
        right_ranges = msg.ranges[0:45]
        
        threshold = SCAN_THRESHOLD
        if self.enroute_home:
            threshold = 0.275
        
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
        
    def go_home(self):
        self.enroute_home = True
        self.fn = lambda: not self.grabbed_item.holding_item
        self.go_to_pose(self.initial_x, self.initial_y)
        
    def go_to_pose(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = DEFAULT_POSE_ORIENTATION_Z
        goal_pose.pose.orientation.w = DEFAULT_POSE_ORIENTATION_W
        
        self.navigator.goToPose(goal_pose)
        self.state = State.MOVING_TO_GOAL
        

    def scout(self):
        # if self.get_clock().now() - self.scout_ts > 10:
            # self.state = State.HOMING
        
        if len(self.fov_items.data) != 0:
            self.state = State.COLLECTING
        
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = ANGULAR_VELOCITY * self.scout_direction
        self.cmd_vel_publisher.publish(cmd)
    
    def collect(self):
        if len(self.fov_items.data) == 0:
            # self.previous_pose = self.pose
            # self.state = State.FORWARD
            self.state = State.HOMING
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
            self.previous_pose = self.pose
            self.goal_distance = 0.15
            self.state = State.GRABBING
            return

        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL * Z_world
        msg.angular.z = item.x / 320.0

        self.cmd_vel_publisher.publish(msg)
    
    def grab(self):
        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL
        self.cmd_vel_publisher.publish(msg)

        difference_x = self.pose.position.x - self.previous_pose.position.x
        difference_y = self.pose.position.y - self.previous_pose.position.y
        distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

        if distance_travelled >= self.goal_distance: #check if the bot have got item
            # self.previous_pose = self.pose
            if not self.grabbed_item.holding_item:
                self.set_state(State.SCOUTING)
            self.state = State.HOMING
        
    def obstacle_detection(self):
        if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
            self.logger.info(f"Enroute? {self.enroute_home} {self.state}")
            self.cmd_vel_publisher.publish(Twist())
            
            self.previous_yaw = self.yaw
            self.state = State.TURNING
            self.turn_angle = 5 # 5

            if self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT]:
                self.turn_angle = 135
                self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                self.get_logger().info("Detected obstacle to both the left and right, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
            elif self.scan_triggered[SCAN_LEFT]:
                self.turn_direction = TURN_RIGHT
                self.get_logger().info(f"Detected obstacle to the left, turning right by {self.turn_angle} degrees")
            else: # self.scan_triggered[SCAN_RIGHT]
                self.turn_direction = TURN_LEFT
                self.get_logger().info(f"Detected obstacle to the right, turning left by {self.turn_angle} degrees")
            return True
    
    def turn(self):
        if len(self.fov_items.data) != 0:
            self.state = State.COLLECTING
        
        if self.enroute_home:
            self.state = State.HOMING
        
        msg = Twist()
        msg.angular.z = self.turn_direction * ANGULAR_VELOCITY * DISTANCE_PROPRTIONAL
        self.cmd_vel_publisher.publish(msg)    
        
        self.logger.info(f"Turning {self.turn_direction * self.turn_angle:.2f} degrees")
        
        self.goal_distance = random.uniform(0.075, 0.125) # random.uniform(0.125, 0.175)
        self.state = State.FORWARD
        
        # yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)                

        # if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
        #     self.previous_pose = self.pose
        #     self.goal_distance = random.uniform(0.3, 0.9)
        #     self.state = State.FORWARD
        #     self.get_logger().info(f"Finished turning, driving forward by {self.goal_distance:.2f} metres")

    def forward(self):
        # if len(self.fov_items.data) != 0:
        #     self.state = State.COLLECTING
        
        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL
        self.cmd_vel_publisher.publish(msg)
        
        self.logger.info(f"Driving forward by {self.goal_distance:.2f} metres")

        difference_x = self.pose.position.x - self.previous_pose.position.x
        difference_y = self.pose.position.y - self.previous_pose.position.y
        distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)
        
        if distance_travelled >= self.goal_distance:
            self.logger.info(f"Finished driving forward by {self.goal_distance:.2f} metres")
            self.set_state(State.SCOUTING)
            if self.enroute_home:
                self.state = State.HOMING

    
    def control_loop(self):
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
            case State.HOMING:
                self.go_home()
            case State.MOVING_TO_GOAL:  # New case for MOVING_TO_GOAL state
                if self.navigator.isTaskComplete():
                    self.enroute_home = False
                    result = self.navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.logger.info('Reached the goal successfully.')
                        self.set_state(State.SCOUTING)
                    elif result == TaskResult.CANCELED:
                        self.logger.info('Goal was canceled.')
                        self.set_state(State.SCOUTING)
                    elif result == TaskResult.FAILED:
                        self.logger.info('Goal failed.')
                        self.set_state(State.SCOUTING)
                    else:
                        self.logger.info('Goal has an invalid return status.')
                        self.set_state(State.SCOUTING)
                else:
                    if self.fn():
                        self.navigator.cancelTask()
                        self.logger.info(f"Fn returned true, cancelling task.")
                    elif self.obstacle_detection():
                        self.navigator.cancelTask()
                        self.previous_state = State.HOMING
                    # self.logger.info(f"Moving to goal, current state: {self.state}")
            case State.TURNING:
                self.turn()
            case State.FORWARD:
                self.forward()
                if self.obstacle_detection():
                    return
    
    # def camdar_loop(self):
    #     pass
        

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

    node = RobotController()

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