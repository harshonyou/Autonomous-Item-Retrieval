import math
import sys
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Pose, Twist
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from assessment_interfaces.msg import ItemList, Item, HomeZone, ItemHolders, ItemHolder
from rclpy.qos import QoSPresetProfiles
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion


DEFAULT_POSE_ORIENTATION_Z = 0.0
DEFAULT_POSE_ORIENTATION_W = 0.99

LINEAR_VELOCITY = 0.3
ANGULAR_VELOCITY = 0.5
DISTANCE_PROPRTIONAL = 0.5

SCAN_THRESHOLD = 0.5
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

class State(Enum):
    SCOUTING = 0
    COLLECTING = 1
    GRABBING = 2
    HOMING = 3
    MOVING_TO_GOAL = 4

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
        
        # Grabbed Item
        self.grabbed_item = ItemHolder()

        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_callback,
            10)
        
        self.lidiar_subscriber = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.lidar_callback,
            QoSPresetProfiles.SENSOR_DATA.value)
        
        self.fov_items_subscriber = self.create_subscription(
            ItemList,
            '/robot1/items',
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
        self.cmd_vel_publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        
        self.state = State.SCOUTING
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
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
        self.logger.info(f"{msg.data}, {self.grabbed_item}")
        for item_holder in msg.data:
            if item_holder.robot_id == "robot1":
                self.grabbed_item = item_holder
                break

    
    def lidar_callback(self, msg):
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30] # 30 to 331 degrees (30 to -30 degrees)
        left_ranges  = msg.ranges[31:90] # 31 to 90 degrees (31 to 90 degrees)
        back_ranges  = msg.ranges[91:270] # 91 to 270 degrees (91 to -90 degrees)
        right_ranges = msg.ranges[271:330] # 271 to 330 degrees (-30 to -91 degrees)

        # Store True/False values for each sensor segment, based on whether the nearest detected obstacle is closer than SCAN_THRESHOLD
        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD
    
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
        cmd.angular.z = ANGULAR_VELOCITY
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

        estimated_distance = 69.0 * float(item.diameter) ** -0.89
        
        if estimated_distance < 0.3:
            self.previous_pose = self.pose
            self.goal_distance = 0.15
            self.state = State.GRABBING
            return

        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL * estimated_distance
        msg.angular.z = item.x / 320.0

        self.cmd_vel_publisher.publish(msg)
    
    def grab(self):
        msg = Twist()
        msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL
        self.cmd_vel_publisher.publish(msg)

        difference_x = self.pose.position.x - self.previous_pose.position.x
        difference_y = self.pose.position.y - self.previous_pose.position.y
        distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

        if distance_travelled >= self.goal_distance:
            # self.previous_pose = self.pose
            self.state = State.HOMING
            
    def control_loop(self):
        match self.state:
            case State.SCOUTING:
                self.scout()
            case State.COLLECTING:
                self.collect()
            case State.GRABBING:
                self.grab()
            case State.HOMING:
                self.go_home()
            case State.MOVING_TO_GOAL:  # New case for MOVING_TO_GOAL state
                if self.navigator.isTaskComplete():
                    result = self.navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.logger.info('Reached the goal successfully.')
                        self.state = State.SCOUTING
                    elif result == TaskResult.CANCELED:
                        self.logger.info('Goal was canceled.')
                        self.state = State.SCOUTING
                    elif result == TaskResult.FAILED:
                        self.logger.info('Goal failed.')
                        self.state = State.SCOUTING
                    else:
                        self.logger.info('Goal has an invalid return status.')
                        self.state = State.SCOUTING
                else:
                    if self.fn():
                        self.navigator.cancelTask()
                        self.logger.info('Goal was canceled.')
                        self.state = State.SCOUTING
                    self.logger.info(f"Moving to goal, current state: {self.state}")
        

    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()
    
    executor = MultiThreadedExecutor()

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