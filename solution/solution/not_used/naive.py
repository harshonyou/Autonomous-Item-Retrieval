from enum import Enum
import math
import random
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles

from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
# from auro_interfaces.msg import StringWithPose, Item, ItemList
from assessment_interfaces.msg import ItemList, Item, HomeZone, ItemHolders
from auro_interfaces.msg import StringWithPose

# from tf_transformations import euler_from_quaternion
import angles

LINEAR_VELOCITY  = 0.3 # Metres per second
ANGULAR_VELOCITY = 0.5 # Radians per second
DISTANCE_PROPRTIONAL = 0.5

TURN_LEFT = 1 # Postive angular velocity turns left
TURN_RIGHT = -1 # Negative angular velocity turns right


SCAN_THRESHOLD = 0.5 # Metres per second
 # Array indexes for sensor sectors
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

# Finite state machine (FSM) states
class State(Enum):
    FORWARD = 0
    TURNING = 1
    COLLECTING = 2
    GRABING = 3
    SEARCHING = 4
    HOMING = 5


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

# Missing features:
# Prone to drift
# No priority for items

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        
        # Class variables used to store persistent values between executions of callbacks and control loop
        self.state = State.FORWARD # Current FSM state
        self.pose = Pose() # Current pose (position and orientation), relative to the odom reference frame
        self.previous_pose = Pose() # Store a snapshot of the pose for comparison against future poses
        self.yaw = 0.0 # Angle the robot is facing (rotation around the Z axis, in radians), relative to the odom reference frame
        self.previous_yaw = 0.0 # Snapshot of the angle for comparison against future angles
        self.turn_angle = 0.0 # Relative angle to turn to in the TURNING state
        self.turn_direction = TURN_LEFT # Direction to turn in the TURNING state
        self.goal_distance = random.uniform(1.0, 2.0) # Goal distance to travel in FORWARD state
        self.scan_triggered = [False] * 4 # Boolean value for each of the 4 LiDAR sensor sectors. True if obstacle detected within SCAN_THRESHOLD
        self.items = ItemList()
        self.home = HomeZone()
        self.target_position = (0.0, 0.0)
        self.arrived = False
        self.hold = ItemHolders()
        
        self.item_subscriber = self.create_subscription(
            ItemList,
            '/robot1/items',
            self.item_callback,
            10
        )
        
        self.home_subscriber = self.create_subscription(
            HomeZone,
            '/robot1/home_zone',
            self.home_callback,
            10
        )
        
        self.hold_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.hold_callback,
            10
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_callback,
            10)
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        
        self.marker_publisher = self.create_publisher(StringWithPose, '/robot1/marker_input', 10)

        self.items:ItemList = ItemList()
        

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def item_callback(self, msg):
        self.items = msg
    
    def home_callback(self, msg):
        self.home = msg
    
    def hold_callback(self, msg):
        self.hold = msg
       
    def odom_callback(self, msg):
        self.pose = msg.pose.pose # Store the pose in a class variable

        # Uses tf_transformations package to convert orientation from quaternion to Euler angles (RPY = roll, pitch, yaw)
        # https://github.com/DLu/tf_transformations
        #
        # Roll (rotation around X axis) and pitch (rotation around Y axis) are discarded
        (roll, pitch, yaw) = get_euler_from_quaternion(self.pose.orientation)
        
        self.yaw = yaw # Store the yaw in a class variable

    def scan_callback(self, msg):
        # Group scan ranges into 4 segments
        # Front, left, and right segments are each 60 degrees
        # Back segment is 180 degrees
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30] # 30 to 331 degrees (30 to -30 degrees)
        left_ranges  = msg.ranges[31:90] # 31 to 90 degrees (31 to 90 degrees)
        back_ranges  = msg.ranges[91:270] # 91 to 270 degrees (91 to -90 degrees)
        right_ranges = msg.ranges[271:330] # 271 to 330 degrees (-30 to -91 degrees)

        # Store True/False values for each sensor segment, based on whether the nearest detected obstacle is closer than SCAN_THRESHOLD
        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD


    def control_loop(self):
        # self.get_logger().info(f"{self.state}")
        marker_input = StringWithPose()
        marker_input.text = str(self.state) # Visualise robot state as an RViz marker
        marker_input.pose = self.pose # Set the pose of the RViz marker to track the robot's pose
        self.marker_publisher.publish(marker_input)
        
        # self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")
        # self.get_logger().info(f"Current pose - x: {self.pose.position.x}, y: {self.pose.position.y}, yaw: {self.yaw}")
        
        match self.state:

            case State.FORWARD:

                if self.scan_triggered[SCAN_FRONT]:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(150, 170)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("Detected obstacle in front, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    return
                
                if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = 45

                    if self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT]:
                        self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                        self.get_logger().info("Detected obstacle to both the left and right, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    elif self.scan_triggered[SCAN_LEFT]:
                        self.turn_direction = TURN_RIGHT
                        self.get_logger().info(f"Detected obstacle to the left, turning right by {self.turn_angle} degrees")
                    else: # self.scan_triggered[SCAN_RIGHT]
                        self.turn_direction = TURN_LEFT
                        self.get_logger().info(f"Detected obstacle to the right, turning left by {self.turn_angle} degrees")
                    return
                
                if self.home.visible and len(self.hold.data) > 0 and self.hold.data[0].holding_item:
                    self.state = State.HOMING
                    return
                
                if len(self.items.data) > 0:
                    self.state = State.COLLECTING
                    return

                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                difference_x = self.pose.position.x - self.previous_pose.position.x
                difference_y = self.pose.position.y - self.previous_pose.position.y
                distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

                # self.get_logger().info(f"Driven {distance_travelled:.2f} out of {self.goal_distance:.2f} metres")

                if distance_travelled >= self.goal_distance:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(30, 150)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("Goal reached, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")

            case State.TURNING:

                if len(self.items.data) > 0:
                    self.state = State.COLLECTING
                    return

                msg = Twist()
                msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                # self.get_logger().info(f"Turned {math.degrees(math.fabs(yaw_difference)):.2f} out of {self.turn_angle:.2f} degrees")

                yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)                

                if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
                    self.previous_pose = self.pose
                    self.goal_distance = random.uniform(1.0, 2.0)
                    self.state = State.FORWARD
                    self.get_logger().info(f"Finished turning, driving forward by {self.goal_distance:.2f} metres")

            case State.COLLECTING:

                if len(self.items.data) == 0:
                    self.previous_pose = self.pose
                    self.state = State.FORWARD
                    return
                
                # item  = max(self.items.data, key=lambda item: (item.value, item.diameter))
                # def custom_key(item):
                #     return item.value / item.diameter
                
                # item = max(self.items.data, key=custom_key)
                w_value = 1.0
                w_diameter = 0.5
                def custom_key(item):
                    return w_value * item.value / 15.0 + w_diameter * item.diameter / 640.0
                
                item = max(self.items.data, key=custom_key)

                estimated_distance = 69.0 * float(item.diameter) ** -0.89
                
                self.get_logger().info(f"{estimated_distance}")
                # .3 if less than that then we start grabbing procedure
                if estimated_distance < 0.3:
                    self.previous_pose = self.pose
                    self.goal_distance = 0.15
                    self.state = State.GRABING
                    return
                

                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL * estimated_distance
                msg.angular.z = item.x / 320.0

                self.cmd_vel_publisher.publish(msg)

            case State.GRABING:
                self.get_logger().info(f"GRABING")
                
                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL
                self.cmd_vel_publisher.publish(msg)

                difference_x = self.pose.position.x - self.previous_pose.position.x
                difference_y = self.pose.position.y - self.previous_pose.position.y
                distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

                if distance_travelled >= self.goal_distance:
                    self.previous_pose = self.pose
                    self.state = State.HOMING
                
                return

            case State.HOMING:
                # self.get_logger().info(f"HOMING")
                
                if not self.hold.data[0].holding_item:
                    self.goal_distance = random.uniform(1.0, 2.0)
                    self.state = State.FORWARD
                    return
                
                if self.arrived:
                    self.arrived = False
                    self.previous_pose = self.pose
                    self.goal_distance = random.uniform(1.0, 2.0)
                    self.state = State.FORWARD
                    return
                
                # dx = self.initial_x - self.pose.position.x
                # dy = self.initial_y - self.pose.position.y
                
                # distance = math.sqrt(dx**2 + dy**2)
                
                # target_angle = math.atan2(dy, dx)
                # angle_diff = angles.normalize_angle(target_angle - self.yaw)

                # cmd_vel = Twist()

                # if abs(angle_diff) > 0.1:  
                #     cmd_vel.angular.z = ANGULAR_VELOCITY * 0.5 * angle_diff
                # elif distance > 0.1:  
                #     cmd_vel.linear.x = LINEAR_VELOCITY * DISTANCE_PROPRTIONAL * distance
                # else:
                #     self.arrived = True
                #     self.get_logger().info('Arrived at target!')

                # self.cmd_vel_publisher.publish(cmd_vel)
                
                dx = self.initial_x - self.pose.position.x
                dy = self.initial_y - self.pose.position.y
                
                distance = math.sqrt(dx**2 + dy**2)
                
                if distance < 0.1:
                    self.arrived = True
                    self.get_logger().info('Arrived at target!')
                
                target_angle = math.atan2(dy, dx)
                angle_diff = angles.normalize_angle(target_angle - self.yaw)

                cmd_vel = Twist()
                
                val = 0.25
                
                if abs(angle_diff) > math.pi / 2:
                    val = 1.0
                elif abs(angle_diff) > math.pi / 4:
                    val = 0.5
                elif abs(angle_diff) > math.pi / 8:
                    val = 0.25
                elif abs(angle_diff) > math.pi / 12:
                    val = 0.125
                else:
                    val = 0.0625
                    
                
                if angle_diff > 0:
                    val *= 1
                else:
                    val *= -1
                
                
                if abs(angle_diff) > 0.1:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = val
                else:
                    cmd_vel.linear.x = 0.5
                    cmd_vel.angular.z = 0.0
                
                self.get_logger().info(f"Distance: {distance}; Angle: {angle_diff}; Val: {cmd_vel.angular.z}")

                self.cmd_vel_publisher.publish(cmd_vel)
                
                return

            case _:
                pass
        


    # def control_loop(self):
        # self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")
        # if len(self.items.data) == 0:
        #     msg = Twist()
        #     msg.linear.x = 0.25 
        #     self.cmd_vel_publisher.publish(msg)
        #     return
                
        # self.get_logger().info(f"{self.items.data}")
        
        # closest_item = max(self.items.data, key=lambda item: item.diameter)
        
        # self.get_logger().info(f"{closest_item.diameter}")        

        # estimated_distance = 69.0 * float(closest_item.diameter) ** -0.89
        
        # x_offset = closest_item.x / 320.0

        # msg = Twist()
        # msg.linear.x = 0.25 
        # msg.angular.z = x_offset

        # self.cmd_vel_publisher.publish(msg)


    def destroy_node(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Stopping: {msg}")
        super().destroy_node()


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