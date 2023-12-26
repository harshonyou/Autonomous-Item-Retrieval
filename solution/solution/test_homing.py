import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion
import angles

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.pose = Pose()
        
        self.velocity_publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)


        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_callback,
            10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def method_1(self):
        # self.get_logger().info(f"Current pose - x: {self.pose.position.x}, y: {self.pose.position.y}, yaw: {self.pose.orientation.z}")
        # Compute the current yaw from quaternion
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        # Compute distance and angle to the target
        dx = self.initial_x - self.pose.position.x
        dy = self.initial_y - self.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)

        # Compute required angular velocity
        angle_diff = angles.normalize_angle(target_angle - yaw)
        angular_velocity = 0.5 * angle_diff
        # angular_velocity = 1 * angle_diff
        
        # Adjust the proportional factor for angular velocity based on angle difference
        # if abs(angle_diff) < math.pi / 2:
        #     angular_velocity = 0.25 * angle_diff  # Smaller factor for smaller angle difference
        # else:
        #     angular_velocity = 0.5 * angle_diff  # Larger factor for larger angle difference

        # Compute required linear velocity
        linear_velocity = 0.2 * distance if abs(angle_diff) < math.pi / 6 else 0.0
        # linear_velocity = 0.5 * distance if abs(angle_diff) < math.pi / 4 else 0.0
        


        # Thresholds for stopping
        position_threshold = 0.1  # meters
        orientation_threshold = math.radians(10)  # radians

        if distance < position_threshold and abs(angle_diff) < orientation_threshold:
            linear_velocity = 0.0
            angular_velocity = 0.0
            self.get_logger().info("Robot has returned to the initial position.")


        # Publish velocity command
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.velocity_publisher.publish(twist)

        self.get_logger().info(f"Twist - linear: {twist.linear.x}, angular: {twist.angular.z}")
        # self.get_logger().info(f"Current pose - x: {self.pose.position.x}, y: {self.pose.position.y}, yaw: {yaw}")

    def method_2(self):
        # self.get_logger().info(f"Current pose - x: {self.pose.position.x}, y: {self.pose.position.y}, yaw: {self.pose.orientation.z}")
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        dx = self.initial_x - self.pose.position.x
        dy = self.initial_y - self.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)

        angle_diff = angles.normalize_angle(target_angle - yaw)
        
        linear_velocity = angular_velocity = 0.0
        
        if abs(angle_diff) > 0.1:  # Threshold to stop rotation
            angular_velocity = 0.3 * angle_diff

        # Proportional controller for translation
        elif distance > 0.1:  # Threshold to stop translation
            linear_velocity = 0.5 * distance

        else:
            self.arrived = True
            self.get_logger().info('Arrived at target!')
        
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.velocity_publisher.publish(twist)

    def method_3(self):
        speed = Twist()
        goal = Point()
        goal.x = 1.1
        goal.y = 2.3
        
        
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, theta = euler_from_quaternion(orientation_list)
        
        inc_x = goal.x - self.pose.position.x
        inc_y = goal.y - self.pose.position.y
        angle_to_goal = math.atan2(inc_y, inc_x)
        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
       
        self.velocity_publisher.publish(speed) 
        

    def control_loop(self):
        self.method_3()

    def destroy_node(self):
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