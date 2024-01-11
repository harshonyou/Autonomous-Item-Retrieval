import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from sensor_msgs.msg import LaserScan
from assessment_interfaces.msg import ItemList, Item
import numpy as np
from solution.lidar import Lidar
from solution.pid import PidController
from tf_transformations import euler_from_quaternion

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.get_logger().info("creating publisher...")
        self.twist_publisher = self.create_publisher(
            Twist,
            '/robot1/cmd_vel',
            10
        )
        
        self.get_logger().info("creating odometry subscriber...")
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_callback,
            10
        )
        self.odom_subscription
        
        self.get_logger().info("creating laser subscriber...")
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.lidar_callback,
            10
        )
        self.lidar_subscription
        self.lidar = Lidar(self)
        
        # important variables
        self.is_moving = False
        self.pos_tolerance = 0.05
        self.goal_x = 0
        self.goal_y = 0
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.gtg_r = 0
        self.gtg_theta = 0
        self.ao_r = 0
        self.ao_theta = 0
        self.theta_desired = 0
        self.r_desired = 0

        self.sample_time = 0.1
        self.max_linear_v = 0.08 # 0.08
        self.alpha = 5
        self.blend_dist_threshold = 1.0 # 0.4 # m
        self.avoid_dist_threshold = 0.5 # 0.2
        self.avoid_angle_threshold = math.pi / 2.0
        self.angle_pid = PidController(0.3, 0.01, 0.002, self.sample_time, True)
        self.angle_pid.set_output_limits(-3, 3)

        self.get_logger().info("initialization finished")

        self.linear_command = 0
        self.angular_command = 0
        
        self.goal_callback(5, 1)
        
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
    def goal_callback(self, x, y):
        self.goal_x = x
        self.goal_y = y
        self.get_logger().info(f'Received goal request: ({self.goal_x}, {self.goal_y})')
        self.get_logger().info(f'Distance to goal: {self.get_distance_to_goal()}')

    def odom_callback(self, odom: Odometry):
        self.current_x = odom.pose.pose.position.x
        self.current_y = odom.pose.pose.position.y
        self.current_theta = euler_from_quaternion([odom.pose.pose.orientation.x,
                                                    odom.pose.pose.orientation.y,
                                                    odom.pose.pose.orientation.z,
                                                    odom.pose.pose.orientation.w])[2]
        self.gtg_r, self.gtg_theta = self.get_go_to_goal_vector()
        self.ao_r, self.ao_theta = self.get_obstacle_vector()
    
    def lidar_callback(self, scan: LaserScan):
        self.lidar.update(scan)

    def get_go_to_goal_vector(self):
        error_x = self.goal_x - self.current_x
        error_y = self.goal_y - self.current_y
        distance_to_goal = self.get_distance_to_goal()

        K = (self.max_linear_v * (1 - math.exp(-self.alpha * (distance_to_goal * distance_to_goal))) / (distance_to_goal * distance_to_goal)) if distance_to_goal > 0.0001 else 0

        u_x = K * error_x
        u_y = K * error_y

        r = math.sqrt((u_x * u_x) + (u_y * u_y))
        theta = math.atan2(u_y, u_x)
        return r, theta
    
    def get_distance_to_goal(self):
        return math.sqrt(math.pow(self.goal_x - self.current_x, 2) + math.pow(self.goal_y - self.current_y, 2))
    
    def get_obstacle_vector(self):
        r_obs, theta_obs = self.lidar.get_closest_obstacle()
        return r_obs, theta_obs
    
    def send_twist_command(self):
        twist_msg = Twist()
        twist_msg.linear.x = float(self.linear_command)
        twist_msg.angular.z = float(self.angular_command)
        # self.get_logger().info(f'Publishing twist: {twist_msg}')
        self.twist_publisher.publish(twist_msg)
    
    def control_loop(self):
        
        # self.gtg_r, self.gtg_theta = self.get_go_to_goal_vector()
        # self.ao_r, self.ao_theta = self.get_obstacle_vector()
        
        self.get_logger().info(f'Go to goal vector: ({self.gtg_r}, {self.gtg_theta}); Obstacle vector: ({self.ao_r}, {self.ao_theta})')
        
        self.r_desired = self.gtg_r
        delta_theta = self.gtg_theta - self.ao_theta
        delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))

        # self.get_logger().info(f'Delta theta: {delta_theta}')
        
        if self.ao_r <= self.avoid_dist_threshold:
            avoid_theta = self.ao_theta + math.pi
            avoid_theta = math.atan2(math.sin(avoid_theta), math.cos(avoid_theta))
            self.theta_desired = avoid_theta

        elif self.ao_r > self.avoid_dist_threshold and self.ao_r <= self.blend_dist_threshold and abs(delta_theta) < self.avoid_angle_threshold:
            avoid_theta = self.ao_theta + (math.copysign(1, delta_theta)) * (math.pi / 2.0) 
            avoid_theta = math.atan2(math.sin(avoid_theta), math.cos(avoid_theta))
            self.theta_desired = self.gtg_theta + math.atan2(self.ao_r * math.sin(self.ao_theta - self.gtg_theta), self.gtg_r + self.ao_r * math.cos(self.ao_theta - self.gtg_theta))
        
        else:
            self.theta_desired = self.gtg_theta

        if self.get_distance_to_goal() <= self.pos_tolerance:
            self.linear_command = 0
            self.angular_command = 0
            self.send_twist_command()
            self.get_logger().info(f"Already near goal, distance: {self.get_distance_to_goal()}")
            return True     

        self.angle_pid.set_input(self.current_theta)
        self.angle_pid.set_setpoint(self.theta_desired)

        self.angle_pid.compute()

        self.linear_command = self.r_desired
        self.angular_command = self.angle_pid.get_output()
        
        self.send_twist_command()
        # self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")


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