import sys
import argparse

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import csv

from assessment_interfaces.msg import ItemLog, ItemHolders, ItemHolder

class DataLogger(Node):

    def __init__(self, args):
        super().__init__('data_logger')

        parser = argparse.ArgumentParser()
        group = parser.add_argument_group()
        group.add_argument('--path', type=str, metavar='PATH', help='Path')
        group.add_argument('--filename', type=str, metavar='FILENAME', help='Filename')
        group.add_argument('--random_seed', type=str, metavar='RANDOM_SEED', help='Random seed')
        self.args = parser.parse_args(args[1:])

        full_filepath = self.args.path + self.args.filename + '_' + self.args.random_seed + '.csv'
        self.get_logger().info(f"Logging data to file: {full_filepath}")

        self.counter = 0
        self.previous_msg = None
        self.log_file = open(full_filepath, 'w')
        self.data_file = open('turtlebot_data.csv', 'w', newline='')
        self.item_data_file = open('item_data.csv', 'w', newline='')

        self.log_file.write('timestamp,counter,')
        self.log_file.write('red_count,green_count,blue_count,total_count,')
        self.log_file.write('red_value,green_value,blue_value,total_value\n')
        self.log_file.flush()        
        
        self.data_writer = csv.writer(self.data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.data_writer.writerow(['Time', 'Robot ID', 'Linear Velocity', 'Angular Velocity', 'Linear Acceleration', 'Distance Traveled'])

        self.item_data_writer = csv.writer(self.item_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.item_data_writer.writerow(['Time', 'Robot ID', 'Item Value', 'Item Colour'])
        
        self.item_log_subscriber = self.create_subscription(
            ItemLog,
            '/item_log',
            self.item_log_callback,
            10
        )
        
        self.item_holders_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.item_holders_callback,
            10,
        )
        
        self.num_robots = 1
        
        self.robot_items = {}
        
        for i in range(1, self.num_robots + 1):
            robot_name = f'robot{i}'
            
            self.robot_items[robot_name] = ItemHolder()
        
        self.odom_subscriber = self.create_subscription(Odometry, '/robot1/odom', self.odom_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/robot1/imu', self.imu_callback, 10)
        
        self.total_distance = 0.0
        
        self.last_control_time = self.get_clock().now()
        self.velocity_sum = 0.0
        self.angular_velocity_sum = 0.0
        self.velocity_count = 0
        
        self.linear_accel_sum = 0.0
        self.linear_accel_count = 0
        
        self.timer_period = 2
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
    def odom_callback(self, msg):
        self.velocity_sum += msg.twist.twist.linear.x
        self.angular_velocity_sum += msg.twist.twist.angular.z
        self.velocity_count += 1

    def imu_callback(self, msg):
        self.linear_accel_sum += msg.linear_acceleration.x
        self.linear_accel_count += 1
        
    def item_holders_callback(self, msg):
        for data in msg.data:
            if self.robot_items[data.robot_id].item_value != data.item_value:
                self.robot_items[data.robot_id] = data
                self.item_data_writer.writerow([self.get_clock().now().to_msg().sec, data.robot_id, data.item_value, data.item_colour])

    def control_loop(self):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_control_time).nanoseconds / 1e9  # Convert to seconds
        
        average_velocity = self.velocity_sum / self.velocity_count if self.velocity_count > 0 else 0
        average_angular_velocity = self.angular_velocity_sum / self.velocity_count if self.velocity_count > 0 else 0
        average_linear_accel = self.linear_accel_sum / self.linear_accel_count if self.linear_accel_count > 0 else 0
        
        # self.get_logger().info(f"Average velocity: {average_velocity} m/s, Average angular velocity: {average_angular_velocity} rad/s, Average linear acceleration: {average_linear_accel} m/s^2")
        
        # Calculate distance and update total distance
        distance_covered = average_velocity * time_diff
        self.total_distance += distance_covered

        # Log data
        self.data_writer.writerow([current_time.to_msg().sec, 'robot1', average_velocity, average_angular_velocity, average_linear_accel, self.total_distance])

        self.last_control_time = current_time
        self.velocity_sum = 0.0
        self.velocity_count = 0
        self.angular_velocity_sum = 0.0
        
        self.linear_accel_sum = 0.0
        self.linear_accel_count = 0

    def item_log_callback(self, msg):
        if self.previous_msg is None or self.is_different(msg, self.previous_msg):
            self.log_file.write(f'{self.get_clock().now().to_msg().sec},{self.counter},')
            self.log_file.write(f'{msg.red_count},{msg.green_count},{msg.blue_count},{msg.total_count},')
            self.log_file.write(f'{msg.red_value},{msg.green_value},{msg.blue_value},{msg.total_value}\n')
            self.log_file.flush()
            
            self.counter += 1
        
        self.previous_msg = msg
    
    def is_different(self, current_msg, previous_msg):
        return (
            current_msg.red_count != previous_msg.red_count or
            current_msg.green_count != previous_msg.green_count or
            current_msg.blue_count != previous_msg.blue_count or
            current_msg.total_count != previous_msg.total_count or
            current_msg.red_value != previous_msg.red_value or
            current_msg.green_value != previous_msg.green_value or
            current_msg.blue_value != previous_msg.blue_value or
            current_msg.total_value != previous_msg.total_value
        )

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()


def main(args=sys.argv):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    args_without_ros = rclpy.utilities.remove_ros_args(args)

    node = DataLogger(args_without_ros)

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