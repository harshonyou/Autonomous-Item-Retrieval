import os
import sys
import argparse

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPose
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu
import csv

from assessment_interfaces.msg import ItemLog, ItemHolders, ItemHolder

from tf2_ros import TransformException, Buffer, TransformListener

class DataLogger(Node):

    def __init__(self, args):
        super().__init__('data_logger')

        parser = argparse.ArgumentParser()
        group = parser.add_argument_group()
        group.add_argument('--path', type=str, metavar='PATH', help='Path')
        group.add_argument('--run_name', type=str, metavar='RUN_NAME', help='Run Name')
        group.add_argument('--num_robots', type=int, metavar='NUM_ROBOTS', help='Number of robots')
        self.args = parser.parse_args(args[1:])

        full_dirpath = os.path.join(self.args.path, self.args.run_name)
        self.get_logger().info(f"Logging data to dir: {full_dirpath}")
        
        if not os.path.exists(full_dirpath):
            os.makedirs(full_dirpath)

        self.start_time = self.get_clock().now()
        self.counter = 0
        self.previous_msg = None
        
        self.item_log_file = open(os.path.join(full_dirpath, 'item_log.csv'), 'w', newline='')
        self.tb_data_file = open(os.path.join(full_dirpath, 'turtlebot_data.csv'), 'w', newline='')
        self.item_data_file = open(os.path.join(full_dirpath, 'item_data.csv'), 'w', newline='')
        self.location_data_file = open(os.path.join(full_dirpath, 'location_data.csv'), 'w', newline='')

        self.item_log_file.write('timestamp,counter,')
        self.item_log_file.write('red_count,green_count,blue_count,total_count,')
        self.item_log_file.write('red_value,green_value,blue_value,total_value\n')
        
        self.data_writer = csv.writer(self.tb_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.data_writer.writerow(['Time', 'Robot ID', 'Linear Velocity', 'Angular Velocity', 'Linear Acceleration', 'Distance Traveled'])

        self.item_data_writer = csv.writer(self.item_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.item_data_writer.writerow(['Time', 'Robot ID', 'Item Value', 'Item Colour'])
        
        self.location_data_writer = csv.writer(self.location_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.location_data_writer.writerow(['Time', 'Robot ID', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])
        
        self.robot_odom_subscribers = {}
        self.robot_imu_subscribers = {}
        self.robot_items = {}
        
        self.total_distances = {}
        self.velocity_sums = {}
        self.angular_velocity_sums = {}
        self.velocity_counts = {}
        
        self.linear_accel_sums = {}
        self.linear_accel_counts = {}
        
        self.odoms = {}
        self.locations = {}
        self.robot_stamp = {}
        
        for i in range(1, self.args.num_robots + 1):
            robot_name = f'robot{i}'
            
            self.robot_odom_subscribers[robot_name] = self.create_subscription(
                Odometry,
                f'/{robot_name}/odom',
                lambda msg, robot_name=robot_name: self.odom_callback(msg, robot_name),
                10
            )
            
            self.robot_imu_subscribers[robot_name] = self.create_subscription(
                Imu,
                f'/{robot_name}/imu',
                lambda msg, robot_name=robot_name: self.imu_callback(msg, robot_name),
                10
            )
            
            self.robot_items[robot_name] = ItemHolder()
            
            self.total_distances[robot_name] = 0.0
            self.velocity_sums[robot_name] = 0.0
            self.angular_velocity_sums[robot_name] = 0.0
            self.velocity_counts[robot_name] = 0
            
            self.linear_accel_sums[robot_name] = 0.0
            self.linear_accel_counts[robot_name] = 0
            
            self.odoms[robot_name] = Odometry()
            self.locations[robot_name] = GeoPose()
            self.robot_stamp[robot_name] = Time()
            
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
            
        
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
        
        self.control_loop_counter = 0
        self.last_control_time = self.get_clock().now()
        
        self.timer_period = 2
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
    
    def odom_callback(self, msg, robot_name):
        self.odoms[robot_name] = msg
        self.velocity_sums[robot_name] += msg.twist.twist.linear.x
        self.angular_velocity_sums[robot_name] += msg.twist.twist.angular.z
        self.velocity_counts[robot_name] += 1
        self.robot_stamp[robot_name] = msg.header.stamp

    def imu_callback(self, msg, robot_name):
        self.linear_accel_sums[robot_name] += msg.linear_acceleration.x
        self.linear_accel_counts[robot_name] += 1
        
    def item_holders_callback(self, msg):
        for data in msg.data:
            if self.robot_items[data.robot_id].item_value != data.item_value:
                ts = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.robot_items[data.robot_id] = data
                self.item_data_writer.writerow([f"{ts:.2f}", data.robot_id, data.item_value, data.item_colour])

    def odom_to_map(self, robot_name):
        namespaced_source_frame = f"{robot_name}/odom"
        namespaced_target_frame = f"{robot_name}/map"
        
        # Try to get the transform from the source frame to the target frame
        try:
            transform = self.tf_buffer.lookup_transform(namespaced_target_frame, namespaced_source_frame, self.robot_stamp[robot_name])

        except TransformException as ex:
            # If the transform could not be found, log the error and return
            self.get_logger().info(
                f'Could not transform {namespaced_source_frame} to {namespaced_target_frame}: {ex}')
            return
        
        self.locations[robot_name].position.latitude = self.odoms[robot_name].pose.pose.position.x + transform.transform.translation.x
        self.locations[robot_name].position.longitude = self.odoms[robot_name].pose.pose.position.y + transform.transform.translation.y
        self.locations[robot_name].position.altitude = self.odoms[robot_name].pose.pose.position.z + transform.transform.translation.z
        
        self.locations[robot_name].orientation.x = self.odoms[robot_name].pose.pose.orientation.x + transform.transform.rotation.x
        self.locations[robot_name].orientation.y = self.odoms[robot_name].pose.pose.orientation.y + transform.transform.rotation.y
        self.locations[robot_name].orientation.z = self.odoms[robot_name].pose.pose.orientation.z + transform.transform.rotation.z
        self.locations[robot_name].orientation.w = self.odoms[robot_name].pose.pose.orientation.w + transform.transform.rotation.w
    
    def flush_all(self):
        self.item_log_file.flush()
        self.tb_data_file.flush()
        self.item_data_file.flush()
        self.location_data_file.flush()
    
    def control_loop(self):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_control_time).nanoseconds / 1e9  # Convert to seconds     
        
        self.control_loop_counter += 1
        if self.control_loop_counter % 25 == 0:
            self.get_logger().info("Flushing files")
            self.flush_all()
            self.control_loop_counter = 0   
        
        for i in range(1, self.args.num_robots + 1):
            ts = (current_time - self.start_time).nanoseconds / 1e9
            
            robot_name = f'robot{i}'
            
            self.odom_to_map(robot_name)
            self.location_data_writer.writerow([f"{ts:.2f}", robot_name, self.locations[robot_name].position.latitude, self.locations[robot_name].position.longitude, self.locations[robot_name].position.altitude, self.locations[robot_name].orientation.x, self.locations[robot_name].orientation.y, self.locations[robot_name].orientation.z, self.locations[robot_name].orientation.w])
            
            average_velocity = self.velocity_sums[robot_name] / self.velocity_counts[robot_name] if self.velocity_counts[robot_name] > 0 else 0
            average_angular_velocity = self.angular_velocity_sums[robot_name] / self.velocity_counts[robot_name] if self.velocity_counts[robot_name] > 0 else 0
            average_linear_accel = self.linear_accel_sums[robot_name] / self.linear_accel_counts[robot_name] if self.linear_accel_counts[robot_name] > 0 else 0
            
            # self.get_logger().info(f"Average velocity: {average_velocity} m/s, Average angular velocity: {average_angular_velocity} rad/s, Average linear acceleration: {average_linear_accel} m/s^2")
            
            # Calculate distance and update total distance
            distance_covered = average_velocity * time_diff
            self.total_distances[robot_name] += distance_covered
            
            # Log data
            self.data_writer.writerow([f"{ts:.2f}", robot_name, average_velocity, average_angular_velocity, average_linear_accel, self.total_distances[robot_name]])
            
            self.velocity_sums[robot_name] = 0.0
            self.velocity_counts[robot_name] = 0
            self.angular_velocity_sums[robot_name] = 0.0
            
            self.linear_accel_sums[robot_name] = 0.0
            self.linear_accel_counts[robot_name] = 0
            
        self.last_control_time = current_time

    def item_log_callback(self, msg):
        if self.previous_msg is None or self.is_different(msg, self.previous_msg):
            ts = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.item_log_file.write(f'{ts:.2f},{self.counter},')
            self.item_log_file.write(f'{msg.red_count},{msg.green_count},{msg.blue_count},{msg.total_count},')
            self.item_log_file.write(f'{msg.red_value},{msg.green_value},{msg.blue_value},{msg.total_value}\n')
            
            self.get_logger().info(f"Item log: {msg}")
            
            self.counter += 1
        
        self.previous_msg = msg
    
    def is_different(self, current_msg, previous_msg):
        return (
            current_msg.total_value != previous_msg.total_value
        )

    def close_files(self):
        self.item_log_file.close()
        self.tb_data_file.close()
        self.item_data_file.close()
        self.location_data_file.close()

    def destroy_node(self):
        self.close_files()
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