import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import LaserScan

import numpy as np
import math

def uv_to_depth_array(u, diameter_pixels, z_value, image_width, h_fov):
    """
    Create a depth array based on UV coordinates, diameter of the ball, and Z value.

    Parameters:
    u (int): U-coordinate (x-axis) of the ball center in pixels.
    diameter_pixels (int): Diameter of the ball in pixels.
    z_value (float): Depth value (Z) of the ball.
    image_width (int): Width of the image in pixels.
    h_fov (float): Horizontal field of view of the camera in radians.

    Returns:
    np.array: Depth array.
    """
    h_fov_deg = math.degrees(h_fov)
    depth_array_length = int(h_fov_deg)  # Array size based on h_fov in degrees
    depth_array = np.full(depth_array_length, float('inf'))

    radius_pixels = diameter_pixels / 2
    start_pixel = int(max(u - radius_pixels, 0))
    end_pixel = int(min(u + radius_pixels, image_width))

    angle_per_pixel = h_fov / image_width
    start_angle = math.degrees((start_pixel - image_width / 2) * angle_per_pixel)
    end_angle = math.degrees((end_pixel - image_width / 2) * angle_per_pixel)

    start_index = int((start_angle + h_fov_deg / 2))
    end_index = int((end_angle + h_fov_deg / 2))

    # Assign Z value to corresponding indices
    depth_array[start_index:end_index] = z_value

    return depth_array

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        self.laserscan_pub = self.create_publisher(LaserScan, '/scan_updated', 10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def control_loop(self):
        u = 320  # Example U-coordinate
        diameter_pixels = 50  # Diameter of the ball in pixels
        z_value = 2.0  # Z value (depth)
        image_width = 640  # Image width in pixels
        h_fov = 1.085595  # Horizontal field of view in radians
        depth_arr = uv_to_depth_array(u, diameter_pixels, z_value, image_width, h_fov)
        self.publish_laserscan_from_depth(depth_arr, -h_fov / 2, h_fov / 2, 0.01749303564429283, 'base_link')
    
    def publish_laserscan_from_depth(self, depth_array, angle_min, angle_max, angle_increment, frame_id):
        """
        Publish a LaserScan message from depth array (Z values).
        
        Parameters:
        depth_array (list or np.array): Array of depth (Z) values.
        angle_min (float): Start angle of the scan [rad].
        angle_max (float): End angle of the scan [rad].
        angle_increment (float): Angular distance between measurements [rad].
        frame_id (str): The frame ID to which this scan belongs.
        """
        
        print(depth_array)
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = frame_id
        
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0  # Update if necessary
        scan.scan_time = 0.0  # Update if necessary
        scan.range_min = 0.0  # Set according to your sensor's specification
        scan.range_max = 100.0  # Set according to your sensor's specification

        # Populate the ranges array
        scan.ranges = depth_array.tolist()

        # Publish the LaserScan message
        self.laserscan_pub.publish(scan)

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