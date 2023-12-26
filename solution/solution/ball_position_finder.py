
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from assessment_interfaces.msg import Item, ItemList
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np
from enum import Enum

class Colour(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3

class ItemSensor(Node):
    def __init__(self):
        super().__init__('item_sensor')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.item_publisher = self.create_publisher(ItemList, 'items', 10)

    def calculate_ball_position(self, msg):
        # Camera intrinsic parameters
        image_width = 640
        image_height = 480
        horizontal_fov = 1.085595
        focal_length = (image_width / 2) / math.tan(horizontal_fov / 2)

        # Camera extrinsic parameters (pose relative to the robot)
        camera_position_x = 0.076
        camera_position_z = 0.093

        # Assuming the known diameter of the ball in meters
        known_ball_diameter = 0.1  # Example value, adjust as needed

        # Calculate distance to the ball
        ball_diameter_in_pixels = msg.diameter
        distance = (focal_length * known_ball_diameter) / ball_diameter_in_pixels

        # Calculate ball's position in camera coordinates
        ball_position_x_camera = (msg.x - (image_width / 2)) * (distance / focal_length)
        ball_position_y_camera = (msg.y - (image_height / 2)) * (distance / focal_length)

        # Adjust for camera's position relative to the robot
        ball_position_x_robot = camera_position_x + ball_position_x_camera
        ball_position_z_robot = camera_position_z - ball_position_y_camera

        return ball_position_x_robot, distance, ball_position_z_robot

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info(f"CvBridgeError: {e}")
            return

        # Process frame to find balls and calculate their positions
        # [Ball detection logic goes here]

        # Example: Publish dummy ball data for testing
        test_msg = Item()
        test_msg.x = 100  # Example value
        test_msg.y = 100  # Example value
        test_msg.diameter = 50  # Example value
        ball_position = self.calculate_ball_position(test_msg)
        self.get_logger().info(f'Ball Position Relative to Robot: {ball_position}')

def main(args=None):
    rclpy.init(args=args)
    node = ItemSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
