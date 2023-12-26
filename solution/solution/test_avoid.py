import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from assessment_interfaces.msg import ItemList, Item

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
    
        self.image_width = 640
        self.image_height = 480
        self.horizontal_fov = 1.085595
        self.focal_length = (self.image_width / 2) / math.tan(self.horizontal_fov / 2)
        
        self.camera_position_x = 0.076
        self.camera_position_z = 0.093
        
        self.known_ball_diameter = 0.075
    
        self.item_subscriber = self.create_subscription(
            ItemList,
            '/robot1/items',
            self.listener_callback,
            10
        )
        
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def listener_callback(self, msg):
        for ball in msg.data:
            self.get_logger().info(f"Estimated distance to ball: {self.estimate_distance(ball.diameter)}")

    def estimate_distance(self, diameter_pixels):
        """
        Estimate the distance from the robot to the ball.

        :param diameter_pixels: Diameter of the ball in pixels as detected in the image
        :return: Estimated distance from the robot to the ball in meters
        """
        # Calculate the distance from the camera to the ball using similar triangles
        distance_camera_to_ball = (self.known_ball_diameter * self.focal_length) / diameter_pixels

        # Adjust for the camera position offset
        # Assuming the offset in the y-direction (height) is negligible for distance estimation
        distance_robot_to_ball = math.sqrt((distance_camera_to_ball ** 2) +
                                           (self.camera_position_x ** 2) +
                                           (self.camera_position_z ** 2))

        return distance_robot_to_ball

    # def process_ball(self, x, y, diameter):
    #     ball_diameter_in_pixels = diameter
    #     distance = (self.focal_length * self.known_ball_diameter) / ball_diameter_in_pixels

    #     # Calculate ball's position in camera coordinates
    #     # Convert pixel coordinates to real-world coordinates
    #     ball_position_x_camera = (x - (self.image_width / 2)) * (distance / self.focal_length)
    #     ball_position_y_camera = (y - (self.image_height / 2)) * (distance / self.focal_length)

    #     # Assuming camera is mounted on the front of the robot and facing forward
    #     # Adjust these values based on your robot's specific camera mounting
    #     camera_offset_x = 0.076  # meters
    #     camera_offset_z = 0.093  # meters

    #     # Calculate ball's position relative to the robot
    #     ball_position_x_robot = camera_offset_x + ball_position_x_camera
    #     ball_position_y_robot = distance  # Forward distance from camera
    #     ball_position_z_robot = camera_offset_z - ball_position_y_camera
        
    #     self.get_logger().info(f"Ball position - x: {ball_position_x_robot}, y: {ball_position_y_robot}, z: {ball_position_z_robot}")
        
    def control_loop(self):
        pass
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