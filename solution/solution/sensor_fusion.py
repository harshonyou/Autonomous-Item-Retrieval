import os
import rclpy
from rclpy.node import Node
from assessment_interfaces.msg import ItemList
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
import numpy as np
import math

from rclpy.parameter import Parameter

def camera_model():
    """
    Create and configure a PinholeCameraModel.

    This function initializes a CameraInfo object with specific parameters like height, width,
    distortion model, and camera matrix values. It then uses this information to create and return
    a configured PinholeCameraModel object.

    Returns:
        PinholeCameraModel: Configured camera model object.
    """
    
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

def uv_to_depth_array(u, diameter_pixels, z_value, image_width, h_fov):
    """
    Convert image coordinates to a depth array.

    This function takes image coordinates and other camera parameters to create a depth array.
    It calculates the array length based on the horizontal field of view and assigns Z values to corresponding indices.

    Args:
        u (int): The u-coordinate in the image.
        diameter_pixels (float): Diameter of the object in pixels.
        z_value (float): The depth value to be assigned in the array.
        image_width (int): Width of the image in pixels.
        h_fov (float): Horizontal field of view in radians.

    Returns:
        numpy.ndarray: The depth array with assigned depth values.
    """
    
    # Convert horizontal field of view from radians to degrees
    h_fov_deg = math.degrees(h_fov)
    # Calculate the length of the depth array based on the horizontal field of view
    depth_array_length = int(h_fov_deg)
    # Initialize the depth array with infinite values
    depth_array = np.full(depth_array_length, float('inf'))

    # Calculate the radius of the object in pixels
    radius_pixels = diameter_pixels / 2
    
    # Calculate the start and end pixels of the object in the image
    start_pixel = int(max(u - radius_pixels, 0))
    end_pixel = int(min(u + radius_pixels, image_width))

    # Calculate the angle per pixel based on the horizontal field of view and image width
    angle_per_pixel = h_fov / image_width
    
    # Calculate the start and end angles of the object in the image
    start_angle = math.degrees((start_pixel - image_width / 2) * angle_per_pixel)
    end_angle = math.degrees((end_pixel - image_width / 2) * angle_per_pixel)

    # Calculate the start and end indices in the depth array based on the start and end angles
    start_index = int((start_angle + h_fov_deg / 2))
    end_index = int((end_angle + h_fov_deg / 2))

    # Assign the depth value to the corresponding indices in the depth array
    depth_array[start_index:end_index] = z_value

    # Return the depth array
    return depth_array

class SensorFusion(Node):
    """
    A ROS2 node for sensor fusion.

    This class extends the ROS2 Node class to create a sensor fusion node. It subscribes to item and laser scan topics,
    processes the incoming data, and publishes updated laser scan information.

    Attributes:
        items (ItemList): A list of detected items.
        camera_model (PinholeCameraModel): The camera model used for calculations.
        h_fov (float): Horizontal field of view in radians.
        camera_pos_gazebo (tuple): The camera position in Gazebo.
        actual_radius (float): The actual radius of the item in meters.
        actual_diameter (float): The actual diameter of the item in meters.
    """
    
    def __init__(self):
        """
        Initialize the SensorFusion node.
        
        Sets up the subscriptions for item and laser scan data and initializes the publisher for the updated scan data.
        """
        super().__init__('sensor_fusion')
        
        self.items:ItemList = ItemList()
        self.camera_model:PinholeCameraModel = camera_model()
        
        self.h_fov = 1.085595
        self.camera_pos_gazebo = (0.076, 0.0, 0.093) 
        self.actual_radius = 0.075
        self.actual_diameter = self.actual_radius * 2
        
        self.item_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10
        )
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.publisher = self.create_publisher(LaserScan, 'camdar', 10)

    def item_callback(self, msg):
        """
        Callback for the item data subscription.

        This method updates the node's item list with the received data.

        Args:
            msg (ItemList): The received item list message.
        """
        self.items = msg

    def scan_callback(self, msg: LaserScan):
        """
        Callback for the laser scan data subscription.

        Processes the received laser scan data along with the item data to produce an updated laser scan.
        Publishes the updated scan data.

        Args:
            msg (LaserScan): The received laser scan message.
        """
        # Skip update if no items are detected
        # This is to not update local costmap if items get out of sight
        if len(self.items.data) == 0:
            return
        
        depth_arrs = []
        
        for item in self.items.data:
            # Extract x and y coordinates of the item
            x = item.x
            y = item.y
            
            # Move origin from center of image to top left corner
            uv = (self.camera_model.cx() + x, self.camera_model.cy() + y)
            
            diameter_pixels = item.diameter
            
            scale_factor = self.actual_diameter / diameter_pixels
            focal_length = self.camera_model.fx()
            
            # Calculate the depth in the camera's coordinate system
            Z_camera = focal_length * scale_factor
            
            # Convert the depth to the world's coordinate system
            Z_world = Z_camera + self.camera_pos_gazebo[2]
            
            # Convert the UV coordinates to a depth array
            depth_arr = uv_to_depth_array(uv[0], diameter_pixels, Z_world, self.camera_model.width, self.h_fov)
            
            depth_arrs.append(depth_arr)
        
        # If there's only one depth array, use it as the minimum values
        if len(depth_arrs) == 1:
            min_values = depth_arrs[0]
        else:
            # Otherwise, stack the depth arrays and find the minimum values along the first axis
            stacked_depth_arrs = np.stack(depth_arrs, axis=0)
            min_values = np.min(stacked_depth_arrs, axis=0)
        
        # Calculate the start angle of the minimum values in degrees    
        min_values_start_angle_deg = math.degrees(-self.h_fov / 2)
        # Get the start angle of the laser scan in degrees
        laser_scan_start_angle_deg = math.degrees(msg.angle_min)
        # Get the angle increment of the laser scan in degrees
        angle_increment = math.degrees(msg.angle_increment)
        
        # Calculate the index offset based on the difference in start angles and the angle increment
        index_offset = int((min_values_start_angle_deg - laser_scan_start_angle_deg) / angle_increment)

        # Update the ranges in the laser scan message with the minimum values
        for i, depth in enumerate(min_values):
            range_index = index_offset + i
            msg.ranges[range_index] = min(depth, msg.ranges[range_index])
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    laser_scan_republisher = SensorFusion()
    rclpy.spin(laser_scan_republisher)
    laser_scan_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
