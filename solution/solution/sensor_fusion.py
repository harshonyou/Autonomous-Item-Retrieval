import os
import rclpy
from rclpy.node import Node
from assessment_interfaces.msg import ItemList
from sensor_msgs.msg import LaserScan
from image_geometry import PinholeCameraModel
import numpy as np
import math

from rclpy.parameter import Parameter

from .pkgs.camera import camera_model
from .pkgs.object_estimation import uv_to_depth_array


class SensorFusion(Node):
    """
    A ROS 2 node for fusing camera and laser scan data to enhance obstacle detection. This node subscribes to a topic publishing items detected by a camera and a topic for laser scan data, merges these data sources, and republishes the enhanced laser scan data.

    Attributes:
        items (ItemList): Storage for the latest detected items.
        camera_model (PinholeCameraModel): The camera model for projecting 2D detections into 3D space.
        h_fov (float): The horizontal field of view of the camera.
        camera_pos_gazebo (tuple): The position of the camera in the Gazebo simulation environment.
        actual_radius (float): The actual radius of the detected objects in meters.
        actual_diameter (float): The actual diameter of the detected objects, derived from the actual radius.
        persistent_items (dict): A dictionary for tracking detected items over time.
        item_decay_time (float): The time in seconds after which undetected items are removed from tracking.
        item_subscriber (Subscription): The subscriber to the topic publishing detected items.
        scan_subscriber (Subscription): The subscriber to the laser scan data topic.
        publisher (Publisher): The publisher for the enhanced laser scan data.
    """
    
    def __init__(self):
        """
        Initializes the SensorFusion node, setting up subscriptions to detected items and laser scan data, and prepares the publisher for the enhanced laser scan data.
        """
        super().__init__('sensor_fusion')
        
        self.items:ItemList = ItemList()
        self.camera_model:PinholeCameraModel = camera_model()
        
        self.h_fov = 1.085595
        self.camera_pos_gazebo = (0.076, 0.0, 0.093) 
        self.actual_radius = 0.075
        self.actual_diameter = self.actual_radius * 2
        
        self.persistent_items = {}
        self.item_decay_time = 0.3
        self.skip_counter = 0
        
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
        Callback for processing detected items. Updates tracking of persistent items and manages item decay over time.

        Args:
            msg (ItemList): The message containing detected items.
        """
        self.skip_counter += 1
        if self.skip_counter % 10 != 0:
            return
        
        self.items = msg
        
        current_time_seconds, current_time_nanoseconds = self.get_clock().now().seconds_nanoseconds()
        current_time = current_time_seconds + current_time_nanoseconds / 1e9

        new_items = {}
        
        for item in msg.data:
            item_id = f"obstacle_{item.x}_{item.y}"
            new_items[item_id] = {
                'item': item,
                'last_seen': current_time
            }

        self.persistent_items.update(new_items)
        
        self.persistent_items = {
            item_id: item_data for item_id, item_data in self.persistent_items.items()
            if current_time - item_data['last_seen'] <= self.item_decay_time
        }

    def scan_callback(self, msg: LaserScan):
        """
        Callback for processing laser scan data. Integrates the depth information derived from camera-detected items into the laser scan data and publishes the enhanced scan.

        Args:
            msg (LaserScan): The laser scan data to be enhanced.
        """
        depth_arrs = []
        
        for value in self.persistent_items.values():
            item = value['item']
            
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
        if len(depth_arrs) == 0:
            return
        elif len(depth_arrs) == 1:
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
    """
    Main function for running the SensorFusion node. Initializes the node, spins it to keep processing callbacks, and ensures clean shutdown.

    Args:
        args: Command-line arguments passed to the node (default is None).
    """
    rclpy.init(args=args)
    laser_scan_republisher = SensorFusion()
    rclpy.spin(laser_scan_republisher)
    laser_scan_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
