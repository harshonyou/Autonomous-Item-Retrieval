import os
import rclpy
from rclpy.node import Node
from assessment_interfaces.msg import ItemList
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
import numpy as np
import math

# This Python script creates a ROS2 node named SimpleLaserScanRepublisher in the "robot1" namespace. It subscribes to two topics: 'items' (an ItemList message type) and 'scan' (a LaserScan message type). The node integrates detected items' positions from the camera's field of view into the laser scan data. It calculates the depth array from camera image coordinates and updates the LaserScan data with these values, republishing the modified LaserScan on the 'updated_scan' topic. The script includes functions for setting up a camera model, converting image coordinates to depth values, and processing item and scan data.

def camera_model():
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

class SimpleLaserScanRepublisher(Node):
    def __init__(self):
        super().__init__('simple_laser_scan_republisher', namespace="robot1")
        self.items:ItemList = ItemList()
        self.camera_model:PinholeCameraModel = camera_model()
        self.h_fov = 1.085595  # Horizontal field of view in radians
        self.camera_pos_gazebo = (0.076, 0.0, 0.093)
        self.actual_radius = 0.075
        self.actual_diameter = self.actual_radius * 2
        
        self.item_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10
        )
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, 'updated_scan', 10)

    def item_callback(self, msg):
        self.items = msg

    def scan_callback(self, msg: LaserScan):
        # Directly republish the received LaserScan data
        if len(self.items.data) == 0: # the rviz wont get updated if there are no items
            return
        
        self.get_logger().info(f"Number of items: {len(self.items.data)}")
        depth_arrs = []
        
        for item in self.items.data:
            x = item.x
            y = item.y
            uv = (self.camera_model.cx() + x, self.camera_model.cy() + y)
            diameter_pixels = item.diameter
            scale_factor = self.actual_diameter / diameter_pixels
            focal_length = self.camera_model.fx()
            
            Z_camera = focal_length * scale_factor
            
            Z_world = Z_camera + self.camera_pos_gazebo[2]
            
            depth_arr = uv_to_depth_array(uv[0], diameter_pixels, Z_world, self.camera_model.width, self.h_fov)
            
            depth_arrs.append(depth_arr)
        
        if len(depth_arrs) == 1:
            min_values = depth_arrs[0]
        else:
            stacked_depth_arrs = np.stack(depth_arrs, axis=0)
            min_values = np.min(stacked_depth_arrs, axis=0)
            
        min_values_start_angle_deg = math.degrees(-self.h_fov / 2)
        laser_scan_start_angle_deg = math.degrees(msg.angle_min)
        angle_increment = math.degrees(msg.angle_increment)
        
        index_offset = int((min_values_start_angle_deg - laser_scan_start_angle_deg) / angle_increment)

        for i, depth in enumerate(min_values):
            range_index = index_offset + i
            msg.ranges[range_index] = min(depth, msg.ranges[range_index])
        
        # min_values_start_angle_deg = math.degrees(-self.h_fov / 2)
        # laser_scan_start_angle_deg = math.degrees(msg.angle_min)
        # angle_increment = math.degrees(msg.angle_increment)
        
        # index_offset = int((min_values_start_angle_deg - laser_scan_start_angle_deg) / angle_increment)

        # for i, depth in enumerate(min_values):
        #     range_index = index_offset + i
        #     msg.ranges[range_index] = min(depth, msg.ranges[range_index])
            # if 0 <= range_index < len(msg.ranges) and depth != float('inf'):
            #     msg.ranges[range_index] = depth
        
        # check if copy is still the same as msg.ranges
        # for i in range(len(copy)):
        #     if copy[i] != msg.ranges[i]:
        #         print("copy and msg.ranges are different")
        
        # In this loop
        # for i in range(min_values.size):
        #     if min_values[i] != float('inf'):
        #         msg.ranges[i] = min_values[i]
        
        # os._exit(1)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    laser_scan_republisher = SimpleLaserScanRepublisher()
    rclpy.spin(laser_scan_republisher)
    laser_scan_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
