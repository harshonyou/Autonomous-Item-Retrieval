import os
import rclpy
from rclpy.node import Node
from assessment_interfaces.msg import ItemList
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg      import Point, Point32
from image_geometry import PinholeCameraModel
import numpy as np
import math
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import struct

# This Python script is a ROS2 node named SimpleLaserScanRepublisher, operating in the "robot1" namespace. It subscribes to 'items' (an ItemList message type) and 'scan' (a LaserScan message type). The node converts item positions detected in the camera's field of view into a 3D PointCloud2 representation.
# For each item detected, it calculates 3D points representing a sphere (representing the detected item) and publishes these points as a PointCloud2 message on the 'updated_scan' topic. This conversion involves calculating the item's angular size in the camera's field of view, estimating its distance and position in 3D space, and generating points for a sphere around this calculated center. The node uses camera characteristics like field of view and image dimensions in this calculation.
# Additionally, it logs the timestamp information for verification and debugging purposes.


class SimpleLaserScanRepublisher(Node):
    def __init__(self):
        super().__init__('simple_laser_scan_republisher', namespace="robot1")
        self.items:ItemList = ItemList()
        self.camera_frame = "camera_link"
        
        self.width = 640
        self.height = 480
        self.aspect_ratio = self.width/self.height
        self.h_fov = 1.085595
        self.v_fov = self.h_fov / self.aspect_ratio
        self.ball_radius = 0.075
        
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
        self.publisher = self.create_publisher(PointCloud2, 'updated_scan', 10)

    def item_callback(self, msg):
        self.items = msg

    def scan_callback(self, msg: LaserScan):
        if len(self.items.data) == 0: 
            return
        
        points = []
        
        for item in self.items.data:
            u = item.x / (self.width / 2)
            v = item.y / (self.height / 2)
            z = item.diameter / self.width
            ang_size = z*self.h_fov
            d = self.ball_radius/(math.atan(ang_size/2))
            y_ang = v*self.v_fov/2
            y = d*math.sin(y_ang)
            d_proj = d*math.cos(y_ang)
            x_ang = u*self.h_fov/2
            x = d_proj*math.sin(x_ang)
            z = d_proj*math.cos(x_ang)
            
            p = Point32()
            sphere_center_x = p.x = z
            sphere_center_y = p.y = x
            sphere_center_z = p.z = 0.0
            
            
            sphere_radius = 0.075  
            for phi in np.linspace(0, math.pi, 10):  
                for theta in np.linspace(0, 2 * math.pi, 20): 
                    dx = sphere_radius * math.sin(phi) * math.cos(theta)
                    dy = sphere_radius * math.sin(phi) * math.sin(theta)
                    dz = sphere_radius * math.cos(phi)
                    r, g, b = 255, 0, 0 
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]  
                    points.append([sphere_center_x + dx, sphere_center_y + dy, sphere_center_z + dz, rgb])

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.camera_frame
        
        self.get_logger().info(f"Stamp: {header.stamp}; Nodetime: {self.get_clock().now()}")

        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]

        cloud = point_cloud2.create_cloud(header, fields, points)
        self.publisher.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    laser_scan_republisher = SimpleLaserScanRepublisher()
    rclpy.spin(laser_scan_republisher)
    laser_scan_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
