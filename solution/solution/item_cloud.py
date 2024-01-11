import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg      import  Point32
from assessment_interfaces.msg import ItemList
import math
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import struct

class ItemsToPointCloud(Node):
    """Node for converting detected items into a point cloud.

    This ROS node subscribes to a topic publishing detected items and converts
    them into a 3D point cloud representation, which is then published for
    visualization in RViz.

    Attributes:
        ball2d_sub: A subscriber to the items topic.
        ball_cloud_pub: Publisher for the point cloud data.
        camera_frame: Reference frame for the camera.
        width: Width of the camera frame.
        height: Height of the camera frame.
        aspect_ratio: Aspect ratio of the camera frame.
        h_fov: Horizontal field of view of the camera.
        v_fov: Vertical field of view of the camera.
        ball_radius: Radius of the detected balls.
    """
    
    def __init__(self):
        """Initializes the ItemsToPointCloud node."""
        super().__init__('items_to_pointcloud')

        self.get_logger().info('Detect Ball 3D Node Started')

        self.items_subscriber  = self.create_subscription(ItemList,"items",self.items_callback, 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, "items_pointcloud", 1)

        self.camera_frame = "camera_link"
        
        self.width = 640
        self.height = 480
        self.aspect_ratio = self.width/self.height
        self.h_fov = 1.085595
        self.v_fov = self.h_fov / self.aspect_ratio
        self.ball_radius = 0.075
    
    def items_callback(self, items: ItemList):
        """Callback for processing received items and publishing point cloud.

        Args:
            items: The list of detected items.
        """
        
        # Check if there are no items in the data
        if len(items.data) == 0: 
            return
        
        # Initialize an empty list to store points
        points = []
        
        for i, item in enumerate(items.data):
            # Normalize the x and y coordinates of the item
            u = item.x / (self.width / 2)
            v = item.y / (self.height / 2)
            
            # Normalize the diameter of the item
            z = item.diameter / self.width

            # Calculate the angular size of the item and its distance
            ang_size = z*self.h_fov
            d = self.ball_radius/(math.atan(ang_size/2))
            
            # Calculate the angular and distance deviations in X and Y
            y_ang = v*self.v_fov/2
            y = d*math.sin(y_ang)
            d_proj = d*math.cos(y_ang)

            x_ang = u*self.h_fov/2
            x = d_proj*math.sin(x_ang)
            Z = d_proj*math.cos(x_ang)
            
            p = Point32()
            sphere_center_x = p.x = Z
            sphere_center_y = p.y = x
            sphere_center_z = p.z = 0.0
            
            # Set the radius of the point cloud sphere
            sphere_radius = 0.075
            
            # Set the RGB values based on the color of the item
            if item.colour == "RED":
                r, g, b = 0, 0, 0
            elif item.colour == "GREEN":
                r, g, b = 125, 125, 125
            elif item.colour == "BLUE":
                r, g, b = 255, 255, 255
                
            # Pack the RGB values into a single float value
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
            
            # Generate points on the surface of the sphere
            for phi in np.linspace(0, math.pi, 10):
                for theta in np.linspace(0, 2 * math.pi, 20):
                    dx = sphere_radius * math.sin(phi) * math.cos(theta)
                    dy = sphere_radius * math.sin(phi) * math.sin(theta)
                    dz = sphere_radius * math.cos(phi)
                    
                    # Append each point to the points list
                    points.append([sphere_center_x + dx, sphere_center_y + dy, sphere_center_z + dz, rgb])

        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.camera_frame

        # Define the fields for the point cloud
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  # Add an RGB field. The datatype here is a 32-bit float, but it will be interpreted as packed 8-bit R, G, B, and unused.
                  PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]

        # Create the point cloud and publish it
        cloud = point_cloud2.create_cloud(header, fields, points)
        self.pointcloud_publisher.publish(cloud)

    def destroy_node(self):
        """Cleans up resources before node is destroyed."""
        super().destroy_node()


def main(args=None):
    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = ItemsToPointCloud()

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