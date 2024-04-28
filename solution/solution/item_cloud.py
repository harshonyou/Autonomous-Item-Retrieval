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

from solution_interfaces.msg import ProcessedItem, ProcessedItemList

class ItemsToPointCloud(Node):
    """
    A ROS 2 node that converts items detected in 2D space into a 3D point cloud representation. Additionally, it processes these items to calculate their 3D coordinates based on their 2D positions and publishes this processed item list.

    Attributes:
        items_subscriber: Subscriber to ItemList messages, representing detected items.
        items_publisher: Publisher for ProcessedItemList messages, containing processed item information.
        pointcloud_publisher: Publisher for PointCloud2 messages, representing the items in a 3D point cloud.
        camera_frame: The frame ID of the camera used for detection.
        width, height: Dimensions of the camera's view.
        aspect_ratio: Aspect ratio of the camera's view.
        h_fov, v_fov: Horizontal and vertical field of view of the camera.
        ball_radius: Radius of the detected items, assumed to be spherical.
    """
    
    def __init__(self):
        """
        Initializes the ItemsToPointCloud node, sets up publishers and subscribers, and initializes camera parameters.
        """
        super().__init__('items_to_pointcloud')

        self.get_logger().info('Detect Ball 3D Node Started')

        self.items_subscriber  = self.create_subscription(ItemList, "items", self.items_callback, 10)
        self.items_publisher = self.create_publisher(ProcessedItemList, "processed_items", 1)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, "items_pointcloud", 1)

        self.camera_frame = "camera_link"
        
        # Camera parameters initialization
        self.width = 640
        self.height = 480
        self.aspect_ratio = self.width/self.height
        self.h_fov = 1.085595
        self.v_fov = self.h_fov / self.aspect_ratio
        self.ball_radius = 0.075
    
    def items_callback(self, items: ItemList):   
        """
        Callback for ItemList messages. Processes each item to calculate its 3D position and constructs a 3D point cloud representing these items.

        Args:
            items: An ItemList message containing detected items.
        """     
        # Initialize an empty list to store points
        points = []
        processed_items = []
        
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
            
            
            processed_item = ProcessedItem()
            processed_item.x = item.x
            processed_item.y = item.y
            processed_item.diameter = item.diameter
            processed_item.colour = item.colour
            processed_item.value = item.value
            processed_item.x_coord = sphere_center_x
            processed_item.y_coord = sphere_center_y
            processed_item.distance = d
            
            processed_items.append(processed_item)
            
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
        
        processed_item_list = ProcessedItemList()
        processed_item_list.header = header
        processed_item_list.data = processed_items
        self.items_publisher.publish(processed_item_list)
        

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
        super().destroy_node()


def main(args=None):
    """
    Main function for the ItemsToPointCloud node. Initializes the node, handles ROS 2 spin, and ensures clean shutdown.

    Args:
        args: Arguments passed to the node (default is None).
    """
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