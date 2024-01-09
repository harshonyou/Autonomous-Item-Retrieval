import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from geometry_msgs.msg      import Point, Point32
from visualization_msgs.msg import Marker
from assessment_interfaces.msg import ItemList, Item, HomeZone, ItemHolders
import math
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import struct
from rclpy.parameter import Parameter

# This Python script is a ROS2 node for a robot, designed to detect 3D positions of objects (like balls) using 2D camera data. It subscribes to 2D item positions, calculates their 3D coordinates based on camera characteristics, and publishes these as a PointCloud2 message for visualization and further processing.

class RobotController(Node):

    def __init__(self):
        # super().__init__('detect_ball_3d', namespace='robot1', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        super().__init__('detect_ball_3d')

        self.get_logger().info('Detecting in 3D')

        self.ball2d_sub  = self.create_subscription(ItemList,"items",self.ball_rcv_callback, 10)
        self.ball_cloud_pub = self.create_publisher(PointCloud2, "ball_3d_cloud", 1)

        self.camera_frame = "camera_link"
        
        self.width = 640
        self.height = 480
        self.aspect_ratio = self.width/self.height
        self.h_fov = 1.085595
        self.v_fov = self.h_fov / self.aspect_ratio
        self.ball_radius = 0.075
    
    def ball_rcv_callback(self, items: ItemList):
        # the rviz wont get updated if there are no items
        # keeping the last seen location of the ball
        # so no collision occurs
        if len(items.data) == 0: 
            return
        
        # cloud = PointCloud()
        # cloud.header.frame_id = self.camera_frame
        points = []
        
        for i, item in enumerate(items.data):
            u = item.x / (self.width / 2)
            v = item.y / (self.height / 2)
            z = item.diameter / self.width

            # Calculate angular size and consequently distance
            ang_size = z*self.h_fov
            d = self.ball_radius/(math.atan(ang_size/2))
            
            # Calculate angular and distance deviations in X and Y
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
            
            sphere_radius = 0.075  # Adjust radius of the sphere as needed
            if item.colour == "RED":
                r, g, b = 0, 0, 0
            elif item.colour == "GREEN":
                r, g, b = 125, 125, 125
            elif item.colour == "BLUE":
                r, g, b = 255, 255, 255
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]  # Pack RGB into a single float value
            
            for phi in np.linspace(0, math.pi, 10):  # Adjust granularity
                for theta in np.linspace(0, 2 * math.pi, 20):  # Adjust granularity
                    dx = sphere_radius * math.sin(phi) * math.cos(theta)
                    dy = sphere_radius * math.sin(phi) * math.sin(theta)
                    dz = sphere_radius * math.cos(phi)
                    
                    # Append each point to the points list
                    points.append([sphere_center_x + dx, sphere_center_y + dy, sphere_center_z + dz, rgb])

        
        # self.ball_cloud_pub.publish(cloud)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        # self.get_logger().info('Current time: "%s"' % self.get_clock().now().to_msg())
        header.frame_id = self.camera_frame

        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  # Add an RGB field. The datatype here is a 32-bit float, but it will be interpreted as packed 8-bit R, G, B, and unused.
                  PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]

        cloud = point_cloud2.create_cloud(header, fields, points)
        self.ball_cloud_pub.publish(cloud)

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