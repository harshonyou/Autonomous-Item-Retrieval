import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry
from sensor_msgs_py import point_cloud2
import numpy as np

class LaserScanToPointCloud(Node):
    def __init__(self):
        # super().__init__('laserscan_to_pointcloud', namespace="robot1")
        super().__init__('laserscan_to_pointcloud')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.laser_projector = laser_geometry.LaserProjection()
        self.height = 1.0  # Constant height for all points
        self.wall_height_intervals = 25


    def listener_callback(self, data):
        cloud2d = self.laser_projector.projectLaser(data)
        cloud3d = self.add_height(cloud2d)
        self.publisher.publish(cloud3d)

    def add_height(self, cloud2d):
        # Convert PointCloud2 to a list of points
        gen = point_cloud2.read_points(cloud2d, skip_nans=True, field_names=("x", "y", "z"))

        # Add height to each point and create wall points
        points_with_walls = []
        for point in gen:
            x, y, _ = point
            for i in range(self.wall_height_intervals + 1):
                z = (self.height / self.wall_height_intervals) * i  # Interpolate height
                points_with_walls.append((x, y, z))

        # Convert back to PointCloud2
        cloud3d = point_cloud2.create_cloud_xyz32(cloud2d.header, points_with_walls)
        return cloud3d



def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloud()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
