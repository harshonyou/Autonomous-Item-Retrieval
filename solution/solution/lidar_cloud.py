import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry
from sensor_msgs_py import point_cloud2

class LaserScanToPointCloud(Node):
    """
    A ROS 2 node that converts 2D laser scan data into a 3D point cloud by extruding the 2D scan points into the vertical dimension.

    Attributes:
        subscription: A subscription to LaserScan messages.
        publisher: A publisher for PointCloud2 messages, representing the extruded 3D point cloud.
        laser_projector: A laser_geometry.LaserProjection object for converting LaserScan messages to PointCloud2.
        height: The height to which the 2D scan points are extruded.
        wall_height_intervals: The number of intervals (steps) used for extrusion, affecting the point cloud density.
    """

    def __init__(self):
        """
        Initializes the LaserScanToPointCloud node, setting up the subscription to laser scan data and the publisher for the 3D point cloud.
        """
        super().__init__('laserscan_to_pointcloud')
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, 'obstacles_pointcloud', 10)
        
        self.laser_projector = laser_geometry.LaserProjection()
        self.height = 1.0
        self.wall_height_intervals = 25


    def listener_callback(self, data):
        """
        Callback for the LaserScan messages. Converts the 2D laser scan data to a 3D point cloud by extruding the points vertically.

        Args:
            data: A LaserScan message containing the 2D scan data.
        """
        cloud2d = self.laser_projector.projectLaser(data)
        cloud3d = self.add_height(cloud2d)
        self.publisher.publish(cloud3d)

    def add_height(self, cloud2d):
        """
        Adds height to the 2D point cloud data to create a 3D point cloud representation by extruding the scan points vertically.

        Args:
            cloud2d: The 2D point cloud generated from laser scan data.

        Returns:
            A PointCloud2 message containing the 3D point cloud with added height.
        """
        gen = point_cloud2.read_points(cloud2d, skip_nans=True, field_names=("x", "y", "z"))

        points_with_walls = []
        for point in gen:
            x, y, _ = point
            for i in range(self.wall_height_intervals + 1):
                z = (self.height / self.wall_height_intervals) * i
                points_with_walls.append((x, y, z))

        cloud3d = point_cloud2.create_cloud_xyz32(cloud2d.header, points_with_walls)
        return cloud3d

def main(args=None):
    """
    Main function for the LaserScanToPointCloud node. Initializes the node, handles ROS 2 spin, and ensures clean shutdown.

    Args:
        args: Arguments passed to the node (default is None).
    """
    rclpy.init(args=args)
    node = LaserScanToPointCloud()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
