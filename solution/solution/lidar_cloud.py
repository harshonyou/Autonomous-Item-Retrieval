import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry
from sensor_msgs_py import point_cloud2

class LaserScanToPointCloud(Node):
    """Node for converting LaserScan data to a PointCloud2 format.

    This ROS node subscribes to a topic that provides LaserScan data and
    converts this data into a 3D point cloud representation, which is then
    published for use in various applications like obstacle detection or
    navigation.

    Attributes:
        subscription: Subscriber to the LaserScan topic.
        publisher: Publisher for the PointCloud2 data.
        laser_projector: Object for projecting laser scan data.
        height: The height to which the laser scan points are extruded.
        wall_height_intervals: Number of intervals for the height division.
    """

    def __init__(self):
        """Initializes the LaserScanToPointCloud node."""
       
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
        """Callback for processing received LaserScan data.

        Args:
            data: The LaserScan data received from the subscription.
        """
       
        cloud2d = self.laser_projector.projectLaser(data)
        cloud3d = self.add_height(cloud2d)
        self.publisher.publish(cloud3d)

    def add_height(self, cloud2d):
        """Adds height to 2D laser scan data to create a 3D point cloud.

        Args:
            cloud2d: The 2D point cloud data.

        Returns:
            A 3D point cloud with height information added.
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
    rclpy.init(args=args)
    node = LaserScanToPointCloud()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
