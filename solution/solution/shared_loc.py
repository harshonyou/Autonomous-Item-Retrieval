import struct
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped


class OdomToPointCloud2Node(Node):
    def __init__(self):
        super().__init__('odom_to_pointcloud2_node')
        self.subscription_robot1 = self.create_subscription(
            PoseWithCovarianceStamped, '/robot1/amcl_pose', self.robot1_odom_callback, 10)
        self.subscription_robot2 = self.create_subscription(
            PoseWithCovarianceStamped, '/robot2/amcl_pose', self.robot2_odom_callback, 10)
        self.publisher_robot1 = self.create_publisher(PointCloud2, '/robot1/combined_pointcloud2', 10)
        self.publisher_robot2 = self.create_publisher(PointCloud2, '/robot2/combined_pointcloud2', 10)
        # You might need to store the Odom data for processing
        self.robot1_odom_data = None
        self.robot2_odom_data = None

    def robot1_odom_callback(self, msg):
        self.robot1_odom_data = msg
        self.process_and_publish()

    def robot2_odom_callback(self, msg):
        print("robot2 odom callback")
        self.robot2_odom_data = msg
        self.process_and_publish()

    def process_and_publish(self):
        if self.robot1_odom_data is None or self.robot2_odom_data is None:
            return
        
        self.get_logger().info(f"Odom: {self.robot2_odom_data}")
        # Parameters for the point cloud
        square_side_length = 0.14  # Side length of the square in meters
        point_density = 50  # number of points per meter
        
        # Generate points for robot 2
        points_robot1 = self.generate_cube_cloud(self.robot1_odom_data, square_side_length, point_density)
        points_robot2 = self.generate_cube_cloud(self.robot2_odom_data, square_side_length, point_density)
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]
        
        point_cloud2_msg_robot1 = pc2.create_cloud(header, fields, points_robot2)
        point_cloud2_msg_robot2 = pc2.create_cloud(header, fields, points_robot1)
        
        self.publisher_robot1.publish(point_cloud2_msg_robot1)
        self.publisher_robot2.publish(point_cloud2_msg_robot2)

        

    def generate_cube_cloud(self, odom_data, side_length, density):
        points = []
        center_x = odom_data.pose.pose.position.x
        center_y = odom_data.pose.pose.position.y
        center_z = float(side_length / 2)
        r, g, b = 255, 0, 0 
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]  

        half_side = side_length / 2
        for x in np.linspace(center_x - half_side, center_x + half_side, density):
            for y in np.linspace(center_y - half_side, center_y + half_side, density):
                for z in np.linspace(center_z - half_side, center_z + half_side, density):
                    points.append([x, y, z, rgb])

        return points

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPointCloud2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
