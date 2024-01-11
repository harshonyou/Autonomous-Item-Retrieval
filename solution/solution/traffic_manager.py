import struct
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

SAFE_DISTANCE = 0.69  # Safe distance threshold

class TrafficManager(Node):
    """Node for managing traffic among a group of robots.

    This node subscribes to the AMCL pose of each robot and publishes a PointCloud2
    for each robot, which is a combination of the point clouds of the other robots.
    It also publishes a halt signal to the robots if they are too close to each other.

    Attributes:
        num_robots (int): Number of robots to manage.
        robot_subscriptions (dict): Subscriptions to robot's AMCL pose topics.
        robot_publishers (dict): Publishers for each robot's combined pose.
        halt_publishers (dict): Publishers to send halt signals to robots.
        odom_data (dict): Stores odometry data for each robot.
    """
    
    def __init__(self):
        """Initializes the TrafficManager node."""
        super().__init__('traffic_manager')
        
        self.declare_parameter('num_robots', 1)
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        
        self.robot_subscriptions = {}
        self.robot_publishers = {}
        self.halt_publishers = {}
        self.odom_data = {}

        # Setting up subscriptions and publishers for each robot
        for i in range(1, self.num_robots + 1):
            robot_name = f'robot{i}'
            
            self.robot_subscriptions[robot_name] = self.create_subscription(
                PoseWithCovarianceStamped, f'/{robot_name}/amcl_pose', 
                lambda msg, robot_name=robot_name: self.odom_callback(msg, robot_name), 10)
            
            self.robot_publishers[robot_name] = self.create_publisher(
                PointCloud2, f'/{robot_name}/combined_pose', 10)
            
            self.halt_publishers[robot_name] = self.create_publisher(
                Bool, f'/{robot_name}/halt', 10)
            
            self.odom_data[robot_name] = Odometry()
            
        self.default_odometry()
        
        self.true = Bool(data=True)
        self.false = Bool(data=False) 
        
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.controller_timer = self.create_timer(self.timer_period, self.control_loop)

    def odom_callback(self, msg, robot_name):
        """Callback function for odometry data.

        Args:
            msg (PoseWithCovarianceStamped): The incoming pose message.
            robot_name (str): Name of the robot.
        """
        self.odom_data[robot_name] = msg

    def control_loop(self):
        """Main control loop for managing traffic."""
        
        for i in range(1, self.num_robots + 1):
            for j in range(i + 1, self.num_robots + 1):
                robot1_name = f'robot{i}'
                robot2_name = f'robot{j}'

                # Calculate the distance between robot i and robot j
                dx = self.odom_data[robot1_name].pose.pose.position.x - self.odom_data[robot2_name].pose.pose.position.x
                dy = self.odom_data[robot1_name].pose.pose.position.y - self.odom_data[robot2_name].pose.pose.position.y
                distance = np.sqrt(dx ** 2 + dy ** 2)

                # Halting robots if they are too close to each other
                if distance < SAFE_DISTANCE:
                    self.get_logger().info(f"Distance between {robot1_name} and {robot2_name}: {distance}, Halting {robot1_name}")
                    self.halt_publishers[robot1_name].publish(self.true)
                    self.halt_publishers[robot2_name].publish(self.false)
                else:
                    self.halt_publishers[robot1_name].publish(self.false)
                    self.halt_publishers[robot2_name].publish(self.false)
        
        self.process_and_publish()

    def process_and_publish(self):
        """Process odometry data and publish point clouds."""
        # Parameters for the point cloud
        square_side_length = 0.14  # Side length of the square in meters
        point_density = 50  # Number of points per meter

        robot_point_clouds = self.generate_robot_point_clouds(square_side_length, point_density)
        self.publish_combined_point_clouds(robot_point_clouds)


    def generate_robot_point_clouds(self, side_length, density):
        """Generate point clouds for each robot."""
        robot_point_clouds = {}
        for i in range(1, self.num_robots + 1):
            robot_name = f'robot{i}'
            if self.odom_data[robot_name] is not None:
                robot_point_clouds[robot_name] = self.generate_cube_cloud(
                    self.odom_data[robot_name], side_length, density)
        return robot_point_clouds

    def publish_combined_point_clouds(self, robot_point_clouds):
        """Publish combined point clouds for each robot."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]

        for i in range(1, self.num_robots + 1):
            robot_name = f'robot{i}'
            combined_points = [point for other_robot, points in robot_point_clouds.items() if other_robot != robot_name for point in points]

            if combined_points:
                combined_point_cloud_msg = pc2.create_cloud(header, fields, combined_points)
                self.robot_publishers[robot_name].publish(combined_point_cloud_msg)
    
    def generate_cube_cloud(self, odom_data, side_length, density):
        """Generate a cube-shaped point cloud based on odometry data."""
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

    def default_odometry(self):
        """Set default odometry values based on the number of robots."""
        if self.num_robots == 1:
            self.odom_data["robot1"].pose.pose.position.x = -3.5
            self.odom_data["robot1"].pose.pose.position.y = 0.0
        elif self.num_robots == 2:
            self.odom_data["robot1"].pose.pose.position.x = -3.5
            self.odom_data["robot1"].pose.pose.position.y = 1.0
            self.odom_data["robot2"].pose.pose.position.x = -3.5
            self.odom_data["robot2"].pose.pose.position.y = -1.0
        elif self.num_robots == 3:
            self.odom_data["robot1"].pose.pose.position.x = -3.5
            self.odom_data["robot1"].pose.pose.position.y = 2.0
            self.odom_data["robot2"].pose.pose.position.x = -3.5
            self.odom_data["robot2"].pose.pose.position.y = 0.0
            self.odom_data["robot3"].pose.pose.position.x = -3.5
            self.odom_data["robot3"].pose.pose.position.y = -2.0
    
def main(args=None):
    rclpy.init(args=args)
    node = TrafficManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
