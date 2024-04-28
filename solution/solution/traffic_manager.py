import struct
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

from assessment_interfaces.msg import ItemHolders, ItemHolder
from solution_interfaces.msg import Peers, PeersList, Halt

SAFE_DISTANCE = 0.9  # Safe distance threshold

class TrafficManager(Node):
    """
    A ROS 2 node responsible for managing the traffic of a fleet of robots by ensuring they maintain a safe distance from each other. It subscribes to the pose of each robot and their item holding status, calculates the proximity between robots, and publishes halt commands to prevent collisions. Additionally, it publishes peer lists and combined point clouds for visualization purposes.

    Attributes:
        num_robots (int): Number of robots in the fleet.
        robot_subscriptions (dict): Stores subscriptions to the pose topics of each robot.
        robot_publishers (dict): Publishers for publishing combined point clouds for each robot.
        peers_publishers (dict): Publishers for publishing lists of peers (other robots) for each robot.
        halt_publishers (dict): Publishers for issuing halt commands to each robot.
        odom_data (dict): Stores the latest odometry data for each robot.
        halt_status (dict): Tracks the halt status (True for halted, False for moving) of each robot.
        robot_item_values (dict): Stores the latest item value held by each robot.
    """

    def __init__(self):
        """
        Initializes the TrafficManager node, setting up parameters, subscriptions, and publishers for managing robot traffic.
        """
        super().__init__('traffic_manager')
        
        self.declare_parameter('num_robots', 1)
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        
        self.robot_subscriptions = {}
        self.robot_publishers = {}
        self.peers_publishers = {}
        self.halt_publishers = {}
        self.odom_data = {}
        self.halt_status = {}
        self.robot_item_values = {}

        # Setting up subscriptions and publishers for each robot
        for i in range(1, self.num_robots + 1):
            robot_name = f'robot{i}'
            
            self.robot_subscriptions[robot_name] = self.create_subscription(
                PoseWithCovarianceStamped, f'/{robot_name}/amcl_pose', 
                lambda msg, robot_name=robot_name: self.odom_callback(msg, robot_name), 10)
            
            self.robot_publishers[robot_name] = self.create_publisher(
                PointCloud2, f'/{robot_name}/combined_pose', 10)
            
            self.peers_publishers[robot_name] = self.create_publisher(
                PeersList, f'/{robot_name}/peers', 10)
            
            self.halt_publishers[robot_name] = self.create_publisher(
                Halt, f'/{robot_name}/halt', 10)
            
            self.odom_data[robot_name] = Odometry()
            
            self.halt_status[robot_name] = False
            
            self.robot_item_values[robot_name] = 0
        
        self.item_holders_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.item_holders_callback,
            10,
        )
         
        self.default_odometry()
        
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.controller_timer = self.create_timer(self.timer_period, self.control_loop)

    def item_holders_callback(self, msg):
        """
        Callback for item holders information. Updates the item value held by each robot.

        Args:
            msg (ItemHolders): The message containing item holders information.
        """
        for data in msg.data:
            self.robot_item_values[data.robot_id] = data.item_value

    def odom_callback(self, msg, robot_name):
        """
        Callback for receiving odometry data from each robot.

        Args:
            msg (PoseWithCovarianceStamped): The message containing the robot's pose.
            robot_name (str): The name of the robot from which the message was received.
        """
        self.odom_data[robot_name] = msg

    def control_loop(self):
        """
        Main control loop for the TrafficManager. It calculates the proximity between robots and issues halt commands as necessary to maintain a safe distance. It also processes and publishes combined point clouds and peer lists for visualization.
        """
        robots_to_halt = set()
        
        for i in range(1, self.num_robots + 1):
            for j in range(i + 1, self.num_robots + 1):
                robot1_name = f'robot{i}'
                robot2_name = f'robot{j}'

                # Calculate the distance between robot i and robot j
                dx = self.odom_data[robot1_name].pose.pose.position.x - self.odom_data[robot2_name].pose.pose.position.x
                dy = self.odom_data[robot1_name].pose.pose.position.y - self.odom_data[robot2_name].pose.pose.position.y
                distance = np.sqrt(dx ** 2 + dy ** 2)

                # If the distance is less than the safe distance, determine which robot to halt based on item value and number
                if distance < SAFE_DISTANCE:
                    # Compare item values
                    item_value_robot1 = self.robot_item_values[robot1_name]
                    item_value_robot2 = self.robot_item_values[robot2_name]

                    if item_value_robot1 > item_value_robot2:
                        robot_to_halt = robot2_name
                    elif item_value_robot1 < item_value_robot2:
                        robot_to_halt = robot1_name
                    else:
                        # If item values are equal, use robot number as tiebreaker
                        robot_to_halt = robot1_name if i < j else robot2_name

                    robots_to_halt.add(robot_to_halt)
                    # self.get_logger().info(f"Distance between {robot1_name} and {robot2_name}: {distance}, Halting {robot_to_halt}")


        for robot_name in self.halt_status:
            if robot_name in robots_to_halt and not self.halt_status[robot_name]:
                self.get_logger().info(f"Halting {robot_name}")
                halt = Halt()
                halt.status = True
                halt.message = f"Robot {robot_name} is too close to another robot."
                self.halt_publishers[robot_name].publish(halt)
                self.halt_status[robot_name] = True
            elif robot_name not in robots_to_halt and self.halt_status[robot_name]:
                self.get_logger().info(f"Resuming {robot_name}")
                halt = Halt()
                halt.status = False
                halt.message = f"Robot {robot_name} is not too close to another robot."
                self.halt_publishers[robot_name].publish(halt)
                self.halt_status[robot_name] = False
            
        self.process_and_publish()

    def process_and_publish(self):
        """
        Processes and publishes combined point clouds and other robots' positions for each robot.
        """
        # Parameters for the point cloud
        square_side_length = 0.14  # Side length of the square in meters
        point_density = 50  # Number of points per meter

        robot_point_clouds = self.generate_robot_point_clouds(square_side_length, point_density)
        self.publish_combined_point_clouds(robot_point_clouds)
        
        self.publish_other_robots_positions()

    def publish_other_robots_positions(self):
        """
        Publishes the positions of other robots to each robot for situational awareness.
        """
        for i in range(1, self.num_robots + 1):
            robot_name = f'robot{i}'
            peers_list_msg = PeersList()
            peers_list_msg.header.stamp = self.get_clock().now().to_msg()
            peers_list_msg.header.frame_id = "map"

            for other_robot_name, data in self.odom_data.items():
                if other_robot_name != robot_name:
                    peer = Peers()
                    peer.x_coord = data.pose.pose.position.x
                    peer.y_coord = data.pose.pose.position.y
                    peers_list_msg.data.append(peer)

            if peers_list_msg.data:
                self.peers_publishers[robot_name].publish(peers_list_msg)

    def generate_robot_point_clouds(self, side_length, density):
        """
        Generates point clouds representing each robot for visualization purposes.

        Args:
            side_length (float): The side length of the cube representing a robot in the point cloud.
            density (int): The density of points per meter in the generated point cloud.

        Returns:
            dict: A dictionary containing the generated point clouds for each robot.
        """
        robot_point_clouds = {}
        for i in range(1, self.num_robots + 1):
            robot_name = f'robot{i}'
            if self.odom_data[robot_name] is not None:
                robot_point_clouds[robot_name] = self.generate_cube_cloud(
                    self.odom_data[robot_name], side_length, density)
        return robot_point_clouds

    def publish_combined_point_clouds(self, robot_point_clouds):
        """
        Publishes combined point clouds for each robot, excluding the robot itself from its own point cloud.

        Args:
            robot_point_clouds (dict): A dictionary containing the point clouds for each robot.
        """
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
        """
        Generates a cubic point cloud around a robot's position.

        Args:
            odom_data (Odometry): The odometry data of the robot.
            side_length (float): The side length of the cube.
            density (int): The density of points per meter.

        Returns:
            list: A list of points representing the cube cloud.
        """
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
        """
        Sets default odometry values for robots. Used for testing or in the absence of initial pose information.
        """
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
    """
    Main function for running the TrafficManager node. Initializes the node, spins it to process callbacks, and ensures clean shutdown.
    """
    rclpy.init(args=args)
    node = TrafficManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
