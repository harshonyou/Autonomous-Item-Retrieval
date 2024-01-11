import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from visualization_msgs.msg import Marker
from auro_interfaces.msg import StringWithPose

from tf2_ros import TransformException, Buffer, TransformListener

TRANSFORM_OFFSET = 0.3

class StateMarkerTransformer(Node):
    """Node for transforming state data into visual markers.

    This class subscribes to a topic that provides state data with pose
    information, transforms this data according to the given frames, and
    then publishes the data as visual markers.

    Attributes:
        r: Red component of the marker's color.
        g: Green component of the marker's color.
        b: Blue component of the marker's color.
        subscriber: Subscriber to the state data topic.
        publisher: Publisher for the Marker data.
        source_frame: Source frame for the transformation.
        target_frame: Target frame for the transformation.
        tf_buffer: Buffer for storing transform data.
        tf_listener: Listener for transform data.
    """

    def __init__(self):
        """Initializes the StateMarkerTransformer node."""
        
        super().__init__('state_marker')
        
        self.declare_parameter('r', 0.0)
        self.declare_parameter('g', 0.0)
        self.declare_parameter('b', 0.0)
        self.r = self.get_parameter('r').get_parameter_value().double_value
        self.g = self.get_parameter('g').get_parameter_value().double_value
        self.b = self.get_parameter('b').get_parameter_value().double_value

        self.subscriber = self.create_subscription(
            StringWithPose,
            'raw_state_marker',
            self.subscriber_callback,
            10)

        self.publisher = self.create_publisher(Marker, 'state_marker', 10)
        
        self.source_frame = 'odom'
        self.target_frame = 'map'
        
        self.namespaced_source_frame = f"{self.get_namespace().strip('/')}/{self.source_frame}"
        self.namespaced_target_frame = f"{self.get_namespace().strip('/')}/{self.target_frame}"
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

    def subscriber_callback(self, data):
        """Callback for processing received state data.

        Transforms the received state data and publishes it as a visual marker.

        Args:
            data: The received state data with pose information.
        """
        
        timestamp = self.get_clock().now()
        
        # Try to get the transform from the source frame to the target frame
        try:
            transform = self.tf_buffer.lookup_transform(self.namespaced_target_frame, self.namespaced_source_frame, timestamp)

        except TransformException as ex:
            # If the transform could not be found, log the error and return
            self.get_logger().info(
                f'Could not transform {self.namespaced_source_frame} to {self.namespaced_target_frame}: {ex}')
            return
        
        # Apply the transform to the position of the data
        data.pose.position.x += transform.transform.translation.x - TRANSFORM_OFFSET
        data.pose.position.y += transform.transform.translation.y
        data.pose.position.z += transform.transform.translation.z
        
        # Create a marker at the transformed position
        marker = self.create_marker(data)
        
        # Publish the marker
        self.publisher.publish(marker)

    def create_marker(self, data):
        """Creates a visual marker from the provided state data.

        Args:
            data: The state data with pose information.

        Returns:
            A Marker object representing the transformed state data.
        """
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.id = 0
        marker.scale.z = 0.1
        marker.color.r = self.r
        marker.color.g = self.g
        marker.color.b = self.b
        marker.color.a = 1.0
        marker.pose = data.pose
        marker.pose.position.z += 0.2
        marker.text = data.text
        return marker

def main(args=None):
    rclpy.init(args = args)

    node = StateMarkerTransformer()

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