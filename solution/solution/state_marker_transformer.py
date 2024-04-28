import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from visualization_msgs.msg import Marker
from solution_interfaces.msg import StateMarker

from tf2_ros import TransformException, Buffer, TransformListener

TRANSFORM_OFFSET = 0.3

class StateMarkerTransformer(Node):
    """
    A ROS 2 node that subscribes to StateMarker messages, transforms their positions from the 'odom' coordinate frame to the 'map' coordinate frame, and publishes the transformed state as a Marker message for visualization purposes.

    Attributes:
        subscriber (Subscription): Subscriber to StateMarker messages.
        publisher (Publisher): Publisher for Marker messages.
        source_frame (str): The source coordinate frame for transformations ('odom').
        target_frame (str): The target coordinate frame for transformations ('map').
        namespaced_source_frame (str): The namespaced source frame based on the node's namespace.
        namespaced_target_frame (str): The namespaced target frame based on the node's namespace.
        tf_buffer (Buffer): TF2 buffer for storing coordinate frame transformations.
        tf_listener (TransformListener): TF2 listener for receiving coordinate frame transformations.
    """

    def __init__(self):
        """
        Initializes the StateMarkerTransformer node, setting up the subscription to StateMarker messages, the publisher for Marker messages, and the TF2 listener for coordinate frame transformations.
        """
        super().__init__('state_marker')
        
        self.subscriber = self.create_subscription(
            StateMarker,
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
        """
        Callback for StateMarker messages. Transforms the position of the state marker from the 'odom' to the 'map' coordinate frame and publishes the transformed state as a Marker message.

        Args:
            data (StateMarker): The received StateMarker message.
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
        """
        Creates a Marker message based on the transformed StateMarker data.

        Args:
            data (StateMarker): The StateMarker message with transformed position.

        Returns:
            Marker: The Marker message for visualization.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.id = 0
        marker.scale.z = 0.1
        marker.color = data.color
        marker.pose = data.pose
        marker.pose.position.z += 0.2
        marker.text = data.text
        return marker

def main(args=None):
    """
    Main function for running the StateMarkerTransformer node. Initializes the node, handles ROS 2 spin, and ensures clean shutdown.

    Args:
        args: Command-line arguments passed to the node (default is None).
    """
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