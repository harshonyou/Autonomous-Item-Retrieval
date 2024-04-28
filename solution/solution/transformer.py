import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from visualization_msgs.msg import Marker
from solution_interfaces.msg import PeersList, ProcessedItemList

from tf2_ros import TransformException, Buffer, TransformListener

class Transformer(Node):
    """
    A ROS 2 node for transforming the coordinates of peers and processed items from their original frames to the robot's base frame.

    Attributes:
        peers_subscriber (Subscription): Subscriber to PeersList messages.
        processed_items_subscriber (Subscription): Subscriber to ProcessedItemList messages.
        peers_publisher (Publisher): Publisher for transformed PeersList messages.
        processed_items_publisher (Publisher): Publisher for transformed ProcessedItemList messages.
        target_frame (str): The target coordinate frame to which the data will be transformed.
        namespaced_target_frame (str): The namespaced version of the target frame based on the node's namespace.
        tf_buffer (Buffer): TF2 buffer for storing coordinate frame transformations.
        tf_listener (TransformListener): TF2 listener for receiving coordinate frame transformations.
    """
    
    def __init__(self):
        """
        Initializes the Transformer node, setting up subscriptions to peers and processed items, and prepares the publishers for transformed data.
        """
        super().__init__('transformer')
        
        self.peers_subscriber = self.create_subscription(
            PeersList,
            'peers',
            self.peers_callback,
            10
        )
        
        self.processed_items_subscriber = self.create_subscription(
            ProcessedItemList,
            'processed_items',
            self.processed_items_callback,
            10
        )

        self.peers_publisher = self.create_publisher(PeersList, 'tf_peers', 10)
        
        self.processed_items_publisher = self.create_publisher(ProcessedItemList, 'tf_processed_items', 10)
        
        self.target_frame = 'base_link'
        
        self.namespaced_target_frame = f"{self.get_namespace().strip('/')}/{self.target_frame}"
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

    def peers_callback(self, peers:PeersList):
        """
        Callback for PeersList messages. Transforms the coordinates of each peer in the list to the target frame and publishes the transformed list.

        Args:
            peers (PeersList): The received list of peers with coordinates in the original frame.
        """
        namespaced_source_frame = f"{self.get_namespace().strip('/')}/{peers.header.frame_id}"
        
        
        # Try to get the transform from the source frame to the target frame
        try:
            transform = self.tf_buffer.lookup_transform(self.namespaced_target_frame, namespaced_source_frame, peers.header.stamp)

        except TransformException as ex:
            # If the transform could not be found, log the error and return
            self.get_logger().info(
                f'Could not transform {namespaced_source_frame} to {self.namespaced_target_frame}: {ex}')
            return
        
        for p in peers.data:
            p.x_coord += transform.transform.translation.x
            p.y_coord += transform.transform.translation.y
        
        peers.header.frame_id = self.target_frame
        
        self.peers_publisher.publish(peers)
    
    def processed_items_callback(self, processed_items: ProcessedItemList):
        """
        Callback for ProcessedItemList messages. Transforms the coordinates of each processed item in the list to the target frame and publishes the transformed list.

        Args:
            processed_items (ProcessedItemList): The received list of processed items with coordinates in the original frame.
        """
        namespaced_source_frame = f"{self.get_namespace().strip('/')}/{processed_items.header.frame_id}"
        
        # Try to get the transform from the source frame to the target frame
        try:
            transform = self.tf_buffer.lookup_transform(self.namespaced_target_frame, namespaced_source_frame, processed_items.header.stamp)

        except TransformException as ex:
            # If the transform could not be found, log the error and return
            self.get_logger().info(
                f'Could not transform {namespaced_source_frame} to {self.namespaced_target_frame}: {ex}')
            return
        
        for p in processed_items.data:
            p.x_coord += transform.transform.translation.x
            p.y_coord += transform.transform.translation.y
        
        processed_items.header.frame_id = self.target_frame
        
        self.processed_items_publisher.publish(processed_items)

def main(args=None):
    """
    Main function for running the Transformer node. Initializes the node, handles ROS 2 spin, and ensures clean shutdown.

    Args:
        args: Command-line arguments passed to the node (default is None).
    """
    rclpy.init(args = args)

    node = Transformer()

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