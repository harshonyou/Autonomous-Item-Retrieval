import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from visualization_msgs.msg import Marker
from solution_interfaces.msg import PeersList, ProcessedItemList

from tf2_ros import TransformException, Buffer, TransformListener

class Transformer(Node):
    def __init__(self):
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