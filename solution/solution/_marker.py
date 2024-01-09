import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from visualization_msgs.msg import Marker
from auro_interfaces.msg import StringWithPose

from tf2_ros import TransformException, Buffer, TransformListener

class RVizTextMarker(Node):

    def __init__(self):
        super().__init__('rviz_text_marker')

        self.subscriber = self.create_subscription(
            StringWithPose,
            'marker_input',
            self.subscriber_callback,
            10)
        self.subscriber_callback  # prevent unused variable warning

        self.publisher = self.create_publisher(Marker, 'marker_output', 10)
        
        self.source_frame = 'odom'
        self.target_frame = 'map'
        
        self.namespaced_source_frame = f"{self.get_namespace().strip('/')}/{self.source_frame}"
        self.namespaced_target_frame = f"{self.get_namespace().strip('/')}/{self.target_frame}"
        
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

    def subscriber_callback(self, input):
        # self.get_logger().info(f"input: {input}")
        timestamp = self.get_clock().now()
        
        try:
            transform = self.tf_buffer.lookup_transform(self.namespaced_target_frame, self.namespaced_source_frame, timestamp)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.namespaced_source_frame} to {self.namespaced_target_frame}: {ex}')
            return
        
        # transformed_pose = do_transform_pose(msg.pose, transform)
        
        input.pose.position.x += transform.transform.translation.x - 0.3
        input.pose.position.y += transform.transform.translation.y
        input.pose.position.z += transform.transform.translation.z
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.id = 0
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose = input.pose
        marker.pose.position.z += 0.2
        marker.text = input.text
        self.publisher.publish(marker)


def main(args=None):

    rclpy.init(args = args)

    node = RVizTextMarker()

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