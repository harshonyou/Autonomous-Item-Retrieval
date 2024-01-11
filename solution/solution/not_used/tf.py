import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, Buffer, TransformListener
from sensor_msgs.msg import PointCloud, PointCloud2
import tf_transformations
import numpy as np
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


def transform_point(trans, pt):
    quat = [
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w
    ]
    mat = tf_transformations.quaternion_matrix(quat)
    pt_np = [pt.x, pt.y, pt.z, 1.0]
    pt_in_map_np = np.dot(mat, pt_np)

    pt_in_map = Point()
    pt_in_map.x = pt_in_map_np[0] + trans.transform.translation.x
    pt_in_map.y = pt_in_map_np[1] + trans.transform.translation.y
    pt_in_map.z = pt_in_map_np[2] + trans.transform.translation.z

    return pt_in_map

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_transform_listener', namespace='robot1')

        # Set the target frame to robot1/camera_link and to_frame to robot1/base_link
        self.source_frame = 'camera_link'
        self.target_frame = 'base_link'
        
        self.namespaced_source_frame = f"{self.get_namespace().strip('/')}/{self.source_frame}"
        self.namespaced_target_frame = f"{self.get_namespace().strip('/')}/{self.target_frame}"
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.point_cloud_pub = self.create_publisher(PointCloud2, "point_3d_cloud", 1)
        self.point_cloud_sub = self.create_subscription(PointCloud2,  "ball_3d_cloud", self.point_cloud_callback, 10)

    def point_cloud_callback(self, msg: PointCloud2):
        timestamp = msg.header.stamp
        source_frame = msg.header.frame_id
        self.get_logger().info(f"Received point cloud from {self.source_frame} at {timestamp}")
        
        try:
            # Look up for the transformation between from_frame and to_frame
            transform = self.tf_buffer.lookup_transform(
                self.namespaced_target_frame,
                self.namespaced_source_frame,
                timestamp)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.namespaced_source_frame} to {self.namespaced_target_frame}: {ex}')
            return
        
        new_points = []
        for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z')):
            pt = Point()
            pt.x, pt.y, pt.z = float(x), float(y), float(z)

            new_pt = transform_point(transform, pt)
            new_points.append((new_pt.x, new_pt.y, new_pt.z))
        
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.target_frame
        pc_relative = pc2.create_cloud_xyz32(header=header, points=new_points)
        self.point_cloud_pub.publish(pc_relative)

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
