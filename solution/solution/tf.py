import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, Buffer, TransformListener

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_transform_listener')

        # Set the target frame to robot1/camera_link and to_frame to robot1/base_link
        self.from_frame = 'robot1/camera_link'
        self.to_frame = 'robot1/base_link'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            # Look up for the transformation between from_frame and to_frame
            transform = self.tf_buffer.lookup_transform(
                self.to_frame,
                self.from_frame,
                rclpy.time.Time())

            # Log the transform
            self.get_logger().info(f'Transform: {transform}')

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.to_frame} to {self.from_frame}: {ex}')

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
