import math
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        camera_info = CameraInfo()

        camera_info.height = 480
        camera_info.width = 640

        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        camera_info.k = [530.4669406576809, 0.0, 320.5, 0.0, 530.4669406576809, 240.5, 0.0, 0.0, 1.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [530.4669406576809, 0.0, 320.5, -0.0, 0.0, 530.4669406576809, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]

        camera_info.binning_x = 0
        camera_info.binning_y = 0
        camera_info.roi.x_offset = 0
        camera_info.roi.y_offset = 0
        camera_info.roi.height = 0
        camera_info.roi.width = 0
        camera_info.roi.do_rectify = False
        
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)
        
        # rectified_pixel = (17, 3)
        # disparity = 84

        # point_3d = camera_model.projectPixelTo3dRay(rectified_pixel)


        # print(point_3d)
        
        actual_radius = 0.075
        actual_diameter = actual_radius * 2
        diameter_pixels = 86
        
        uv = (camera_model.cx() , camera_model.cy() + 3)
        
        scale_factor = actual_diameter / diameter_pixels
        focal_length = camera_model.fx()
        Z_camera = focal_length * scale_factor
        
        camera_pos_gazebo = (0.076, 0.0, 0.093)
        
        ray = camera_model.projectPixelTo3dRay(uv)
        
        X_camera = ray[0] * Z_camera
        Y_camera = ray[1] * Z_camera
        
        X_world = X_camera + camera_pos_gazebo[0]
        Y_world = Y_camera + camera_pos_gazebo[1]
        Z_world = Z_camera + camera_pos_gazebo[2]
        
        print("Real-world X, Y, and Z coordinates in Gazebo:", X_world, Y_world, Z_world)
        
        
        def pixel_to_normalized(uv, image_width, image_height):
            u, v = uv

            # Normalize u and v coordinates
            u_normalized = (u - (image_width / 2)) / (image_width / 2)
            v_normalized = (v - (image_height / 2)) / (image_height / 2)

            return u_normalized, -v_normalized  # Negate v to convert from image to Cartesian coordinates

        def normalize_apparent_size(apparent_size_pixels, image_width, image_height):
            # Use the diagonal of the image as a reference for normalization
            diagonal = math.sqrt(image_width ** 2 + image_height ** 2)
            normalized_size = apparent_size_pixels / diagonal

            return normalized_size

        normalized_uv = pixel_to_normalized(uv, camera_model.width, camera_model.height)
        normalized_diameter = normalize_apparent_size(diameter_pixels, camera_model.width, camera_model.height)
        
        print(f"Normalized UV: {normalized_uv}; Normalized Diameter: {normalized_diameter}")
        
        # More calculations
        # Sample input data (replace with actual data from your subscription)
        data_x = normalized_uv[0] # Normalized X coordinate in the image
        data_y = normalized_uv[1]  # Normalized Y coordinate in the image
        data_z = normalized_diameter   # Apparent size of the ball in pixels

        # Camera and ball parameters (set these values)
        h_fov = 1.085595  # Horizontal field of view of the camera in radians
        v_fov = h_fov / (4.0 / 3.0)  # Vertical field of view (based on aspect ratio)
        ball_radius = actual_radius  # Actual radius of the ball in meters

        # Calculate angular size and distance
        ang_size = data_z * h_fov
        d = ball_radius / math.atan(ang_size / 2)

        # Calculate angular and distance deviations in X and Y
        y_ang = data_y * v_fov / 2
        y = d * math.sin(y_ang)
        d_proj = d * math.cos(y_ang)

        x_ang = data_x * h_fov / 2
        x = d_proj * math.sin(x_ang)
        z = d_proj * math.cos(x_ang)

        # Adjust for camera position in Gazebo
        X_world = x + camera_pos_gazebo[0]
        Y_world = y + camera_pos_gazebo[1]
        Z_world = z + camera_pos_gazebo[2]

        print("Calculated 3D Position in Gazebo World Frame:", X_world, Y_world, Z_world)

        
        print("END")
        os._exit(0)




    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()

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