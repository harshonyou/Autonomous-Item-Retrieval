import numpy as np
import math

def uv_to_depth_array(u, diameter_pixels, z_value, image_width, h_fov):
    """
    Convert image coordinates to a depth array.

    This function takes image coordinates and other camera parameters to create a depth array.
    It calculates the array length based on the horizontal field of view and assigns Z values to corresponding indices.

    Args:
        u (int): The u-coordinate in the image.
        diameter_pixels (float): Diameter of the object in pixels.
        z_value (float): The depth value to be assigned in the array.
        image_width (int): Width of the image in pixels.
        h_fov (float): Horizontal field of view in radians.

    Returns:
        numpy.ndarray: The depth array with assigned depth values.
    """
    
    # Convert horizontal field of view from radians to degrees
    h_fov_deg = math.degrees(h_fov)
    # Calculate the length of the depth array based on the horizontal field of view
    depth_array_length = int(h_fov_deg)
    # Initialize the depth array with infinite values
    depth_array = np.full(depth_array_length, float('inf'))

    # Calculate the radius of the object in pixels
    radius_pixels = diameter_pixels / 2
    
    # Calculate the start and end pixels of the object in the image
    start_pixel = int(max(u - radius_pixels, 0))
    end_pixel = int(min(u + radius_pixels, image_width))

    # Calculate the angle per pixel based on the horizontal field of view and image width
    angle_per_pixel = h_fov / image_width
    
    # Calculate the start and end angles of the object in the image
    start_angle = math.degrees((start_pixel - image_width / 2) * angle_per_pixel)
    end_angle = math.degrees((end_pixel - image_width / 2) * angle_per_pixel)

    # Calculate the start and end indices in the depth array based on the start and end angles
    start_index = int((start_angle + h_fov_deg / 2))
    end_index = int((end_angle + h_fov_deg / 2))

    # Assign the depth value to the corresponding indices in the depth array
    depth_array[start_index:end_index] = z_value

    # Return the depth array
    return depth_array