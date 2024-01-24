from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel

def camera_model():
    """
    Create and configure a PinholeCameraModel.

    This function initializes a CameraInfo object with specific parameters like height, width,
    distortion model, and camera matrix values. It then uses this information to create and return
    a configured PinholeCameraModel object.

    Returns:
        PinholeCameraModel: Configured camera model object.
    """
    
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
    
    return camera_model