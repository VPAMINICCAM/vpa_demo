import os
import yaml
import cv2
import numpy as np
from pupil_apriltags import Detector

# if you are python 3.8
import pupil_apriltags
package_dir = os.path.dirname(pupil_apriltags.__file__)
parent_dir = os.path.dirname(package_dir)
dll_dir = os.path.join(parent_dir, "pupil_apriltags.libs")

os.add_dll_directory(dll_dir)

try:
    from cv_bridge import CvBridge, CvBridgeError
except (ModuleNotFoundError, ImportError):
    class CvBridge:
        def cv2_to_imgmsg(self, image, encoding="bgr8"):
            # For testing, just return the image itself
            return image
    class CvBridgeError(Exception):
        pass

try:
    from sensor_msgs.msg import Image
except ImportError:
    from unittest.mock import MagicMock
    Image = MagicMock()

def load_camera_intrinsics(yaml_file_path=None):
    if yaml_file_path is None:
        # Default camera parameters (fx, fy, cx, cy) from the demo
        return [305.5718893575089, 308.8338858195428, 303.0797142544728, 231.8845403702499]
    with open(yaml_file_path, 'r') as file:
        cam_data = yaml.safe_load(file)
    # Extract fx, fy, cx, cy from the camera_matrix/data field
    cam_matrix = cam_data['camera_matrix']['data']
    fx = cam_matrix[0]
    fy = cam_matrix[4]
    cx = cam_matrix[2]
    cy = cam_matrix[5]
    return [fx, fy, cx, cy]

def get_camera_pose_in_base(image, base_to_camera_transform=None, yaml_file_path=None, tag_size=0.064, cv_debug=False):
    """
    Given an input image (cv2 format) and optionally a YAML file with camera intrinsics,
    detect one or more AprilTags using pupil_apriltags, and compute each tag’s pose in the 
    robot base frame via the provided base-to-camera transformation.
    
    Parameters:
        image: Input image (if not in grayscale, it will be converted).
        base_to_camera_transform: Optional 4x4 numpy array representing the transformation 
                                  from the robot base to the camera. If None, a default transform is used.
        yaml_file_path: Path to the YAML file with camera intrinsics. If None, default intrinsics are used.
        tag_size: Physical size of the AprilTag in meters.
        cv_debug: If True, assumes the input image is already in OpenCV format.
    
    Returns:
        tag_poses: A list of tuples, each containing:
                   (T_base_to_tag: 4x4 numpy array representing the transformation from the robot base 
                   to the tag, detection: the detection object for the AprilTag).
    """
    bridge = CvBridge()
    # Instead of checking isinstance(image, Image), check if image is a numpy array.
    if not cv_debug:
        if not isinstance(image, np.ndarray):  # if it's not already an OpenCV image
            try:
                image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            except CvBridgeError as e:
                raise ValueError("Error converting ROS image to OpenCV format: {}".format(e))
    else:
        print("cv_debug is enabled: assuming input image is already a valid OpenCV image.")
    
    if base_to_camera_transform is None:
        base_to_camera_transform = np.array([
            [0,           -0.258819045,  0.965925826,  0.0585 ],
            [1,            0,            0.0,          0.0    ],
            [0,           -0.965925826, -0.258819045,  0.0742 ],
            [0.0,          0.0,          0.0,          1.0]    
        ])
    
    # Ensure the image is grayscale
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    # Load camera intrinsics from YAML file
    camera_params = load_camera_intrinsics(yaml_file_path)
    
    # Initialize the AprilTag detector without apriltag_path parameter
    detector = Detector(families="tag36h11", nthreads=1, quad_decimate=1.0, quad_sigma=0.0,
                        refine_edges=1, decode_sharpening=0.25, debug=0)
    
    # Detect AprilTags in the image
    detections = detector.detect(gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)
    
    if len(detections) == 0:
        raise ValueError("No AprilTags detected in the image.")
    
    tag_poses = []
    for detection in detections:
        # Build the transformation from camera to tag for each detection
        T_camera_to_tag = np.eye(4)
        T_camera_to_tag[:3, :3] = np.array(detection.pose_R)
        T_camera_to_tag[:3, 3] = np.array(detection.pose_t).flatten()
        
        # Compute the transformation from base to tag
        T_base_to_tag = base_to_camera_transform @ T_camera_to_tag
        tag_poses.append((T_base_to_tag, detection))
    
    return tag_poses
