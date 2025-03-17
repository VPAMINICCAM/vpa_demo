import os
import yaml
import cv2
import numpy as np
import math
from dt_apriltags import Detector

try:
    from cv_bridge import CvBridge, CvBridgeError
except (ModuleNotFoundError, ImportError):
    class CvBridge:
        def cv2_to_imgmsg(self, image, encoding="bgr8"):
            return image  # For testing, just return the image itself

    class CvBridgeError(Exception):
        pass

try:
    from sensor_msgs.msg import Image
except ImportError:
    from unittest.mock import MagicMock
    Image = MagicMock()

def extract_position_and_yaw(T):
    """ Extract position and yaw angle from transformation matrix T """
    position = T[:3, 3]  # ✅ Always take the last column

    yaw = math.atan2(T[1, 0], T[0, 0])  # Extract yaw from rotation matrix
    return position, yaw

def load_camera_intrinsics(yaml_file_path=None):
    if yaml_file_path is None:
        # Default camera parameters (fx, fy, cx, cy)
        return [305.5718893575089 / 2, 308.8338858195428 / 2, 303.0797142544728 / 2, 231.8845403702499 / 2]

    with open(yaml_file_path, 'r') as file:
        cam_data = yaml.safe_load(file)

    cam_matrix = cam_data['camera_matrix']['data']
    fx, fy, cx, cy = cam_matrix[0], cam_matrix[4], cam_matrix[2], cam_matrix[5]
    return [fx, fy, cx, cy]

def get_camera_pose_in_base(image, base_to_camera_transform=None, yaml_file_path=None, tag_size=0.064, cv_debug=False):
    """
    Detects AprilTags using dt_apriltags and computes their pose relative to the robot base.
    """
    bridge = CvBridge()

    if not cv_debug:
        if not isinstance(image, np.ndarray):
            try:
                image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            except CvBridgeError as e:
                raise ValueError(f"Error converting ROS image to OpenCV format: {e}")
    else:
        print("cv_debug enabled: assuming input image is an OpenCV image.")

    if base_to_camera_transform is None:
        base_to_camera_transform = np.array([
            [0, -0.258819045, 0.965925826, 0.0585],
            [1, 0, 0.0, 0.0],
            [0, -0.965925826, -0.258819045, 0.0742],
            [0.0, 0.0, 0.0, 1.0]
        ])

    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    

    camera_params = load_camera_intrinsics(yaml_file_path)

    detector = Detector(families="tag36h11", nthreads=1, quad_decimate=1.0,
                        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)

    detections = detector.detect(gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)

    if not detections:
        raise ValueError("No AprilTags detected in the image.")

    tag_poses = []
    for detection in detections:
        T_camera_to_tag = np.eye(4)
        T_camera_to_tag[:3, :3] = detection.pose_R
        T_camera_to_tag[:3, 3] = detection.pose_t.flatten()

        T_base_to_tag = base_to_camera_transform @ T_camera_to_tag
        tag_poses.append((T_base_to_tag, detection))

    return tag_poses

def get_robot_x_y_theta(image, t_tag_to_inter=np.array([0, 0.06, 0])):
    """
    Computes the robot's (x, y, theta) in the world frame given an image.

    Parameters:
        - image: The input image for detecting AprilTags.
        - t_tag_to_world: 3D translation of the tag in the world frame (default: [0, 0.06, 0] meters).

    Returns:
        - x (meters)
        - y (meters)
        - theta (radians)
    """
    # Get detected tag pose in the base frame
    tag_poses = get_camera_pose_in_base(image, yaml_file_path=None, cv_debug=False)
   
    if not tag_poses:
        raise ValueError("No valid tag pose detected.")

    # Use the first detected tag
    T_base_to_tag, _ = tag_poses[0]

    # Transformation from tag to world (intersection base)
    R_tag_to_inter = np.array([
        [0, -1, 0],  # World X → Tag -Y
        [-1, 0, 0],  # World Y → Tag -X
        [0,  0, 1]    # World Z → Tag Z
    ])

    T_tag_to_inter = np.eye(4)
    T_tag_to_inter[:3, :3] = R_tag_to_inter
    T_tag_to_inter[:3, 3]  = t_tag_to_inter
    # Compute transformations
    T_base_to_inter = T_base_to_tag @ T_tag_to_inter  
    T_inter_to_base = np.linalg.inv(T_base_to_inter)

    robot_position, robot_yaw = extract_position_and_yaw(T_inter_to_base)
    x = robot_position[0]
    y = robot_position[1]
    return x, y, robot_yaw
