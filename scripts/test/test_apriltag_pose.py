import os
import sys
import cv2
import unittest
from unittest.mock import MagicMock

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

sys.path.append(os.path.join(os.path.dirname(__file__), '../middleware'))
from apriltag_pose import get_camera_pose_in_base

class TestAprilTagPose(unittest.TestCase):
    def setUp(self):
        # Load a test image from file (adjust the path as needed)
        image_path = os.path.join(os.path.dirname(__file__), 'images', 'test_image.png')
        self.test_image = cv2.imread(image_path)
        self.assertIsNotNone(self.test_image, "Test image could not be loaded.")
        
        # Create a dummy ROS image message using CvBridge
        bridge = CvBridge()
        try:
            self.dummy_ros_image = bridge.cv2_to_imgmsg(self.test_image, encoding="bgr8")
        except CvBridgeError as e:
            self.fail("CvBridge conversion failed: {}".format(e))
    
    def test_with_cv_debug_true(self):
        # Test assuming the image is already an OpenCV image
        try:
            tag_poses = get_camera_pose_in_base(self.test_image, cv_debug=True)
            for T_base_to_tag, detection in tag_poses:
                print("Tag ID:", detection.tag_id)
                print("Transformation Matrix:\n", T_base_to_tag)
        except Exception as e:
            self.fail("get_camera_pose_in_base failed with cv_debug=True: {}".format(e))
        
        self.assertIsInstance(tag_poses, list, "Output should be a list of tag poses.")
        for T_base_to_tag, detection in tag_poses:
            self.assertEqual(T_base_to_tag.shape, (4, 4), "Transformation matrix must be 4x4.")
    
    def test_with_ros_image(self):
        # Test with a simulated ROS image (conversion will be executed)
        try:
            tag_poses = get_camera_pose_in_base(self.dummy_ros_image, cv_debug=True)
        except Exception as e:
            self.fail("get_camera_pose_in_base failed with ROS image input: {}".format(e))
        
        self.assertIsInstance(tag_poses, list, "Output should be a list of tag poses.")
        for T_base_to_tag, detection in tag_poses:
            self.assertEqual(T_base_to_tag.shape, (4, 4), "Transformation matrix must be 4x4.")

if __name__ == '__main__':
    unittest.main()
