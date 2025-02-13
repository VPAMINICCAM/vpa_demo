#!/usr/bin/env python3

import rospy
import numpy as np
import socket
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from middleware.apriltag_pose import get_camera_pose_in_base
from nav_msgs.msg import Odometry

class IntersectionNav:
    # Initialize IntersectionNav class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('intersection_nav', anonymous=False)
        self.robot_name = socket.gethostname()
        # Subscribe to a trigger, this is called when the robot reaches an intersection'
        self.is_intersection = False
        rospy.Subscriber('intersection_trigger', Bool, self.intersection_callback)

        # get turn index from launch file for debug, #TODO: requested from higher planning in real operation
        self.turn_index = rospy.get_param('~turn_index', 0)
        # publish this to pure_pursuit to navigate the intersection
        self.turn_pub = rospy.Publisher('turn_index', Bool, queue_size=1)

        # flag allowing the robot to start turning
        self.start_turn = False
        self.start_turn_pub = rospy.Publisher('start_turn', Bool, queue_size=1)

        # publish initial pose for pure pursuit
        self.odom_pub = rospy.Publisher('initial_pose', Odometry, queue_size=1)

        # define reference mark id = 200, size = 0.064m, coordinates (0.06,0,0)
        self.ref_tag_id = 200
        self.ref_tag_size = 0.064
        self.ref_tag_pose = np.array([0.06, 0, 0])
        # loginfo ready
        rospy.loginfo("%s: IntersectionNav node ready", self.robot_name)
        # run
        self.run()

    def intersection_callback(self, msg):
        # Callback function for intersection trigger
        self.is_intersection = msg.data
        if self.is_intersection:
            rospy.loginfo("%s: Intersection detected, navigating...", self.robot_name)

    def run(self):
        # Main loop
        rate = rospy.Rate(10)
        attempt_count = 0
        while not rospy.is_shutdown():
            if self.is_intersection:
                # get initial pose from apriltag_pose
                # wait_for_message from camera, just one frame. topic name 'robot_cam/image_raw'
                self.image = rospy.wait_for_message('robot_cam/image_raw', Image)
                tag_poses = get_camera_pose_in_base(self.image, tag_size=self.ref_tag_size, cv_debug=False)
                attempt_count += 1
                found_ref_tag = False
                for T_base_to_tag, detection in tag_poses:
                    # get id from detection
                    id = detection.tag_id
                    # check if id is the reference tag
                    if id == self.ref_tag_id:
                        found_ref_tag = True
                        # this is the tag mark
                        # Calculate the pose of the robot (base) relative to the reference tag
                        tag_pose = T_base_to_tag[:3, 3]
                        robot_pose = tag_pose - self.ref_tag_pose

                        # Log the calculated robot pose (x, y)
                        rospy.loginfo("%s: Robot pose calculated: (x: %f, y: %f)", self.robot_name, robot_pose[0], robot_pose[1])

                        # Publish the initial pose for pure pursuit
                        odom_msg = Odometry()
                        odom_msg.pose.pose.position.x = robot_pose[0]
                        odom_msg.pose.pose.position.y = robot_pose[1]
                        odom_msg.pose.pose.position.z = robot_pose[2]
                        self.odom_pub.publish(odom_msg)

                        # Publish the turn index
                        self.turn_pub.publish(Bool(self.turn_index))

                        # Set the start_turn flag to True and publish it
                        self.start_turn = True
                        self.start_turn_pub.publish(Bool(self.start_turn))
                        attempt_count = 0
                        self.is_intersection = False
                        break

                if not found_ref_tag:
                    rospy.logwarn("%s: Reference tag ID %d not found", self.robot_name, self.ref_tag_id)
                if attempt_count > 10:
                    rospy.logwarn("%s: Reference tag not found after 10 attempts", self.robot_name)
                    self.is_intersection = False
                    attempt_count = 0
                    # Signal shutdown
                    rospy.signal_shutdown("Reference tag not found after 10 attempts")
                    break

            rate.sleep()

if __name__ == '__main__':
    intersection_nav = IntersectionNav()
    rospy.spin()