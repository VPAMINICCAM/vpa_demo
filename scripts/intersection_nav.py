#!/usr/bin/env python3

import rospy
import numpy as np
import socket
import math
from std_msgs.msg import Bool,Int32
from sensor_msgs.msg import Image
# from middleware.apriltag_pose import get_camera_pose_in_base
from middleware.apriltag_pose_dt import get_robot_x_y_theta
from nav_msgs.msg import Odometry

def yaw_to_quaternion(yaw_angle_rad):
    # Since roll = pitch = 0, the quaternion conversion simplifies:
    qz = math.sin(yaw_angle_rad / 2.0)
    qw = math.cos(yaw_angle_rad / 2.0)
    return [0.0, 0.0, qz, qw]

# def extract_position_and_yaw(T):
#     """ Extract position and yaw angle from transformation matrix T """
#     position = T[:3, 3]  # ✅ Always take the last column

#     yaw = math.atan2(T[1, 0], T[0, 0])  # Extract yaw from rotation matrix
#     return position, yaw

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
        self.turn_pub = rospy.Publisher('turn_direction', Int32, queue_size=1)

        # flag allowing the robot to start turning
        self.start_turn = False
        self.start_turn_pub = rospy.Publisher('start_turn', Bool, queue_size=1)

        # finish turn
        self.is_turn_finish = False
        self.fin_sub = rospy.Subscriber('turn_end', Bool, self.fin_callback)
        # publish initial pose for pure pursuit
        self.odom_pub = rospy.Publisher('initial_pose', Odometry, queue_size=1)
        
        self.found_ref_tag = False

        # start odom
        self.control_odom_pub = rospy.Publisher('control_odometry',Bool,queue_size=1)

        self.ref_tag_id = 200
        self.ref_tag_size = 0.064

        # ✅ Use the new pose estimator class

        # self.T_reftag_to_inter = np.array([
        #     [ 0, -1, 0, 0   ],
        #     [-1,  0, 0, 0.06],
        #     [ 0,  0, 1, 0   ],
        #     [ 0,  0, 0, 1   ]
        # ])
        # loginfo ready
        rospy.loginfo("%s: IntersectionNav node ready", self.robot_name)
        # run
        self.run()

    def fin_callback(self,msg):
        if self.is_intersection:
            self.is_turn_finish = msg.data
            if self.is_turn_finish:
                rospy.loginfo('%s: Inter Nav complete, sleeping...',self.robot_name)
                self.control_odom_pub.publish(Bool(False))
                self.start_turn_pub.publish(Bool(False))
                self.is_intersection    = False
                self.is_turn_finish     = False


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
                if not self.found_ref_tag:
                    self.image = rospy.wait_for_message('robot_cam/image_raw', Image)
        
                    attempt_count += 1
                    try:
                        x, y, theta = get_robot_x_y_theta(self.image)
                        self.found_ref_tag = True
                        self.turn_pub.publish(Int32(self.turn_index))
                        rospy.loginfo("%s: Robot position: x=%.2f, y=%.2f", self.robot_name, x,y)
                        rospy.loginfo("%s: Robot yaw: %.2f degrees", self.robot_name, math.degrees(theta))


                        # Publish the initial pose for pure pursuit
                        odom_msg = Odometry()
                        odom_msg.pose.pose.position.x = x
                        odom_msg.pose.pose.position.y = y
                        odom_msg.pose.pose.position.z = 0
                        quat = yaw_to_quaternion(theta)
                        odom_msg.pose.pose.orientation.x = quat[0]
                        odom_msg.pose.pose.orientation.y = quat[1]
                        odom_msg.pose.pose.orientation.z = quat[2]
                        odom_msg.pose.pose.orientation.w = quat[3]
                        rospy.sleep(0.2)
                        self.odom_pub.publish(odom_msg)

                        self.control_odom_pub.publish(Bool(True))
                        rospy.sleep(0.2)
                        # Set the start_turn flag to True and publish it
                        self.start_turn_pub.publish(Bool(True))
                        rospy.loginfo('%s: InterNav: OK to Turn', self.robot_name)
                        attempt_count = 0
                        break
                    except ValueError as e:
                        rospy.logwarn("%s: AprilTag detection failed: %s", self.robot_name, str(e))

                if not self.found_ref_tag:
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