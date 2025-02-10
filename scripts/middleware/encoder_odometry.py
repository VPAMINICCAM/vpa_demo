#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Header, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose
from vpa_robot_interface.msg import WheelsEncoder

class EncoderOdometry:
    def __init__(self):
        rospy.init_node('encoder_odometry_node')
        
        # Parameters
        self.ticks_per_rev = 145
        self.wheel_radius = 0.0318  # meters
        self.wheel_base = 0.1  # meters
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = rospy.Time.now()
        self.running = False  # Default state is stopped
        
        # Subscribers
        self.encoder_sub = rospy.Subscriber('wheels_encoder', WheelsEncoder, self.encoder_callback)
        self.control_sub = rospy.Subscriber('control_odometry', Bool, self.control_callback)
        
        # Publisher
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        
        rospy.spin()
    
    def encoder_callback(self, msg):
        if not self.running:
            return
        
        current_time = msg.header.stamp
        dt = (current_time - self.prev_time).to_sec()
        
        # Calculate the distance traveled by each wheel
        delta_left_ticks = msg.left_ticks - self.prev_left_ticks
        delta_right_ticks = msg.right_ticks - self.prev_right_ticks
        distance_left = (2 * math.pi * self.wheel_radius * delta_left_ticks) / self.ticks_per_rev
        distance_right = (2 * math.pi * self.wheel_radius * delta_right_ticks) / self.ticks_per_rev
        
        # Calculate the change in position and orientation
        delta_distance = (distance_left + distance_right) / 2.0
        delta_theta = (distance_right - distance_left) / self.wheel_base
        
        # Update the robot's position and orientation
        self.x += delta_distance * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_distance * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        
        # Normalize theta to the range [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        
        # Create the odometry message
        odom = Odometry()
        odom.header = msg.header
        odom.pose.pose = Pose()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*self.euler_to_quaternion(0, 0, self.theta))
        odom.twist.twist = Twist()
        odom.twist.twist.linear.x = delta_distance / dt
        odom.twist.twist.angular.z = delta_theta / dt
        
        # Publish the odometry message
        self.odom_pub.publish(odom)
        
        # Update previous values
        self.prev_left_ticks = msg.left_ticks
        self.prev_right_ticks = msg.right_ticks
        self.prev_time = current_time
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw
    
    def control_callback(self, msg):
        self.running = msg.data
        if self.running:
            rospy.loginfo("Encoder odometry node has been started.")
        else:
            rospy.loginfo("Encoder odometry node has been stopped.")
    
    def reset_odometry(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = rospy.Time.now()
        self.running = True
        rospy.loginfo("Encoder odometry has been reset.")

if __name__ == '__main__':
    try:
        EncoderOdometry()
    except rospy.ROSInterruptException:
        pass
