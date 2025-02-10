#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from vpa_demo.msg import TimeBasedPath, TrajectoryPoint
from geometry_msgs.msg import Twist
import time

class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit_node')
        
        # Load parameters
        self.L_min = rospy.get_param('~L_min', 0.5)
        self.k = rospy.get_param('~k', 1.0)
        self.max_speed = rospy.get_param('~max_speed', 1.0)
        self.max_yaw_rate = rospy.get_param('~max_yaw_rate', 1.0)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.traj_sub = rospy.Subscriber('trajectory', TimeBasedPath, self.traj_callback)
        
        # Publisher
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        self.trajectory = []
        self.current_index = 0  # Track the current index in the trajectory
        self.x_r, self.y_r, self.theta_r, self.v = 0.0, 0.0, 0.0, 0.0
        self.start_time = time.time()
        
        self.control_loop()
    
    def traj_callback(self, msg):
        # Ensure the trajectory is properly read by checking the message type and content
        if isinstance(msg, TimeBasedPath):
            self.trajectory = [(wp.position.x, wp.position.y, self.quaternion_to_euler(wp.orientation)[2], wp.time_from_start) for wp in msg.points]
            self.current_index = 0  # Reset the current index when a new trajectory is received
            self.start_time = time.time()  # Reset the start time
        else:
            rospy.logwarn("Received message is not of type TimeBasedPath")
    
    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.x_r = pose.position.x
        self.y_r = pose.position.y
        _, _, self.theta_r = self.quaternion_to_euler(pose.orientation)
        self.v = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
    
    def quaternion_to_euler(self, orientation):
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy**2 + qz**2)
        return 0.0, 0.0, math.atan2(siny_cosp, cosy_cosp)  # roll, pitch, yaw
    
    def compute_L_d(self):
        return max(self.L_min, self.k * self.v)
    
    def find_pursuit_point(self):
        if not self.trajectory:
            return None
        
        L_d = self.compute_L_d()
        for i in range(self.current_index, len(self.trajectory)):
            x_p, y_p, theta_p, t_p = self.trajectory[i]
            if math.sqrt((x_p - self.x_r)**2 + (y_p - self.y_r)**2) >= L_d:
                self.current_index = i  # Update the current index to the next point
                return x_p, y_p, theta_p, t_p
        return self.trajectory[-1]
    
    def compute_cmd_vel(self):
        pursuit_point = self.find_pursuit_point()
        if pursuit_point is None:
            return None
        
        x_p, y_p, theta_p, t_p = pursuit_point
        distance_to_pursuit = math.sqrt((x_p - self.x_r)**2 + (y_p - self.y_r)**2)
        current_time = time.time() - self.start_time
        time_to_pursuit = t_p - current_time
        
        if time_to_pursuit <= 0:
            linear_speed = 0
        else:
            linear_speed = min(self.max_speed, distance_to_pursuit / time_to_pursuit)
        
        # Ensure the linear speed does not exceed the maximum speed
        linear_speed = min(linear_speed, self.max_speed)
        
        theta_d = math.atan2(y_p - self.y_r, x_p - self.x_r)
        omega = (2 * self.v * math.sin(theta_d - self.theta_r)) / self.compute_L_d()
        
        # Clamp omega
        omega = max(-self.max_yaw_rate, min(self.max_yaw_rate, omega))
        
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = omega
        return cmd
    
    def control_loop(self):
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            cmd = self.compute_cmd_vel()
            if cmd:
                self.cmd_pub.publish(cmd)
            rate.sleep()
    
if __name__ == '__main__':
    try:
        PurePursuit()
    except rospy.ROSInterruptException:
        pass
