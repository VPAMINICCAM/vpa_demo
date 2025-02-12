#!/usr/bin/env python3

import rospy
import socket
from planning.trajectory_generation import generate_trajectory_turn
from middleware.pure_pursuit import PurePursuitController
from std_msgs.msg import Bool,Int32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

class State:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

class PurePursuitTurnNode:
    def __init__(self):
        rospy.init_node('pure_pursuit_turn_node')

        self.robot_name = socket.gethostname()

        self.loop_freq = 40
        self.running = False

        self.trajectory_points = []
        self.already_generated = False
        self.sub_turn = rospy.Subscriber('turn_direction', Int32, self.turn_callback)

        self.cur_time = 0
        
        self.x = 0
        self.y = 0
        self.theta = 0

        self.initial_pose_received = False
        # Wait for the initial pose to be received
        initial_pose_msg = rospy.wait_for_message('initial_pose', Odometry)
        self.initial_pose_callback(initial_pose_msg)
        rospy.loginfo('%s: Initial pose received', self.robot_name)

        self.lookahead_time = 0.5
        self.max_speed      = 1.0
        self.controller     = PurePursuitController(self.trajectory_points, self.lookahead_time, self.max_speed)

        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_callback)

        self.sub_start = rospy.Subscriber('start_turn', Bool, self.start_callback)

        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def initial_pose_callback(self, msg):
        # decode odom message to (x,y,theta)
        # this function should only work once
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def odom_callback(self, msg):
        # Update current position based on odometry
        self.x += msg.pose.pose.position.x
        self.y += msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, delta_theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.theta += delta_theta

    def start_callback(self, msg):
        self.running = msg.data

    def turn_callback(self, msg):
        direction = msg.data
        if not self.already_generated:
            rospy.loginfo('%s: Generating trajectory',self.robot_name)
            self.trajectory_points = generate_trajectory_turn(direction)
            self.already_generated = True
    
    def reset(self):
        self.cur_time = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.trajectory_points = []
        self.already_generated = False

    def run(self):
        rate = rospy.Rate(self.loop_freq)  # 40Hz
        # the odem is 20Hz, this makes the control reacts to its updates fast
        while not rospy.is_shutdown():
            if self.running and len(self.trajectory_points) > 0:
                # Add your control logic here
                cur_state = State(self.x, self.y, self.theta)
                vx,wz = self.controller.compute_control(state=cur_state, current_time=self.cur_time)
                twist = Twist()
                twist.linear.x = vx
                twist.angular.z = wz

                
                # Stop criterion: if the robot is close enough to the last point in the trajectory
                last_point = self.trajectory_points[-1]
                distance_to_goal = ((self.x - last_point[0]) ** 2 + (self.y - last_point[1]) ** 2) ** 0.5
                if distance_to_goal < 0.1:  # Threshold distance to stop
                    rospy.loginfo('%s: Reached the goal', self.robot_name)
                    self.running = False
                    twist.linear.x = 0
                    twist.angular.z = 0
                    
                self.pub_cmd.publish(twist)
                self.cur_time += 1.0 / self.loop_freq

            else:
                self.reset()
            rate.sleep()

if __name__ == '__main__':
    node = PurePursuitTurnNode()
    try:
        node.run()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass