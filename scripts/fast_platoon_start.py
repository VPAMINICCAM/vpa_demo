#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This script is originally design to show a very fast start (no shock wave start of robots)
# It only work with the launch file, the standalone one does not work
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class FastPlatoonStart:
    
    def __init__(self) -> None:
        self.ori_sub = rospy.Subscriber('cmd_vel_raw',Twist,self.repo_cmd)
        self.kick_in = True
        self.kick_in_sub = rospy.Subscriber('/fast_start_flag',Bool,self.kick_cb)
        self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        rospy.loginfo('fast start standby')
    def repo_cmd(self,data:Twist):
        msg = Twist()
        if self.kick_in:
            rever_coff = 0.3/data.linear.x
            
            msg.linear.x = 0.3
            msg.angular.z = data.angular.z * rever_coff
        self.cmd_pub.publish(data)
    
    def kick_cb(self,data:Bool):
        self.kick_in = data.data
        
if __name__ == "__main__":
    rospy.init_node('fast_platoon_start')
    T = FastPlatoonStart()
    rospy.spin()