#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from enum import Enum

# Sensor Message
from sensor_msgs.msg import Image, Range
# Geometry Message
from geometry_msgs.msg import Twist
#
from std_msgs.msg import Bool
# Open CV library
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Service
from vpa_demo.srv import AssignTask,InterManage

# Map
from map.local_map import local_mapper

# Dynamic reconfiguration
from dynamic_reconfigure.server import Server
from vpa_demo.cfg import color_hsvConfig

from visual.hsv import HSVSpace, from_cv_to_hsv, draw_test_mark_at_center
from visual.search_pattern import search_buffer_line,search_line,search_lane_center,search_inter_guide_line,search_front_car,search_inter_guide_line2

from controller.lane_follow_controller import lane_pi_control
from controller.line_follow_controller import line_pi_control
from controller.acc_controller import acc_pi_control
from controller.buffer_follow_control import buffferline_pi_control


# Define action index
class Direction(Enum):
    GO_STRAIGHT = 0
    LEFT_TURN = 1
    RIGHT_TURN = 2

class Track(Enum):
    BUFFER_AREA = 0 # this is queuing area outside the intersections
    LANE = 1
    INTERSECTION = 2

class LaneDirection(Enum):
    RIGHT_HAND = True
    LEFT_HAND  = False

class VehicleMovement:

    def __init__(self,Nodename:str) -> None:
        
        rospy.init_node(Nodename)

        rospy.on_shutdown(self.shutdown_handler)

        # _test_mode will return the hsv value of the desired point, no vehicles action considered
        self._test_mode      = bool(rospy.get_param('~test_mode',False)) 
        
        # publish mask image for debug
        self._publish_mask   = bool(rospy.get_param('~publish_mask',True))
        
        # simple ACC function on or off, default on
        self._acc_mode       = bool(rospy.get_param('~acc_on',True))

        # get the robot name, by default it will be the hostname of the robot
        self._robot_name     = rospy.get_param('~robot_name','db19')

        self.result_pub      = rospy.Publisher("result_image", Image, queue_size=1)
        self.pub_cmd         = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        
        self._bridge         = CvBridge()

        # init hsv color spaces for selecting 
        self._color_space_init()

        self._request_task_init()  
        self._request_inter_init()
        self._status_flag_init()

        # if self._publish_mask:
        #     self.mask_pub_1 = rospy.Publisher("mask_1_image", Image, queue_size=1)
        #     self.mask_pub_2 = rospy.Publisher("mask_2_image", Image, queue_size=1)
        #     self.mask_pub_3 = rospy.Publisher("mask_stop_image", Image, queue_size=1)
        
        self.last_x = 160
        
        if self._acc_mode:
            self.tof_sub    = rospy.Subscriber("tof_distance",Range,self.acc_dis_cb)
            
            self.distance_acc = 100 # meters, as a sufficently big value for missing object
            self.acc_update_time = 0
        
        self.srv_color    = Server(color_hsvConfig,self.dynamic_reconfigure_callback_hsv)
        self.image_sub    = rospy.Subscriber("robot_cam/image_raw", Image, self._image_cb)
        self._timer       = rospy.Timer(rospy.Duration(0.5),self._timer_cb)
        self.shutdown_sub = rospy.Subscriber("task_finish_shutdown",Bool,self.shut_cb)
        self.task_shut_flag = False

    def shut_cb(self,msg:Bool):
        self.task_shut_flag = msg.data

    def acc_dis_cb(self,msg:Range):
        self.acc_update_time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        self.distance_acc = msg.range

    def dynamic_reconfigure_callback_hsv(self,config,level):
        
        self._lane_hsv_1._h_lower = config.h_lower_1
        self._lane_hsv_1._s_lower = config.s_lower_1
        self._lane_hsv_1._v_lower = config.v_lower_1
        
        self._lane_hsv_1._h_upper = config.h_upper_1
        self._lane_hsv_1._s_upper = config.s_upper_1
        self._lane_hsv_1._v_upper = config.v_upper_1

        self._lane_hsv_2._h_lower = config.h_lower_2
        self._lane_hsv_2._s_lower = config.s_lower_2
        self._lane_hsv_2._v_lower = config.v_lower_2

        self._lane_hsv_2._h_upper = config.h_upper_2
        self._lane_hsv_2._s_upper = config.s_upper_2
        self._lane_hsv_2._v_upper = config.v_upper_2
        
        self._stop_line_hsv._h_lower = config.h_lower_s
        self._stop_line_hsv._s_lower = config.s_lower_s
        self._stop_line_hsv._v_lower = config.v_lower_s
        self._stop_line_hsv._h_upper = config.h_upper_s
        self._stop_line_hsv._s_upper = config.s_upper_s
        self._stop_line_hsv._v_upper = config.v_upper_s  
        
        return config

    def _color_space_init(self) -> None:

        self._lane_hsv_1    = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_1',100)),
            h_l=int(rospy.get_param('~h_lower_1',80)),
            s_u=int(rospy.get_param('~s_upper_1',255)),
            s_l=int(rospy.get_param('~s_lower_1',80)),
            v_u=int(rospy.get_param('~v_upper_1',255)),
            v_l=int(rospy.get_param('~v_lower_1',150))
        ) # HSV space for yellow (center lane line)

        self._lane_hsv_2    = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_2',100)),
            h_l=int(rospy.get_param('~h_lower_2',25)),
            s_u=int(rospy.get_param('~s_upper_2',60)),
            s_l=int(rospy.get_param('~s_lower_2',0)),
            v_u=int(rospy.get_param('~v_upper_2',255)),
            v_l=int(rospy.get_param('~v_lower_2',200))
        ) # HSV space for white (side lane line)

        self._stop_line_hsv = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_s',145)),
            h_l=int(rospy.get_param('~h_lower_s',110)),
            s_u=int(rospy.get_param('~s_upper_s',180)),
            s_l=int(rospy.get_param('~s_lower_s',120)),
            v_u=int(rospy.get_param('~v_upper_s',235)),
            v_l=int(rospy.get_param('~v_lower_s',170))
        )

        # guiding lines inside intersections - no dynamic reconfigure
        self._right_guide_hsv = HSVSpace(140,100,120,80,250,200)
        self._left_guide_hsv  = HSVSpace(160,140,180,80,230,160)
        self._thur_guide_hsv  = HSVSpace(30,0,250,170,230,130)  
    
        self.inter_guide_line = [self._thur_guide_hsv,self._left_guide_hsv,self._right_guide_hsv]
        self._acc_aux_hsv     = HSVSpace(150,110,160,100,255,130)
        self._buffer_line_hsv = HSVSpace(160,125,140,10,240,200)
        self._exit_line_hsv   = HSVSpace(50,20,240,140,220,130)
        
        self.task_line_hsv    = HSVSpace(100,50,255,3,255,150)

    def _request_task_init(self):
        rospy.loginfo("%s: Waiting for task server",self._robot_name)
        rospy.wait_for_service("/AssignTask")
        self._task_proxy = rospy.ServiceProxy("/AssignTask",AssignTask)
        rospy.loginfo("%s: Task server online",self._robot_name)

    def _request_inter_init(self):
        rospy.loginfo("%s: Waiting for intersection server",self._robot_name)
        rospy.wait_for_service("/ManageInter")
        self._inter_proxy = rospy.ServiceProxy("/ManageInter",InterManage) 
        rospy.loginfo('%s: Intersection manager online',self._robot_name)

    def _load_task_list(self,task_list:list):
        if len(task_list) < 3:
            rospy.loginfo('Invalid task list, no task loaded, exiting ...')
            raise ValueError
        else:
            rospy.loginfo('%s: task received',self._robot_name)
            self.last_node = task_list[0] 
            self.this_node = task_list[1]
            self.next_node = task_list[2]
            self.task_list = task_list
            self._next_action = local_mapper(self.last_node,self.this_node,self.next_node)
            
    def _status_flag_init(self):

        self._start_flag    = True      # if at start up phase -> indicating which landmarks to follow

        self._pause_flag    = False     # if the twist cmd being sent to robot (stop car if true)

        self.track_part     = Track.BUFFER_AREA

        self.lane_direction = LaneDirection.RIGHT_HAND 

        # Time lock (prohibit unreasonable status change)
        self.enter_inter_time = 0
        self.left_inter_time  = 0

        # Task
        self.find_task_line = False     # did the robot find the task_line already
        self.task_counter   = 0         # this is to count how many tasks has this specific robot conducted
        self.task_list      = []        # a list of intersections to travel through
        self.ask_task_by_period = False # if the robot will check if there is task at certain period

        self.node_pointer   = 2
        
        # Action
        self._next_action = None        # the next action to perfrom 
        self.inquiry_inter_by_period = False # if teh robot will check if can pass the current intersection
        self.lock_inter_source = True

        self.action_dic = {
            0:'go thur',
            1:'left turn',
            2:'right turn',
            3:'stop'
        }

    def test_mode(self,hsv_image,cv_image) -> None:
        cv_image_drawn = draw_test_mark_at_center(cv_image)
        
        width_half  = int(hsv_image.shape[1]/2)
        height_half = int(hsv_image.shape[0]/2)

        rospy.loginfo("Point HSV Value is %s"%hsv_image[height_half,width_half])

        return cv_image_drawn
    
    def _apply_next_action(self):
        self.node_pointer += 1
        if self.node_pointer == len(self.task_list):
            # this is the last node
            self._next_action = None
        else:
            self.last_node    = self.this_node
            self.this_node    = self.next_node
            self.next_node    = self.task_list[self.node_pointer]
            self._next_action = local_mapper(self.last_node,self.this_node,self.next_node)

    def _request_task_service(self,task_index) -> list:
        rospy.loginfo_once('%s: arrive ready line, inquiring task',self._robot_name)
        resp = self._task_proxy(self._robot_name,task_index)
        return resp.node_list
    
    def _request_inter_service(self,_robot_name:str,_next_action:int,_cur_node:int,_last_node:int,_is_release:bool) -> bool:
        _resp = self._inter_proxy(_robot_name,_next_action,_cur_node,_last_node,_is_release)
        return _resp.pass_flag

    def _publish_image(self,pub_name,image,_is_mask):
        if _is_mask:
            img_msg = self._bridge.cv2_to_imgmsg(image, encoding="passthrough")
        else:
            img_msg = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        img_msg.header.stamp = rospy.Time.now()
        pub_name.publish(img_msg)

    def _sent_twist_cmd(self,vx,oz,vf):
        cmd = Twist()
        if self._pause_flag:
            cmd.linear.x = 0
            cmd.angular.z = 0
        else:
            cmd.linear.x = vx * vf
            cmd.angular.z = oz * vf
        self.pub_cmd.publish(cmd)

    def _timer_cb(self,_):
        
        if self.inquiry_inter_by_period:
            _pass = self._request_inter_service(self._robot_name,self._next_action,self.this_node,self.last_node,False)
            if _pass:
                self._pause_flag        = False
                self.inquiry_inter_by_period = False
                self.lock_inter_source  = True
        
        if self.ask_task_by_period:
            resp = self._request_task_service(self.task_counter)
            self.arrive_task_line = rospy.get_time()
            if not len(resp) == 0:
                self._load_task_list(resp)
                self.ask_task_by_period = False
                self._pause_flag = False

    def _image_cb(self,data:Image) -> None:
        
        # the function is supposed to be called at about 10Hz based on fps settins
        
        try:
            cv_image    = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # the picture has been converted to opencv image format
        acc_image   = cv_image[0:int(cv_image.shape[0]/4),:]
        cv_image    = cv_image[int(cv_image.shape[0]/4):cv_image.shape[0],:] 
        # removing the upper 25% of the image when processing
        acc_hsv_image = from_cv_to_hsv(acc_image)
        hsv_image = from_cv_to_hsv(cv_image)
        # convert cv image to the hsv image

        if self._test_mode:
            cv_image = self.test_mode(hsv_image,cv_image)
            # this function creates a mark at the center of the picture (sliced)
            # it also print out the hsv value
        else:
            if self.track_part == Track.BUFFER_AREA:
                # the robot is at the area of buffer queue
                mask_buffer = self._buffer_line_hsv.apply_mask(hsv_image)
                line_x,line_y = search_buffer_line(mask_buffer) # this is assumed to be the targeting direction

                if not line_x == None:
                    # the line is found, and indicate the segement found
                    cv2.circle(cv_image, (line_x,line_y), 5, (255,100,0), 5)
                    self.last_x = line_x
                else:
                    line_x = self.last_x

                if self.find_task_line:
                    # the robot has reached the ready line
                    # It will gradually approach an Intersection stop line
                    dis2exit = search_line(hsv_image,self._lane_hsv_2)
                    if dis2exit > 25:
                        # The robot is sufficently close to the intersection exit (in a lane)
                        if rospy.get_time() - self.arrive_task_line > 2:
                            # there is enough gap between last status change
                            self.enter_inter_time = rospy.get_time() # update time
                            self.track_part = Track.INTERSECTION
                            if not self.inquiry_inter_by_period:
                                _pass = self._request_inter_service(
                                    self._robot_name,self._next_action,self.this_node,self.last_node,False
                                    ) # to ask for and occupy the intersection resources
                                if not _pass:
                                    self._pause_flag = True
                                    self.inquiry_inter_by_period = True #
                                else:
                                    # flag: if the robot has occupied the resource
                                    self.lock_inter_source = True
                                    rospy.loginfo("%s: entering Intersection %s, action is %s",self._robot_name,str(self.this_node),self.action_dic[self._next_action])
                        else:
                            rospy.logwarn("Self status invalid switch, buffer exit, ignoring...")
                else:
                    # check ready line
                    dis2ready = search_line(hsv_image,self._lane_hsv_1)
                    if dis2ready > 25:
                        # the robot is close enough to the ready area
                        self.find_task_line = True # Toggle the status
                        resp = self._request_task_service(self.task_counter)
                        if len(resp) == 0:
                            self.arrive_task_line = rospy.get_time()
                            # empty task returned
                            # this may be either the task assignment requries the robot to wait
                            # or there is no more mission -> there is going to be a signal shutdown
                            self.ask_task_by_period = True
                            self._pause_flag        = True # stop the robot
                        else:
                            self.arrive_task_line = rospy.get_time()
                            self._load_task_list(resp)

            elif self.track_part == Track.LANE:
                # the robot is in the lane,
                # searching for lane boundaries
                
                line_x = search_lane_center(self._lane_hsv_1,self._lane_hsv_2,hsv_image,self.lane_direction)
                cv2.circle(cv_image, (line_x,int(hsv_image.shape[0]/2)), 5, (0,255,0), 5)

                dis2stop = search_line(hsv_image,self._stop_line_hsv)
                if dis2stop > 30:
                    # the robot is close enough to a stop line
                    if rospy.get_time() - self.left_inter_time > 0.2:
                        self.enter_inter_time = rospy.get_time()
                        if not self._next_action == None:
                            self.track_part = Track.INTERSECTION
                            if not self.inquiry_inter_by_period:
                                # attempt to communicate once entering the intersection
                                _pass = self._request_inter_service(
                                    self._robot_name,self._next_action,self.this_node,self.last_node,False
                                    ) # to ask for and occupy the intersection resources
                                if not _pass:
                                    # not approved for passing
                                    self._pause_flag = True
                                    self.inquiry_inter_by_period = True # switch the ask permission later
                                else:
                                    # flag: if the robot has occupied the resource
                                    self.lock_inter_source = True
                                    rospy.loginfo("%s: entering Intersection %s, action is %s",self._robot_name,str(self.this_node),self.action_dic[self._next_action])
                        else:
                            rospy.loginfo("%s: exiting track, back to buffer")
                            self.track_part = Track.BUFFER_AREA
                    else:
                        rospy.logwarn("Self status invalid switch, from Green -> Red, ignoring...")
            elif self.track_part == Track.INTERSECTION:
                # the robot is in the intersection
                if self._next_action == None:
                    print(self.last_node,self.this_node,self.next_node)
                    rospy.signal_shutdown('shutdown due to error next action')
                line_x = search_inter_guide_line2(self.inter_guide_line[self._next_action],hsv_image,self._next_action)
                if line_x == None:
                    line_x = self.last_x
                else:
                    self.last_x = line_x
                if not line_x == None:
                    cv2.circle(cv_image, (int(line_x),int(hsv_image.shape[0]/2)), 5, (255,255,0), 5)
                else:
                    rospy.logwarn('Invaild return of line_x')
                # also search for intersection exit line
                dis2exit = search_line(hsv_image,self._exit_line_hsv)
                if dis2exit > 20:
                    # the robot is sufficently close to exit the intersection
                    if rospy.get_time() - self.enter_inter_time > 0.2:
                        # leave intersections
                        self.left_inter_time = rospy.get_time()
                        if self.lock_inter_source:
                            # to release the intersection resource
                            _pass = self._request_inter_service(self._robot_name,self._next_action,self.this_node,self.last_node,True)
                            self.lock_inter_source = False
                            self._apply_next_action()
                            rospy.loginfo('%s: exiting Intersection %s',self._robot_name,str(self.last_node))
                            if self._next_action == None:
                                rospy.loginfo('%s: return to buffer area',self._robot_name)
                                self.find_task_line = False
                                self.node_pointer = 2 # reset
                                self.track_part = Track.BUFFER_AREA
                                self.task_counter += 1
                            else:
                                self.track_part = Track.LANE
                    else:
                        rospy.logwarn("Self status invalid switch, from Red -> Green, ignoring...")
            else:
                rospy.logwarn('Unknown track part index: %s',str(self.track_part))
                line_x = None
            

            if self.track_part == Track.LANE:
                # call lane follow controller
                v_x, omega_z = lane_pi_control(int(hsv_image.shape[1]/2),line_x)
            elif self.track_part == Track.INTERSECTION:
                v_x, omega_z = line_pi_control(int(hsv_image.shape[1]/2),line_x)
            else:
                v_x, omega_z = buffferline_pi_control(int(hsv_image.shape[1]/2),line_x)


            # since the cmd is sending on a higher frequency than ACC msg, we estimate the distance to further avoid collsions
            if self._acc_mode:
                vis_dis2car = search_front_car(acc_hsv_image,self._acc_aux_hsv)
                if not vis_dis2car == None:
                    delta_last_acc = rospy.get_time() - self.acc_update_time
                    dis_est  = self.distance_acc - delta_last_acc * v_x 
                    v_factor = acc_pi_control(0.35,dis_est)
                    cv2.line(cv_image,(100,90),(220,90),(0,0,0),2)
                else:
                    v_factor = 1
            else:
                v_factor = 1
            
            
            self._sent_twist_cmd(v_x,omega_z,v_factor)

            # after a reconsideration, the check mask should not be locally, but we check on a remote machine with more graphical resources
            # never publish mask from robot side

            self._publish_image(self.result_pub,cv_image,False)

            if self.task_shut_flag:
                # the task commander thinks no more task to offer
                if self.track_part == Track.BUFFER_AREA and self._pause_flag:
                    # stopped at pause line -> no guanrantee in position
                    _pub = rospy.Publisher('robot_interface_shutdown',Bool,queue_size=1)
                    msg = Bool()
                    msg.data = True
                    _pub.publish(msg)
                    rospy.signal_shutdown('Task finish flag set to True, shutting down the node.')
                if self.track_part == Track.BUFFER_AREA and v_factor == 0:
                    # stopped by ACC fully -> no guanrantee in position
                    _pub = rospy.Publisher('robot_interface_shutdown',Bool,queue_size=1)
                    msg = Bool()
                    msg.data = True
                    _pub.publish(msg)
                    rospy.signal_shutdown('Task finish flag set to True, shutting down the node.')
                

    def shutdown_handler(self):
        _cmd = Twist()
        self.pub_cmd.publish(_cmd)
        print("Node is shutting down, closing resources...")

if __name__ == "__main__":

    T = VehicleMovement(Nodename='Lane_operation')
    rospy.spin()