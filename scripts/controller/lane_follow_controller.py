#!/usr/bin/env python3
import rospy
err_intergal = 0
last_time_called = 0

def lane_pi_control(ref,sig):
    
    # kp: the p gain for this controller
    # ki: the i gain for this controller

    global err_intergal
    global last_time_called

    if last_time_called == 0:
        # first time called
        t_gap = 0
    else:
        t_gap = rospy.get_time() - last_time_called
    last_time_called = rospy.get_time()
    
    kp = 5
    ki = 0
    vf = 0.35
    vs = 0.35
    
    err = (ref - sig)/ref

    err_intergal += err * t_gap

    if sig/ref > 0.9 and sig/ref < 1.1:
        v_x = vf
        omega_z = 0
    else:
        v_x = vs
        omega_z = err * kp + err_intergal * ki

    return v_x,omega_z