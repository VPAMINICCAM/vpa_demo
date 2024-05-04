#!/usr/bin/env python3
import rospy
err_intergal_b = 0
last_time_called_b = 0

def buffferline_pi_control(ref,sig):
    
    # kp: the p gain for this controller
    # ki: the i gain for this controller

    global err_intergal_b
    global last_time_called_b

    if last_time_called_b == 0:
        # first time called
        t_gap = 0
    else:
        t_gap = rospy.get_time() - last_time_called_b

    last_time_called_b = rospy.get_time()

    kp = 4
    ki = 0
    vf = 0.3
    vs = 0.3
    
    err = (ref - sig)/ref

    err_intergal_b += err * t_gap

    if sig/ref > 0.9 and sig/ref < 1.1:
        v_x = vf
        omega_z = 0
    else:
        v_x = vs
        omega_z = err * kp + err_intergal_b * ki

    return v_x,omega_z