#!/usr/bin/env python3
import rospy
# Global variables
err_integral_acc = 0  # Integral error initialized
last_updated_acc = 0         
def acc_pi_control(ref, sig):

    global err_integral_acc,last_updated_acc  # Declare the use of the global variable

    kp = -12  # Proportional gain
    ki = 0   # Integral gain

    if last_updated_acc == 0:
        # first call
        last_updated_acc = rospy.get_time()


    if sig > 0.4:
        v_factor = 1
        err_integral_acc = 0  # Reset the integral component when signal is high
    else:
        err = sig - ref
        err_integral_acc += err * (rospy.get_time() - last_updated_acc)  # Update integral of error

        # Calculate the velocity factor with PI control
        v_factor = 1 - (err * kp + err_integral_acc * ki)

    last_updated_acc = rospy.get_time()

        # Clamp v_factor to be between 0.1 and 1
    if v_factor > 1:
        v_factor = 1
    elif v_factor < 0.1:
        v_factor = 0
        
    return v_factor