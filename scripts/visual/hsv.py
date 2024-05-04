import cv2
import numpy as np

from sensor_msgs.msg import Image

class HSVSpace:
    
    def __init__(self,h_u,h_l,s_u,s_l,v_u,v_l) -> None:
        self._h_upper = h_u
        self._h_lower = h_l
        self._s_upper = s_u
        self._s_lower = s_l
        self._v_upper = v_u
        self._v_lower = v_l
    
    def _generate_lower_mask(self):
        return np.array([self._h_lower,self._s_lower,self._v_lower])
    
    def _generate_upper_mask(self):
        return np.array([self._h_upper,self._s_upper,self._v_upper])
    
    def apply_mask(self,hsv_image):
        _mask   = cv2.inRange(hsv_image,self._generate_lower_mask(),self._generate_upper_mask())
        _kernel = np.ones((9,9),np.uint8)
        _mask   = cv2.morphologyEx(_mask,cv2.MORPH_CLOSE,_kernel)
        return _mask
    

def from_cv_to_hsv(in_image):
    return cv2.cvtColor(in_image,cv2.COLOR_RGB2HSV)

def draw_test_mark_at_center(cv_pic):
    
    width_select    = int(cv_pic.shape[1]/2)
    height_select   = int(cv_pic.shape[0]/2)

    cv2.circle(cv_pic, (width_select ,height_select), 5, (0,0,255), 1)

    cv2.line(cv_pic,(width_select -10, height_select), (width_select  +10,height_select), (0,0,255), 1)
    cv2.line(cv_pic,(width_select , height_select-10), (width_select , height_select+10), (0,0,255), 1)
    
    return cv_pic  