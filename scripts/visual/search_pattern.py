import numpy as np
from typing import Union
from visual.hsv import HSVSpace
import os


script_dir  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
filepath    = os.path.join(script_dir,'robot_setting')

if os.path.exists(filepath):
    from robot_setting import LEFT_TURN_L,LEFT_TURN_R,RIGHT_TURN_L,RIGHT_TURN_R,THUR_L,THUR_R,LANE_L,LANE_R
    print('Loading customize robot settings')
else:
    # default values
    LEFT_TURN_R     = 240 
    LEFT_TURN_L     = 100
    RIGHT_TURN_R    = 250
    RIGHT_TURN_L    = 170
    THUR_L          = 100
    THUR_R          = 220
    
    LANE_L          = 80
    LANE_R          = 240

def search_buffer_line(mask) -> list:
    low_bound   = 15
    upper_bound = 75

    _height_center = int(mask.shape[0]/2)
    _line_center   = 0
    for i in range(low_bound,upper_bound,15):
        # search this part of the picture
        _line = np.nonzero(mask[_height_center + i,:])[0]
        
        if len(_line) > 8 and len(_line) < 50:
            # there are proper amount of points at this part
            _line_center = int(np.mean(_line))        
            return _line_center,i + _height_center
    
    return None,None
    
def search_line(hsv_image,hsv_space:HSVSpace) -> Union[int, float]:
    mask = hsv_space.apply_mask(hsv_image)
    lower_bound = int(hsv_image.shape[0]/2)
    upper_bound = 2 * lower_bound
    width_center = int(hsv_image.shape[1]/2)
    for i in range(-50,100,50):
        point = np.nonzero(mask[lower_bound:upper_bound,width_center+i])[0]
        if len(point) > 5:
            return len(point)
        else:
            continue # no valid result found
    return 0
def _search_lane_linecenter(_mask,_upper_bias:int,_lower_bias:int,_height_center:int,_interval:int,_width_range_left:int,_width_range_right:int) -> int:
    for i in range(_lower_bias,_upper_bias,_interval):
        point = np.nonzero(_mask[_height_center+i,_width_range_left:_width_range_right])[0] + _width_range_left
        if len(point) > 8 and len(point) < 45:
            _line_center = int(np.mean(point))
            return _line_center
        else:
            continue
    return 0 # nothing found in the end

def search_lane_center(space1:HSVSpace,space2:HSVSpace,hsv_image,is_yellow_left:bool) -> int:
    mask1 = space1.apply_mask(hsv_image)
    mask2 = space2.apply_mask(hsv_image)
    _line_center1 = _search_lane_linecenter(mask1,50,-20,int(hsv_image.shape[0]/2),10,0,int(hsv_image.shape[1]))

    if _line_center1 == 0 and not is_yellow_left:
        # failed to find the center yellow line
        _line_center1 = hsv_image.shape[1]
        
    # search the while line, but limited area
    if is_yellow_left:
        _line_center2 = _search_lane_linecenter(mask2,50,-20,int(hsv_image.shape[0]/2),10,_line_center1,int(hsv_image.shape[1]))
    else:
        _line_center2 = _search_lane_linecenter(mask2,50,-20,int(hsv_image.shape[0]/2),10,0,_line_center1)

    if _line_center2 == 0 and is_yellow_left:
        # miss detections 
        _line_center2 = hsv_image.shape[1]

    _lane_center = int((_line_center1 + _line_center2)/2)
    return max(min(_lane_center,LANE_R),LANE_L)

def search_inter_guide_line(hsv_space:HSVSpace,hsv_image,action:int):
    mask = hsv_space.apply_mask(hsv_image)
    
    # handle the crossing (X) patterns on the guide lines
    height = int(hsv_image.shape[0])
    x_list = []
    if action == 1:
        # left turn
        for i in range(80,170,20):
            y = height - i
            line = np.nonzero(mask[y,:])[0]
            # print(y,line)
            seg = _break_segs(line)

            if len(seg) == 0:
                continue        
            
            x =  np.mean(seg[0])
            x_list.append(x)
        #     y_list.append(y)
        if len(x_list) > 0:
            return max(LEFT_TURN_L,min(int(np.mean(x_list)),LEFT_TURN_R))
        else:
            return None
    elif action == 2:
        # right turn
        for i in range(70,130,15):
            y = i
            line = np.nonzero(mask[y,:])[0]
            seg = _break_segs(line)
            if len(seg) == 1:
                return min(max(RIGHT_TURN_L,int(np.mean(seg[0]))),RIGHT_TURN_R)
        return None
    else: # thur
        width = hsv_image.shape[1]
        y_hl = []
        for x in range(5,width,50):
            y_keep_out_high = height
            y_keep_out_low = 0
            vline = np.nonzero(mask[:,x])[0]
            if len(vline) > 5 and len(vline) < 80:
                y_hl.append(np.mean(vline))
            elif len(vline) >= 80:
                return x
        if len(y_hl) >=5 :
            # there is a front line here
            y_keep_out_high = min(y_hl)
            y_keep_out_low = max(y_hl)
        for i in range(30,120,15):
            y = height - i
            if y < y_keep_out_high or y > y_keep_out_low:
                line = np.nonzero(mask[y,:])[0]
                seg = _break_segs(line)
                if len(seg) == 1:
                    return max(min(int(np.mean(seg[0])),THUR_R),THUR_L)
        return None

def search_inter_guide_line2(hsv_space:HSVSpace,hsv_image,action:int):
    mask = hsv_space.apply_mask(hsv_image)
    # handle the crossing (X) patterns on the guide lines
    height = int(hsv_image.shape[0])
    x_list = []
    if action == 1:
        # left turn
        res = None
        for i in range(60,170,20):
            y = height - i
            line1 = np.nonzero(mask[y,:])[0]
            line2 = np.nonzero(mask[y-20,:])[0]
            # print(y,line)
            seg1 = _break_segs(line1)
            seg2 = _break_segs(line2)
            if len(seg2) == 0:
                # the further line missing
                if len(seg1) == 0:
                    res =  None
                else:
                    res = int(np.mean(seg1[0]))
            
            if len(seg2) == 1:
                # one line in front
                if len(seg1) == 1:
                    res = int(np.mean(seg1[0]))
                if len(seg1) > 1:
                    res = int(np.mean(seg2[0]))
                
            if len(seg2) == 2:
                if len(seg1) == 1:
                    res = int(np.mean(seg2[1]))
                elif len(seg1) == 2:
                    n1 = abs(np.mean(seg1[0]) - np.mean(seg1[1]))
                    n2 = abs(np.mean(seg2[0]) - np.mean(seg2[1]))
                    if n1 > n2:
                        # closer gap is bigger
                        res = int(np.mean(seg1[1])) # follow lines on the right
                    else:
                        res = int(np.mean(seg1[0]))
                else:
                    res = int(np.mean(seg2[0]))
            
            if res == None:
                return res
            else:
                if res > 250:
                    return None
                temp =  max(min(res,LEFT_TURN_R),LEFT_TURN_L)
                return temp

    elif action == 2:
        # right turn
        for i in range(70,130,15):
            y = i
            line = np.nonzero(mask[y,:])[0]
            seg = _break_segs(line)
            if len(seg) == 1:
                return min(max(RIGHT_TURN_L,int(np.mean(seg[0]))),RIGHT_TURN_R)
        return None
    else: # thur
        width = hsv_image.shape[1]
        y_hl = []
        for x in range(5,width,50):
            y_keep_out_high = height
            y_keep_out_low = 0
            vline = np.nonzero(mask[:,x])[0]
            if len(vline) > 5 and len(vline) < 80:
                y_hl.append(np.mean(vline))
            elif len(vline) >= 80:
                return x
        if len(y_hl) >=5 :
            # there is a front line here
            y_keep_out_high = min(y_hl)
            y_keep_out_low = max(y_hl)
        for i in range(30,120,15):
            y = height - i
            if y < y_keep_out_high or y > y_keep_out_low:
                line = np.nonzero(mask[y,:])[0]
                seg = _break_segs(line)
                if len(seg) == 1:
                    return max(min(int(np.mean(seg[0])),THUR_R),THUR_L)
        return None      

def _break_segs(numbers:list,max_gap=5):
    
    segements = {}
    
    segement_number = 0
    current_segement = []
    
    for index,i in enumerate(numbers):
        
        if not current_segement:
            current_segement.append(i)
        else:
            if i <= numbers[index-1] + max_gap:
                # IN 
                current_segement.append(i)
            else:
                if len(current_segement) > 3:
                    segements[segement_number] = current_segement
                    segement_number += 1
                current_segement = [i]
    
    if len(current_segement) > 3:
        segements[segement_number] = current_segement
        
    return segements

def search_front_car(acc_image_hsv,hsv_space: HSVSpace):
    
    mask = hsv_space.apply_mask(acc_image_hsv)
    height = mask.shape[0]
    for i in range(5,60,5):
        y = height - i
        line = mask[y,:]
        point = np.nonzero(line)[0]
        
        if len(point) > 10:
            
            # there is a line
            center = int(np.mean(point))
            vline = np.nonzero(mask[:,center])
            dis_equiv = len(vline[0])
            return dis_equiv
        else:
            continue
        
    return None
        
