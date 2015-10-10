#!/user/bin/env/ python
## @package third_ronot_lower_step_detector
#  fix original laser scan data to detect down step 
#
#  @author Masaru Morita 

import rospy
import copy
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

DEG_TO_RAD = math.pi / 180
RAD_TO_DEG = 180 / math.pi
NAME_NODE = 'lower_step_detector'

class LowerStapDetector():
    ## public constants
    NAME_LASER_ORI = 'base_scan1'
    NAME_LASER_FIX = 'base_scan1_fix'
    LASER_INTENSITY_MAX = 3.0
    VIRTUAL_LASER_INTENSITY = 1.0
    DETECT_STEP_ANGLE_MIN_DEG = 40
    DETECT_STEP_ANGLE_MAM_DEG = 180 - DETECT_STEP_ANGLE_MIN_DEG
    DETECT_ANGLE_CENTER_DEG = 90

    ## private member variables
    ## @var __laser_ori_sub
    # original laser scan subscriber
    __laser_ori_sub = 0
    ## @var __laser_fix_pub
    # fixed laser scan publisher
    __laser_fix_pub = 0

    ## constractor
    def __init__(self):
        rospy.init_node(NAME_NODE, anonymous=False)
        self.__laser_ori_sub = rospy.Subscriber(self.NAME_LASER_ORI, LaserScan, self.on_subscribe_laser_scan)
        self.__laser_fix_pub = rospy.Publisher(self.NAME_LASER_FIX, LaserScan, queue_size = 1)
        # start callback
        rospy.spin()

    # methods
    def on_subscribe_laser_scan(self, laser_sensor_msg_ori):
        # temporary buffer for publish. This is because tuple(type of LaerScan.ranges) type cann't be overwritten.
        tmp_fix_data = len(laser_sensor_msg_ori.ranges)*[0]
        # calculate parameters
        angle_increment = laser_sensor_msg_ori.angle_increment
        detect_index_min = self.DETECT_STEP_ANGLE_MIN_DEG * DEG_TO_RAD / angle_increment
        detect_index_max = self.DETECT_STEP_ANGLE_MAM_DEG * DEG_TO_RAD / angle_increment
        detect_index_mid = self.DETECT_ANGLE_CENTER_DEG * DEG_TO_RAD / angle_increment

        for i in range(len(laser_sensor_msg_ori.ranges)):
            # copy original data
            tmp_fix_data[i] = copy.deepcopy(laser_sensor_msg_ori.ranges[i])
            # skip when a range cannot detect down step
            if i < detect_index_min or i > detect_index_max:
                continue
            # overwrite only when range can detect down step
            if laser_sensor_msg_ori.ranges[i] > self.LASER_INTENSITY_MAX:
                angle_curr_rad = i * angle_increment
                if i < detect_index_mid:
                    theta = angle_curr_rad
                else:
                    theta = math.pi - angle_curr_rad
                tmp_fix_data[i] = self.VIRTUAL_LASER_INTENSITY / math.sin(theta)

        # create & registar temproray tuple buffer to publishing buffer
        laser_sensor_msg_fix.ranges = tuple(tmp_fix_data)
        # publish fixed laser scan topic
        self.__laser_fix_pub.publish(laser_sensor_msg_fix)

if __name__ == '__main__':
    try:
        print 'start program'
        LowerStapDetector()

    except:
        rospy.loginfo("lower_step_detector finished.")

