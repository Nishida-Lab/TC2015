#!/usr/bin/env python
# coding: utf-8

"""
fix original laser scan data to detect down step
"""
__author__  = "MasaruMorita <p595201m@mail.kyutech.jp>"
__version__ = "1.01"
__date__    = "18 Oct 2015"

import rospy
import copy
import math
import sys
import numpy as np
from sensor_msgs.msg import LaserScan

NAME_NODE = 'lower_step_detector'

class LaserScanEx():
    threshold_intensity = 0
    angle_laser_rad = 0
    angle_laser_deg = 0
    angle_laser_to_calc_intensity = 0
    is_beyond_threshold = False

    def __init__(self):
        self.threshold = 0
        self.angle_laser_rad = 0
        self.angle_laser_deg = 0
        self.angle_laser_to_calc_intensity = 0
        self.is_beyond_threshold = False

class LowerStepDetector():
    ## public constants
    # topic names
    NAME_TOPIC_SUFFIX_TO_PUBLISH = '_fix'
    NAME_TOPIC_LASER_ORI = 'base_scan1'
    NAME_TOPIC_LASER_FIX = NAME_TOPIC_LASER_ORI + NAME_TOPIC_SUFFIX_TO_PUBLISH
    # parameter names
    NAME_PARAM_LASER_INTENSITY = NAME_NODE + '/laser_intensity_max'
    NAME_PARAM_VIRTUAL_LASER_INTENSITY = NAME_NODE + '/virtual_laser_intensity'
    NAME_PARAM_LASER_SCAN_RANGE_DEG = NAME_NODE + '/laser_scan_range_deg'
    NAME_PARAM_DETECT_STEP_ANGLE_MIN_DEG = NAME_NODE + '/detect_step_angle_min_deg'
    NAME_PARAM_MARGIN_BETWEEN_PLANE_AND_DOWN_STEP = NAME_NODE + '/margin_between_plane_and_down_step'
    # default parameter values
    DEFAULT_LASER_INTENSITY_MAX = 1.5
    DEFAULT_MARGIN_BETWEEN_PLANE_AND_DOWN_STEP = 1.5
    DEFAULT_VIRTUAL_LASER_INTENSITY = 1.0
    DEFAULT_LASER_SCAN_RANGE_DEG = 180.0
    DEFAULT_DETECT_STEP_ANGLE_MIN_DEG = 10.0

    ## private member variables
    # original laser scan subscriber
    __laser_ori_sub = 0
    # fixed laser scan publisher
    __laser_fix_pub = 0
    __laser_intensity_max = 0
    __virtual_laser_intensity = 0
    __detect_index_min = 0
    __detect_index_max = 0
    #
    __laser_scan_range_deg = 0
    __detect_step_angle_min_deg = 0
    __detect_step_angle_max_deg = 0
    __detect_angle_center_deg = 0
    __margin_between_plane_and_down_step = 0
    #
    __is_init = True
    __laser_scan_exs = []

    ## constructor
    def __init__(self):
        rospy.init_node(NAME_NODE, anonymous=False)
        self.load_topic_name_to_sub()
        self.load_rosparam()
        self.__laser_ori_sub = rospy.Subscriber(self.NAME_TOPIC_LASER_ORI, LaserScan, self.on_subscribe_laser_scan)
        self.__laser_fix_pub = rospy.Publisher(self.NAME_TOPIC_LASER_FIX, LaserScan, queue_size = 1)
        # start callback
        rospy.spin()

    ## methods
    def load_topic_name_to_sub(self):
        argvs = sys.argv
        num_arg = len(argvs)
        if num_arg > 1:
            self.NAME_TOPIC_LASER_ORI = argvs[1]
            self.NAME_TOPIC_LASER_FIX = self.NAME_TOPIC_LASER_ORI + self.NAME_TOPIC_SUFFIX_TO_PUBLISH

    def load_rosparam(self):
        self.__laser_intensity_max = rospy.get_param(self.NAME_PARAM_LASER_INTENSITY, self.DEFAULT_LASER_INTENSITY_MAX)
        self.__margin_between_plane_and_down_step = rospy.get_param(self.NAME_PARAM_MARGIN_BETWEEN_PLANE_AND_DOWN_STEP, self.DEFAULT_MARGIN_BETWEEN_PLANE_AND_DOWN_STEP)
        self.__virtual_laser_intensity = rospy.get_param(self.NAME_PARAM_VIRTUAL_LASER_INTENSITY, self.DEFAULT_VIRTUAL_LASER_INTENSITY)
        self.__laser_scan_range_deg = rospy.get_param(self.NAME_PARAM_LASER_SCAN_RANGE_DEG, self.DEFAULT_LASER_SCAN_RANGE_DEG)
        self.__detect_step_angle_min_deg = rospy.get_param(self.NAME_PARAM_DETECT_STEP_ANGLE_MIN_DEG, self.DEFAULT_DETECT_STEP_ANGLE_MIN_DEG)
        self.__detect_step_angle_max_deg = self.__laser_scan_range_deg - self.__detect_step_angle_min_deg
        self.__detect_angle_center_deg = self.__laser_scan_range_deg / 2.0

    def calculate_threshold_intensity(self, laser_sensor_msg_ori):
         # calculate parameters
         angle_increment = laser_sensor_msg_ori.angle_increment
         self.__detect_index_min = math.radians(self.__detect_step_angle_min_deg) / angle_increment
         self.__detect_index_max = math.radians(self.__detect_step_angle_max_deg) / angle_increment
         detect_index_mid = math.radians(self.__detect_angle_center_deg) / angle_increment

         for i in range(len(laser_sensor_msg_ori.ranges)):
             laser_scan_ex = LaserScanEx()

             angle_curr_rad = i * angle_increment
             angle_curr_deg = math.degrees(angle_curr_rad)

             if i < detect_index_mid:
                 theta = angle_curr_rad
             else:
                 theta = math.pi - angle_curr_rad

             # to avoid zero division
             if theta != 0:
                 laser_intensity_thresh = self.__laser_intensity_max / math.sin(theta) + self.__margin_between_plane_and_down_step
             else:
                 laser_intensity_thresh = float("inf")

             laser_scan_ex.angle_laser_rad = angle_curr_rad
             laser_scan_ex.angle_laser_deg = angle_curr_deg
             laser_scan_ex.angle_laser_to_calc_intensity = theta

             # beyond the extend of step scan
             if i < self.__detect_index_min or i > self.__detect_index_max:
                 laser_scan_ex.threshold_intensity = float("inf")
             else:
                 laser_scan_ex.threshold_intensity = laser_intensity_thresh
             self.__laser_scan_exs.append(laser_scan_ex)

    def on_subscribe_laser_scan(self, laser_sensor_msg_ori):
        laser_sensor_msg_fix = copy.deepcopy(laser_sensor_msg_ori)
        # temporary buffer for publish. This is because tuple(type of LaserScan.ranges) type can't be overwritten.
        tmp_fix_data = len(laser_sensor_msg_ori.ranges)*[0]

        # calculate threshold only when it's the first time
        if self.__is_init == True:
            # reset initial flag
            self.__is_init = False
            self.calculate_threshold_intensity(laser_sensor_msg_ori)

        for i in range(len(laser_sensor_msg_ori.ranges)):
            # copy original data
            tmp_fix_data[i] = copy.deepcopy(laser_sensor_msg_ori.ranges[i])
            # skip when a range cannot detect down step
            if i < self.__detect_index_min or i > self.__detect_index_max:
                continue

            # overwrite only when range can detect down step
            if laser_sensor_msg_ori.ranges[i] > self.__laser_scan_exs[i].threshold_intensity:
                print 'detected lower step at %f[degree]!' % self.__laser_scan_exs[i].angle_laser_deg
                self.__laser_scan_exs[i].is_beyond_threshold = True
            else:
                self.__laser_scan_exs[i].is_beyond_threshold = False

            if self.__laser_scan_exs[i].is_beyond_threshold == True:
                tmp_fix_data[i] = self.__virtual_laser_intensity / math.sin(self.__laser_scan_exs[i].angle_laser_to_calc_intensity)

        # create & register temporary tuple buffer to publishing buffer
        laser_sensor_msg_fix.ranges = tuple(tmp_fix_data)
        # publish fixed laser scan topic
        self.__laser_fix_pub.publish(laser_sensor_msg_fix)

if __name__ == '__main__':
    try:
        print 'start program'
        LowerStepDetector()

    except:
        rospy.loginfo("lower_step_detector finished.")