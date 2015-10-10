#!/user/bin/env/ python
import rospy
import copy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

NAME_NODE = 'lower_step_detector'

class LowerStapDetector():
    # public constants
    NAME_LASER_ORI = 'base_scan1'
    NAME_LASER_FIX = 'base_scan1_fix'
    MAX_LASER_INTENSITY = 2.0
    VIRTUAL_LASER_INTENSITY = 1.0

   # private variables
    __laser_ori_sub = 0  # original laser scan subscriber
    __laser_fix_pub = 0 # fixed laser scan publisher

    def __init__(self):
        rospy.init_node(NAME_NODE, anonymous=False)
        self.__laser_ori_sub = rospy.Subscriber(self.NAME_LASER_ORI, LaserScan, self.on_subscribe_laser_scan)
        self.__laser_fix_pub = rospy.Publisher(self.NAME_LASER_FIX, LaserScan, queue_size = 1)
        # start callback
        rospy.spin()

    def on_subscribe_laser_scan(self, laser_sensor_msg_ori):
        # do something
        #print 'subscribed'
        #len(message.ranges)
        #print message.ranges[360]
        #laser_sensor_msg_ori = copy.deepcopy(message)
        laser_sensor_msg_fix = copy.deepcopy(laser_sensor_msg_ori)
        tmp_fix_data = len(laser_sensor_msg_ori.ranges)*[0]
        # fix data
        for i in range(len(laser_sensor_msg_ori.ranges)):
            if laser_sensor_msg_ori.ranges[i] > self.MAX_LASER_INTENSITY:
                tmp_fix_data[i] = self.VIRTUAL_LASER_INTENSITY
            else:
                tmp_fix_data[i] = copy.deepcopy(laser_sensor_msg_ori.ranges[i])
                #laser_sensor_msg_fix.ranges[i] = copy.deepcopy(laser_sensor_msg_ori.ranges[i])
                #laser_sensor_msg_fix.ranges[i] = laser_sensor_msg_ori.ranges[i]
        #laser_data_fix = message
        # registar to fix buffer
        laser_sensor_msg_fix.ranges = tuple(tmp_fix_data)
        # publish fixed laser
        self.__laser_fix_pub.publish(laser_sensor_msg_fix)
        #jprint 'published'

if __name__ == '__main__':
    try:
        print 'start program'
        LowerStapDetector()

    except:
        rospy.loginfo("lower_step_detector finished.")

