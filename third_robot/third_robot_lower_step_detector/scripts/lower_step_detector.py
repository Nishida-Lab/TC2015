#!/user/bin/env/ python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

NAME_NODE = 'lower_step_detector'

class LowerStapDetector():
    # public constants
    NAME_LASER_ORI = 'base_scan1'
    NAME_LASER_FIX = 'base_scan1_fix'

    # private variables
    __laser_ori_sub = 0  # original laser scan subscriber
    __laser_fix_pub = 0 # fixed laser scan publisher

    def __init__(self):
        rospy.init_node(NAME_NODE, anonymous=False)
        self.__laser_ori_sub = rospy.Subscriber(self.NAME_LASER_ORI, LaserScan, self.on_subscribe_laser_scan)
        self.__laser_fix_pub = rospy.Publisher(self.NAME_LASER_FIX, LaserScan, queue_size = 1)
        # start callback
        rospy.spin()

    def on_subscribe_laser_scan(self, message):
        # do someting
        # fix data
        laser_data_fix = message
        # publish fixed laser
        self.__laser_fix_pub.publish(laser_data_fix)
        print 'subscribed'

if __name__ == '__main__':
    try:
        print 'start program'
        LowerStapDetector()

    except:
        rospy.loginfo("lower_step_detector finished.")

