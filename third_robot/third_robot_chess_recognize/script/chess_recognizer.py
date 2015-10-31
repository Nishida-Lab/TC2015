#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This source code is based on below URL's code.
# http://www.pyimagesearch.com/2015/01/26/multi-scale-template-matching-using-python-opencv/

# import the necessary packages
import numpy as np
import argparse
import imutils
import glob
import cv2
import rospy
import sys
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class chess_recognizer:

    def __init__(self):
        self.image_pub = rospy.Publisher("chess_recognize", Image, queue_size=10)
        self.num_of_templates = 3
        self.templates_list = []
        self.templates_tH_list = []
        self.templates_tW_list = []
        rospack = rospkg.RosPack()
        for num in range(0, self.num_of_templates):
            tmp = cv2.imread(rospack.get_path("third_robot_chess_recognize") \
                                                  + "/resource/chess_template0" + str(num) + ".png")
            tmp = cv2.cvtColor(tmp, cv2.COLOR_BGR2GRAY)
            tmp = cv2.Canny(tmp, 50, 200)
            self.templates_list.append(tmp)
            height, width = tmp.shape[:2]
            self.templates_tH_list.append(height)
            self.templates_tW_list.append(width)

        self.template = cv2.imread(rospack.get_path("third_robot_chess_recognize") \
                                   + "/resource/chess_template00.png")
        if self.template == None:
            print "Could not open or find template image"
            raise("IO Error")
        self.template = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)
        self.template = cv2.Canny(self.template, 50, 200)
        (self.tH, self.tW) = self.template.shape[:2]
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(0)
        if self.capture.isOpened() is False:
            raise("IO Error")

    def recognize(self):
        ret, image = self.capture.read()
        # load the image, convert it to grayscale, and initialize the
        # bookkeeping variable to keep track of the matched region
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found = None

        # loop over the scales of the image
        for scale in np.linspace(0.1, 0.5, 5)[::-1]:
            # resize the image according to the scale, and keep track
            # of the ratio of the resizing
            resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
            r = gray.shape[1] / float(resized.shape[1])
    
            # if the resized image is smaller than the template, then break
            # from the loop
            if resized.shape[0] < self.tH or resized.shape[1] < self.tW:
                break
            # detect edges in the resized, grayscale image and apply template
            # matching to find the template in the image
            edged = cv2.Canny(resized, 50, 200)
            result = cv2.matchTemplate(edged, self.template, cv2.TM_CCOEFF)
            (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
            
            # if we have found a new maximum correlation value, then ipdate
            # the bookkeeping variable
            if found is None or maxVal > found[0]:
                found = (maxVal, maxLoc, r)
            
            # unpack the bookkeeping varaible and compute the (x, y) coordinates
            # of the bounding box based on the resized ratio
            (_, maxLoc, r) = found
            (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
            (endX, endY) = (int((maxLoc[0] + self.tW) * r), int((maxLoc[1] + self.tH) * r))
        
            if found[0] > 15000000:
                # draw a bounding box around the detected result and display the image
                cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
                
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            except CvBridgeError, e:
                print e

                
    def multi_template_recognize(self):
        ret, image = self.capture.read()
        # load the image, convert it to grayscale, and initialize the
        # bookkeeping variable to keep track of the matched region
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found = None
  
        for i, template in enumerate(self.templates_list):
            # loop over the scales of the image
            for scale in np.linspace(0.1, 0.5, 5)[::-1]:
                # resize the image according to the scale, and keep track
                # of the ratio of the resizing
                resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
                r = gray.shape[1] / float(resized.shape[1])
                
                # if the resized image is smaller than the template, then break
                # from the loop
                if resized.shape[0] < self.templates_tH_list[i] or resized.shape[1] < self.templates_tW_list[i]:
                    break
                    # detect edges in the resized, grayscale image and apply template
                    # matching to find the template in the image
                edged = cv2.Canny(resized, 50, 200)
                result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
                (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
                    
                # if we have found a new maximum correlation value, then ipdate
                # the bookkeeping variable
                if found is None or maxVal > found[0]:
                    found = (maxVal, maxLoc, r)
                        
                # unpack the bookkeeping varaible and compute the (x, y) coordinates
                # of the bounding box based on the resized ratio
                (_, maxLoc, r) = found
                (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
                (endX, endY) = (int((maxLoc[0] + self.tW) * r), int((maxLoc[1] + self.tH) * r))
                
                if found[0] > 10000000:
                    # draw a bounding box around the detected result and display the image
                    cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
                            
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError, e:
            print e
        
        
def main(args):
    recognizer = chess_recognizer()
    rospy.init_node('chess_recognizer', anonymous=True)
    # rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #recognizer.recognize()
        recognizer.multi_template_recognize()
        #rate.sleep()

if __name__ == '__main__':
    main(sys.argv)



