#!/usr/bin/env python

# Python libs
import sys, time
import numpy as np
import cv2
from pyzbar.pyzbar import decode

# Ros libraries
import roslib
import rospy

# Ros Messages
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

# Do we write information ?
VERBOSE=True

class color_detection:
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        
        # topic where we publish
        self.test_pub = rospy.Publisher("/detected_barcode", String, queue_size = 10)
        if VERBOSE:
            rospy.loginfo("Publishing in /detected_barcode topic")
            
        # subscribed topic
        self.color_sub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.callback, queue_size = 1)
        if VERBOSE:
            rospy.loginfo("Subscribing to /camera/color/image_raw/compressed")

    def callback(self, ros_data):
        #"""Detect barcodes and publishes the position informations
        #of the detected barcodes and its name"""
                          
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(np_arr, 1)

        barcodes = decode(image)

        if VERBOSE:
            if len(barcodes) > 0:
                rospy.loginfo("Detected %d barcodes" % len(barcodes))
        
        for barcode in barcodes:
            # x, y: position
            # w: withdraw, h: height
            (x, y, w, h) = barcode.rect
            qr_name_data = barcode.data
            xmoy = x+w/2
            ymoy = y+h/2
            if VERBOSE:
                rospy.loginfo("Barcode data: " + str(qr_name_data))
                rospy.loginfo("Center of barcode: (%d, %d)" % (xmoy, ymoy))
            position_data = (x, y, w, h)
            msg = "(" + str(position_data) + ", " + str(qr_name_data) + ")"
            if VERBOSE:
                rospy.loginfo("published msg:\n" + msg)
            self.test_pub.publish(msg)
        
def main(args):
    '''Initializes and cleanup ros node'''
    color_detect = color_detection()
    rospy.init_node('color_detection', anonymous=True)
    if VERBOSE:
	rospy.loginfo("initializing color detection node")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    if VERBOSE:
        print "start of main"
    main(sys.argv)
    if VERBOSE:
        print "end of main"
