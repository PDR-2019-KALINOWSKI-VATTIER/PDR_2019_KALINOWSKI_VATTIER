# !/usr/bin/env python

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

VERBOSE=True

class depth_detection:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.test_pub = rospy.Publisher("/test_publisher", String, queue_size = 10)
        if VERBOSE:
            print "Publishing in /test_publisher topic"

        # subscribed Topic
        self.qr_sub = rospy.Subscriber("/detected_barcode", String, self.callback, queue_size = 1)
        if VERBOSE:
            print "Subscribing to /detected_barcode topic" 
            
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, queue_size = 1)
        # if VERBOSE:
        #     print "Subscribing to /camera/depth/image_raw"      

    def callback(self, ros_data):
        '''Compute the distance between the qr code and the camera'''

        received_msg = str(ros_data.data)
        if VERBOSE:
            print "Received msg:\n" + received_msg

        # parse the information provided by the /detected_barcode topic
        x, y, w, h, msg = self.convert_msg(received_msg)
        if VERBOSE:
            print "Parsed information:\n"
            print "x: %d y: %d w: %d h: %d" % (x, y, w, h)
            print "msg: " + msg

        x0 = x + w//2
        y0 = y + h//2

        print x0, y0
        
        # get the image
        if VERBOSE:
            print "Getting depth image"
        image_msg = rospy.wait_for_message("/camera/infra2/image_rect_raw/compressedDepth", CompressedImage)
        if VERBOSE:
            print "Got image!"

        # convert the image into an np.array
        depths_np_arr = np.fromstring(image_msg.data, np.uint8)
        depths_image = cv2.imdecode(depths_np_arr, 1)
        
        print depths_image.shape

        # we determine the distance using depths
        print depths_image[y0][x0]
        
        qr_dist = depths_image[y0][x0][0]
        if VERBOSE:
            print "Distance from QR code: %d" % qr_dist
            
    def convert_msg(self, msg):
        splited_msg = msg.split(',')
        print splited_msg
        x = []
        for i in range(4):
            xi_s = splited_msg[i]
            xi_l = [int(c) for c in xi_s if c.isdigit()]        
            x += [self.convert_decimal(xi_l)]
        x += [str(splited_msg[4][:len(splited_msg[4])-1])]
        return tuple(x)
    
    def convert_decimal(self, l):
        x = 0
        n = len(l)-1
        for i in range(len(l)):
            x += l[i]*10**n
            n -= 1
        return x
        
def main(args):
    '''Initializes and cleanup ros node'''
    depth_detect = depth_detection()
    rospy.init_node('depth_detection', anonymous=True)
    if VERBOSE:
        print "initializing detection node"
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
