# !/usr/bin/env python

# Python libs
import sys, time
import numpy as np
import cv2
from pyzbar.pyzbar import decode
import pyrealsense2 as rs

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
            
        # Create a pipeline
        self.pipeline = rs.pipeline()
        
        #Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        
        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        self.clipping_distance_in_meters = 1 #1 meter
        self.clipping_distance = clipping_distance_in_meters / depth_scale
        
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        self.align_to = rs.stream.color
        self.align = rs.align(align_to)

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

        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image
        
        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)
        
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
                
        # convert the image into an np.array
        depths_np_arr = np.asanyarray(aligned_depth_frame.get_data())
        depths_image = cv2.imdecode(depths_np_arr, 1)
        
        print depths_image.shape

        # we determine the distance using depths
        qr_dist = depths_image[y0][x0]
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
