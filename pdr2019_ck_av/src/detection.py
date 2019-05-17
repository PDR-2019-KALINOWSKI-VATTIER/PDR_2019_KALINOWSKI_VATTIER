#!/usr/bin/env python

""" Premier programme de detection mis en place,
    utilisant le topic /barcode.

    Il est necessaire d'installer la bibliotheque zbar_ros
    et d'executer dans le terminal les commandes suivantes :

    $ rosrun zbar_ros barcode_reader_node &
    $ rosrun image_transport republish raw in:=/camera/color/image_raw raw out:=/image

    Le nom du QR code detecte est publie dans le topic /test_publisher"""


# Python libs
import sys, time

# Ros libraries
import roslib
import rospy

# Ros Messages
from std_msgs.msg import String

VERBOSE=True

class qr_detection:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.test_pub = rospy.Publisher("/test_publisher", String, queue_size = 10)
        if VERBOSE:
            rospy.loginfo("Publishing in /test_publisher topic")

        # subscribed topic
        self.sub = rospy.Subscriber("/barcode", String, self.callback, queue_size = 1)
        if VERBOSE:
            rospy.loginfo("Subscribing to /barcode")

    def callback(self, ros_data):
        '''Callback function of subscribed topic.'''
        detected_code = ros_data.data
        
        msg = String()
        msg.data = "New code detected: " + str(detected_code)

        self.test_pub.publish(msg)

        if VERBOSE :
            rospy.loginfo('published the following msg:\n' + str(msg))

        
def main(args):
    '''Initializes and cleanup ros node'''
    qr_detect = qr_detection()
    rospy.init_node('detection', anonymous=True)
    if VERBOSE:
	rospy.loginfo("initializing detection node")
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
x
