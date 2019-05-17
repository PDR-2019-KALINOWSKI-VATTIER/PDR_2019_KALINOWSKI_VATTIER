#!/usr/bin/env python

""" Code permettant de commander le deplacement du robot.

    Le robot tourne a 90 degres lorsqu'un QR code est detecte.
    Pour calibrer la consigne de vitesse anglaire et la duree de rotation,
    nous avons procede de maniere iterative en changeant ces parametres."""

import rospy
from time import time

from geometry_msgs.msg import Twist
from std_msgs.msg import String

VERBOSE=True

class Move_and_Dodge():
    def __init__(self):
        """ Initialization """
        rospy.init_node('move_and_dodge', anonymous=False)

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
	# Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Create a subscriber which can listen to information relative
        # to QR code detection and change the speed
        self.change_vel = rospy.Subscriber("/detected_barcode", String, self.callback, queue_size=1)

        # Parameters related to dodging
        self.linear_speed = 0.2
        self.angular_speed = 0
        self.delta_t = 1
        self.barcode_detected = False
        self.t0 = time()
        
    def move(self):
        """ Move the bot """
	# Define the rate
        r = rospy.Rate(10)

        move_cmd = Twist()

        while not rospy.is_shutdown():
            t = time()
            rospy.loginfo("t - t0 = %f" % (t-self.t0))
            
            if (t - self.t0) >= self.delta_t and (self.barcode_detected):
                rospy.loginfo("Finished dodging !")
                self.angular_speed = 0
                self.barcode_detected = False

            # Define linear and angular speeds
            move_cmd.linear.x = self.linear_speed
	    move_cmd.angular.z = self.angular_speed
        
	    # publish the velocity
            self.cmd_vel.publish(move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    def callback(self, data):
        """ Put the angular_speed of the robot
        to 2.0 when a barcode is detected """
        if not self.barcode_detected:
            rospy.loginfo("Started to dodge !")
            self.angular_speed = 1.5
            self.barcode_detected = True
            self.t0 = time()
            
    def shutdown(self):
        """ Shutdown the robot """
	# a default Twist has linear.x of 0 and angular.z of 0.
        # So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    if VERBOSE:
        print "Start of main"
    move_and_dodge = Move_and_Dodge()
    move_and_dodge.move()
    move_and_dodge.shutdown()
    if VERBOSE:
        print "End of main"
    
