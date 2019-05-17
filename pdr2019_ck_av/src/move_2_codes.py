#!/usr/bin/env python

""" Code permettant de commander le mouvement du robot en differenciant
    les ordres donnes selon le QR code detecte.

    Si le robot detecte le QR code 1, il entreprend une rotation
    de 90 degres pour esquiver l'obstacle.
    S'il detecte le QR code 2, il effectue une rotation de 180
    degres et fait donc demi-tour."""

# Python Lib
from time import time

# Ros Lib
import rospy

# Ros messages
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# QR code names
qr1 = " Hello :)"
qr2 = " http://fr.wikipedia.org/"

# Do we write information ?
VERBOSE = True

# State id
MOVING_FORWARD = 0
DODGING = 1
TURNING_BACK = 2

class Move_2_codes():
    def __init__(self):
        """ Initialization """
        rospy.init_node('move_2_codes', anonymous=False)

        # What function to call when you Ctrl + C    
        rospy.on_shutdown(self.shutdown)
        
	# Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Create a subscribe which can listen to information relative
        # to QR code detection and change the speed
        self.change_vel = rospy.Subscriber("/detected_barcode", String, self.callback, queue_size=1)

        # Parameters related to movement
        self.linear_speed = -0.06
        self.angular_speed = 0
        self.delta_t = 1
        self.barcode_detected = False
        self.t0 = time()
        self.state = MOVING_FORWARD
        
    def move(self):
        """ Move the bot """
	# Define the rate
        r = rospy.Rate(10)

        move_cmd = Twist()

        while not rospy.is_shutdown():
            t = time()
            if VERBOSE:
                rospy.loginfo("t - t0 = %f" % (t-self.t0))
            
            if (t - self.t0) >= self.delta_t and (self.barcode_detected):
                if self.state == DODGING:
                    rospy.loginfo("Finished dodging!")
                elif self.state == TURNING_BACK:
                    rospy.loginfo("Finished turning back!")
                self.reset()
                
            # Update linear and angular speeds
            move_cmd.linear.x = self.linear_speed
	    move_cmd.angular.z = self.angular_speed
        
	    # publish the velocity
            self.cmd_vel.publish(move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    def callback(self, ros_data):
        received_msg = str(ros_data.data)
        x, y, w, h, qr_name = self.convert_msg(received_msg)

        if qr_name == qr1:
            if not self.barcode_detected:
                self.dodge()
                rospy.loginfo("delta_t = %f" % self.delta_t)
        if qr_name == qr2:
            if not self.barcode_detected:
                self.turn_back()
                rospy.loginfo("delta_t = %f" % self.delta_t)

    def reset(self):
        rospy.loginfo("Resetting parameters !")
        self.angular_speed = 0
        self.barcode_detected = False
        self.state = MOVING_FORWARD

    def dodge(self):
        rospy.loginfo("Started to dodge !")
        self.angular_speed = 1.5
        self.barcode_detected = True
        self.t0 = time()
        self.delta_t = 1
        self.state = DODGING

    def turn_back(self):
        rospy.loginfo("Turning back !")
        self.angular_speed = 2.0
        self.barcode_detected = True
        self.t0 = time()
        self.delta_t = 2
        self.state = TURNING_BACK
        
    def shutdown(self):
        """ Shutdown the robot """
	# a default Twist has linear.x of 0 and angular.z of 0.
        # So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

    def convert_msg(self, msg):
        """ Parses the message read the topic /detected_barcode """
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
        """ Convert the power of 10 list of an integer into this number """
        x = 0
        n = len(l)-1
        for i in range(len(l)):
            x += l[i]*10**n
            n -= 1
        return x
        
if __name__ == '__main__':
    move_2_codes = Move_2_codes()
    move_2_codes.move()
    move_2_codes.shutdown()
