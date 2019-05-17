#!/usr/bin/env python

""" Code permettant de commander le deplacement du robot.

    Le robot s'arrete des la detection du premier QR code """

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Move_and_Stop():
    def __init__(self):
        """ Initialization """
        rospy.init_node('move_and_stop', anonymous=False)

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
	# Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Create a subscribe which can listen to information relative
        # to QR code detection and change the speed
        self.change_vel = rospy.Subscriber("/detected_barcode", String, self.callback, queue_size=1)        
        self.speed = 0.2

    def move(self):
        """ Move the bot """
	# Define the rate
        r = rospy.Rate(10)

        move_cmd = Twist()

        # Define linear and angular speeds
        move_cmd.linear.x = self.speed
	move_cmd.angular.z = 0
        
        while not rospy.is_shutdown() and self.speed > 0:
	    # publish the velocity
            self.cmd_vel.publish(move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    def callback(self, data):
        """ Put the speed of the robot to 0 when a barcode is detected """
        self.speed = 0
            
    def shutdown(self):
        """ Shutdown the robot """
	# a default Twist has linear.x of 0 and angular.z of 0.
        # So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    move_and_stop = Move_and_Stop()
    move_and_stop.move()
    move_and_stop.shutdown()
