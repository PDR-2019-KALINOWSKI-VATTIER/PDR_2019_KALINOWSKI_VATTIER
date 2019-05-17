#!/usr/bin/env python

""" Ce code n'a pas fonctionné, nous avons au départ
cherché à utiliser les informations du QR code détecté
afin de donner un ordre de mouvement au robot.

Nous avons ensuite utilisé un Timer afin de faire appel
à une fonction callback() de manière périodique dans le temps.
Cette fonction callback() fait varier périodiquement la vitesse
(passant de 0 à speed de manière discontinue) et actualise les
consignes de vitesse données au robot.

Nous avons remarqué que les informations étaient bien
communiquées au robot dans le bon topic, sans comprendre
pourquoi il ne se déplaçait pas"""

# Python libs
import sys, time

# Ros libraries
import roslib
import rospy

# Ros Messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist

VERBOSE=True

class bot_motion:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.test_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 10)
        if VERBOSE:
            print "Publishing in /cmd_vel_mux/input/navi topic"

        # subscribed Topic
        self.sub = rospy.Subscriber("/barcode", String, queue_size = 1)
        if VERBOSE:
            print "Subscribing to /barcode"
            
        # movement variable
        self.moving = True
        self.speed = 5

        self.vx = 5
        self.vy = 5
        self.vz = 5
        self.wx = 0
        self.wy = 0
        self.wz = 0
        
    def callback(self, event):
        '''Callback function of subscribed topic.
        sets speed to its new value, updates setpoints
        and publish it in the topicx'''

        # Change the speed of the robot
        self.moving = not self.moving
        if self.moving:
            speed = self.speed
        else:
            speed = 0

        self.vx = speed
        self.vy = speed
        self.vz = speed

        vel_msg = Twist()

        # Update setpoint
        vel_msg.linear.x = self.vx
        vel_msg.linear.y = self.vy
        vel_msg.linear.z = self.vz
        vel_msg.angular.x = self.wx
        vel_msg.angular.y = self.wy
        vel_msg.angular.z = self.wz
    
        # Publish the message
        self.test_pub.publish(vel_msg)

        if VERBOSE :
            rospy.loginfo("published the following msg:\n" + str(vel_msg))
           
def main(args):
    '''Initializes and cleanup ros node'''
    move_bot = bot_motion()
    rospy.init_node('motion', anonymous=True)
    rospy.Timer(rospy.Duration(2), move_bot.callback)
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
