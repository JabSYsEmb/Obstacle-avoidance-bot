#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def joy_listener():

    # start node
    rospy.init_node("vehicle_joy", anonymous=True)

    # subscribe to joystick messages on topic "joy"
    rospy.Subscriber("joy", Joy, tj_callback, queue_size=1)

    # keep node alive until stopped
    rospy.spin()

# called when joy message is received
def tj_callback(data):

    # start publisher of cmd_vel to control Turtlesim
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Create Twist message & add linear x and angular z from left joystick
    twist = Twist()
    twist.linear.x = data.axes[1]
    twist.angular.z = data.axes[3]

    # record values to log file and screen
    rospy.loginfo("twist.linear: %f ; angular %f", twist.linear.x, twist.angular.z)

    # publish cmd_vel move command to Turtlesim
    pub.publish(twist)

if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass


   
