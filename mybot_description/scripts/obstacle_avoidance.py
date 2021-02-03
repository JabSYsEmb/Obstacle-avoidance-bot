#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sensor_msgs.msg
import random
import numpy as np
from geometry_msgs.msg import Twist
from itertools import *
from operator import itemgetter

LINX = 0.0 
THRESHOLD = 2.5 #
PI = 3.14
Kp = 0.05
angz = 0

def LaserScanProcess(data):
 
    range_angels = np.arange(len(data.ranges))
    ranges = np.array(data.ranges)
    # Filtering data by a threshold value for accepted values
    range_mask = (ranges > THRESHOLD)
    ranges = list(range_angels[range_mask])
    max_gap = 40

    gap_list = []

    for k, g in groupby(enumerate(ranges), lambda (i,x):i-x):
        gap_list.append(map(itemgetter(1), g))

    # Sorting data that is grouped according to their length
    gap_list.sort(key=len)
         
    largest_gap = gap_list[-1]

    # Calculate the min and max angle of the largest gap 
    min_angle, max_angle = largest_gap[0]*((data.angle_increment)*180/PI), largest_gap[-1]*((data.angle_increment)*180/PI)

    average_angle = (max_angle - min_angle)/2

    # To be turnt angle
    turn_angle = min_angle + average_angle
 
    global LINX
    global angz
    if average_gap < max_gap:
        angz = -0.5 
    else:
        LINX = 0.6  
        angz = Kp*(-1)*(90 - turn_angle)

def main():
    rospy.init_node('listener', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("scan", sensor_msgs.msg.LaserScan , LaserScanProcess)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        command = Twist()
        command.linear.x = LINX
        command.angular.z = angz
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    main()
