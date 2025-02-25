#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan



def callback(msg):
    laser_factor = len(msg.ranges)/360

    print ("Number of scan points: "+ str(len(msg.ranges)))
    # values at 0 degrees
    print ("Distance at 0deg: " + str(msg.ranges[int(laser_factor*0)]))
    # values at 90 degrees
    print ("Distance at 90deg: " + str(msg.ranges[int(laser_factor*90)]))
    # values at 180 degrees
    print ("Distance at 180deg: " + str(msg.ranges[int(laser_factor*180)]))
    # values at 270 degrees
    print ("Distance at 270deg: " + str(msg.ranges[int(laser_factor*270)]))
    # values at 360 degrees
    print ("Distance at 360deg: " + str(msg.ranges[int(laser_factor*360)-1]))

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()