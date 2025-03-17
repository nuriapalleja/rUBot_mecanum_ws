#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import sign

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=False)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(25)

        self.d = rospy.get_param("~distance_laser", 0.3)  # Default value added
        self.vx = rospy.get_param("~forward_speed", 0.2)  # Default value added
        self.wz = rospy.get_param("~rotation_speed", 1.0)  # Default value added
        self.vf = rospy.get_param("~speed_factor", 1.0)  # Default value added

        self.isScanRangesLengthCorrectionFactorCalculated = False
        self.scanRangesLengthCorrectionFactor = 2

        self.shutting_down = False

    def scan_callback(self, msg):
        if self.shutting_down:
            return

        if not self.isScanRangesLengthCorrectionFactorCalculated:
            self.scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
            self.isScanRangesLengthCorrectionFactorCalculated = True

        bright_min = int(30 * self.scanRangesLengthCorrectionFactor)
        bright_max = int(90 * self.scanRangesLengthCorrectionFactor)
        right_min = int(90 * self.scanRangesLengthCorrectionFactor)
        right_max = int(120 * self.scanRangesLengthCorrectionFactor)
        fright_min = int(120 * self.scanRangesLengthCorrectionFactor)
        fright_max = int(170 * self.scanRangesLengthCorrectionFactor)
        front_min = int(170 * self.scanRangesLengthCorrectionFactor)
        front_max = int(190 * self.scanRangesLengthCorrectionFactor)

        
        regions = {
            'bright': min(min(msg.ranges[bright_min:bright_max]), 3),
            'right': min(min(msg.ranges[right_min:right_max]), 3),
            'fright': min(min(msg.ranges[fright_min:fright_max]), 3),
            'front': min(min(msg.ranges[front_min:front_max]), 3),
        }

        self.take_action(regions)

    def take_action(self, regions):
        msg = Twist()

        state_description = ''

        if regions['front'] > self.d and regions['fright'] > self.d * 2 and regions['right'] > self.d * 2 and regions['bright'] > self.d * 2:
            state_description = 'case 1 - nothing'
            msg = self.go_ahead()
        elif regions['front'] < self.d:
            state_description = 'case 2 - front'
            msg = self.turn_left()
        elif regions['fright'] < self.d and regions['right'] > self.d:
            state_description = 'case 3 - fright'
            msg = self.turn_left()
        elif regions['fright'] < self.d and regions['fright'] < regions['right'] * 2: # controla la distancia horizontal
            state_description = 'case 3 - fright'
            msg = self.turn_left()
        elif regions['right'] < self.d:
            state_description = 'case 4 - right'
            msg = self.go_ahead()
        elif regions['bright'] < self.d:
            state_description = 'case 5 - bright'
            msg = self.turn_right()
        else:
            state_description = 'case 6 - Far'
            msg = self.reorient()

        rospy.loginfo(state_description)
        self.pub.publish(msg)
        self.rate.sleep()

    def go_ahead(self):
        msg = Twist()
        msg.linear.x = self.vx * self.vf
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = self.wz * self.vf
        return msg

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -2*self.wz * self.vf
        return msg

    def reorient(self):
        msg = Twist()
        msg.linear.x = self.vx/2 * self.vf
        msg.angular.z = -2*self.wz * self.vf
        return msg

    def shutdown(self):
        self.shutting_down = True
        self.sub.unregister()
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0
        self.pub.publish(msg)
        self.rate.sleep()
        rospy.loginfo("Stop rUBot")

if __name__ == '__main__':
    try:
        wall_follower = WallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass