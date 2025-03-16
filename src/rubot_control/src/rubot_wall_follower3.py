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

        self.d = rospy.get_param("~distance_laser", 0.5)  # Default value added
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

        bright_val, bright_relative_index = min(
            (val, idx) for (idx, val) in enumerate(msg.ranges[bright_min:bright_max])
        )
        bright_val = min(bright_val, 3)
        bright_angle = (bright_relative_index + bright_min) / self.scanRangesLengthCorrectionFactor

        right_val, right_relative_index = min(
            (val, idx) for (idx, val) in enumerate(msg.ranges[right_min:right_max])
        )
        right_val = min(right_val, 3)
        right_angle = (right_relative_index + right_min) / self.scanRangesLengthCorrectionFactor

        fright_val, fright_relative_index = min(
            (val, idx) for (idx, val) in enumerate(msg.ranges[fright_min:fright_max])
        )
        fright_val = min(fright_val, 3)
        fright_angle = (fright_relative_index + fright_min) / self.scanRangesLengthCorrectionFactor

        front_val, front_relative_index = min(
            (val, idx) for (idx, val) in enumerate(msg.ranges[front_min:front_max])
        )
        front_val = min(front_val, 3)
        front_angle = (front_relative_index + front_min) / self.scanRangesLengthCorrectionFactor

        regions = {
            'bright_val': bright_val,
            'bright_angle': bright_angle,
            'right_val': right_val,
            'right_angle': right_angle,
            'fright_val': fright_val,
            'fright_angle': fright_angle,
            'front_val': front_val,
            'front_angle': front_angle,
        }

        self.take_action(regions)

    def take_action(self, regions):
        msg = Twist()
        linear_x = 0
        angular_z = 0

        state_description = ''

        if regions['front'] > self.d and regions['fright'] > 2 * self.d and regions['right'] > 2 * self.d and regions['bright'] > 2 * self.d:
            state_description = 'case 1 - starting'
            linear_x = self.vx
            angular_z = 0
        elif regions['front'] < self.d:
            state_description = 'case 2 - front'
            linear_x = 0
            angular_z = self.wz
        elif regions['fright'] < self.d and regions['right'] > self.d:
            state_description = 'case 3 - fright'
            linear_x = 0
            angular_z = self.wz
        elif regions['right'] < self.d and regions['bright'] > self.d:
            state_description = 'case 4 - right'
            linear_x = self.vx
            angular_z = 0
        elif regions['bright'] < self.d:
            state_description = 'case 5 - bright'
            linear_x = 0
            angular_z = -self.wz
        else:
            state_description = 'case 6 - Far'
            linear_x = self.vx
            angular_z = -self.wz

        rospy.loginfo(state_description)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)
        self.rate.sleep()

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