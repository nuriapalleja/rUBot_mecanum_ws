#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
global msg
msg = Twist()
class SimpleNavigator:
    def __init__(self):
        rospy.init_node("simple_nav")

        # Parameters
        self.safe_distance = rospy.get_param("~safe_distance", 0.5)
        self.forward_speed = rospy.get_param("~forward_speed", 0.2)
        self.backward_speed = rospy.get_param("~backward_speed", -0.1)
        self.rotation_speed = rospy.get_param("~rotation_speed", 1.0)

        self.__isScanRangesLengthCorrectionFactorCalculated = False
        self.__scanRangesLengthCorrectionFactor = 2
        self.angle_closest_distance = -90
        self.element_index = 0

        # Velocity publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=10)

        self.closest_distance = float("inf")
        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, scan):
        if not self.__isScanRangesLengthCorrectionFactorCalculated:
            self.__scanRangesLengthCorrectionFactor = int(len(scan.ranges) / 360)
            self.__isScanRangesLengthCorrectionFactorCalculated = True
        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = 999  # Replace inf values with a large number
        self.closest_distance, self.elementIndex = min(
            (val, idx) for (idx, val) in enumerate(scan.ranges)
            if scan.range_min < val < scan.range_max
        )

    def run(self):
        global msg

        while not rospy.is_shutdown():
            if self.closest_distance < self.safe_distance and -70 < self.angle_closest_distance < 70:
                msg.linear.x = self.backward_speed
                msg.angular.z = -np.sign(self.angle_closest_distance) * self.rotation_speed
            else:
                msg.linear.x = self.forward_speed
                msg.angular.z = 0
            
            self.angle_closest_distance = self.element_index / self.__scanRangesLengthCorrectionFactor
            #rospy.loginfo(f"Angle Closest Distance (raw): {angleClosestDistance:.0f} degrees")
            # To take into account the Lidar Zero angle is on the back
            self.angle_closest_distance = self.angle_closest_distance - 180
            # To wrapp the angle to the ranege [+180, -180]
            self.angle_closest_distance = (self.angle_closest_distance + 180) % 360 - 180

            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()


if __name__ == "__main__":
    node = SimpleNavigator()
    node.run()