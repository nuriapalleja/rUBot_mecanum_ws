#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import sign

class rUBot:
    def __init__(self):
        rospy.init_node("rubot_nav", anonymous=False)

        self._distanceLaser = rospy.get_param("~distance_laser", 0.5)
        self._speedFactor = rospy.get_param("~speed_factor", 1.0)
        self._forwardSpeed = rospy.get_param("~forward_speed", 0.2)
        self._backwardSpeed = rospy.get_param("~backward_speed", -0.1)
        self._rotationSpeed = rospy.get_param("~rotation_speed", 1.0)

        self._msg = Twist()
        self._cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.on_shutdown(self.shutdown_callback)

        self._rate = rospy.Rate(25)

        self.__isScanRangesLengthCorrectionFactorCalculated = False
        self.__scanRangesLengthCorrectionFactor = 2

        self._shutting_down = False  # flag to ensure a proper shutdown
        self.closest_distance = float('inf')
        self.closest_angle = 0
        self.last_distance = float('inf')
        self.last_angle = 0
        self.change_threshold_distance = 0.05  # 5 cm
        self.change_threshold_angle = 5  # 5 degrees
        self.current_scan = None

    def scan_callback(self, scan):
        if self._shutting_down:  # check flag
            return

        # Actualiza los datos del escaneo sin hacer cálculos
        if not self.__isScanRangesLengthCorrectionFactorCalculated:
            self.__scanRangesLengthCorrectionFactor = int(len(scan.ranges) / 360)
            self.__isScanRangesLengthCorrectionFactorCalculated = True

        # Guarda el escaneo actual
        self.current_scan = scan

    def calculate_closest_obstacle(self):
        if not self.current_scan:
            return

        scan = self.current_scan

        # Encuentra la distancia y el ángulo más cercano
        closest_distance, elementIndex = min(
            (val, idx) for (idx, val) in enumerate(scan.ranges)
            if scan.range_min < val < scan.range_max
        )

        angleClosestDistance = elementIndex / self.__scanRangesLengthCorrectionFactor
        angleClosestDistance = angleClosestDistance - 180
        angleClosestDistance = (angleClosestDistance + 180) % 360 - 180

        # Solo actualiza si hay cambios significativos
        if (abs(closest_distance - self.last_distance) > self.change_threshold_distance or
                abs(angleClosestDistance - self.last_angle) > self.change_threshold_angle):
            self.closest_distance = closest_distance
            self.closest_angle = angleClosestDistance
            self.last_distance = closest_distance
            self.last_angle = angleClosestDistance
            #rospy.loginfo(f"Closest distance: {closest_distance:.2f} meters and Angle: {angleClosestDistance:.1f} degrees")

    def control_loop(self):
        while not rospy.is_shutdown():
            self.calculate_closest_obstacle()

            if self.closest_distance < self._distanceLaser and -80 < self.closest_angle < 80:
                self._msg.linear.x = self._backwardSpeed * self._speedFactor
                self._msg.angular.z = -sign(self.closest_angle) * self._rotationSpeed * self._speedFactor
            else:
                self._msg.linear.x = self._forwardSpeed * self._speedFactor
                self._msg.angular.z = 0.0

            self._cmdVel.publish(self._msg)
            self._rate.sleep()

    def shutdown_callback(self):
        self._shutting_down = True  # set flag
        self._scan_sub.unregister()  # unsubscribe
        self._msg.linear.x = 0
        self._msg.linear.y = 0
        self._msg.angular.z = 0
        self._cmdVel.publish(self._msg)
        self._rate.sleep()
        rospy.loginfo("rUBot stopped.")

if __name__ == '__main__':
    rUBot1 = rUBot()
    rUBot1.control_loop()
