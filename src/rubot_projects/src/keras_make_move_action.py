#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class KerasMove:
    def __init__(self):
        # Movement parameters
        self.detection_distance_threshold = 0.4 # Distancia en m a la senyal
        self.detection_timeout = 10 # Segons per considerar una detecció vàlida

        # Odometry
        self.robot_x = 0.0
        self.robot_y = 0.0
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Signal position
        self.signal_x = None
        self.signal_y = None
        self.last_detection_time = None

        # velocity publisher
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # Prediction topic subscription
        rospy.Subscriber("/predicted_class", String, self.prediction_callback)

        rospy.loginfo("[MovementController] nodo activo.")

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.robot_x, self.robot_y = p.x, p.y

    def prediction_callback(self, msg):
        class_name = msg.data
        now = rospy.Time.now()
        self.signal_x = self.robot_x + 0.5
        self.signal_y = self.robot_y
        self.last_detection_time = now
        rospy.loginfo(f"[MovementController] Señal '{class_name}' detectada en ({self.signal_x:.2f}, {self.signal_y:.2f})")
        self.decide_and_move(class_name)

    def decide_and_move(self, class_name):
        if self.last_detection_time is None:
            return
        if (rospy.Time.now() - self.last_detection_time) > self.detection_timeout:
            rospy.loginfo("[MovementController] Detección caducada, no actúo.")
            return

        # Distance calculation
        distance_to_signal = np.sqrt((self.robot_x - self.signal_x)**2 + (self.robot_y - self.signal_y)**2)
        rospy.loginfo(f"[MovementController] Distancia al señal: {distance_to_signal:.2f} m")

        twist_msg = Twist()
        if distance_to_signal <= self.detection_distance_threshold:
            if class_name == "Stop":
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
            elif class_name == "Give_Way":
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0 # Aturar abans de passar
            elif class_name == "Turn_Left":
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.5 # Girar més pronunciadament a l'esquerra
            elif class_name == "Turn_Right":
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = -0.5 # Girar més pronunciadament a la dreta
            self.cmd_vel_pub.publish(twist_msg)

            rospy.loginfo(f"[MovementController] Publicado cmd_vel: linear={twist_msg.linear.x:.2f}, angular={twist_msg.angular.z:.2f}")

if __name__ == "__main__":
    rospy.init_node("keras_move")
    controller = KerasMove()
    rospy.spin()