#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time


class WallFollower:
    def __init__(self):
        rospy.init_node('holonomic_wall_follower', anonymous=False)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(25)

        self.d = rospy.get_param("~distance_laser", 0.3)  # Default value added
        self.vx = rospy.get_param("~forward_speed", 0.2)  # Default value added
        self.wz = rospy.get_param("~rotation_speed", 1.0)  # Default value added
        self.vf = rospy.get_param("~speed_factor", 1.0)  # Default value added
        self.vy = rospy.get_param("~lateral_speed", 0.1) # Default value added

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
        
        back_min_1 = int(350 * self.scanRangesLengthCorrectionFactor)
        back_max_1 = int(360 * self.scanRangesLengthCorrectionFactor)  # 350 a 360 grados

        back_min_2 = int(0 * self.scanRangesLengthCorrectionFactor)
        back_max_2 = int(10 * self.scanRangesLengthCorrectionFactor)  # 0 a 10 grados

        # Ahora tomar los valores de los dos rangos para el 'back'
        back_range_1 = msg.ranges[back_min_1:back_max_1]
        back_range_2 = msg.ranges[back_min_2:back_max_2]

        # Combina los dos rangos y obtiene el valor mínimo
        back_value = min(min(back_range_1), min(back_range_2))


        # Ahora usa 'back_value' en la asignación de la variable 'regions'
        regions = {
            'bright': min(min(msg.ranges[bright_min:bright_max]), 3),
            'right': min(min(msg.ranges[right_min:right_max]), 3),
            'fright': min(min(msg.ranges[fright_min:fright_max]), 3),
            'front': min(min(msg.ranges[front_min:front_max]), 3),
            'back': back_value,  # Usamos el valor combinado de la parte trasera
        }   

        self.take_action(regions)

    def take_action(self, regions):
        msg = Twist()
        linear_x = 0
        linear_y = 0

        state_description = ''

        if regions['back'] < self.d and regions['right'] > self.d:
            state_description = 'case 6 - obstacle back, move sideways'
            linear_x = 0
            linear_y = -self.vy * self.vf  # Movimiento lateral hacia la derecha

        # Caso 2: Obstáculo frontal, mueve lateralmente a la izquierda.
        elif regions['front'] < self.d:
            state_description = 'case 2 - obstacle front, move sideways'
            linear_x = 0
            linear_y = self.vy * self.vf  # Movimiento lateral hacia la izquierda

        # Caso 3: Obstáculo cerca de la parte frontal derecha, ajusta a la izquierda.
        elif regions['fright'] < self.d:
            state_description = 'case 3 - close to front-right, adjust left'
            linear_x = self.vx * self.vf 
            linear_y = self.vy * self.vf / 2

        # Caso 4: Obstáculo en el lado derecho, ajusta hacia la izquierda.
        elif regions['right'] < self.d:
            state_description = 'case 4 - right wall too close, adjust left'
            linear_x = self.vx * self.vf
            linear_y = self.vy * self.vf / 2

        # Caso 5: Obstáculo cerca de la parte trasera derecha, ajusta hacia la izquierda.
        elif regions['bright'] < self.d:
            state_description = 'case 5 - right back too close, adjust left'
            linear_x = self.vx * self.vf 
            linear_y = -self.vy * self.vf


        # Caso 7: No hay obstáculos cerca, sigue recto.
        else:
            state_description = 'case 7 - Far, maintain trajectory'
            linear_x = self.vx * self.vf
            linear_y = 0

        ##rospy.loginfo(state_description)
        msg.linear.x = linear_x
        msg.linear.y = linear_y
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
        #rospy.loginfo("Stop rUBot")

if __name__ == '__main__':
    try:
        wall_follower = WallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass