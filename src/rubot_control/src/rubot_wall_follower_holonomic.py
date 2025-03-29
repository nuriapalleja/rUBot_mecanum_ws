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

        self.d = rospy.get_param("~distance_laser", 0.3)
        self.vx = rospy.get_param("~forward_speed", 0.2)
        self.wz = rospy.get_param("~rotation_speed", 1.0)
        self.vf = rospy.get_param("~speed_factor", 1.0)
        self.vy = rospy.get_param("~lateral_speed", 0.1)

        self.isScanRangesLengthCorrectionFactorCalculated = False
        self.scanRangesLengthCorrectionFactor = 2

        self.shutting_down = False
        self.regions = None

        # Inicializamos los índices de las regiones
        self.bright_min = None
        self.bright_max = None
        self.right_min = None
        self.right_max = None
        self.fright_min = None
        self.fright_max = None
        self.front_min = None
        self.front_max = None
        self.fleft_min = None
        self.fleft_max = None
        self.left_min = None
        self.left_max = None
        self.bleft_min = None
        self.bleft_max = None
        self.back_min_1 = None
        self.back_max_1 = None
        self.back_min_2 = None
        self.back_max_2 = None


    def calculate_region_indices(self, msg):
        # Solo calculamos los índices una vez
        if not self.isScanRangesLengthCorrectionFactorCalculated:
            self.scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
            self.bright_min = int(30 * self.scanRangesLengthCorrectionFactor)
            self.bright_max = int(90 * self.scanRangesLengthCorrectionFactor)
            self.right_min = int(90 * self.scanRangesLengthCorrectionFactor)
            self.right_max = int(120 * self.scanRangesLengthCorrectionFactor)
            self.fright_min = int(120 * self.scanRangesLengthCorrectionFactor)
            self.fright_max = int(170 * self.scanRangesLengthCorrectionFactor)
            self.front_min = int(170 * self.scanRangesLengthCorrectionFactor)
            self.front_max = int(190 * self.scanRangesLengthCorrectionFactor)
            self.fleft_min = int(190 * self.scanRangesLengthCorrectionFactor)
            self.fleft_max = int(240 * self.scanRangesLengthCorrectionFactor)
            self.left_min = int(240 * self.scanRangesLengthCorrectionFactor)
            self.left_max = int(270 * self.scanRangesLengthCorrectionFactor)
            self.bleft_min = int(270 * self.scanRangesLengthCorrectionFactor)
            self.bleft_max = int(350 * self.scanRangesLengthCorrectionFactor)
            self.back_min_1 = int(350 * self.scanRangesLengthCorrectionFactor)
            self.back_max_1 = int(360 * self.scanRangesLengthCorrectionFactor)
            self.back_min_2 = int(0 * self.scanRangesLengthCorrectionFactor)
            self.back_max_2 = int(10 * self.scanRangesLengthCorrectionFactor)
            self.isScanRangesLengthCorrectionFactorCalculated = True

    def scan_callback(self, msg):
        if self.shutting_down:
            return

        # Calcular índices solo si aún no se ha hecho
        self.calculate_region_indices(msg)

        # Obtener el valor mínimo de las regiones traseras
        back_range_1 = msg.ranges[self.back_min_1:self.back_max_1]
        back_range_2 = msg.ranges[self.back_min_2:self.back_max_2]
        back_value = min(min(back_range_1), min(back_range_2))

        # Actualizamos las regiones de forma segura
        self.regions = {
            'bright': min(min(msg.ranges[self.bright_min:self.bright_max]), 3),
            'right': min(min(msg.ranges[self.right_min:self.right_max]), 3),
            'fright': min(min(msg.ranges[self.fright_min:self.fright_max]), 3),
            'front': min(min(msg.ranges[self.front_min:self.front_max]), 3),
            'fleft': min(min(msg.ranges[self.fleft_min:self.fleft_max]), 3),
            'left': min(min(msg.ranges[self.left_min:self.left_max]), 3),
            'bleft': min(min(msg.ranges[self.bleft_min:self.bleft_max]), 3),
            'back': back_value,
        }

    def take_action(self, regions):
        msg = Twist()
        linear_x = 0
        linear_y = 0

        state_description = ''

        if regions['back'] < self.d and regions['right'] > self.d:
            state_description = 'case 6 - obstacle back, move sideways'
            linear_x = 0
            linear_y = -self.vy * self.vf

        elif regions['front'] < self.d and regions['left'] > self.d:
            state_description = 'case 2 - obstacle front, move sideways'
            linear_x = 0
            linear_y = self.vy * self.vf

        elif regions['left'] < self.d:
            state_description = 'case 10 - left wall too close, adjust right'
            linear_x = -self.vx * self.vf
            linear_y = 0

        elif regions['right'] < self.d:
            state_description = 'case 4 - right wall too close, adjust left'
            linear_x = self.vx * self.vf
            linear_y = 0

        elif regions['fright'] < self.d:
            state_description = 'case 3 - close to front-right, adjust left'
            linear_x = self.vx * self.vf
            linear_y = self.vy * self.vf / 2

        elif regions['bright'] < self.d:
            state_description = 'case 5 - right back too close, adjust left'
            linear_x = self.vx * self.vf
            linear_y = -self.vy * self.vf

        elif regions['bleft'] < self.d and regions['left'] > self.d:
            state_description = 'case 8 - obstacle back left, move sideways right'
            linear_x = -self.vx * self.vf
            linear_y = -self.vy * self.vf / 2

        elif regions['fleft'] < self.d:
            state_description = 'case 9 - obstacle front left, move sideways right'
            linear_x = -self.vx * self.vf /2
            linear_y = self.vy * self.vf

        else:
            # Encontrar la región con la menor distancia
            closest_region = min(regions, key=regions.get)
            state_description = f'case 7 - Far, move towards closest wall: {closest_region}'

            if closest_region in ['right', 'bright']:
                linear_x = 0
                linear_y = -self.vy * self.vf  # Moverse hacia la derecha
            elif closest_region in ['left', 'bleft']:
                linear_x = 0
                linear_y = self.vy * self.vf  # Moverse hacia la izquierda
            elif closest_region in ['front', 'fright', 'fleft']:
                linear_x = self.vx * self.vf  # Avanzar
                linear_y = 0
            elif closest_region == 'back':
                linear_x = -self.vx * self.vf  # Retroceder
                linear_y = 0

        rospy.loginfo(state_description)
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        self.pub.publish(msg)


    def shutdown(self):
        self.shutting_down = True
        self.sub.unregister()
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0
        self.pub.publish(msg)
        self.rate.sleep()

if __name__ == '__main__':
    try:
        wall_follower = WallFollower()

        # Ciclo principal donde llamamos a take_action() en cada iteración
        while not rospy.is_shutdown():
            if wall_follower.regions:  # Asegúrate de que las regiones están definidas
                wall_follower.take_action(wall_follower.regions)

            wall_follower.rate.sleep()

    except rospy.ROSInterruptException:
        pass
