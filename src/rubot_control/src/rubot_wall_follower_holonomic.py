#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

pub = None
d = 0
vx = 0
vy = 0
vf = 1  # Valor por defecto, puedes ajustarlo según sea necesario

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 2


def clbk_laser(msg):
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated:
        scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
        isScanRangesLengthCorrectionFactorCalculated = True

    bright_min = int(30 * scanRangesLengthCorrectionFactor)
    bright_max = int(90 * scanRangesLengthCorrectionFactor)
    right_min = int(90 * scanRangesLengthCorrectionFactor)
    right_max = int(120 * scanRangesLengthCorrectionFactor)
    fright_min = int(120 * scanRangesLengthCorrectionFactor)
    fright_max = int(170 * scanRangesLengthCorrectionFactor)
    front_min = int(170 * scanRangesLengthCorrectionFactor)
    front_max = int(190 * scanRangesLengthCorrectionFactor)
    
    back_min_1 = int(350 * scanRangesLengthCorrectionFactor)
    back_max_1 = int(360 * scanRangesLengthCorrectionFactor)  # 350 a 360 grados

    back_min_2 = int(0 * scanRangesLengthCorrectionFactor)
    back_max_2 = int(10 * scanRangesLengthCorrectionFactor)  # 0 a 10 grados

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

    take_action(regions)


def take_action(regions):
    msg = Twist()
    linear_x = 0
    linear_y = 0

    state_description = ''

    if regions['back'] < d and regions['right'] > d:
        state_description = 'case 6 - obstacle back, move sideways'
        linear_x = 0
        linear_y = -vy * vf  # Movimiento lateral hacia la derecha


    # Caso 2: Obstáculo frontal, mueve lateralmente a la izquierda.
    elif regions['front'] < d:
        state_description = 'case 2 - obstacle front, move sideways'
        linear_x = 0
        linear_y = vy * vf  # Movimiento lateral hacia la izquierda

    # Caso 3: Obstáculo cerca de la parte frontal derecha, ajusta a la izquierda.
    elif regions['fright'] < d:
        state_description = 'case 3 - close to front-right, adjust left'
        linear_x = vx * vf 
        linear_y = vy * vf /2

    # Caso 4: Obstáculo en el lado derecho, ajusta hacia la izquierda.
    elif regions['right'] < d:
        state_description = 'case 4 - right wall too close, adjust left'
        linear_x = vx * vf
        linear_y = vy * vf / 2

    # Caso 5: Obstáculo cerca de la parte trasera derecha, ajusta hacia la izquierda.
    elif regions['bright'] < d:
        state_description = 'case 5 - right back too close, adjust left'
        linear_x = vx * vf 
        linear_y = -vy * vf


    # Caso 7: No hay obstáculos cerca, sigue recto.
    else:
        state_description = 'case 7 - Far, maintain trajectory'
        linear_x = vx * vf
        linear_y = 0

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    pub.publish(msg)
    rate.sleep()

def shutdown():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Stop rUBot")


def main():
    global pub
    global sub
    global rate
    global d
    global vx
    global vy
    global wz
    global vf

    rospy.init_node('holonomic_wall_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(25)

    d = rospy.get_param("~distance_laser")
    vx = rospy.get_param("~forward_speed")
    vy = rospy.get_param("~lateral_speed")
    vf = rospy.get_param("~speed_factor")


if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        shutdown()
