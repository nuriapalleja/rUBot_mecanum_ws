#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import sign

class WallFollower:
    def __init__(self):
        # Initialize ros node
        rospy.init_node('wall_follower', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Creates a cmd_vel topic publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Creates a scan topic subscriber
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        # Set node rate
        self.rate = rospy.Rate(25)

        # node parameters
        self.d = rospy.get_param("~distance_laser", 0.3)  # Default value added
        self.vx = rospy.get_param("~forward_speed", 0.2)  # Default value added
        self.wz = rospy.get_param("~rotation_speed", 1.0)  # Default value added
        self.vf = rospy.get_param("~speed_factor", 1.0)  # Default value added

        self.msg = None

        # Global variable userd to stop scanning threads when main node is shutting down
        self.shutting_down = False

    def run(self):
        '''
        Run robot
        '''

        # Setup
        while not self.start():
            rospy.loginfo('Waiting for setup')

        while not self.is_wall_found():
            rospy.loginfo('Looking for a wall')
            self.go_ahead()
        
        # Main loop to continuously publish velocity messages
        while not rospy.is_shutdown():
            self.take_action()
            self.rate.sleep()


    def start(self):
        '''
        Wait util the first scann and sets scanner regions

        Returns:
            bool: True if setup is made, False otherwise
        '''

        if not self.msg:
            return False

        # Set lidar correction factor
        self.scanRangesLengthCorrectionFactor = len(self.msg.ranges) / 360

        # Set lidar ranges
        self.bright_min = int(30 * self.scanRangesLengthCorrectionFactor)
        self.bright_max = int(90 * self.scanRangesLengthCorrectionFactor)
        self.right_min = int(90 * self.scanRangesLengthCorrectionFactor)
        self.right_max = int(120 * self.scanRangesLengthCorrectionFactor)
        self.fright_min = int(120 * self.scanRangesLengthCorrectionFactor)
        self.fright_max = int(170 * self.scanRangesLengthCorrectionFactor)
        self.front_min= int(170 * self.scanRangesLengthCorrectionFactor)
        self.front_max = int(190 * self.scanRangesLengthCorrectionFactor)

        return True

    
    def is_wall_found(self):
        '''
        Check if a front wall is found
        '''
        return min(self.msg.ranges[self.front_min:self.front_max]) < self.d


    def scan_callback(self, msg):
        '''
        Read messages from lidar scanner

        Params:
        -------
            msg: sensor_msgs.msg.LaserScan
                message with distance from all laser beams
        '''

        if self.shutting_down: # main node is shutting down
            return

        self.msg = msg # Read and store the message once


        
    def take_action(self):
        '''
        Main algorithm. Decides where the robot sould go to follow the wall.
        
        Idea:
        ----

        1 - Find a wall
        2 - Turn to the left to follow the wall with the right hand
        3 - Adjust velocities to keep the robot near the wall

    
        '''

        self.regions = {
            'bright': min(min(self.msg.ranges[self.bright_min:self.bright_max]), 3),
            'right': min(min(self.msg.ranges[self.right_min:self.right_max]), 3),
            'fright': min(min(self.msg.ranges[self.fright_min:self.fright_max]), 3),
            'front': min(min(self.msg.ranges[self.front_min:self.front_max]), 3),
        }

        ### ALGORITMO ###

        if self.regions['front'] < self.d:
            rospy.loginfo('case 2 - front')
            self.turn_left()
        elif self.regions['fright'] < self.d and self.regions['right'] > self.d:
            rospy.loginfo('case 3 - fright')
            self.turn_left()
        elif self.regions['fright'] < self.d and self.regions['fright'] < self.regions['right'] * 2: # controla la distancia horizontal
            rospy.loginfo('case 3 - fright')
            self.turn_left()
        elif self.regions['right'] < self.d:
            rospy.loginfo('case 4 - right')
            self.go_ahead()
        elif self.regions['bright'] < self.d:
            rospy.loginfo('case 5 - bright')
            self.turn_right()
        else:
            rospy.loginfo('case 6 - Far')
            self.reorient()

        """
        # Case 1 - wall is in front
        if self.regions['front'] < self.d:
            self.turn_left()
        # Case 2 - wall is on front - right
        elif self.regions['fright'] < self.d and self.regions['fright'] < self.regions['right']:
            self.turn_left()
        # Case 3 - wall is on right    
        elif self.regions['right'] < self.d and self.regions['right'] < self.regions['bright']:
            self.go_ahead()
        # Case 4 - wall is on back - right
        elif self.regions['bright'] < self.d:
            self.turn_right()
        # Case 5 - wall is getting far
        else:
            self.reorient()
        """



    def go_ahead(self):
        msg = Twist()
        msg.linear.x = self.vx * self.vf
        self.pub.publish(msg)


    def turn_left(self):
        msg = Twist()
        msg.angular.z = self.wz * self.vf
        self.pub.publish(msg)


    def turn_right(self):
        msg = Twist()
        msg.angular.z = -self.wz * self.vf
        self.pub.publish(msg)


    def reorient(self):
        msg = Twist()
        msg.linear.x = self.vx/2 * self.vf
        msg.angular.z = -2*self.wz * self.vf
        self.pub.publish(msg)


    def shutdown(self):
        '''
        Stop all node processament when shutdown
        '''
        self.shutting_down = True # global variable 
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
        wall_follower.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass