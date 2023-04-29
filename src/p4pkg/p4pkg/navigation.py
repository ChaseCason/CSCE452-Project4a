from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

import tf2_ros
import geometry_msgs.msg as gm
import rclpy
from rclpy.node import  Node

from nav_msgs.msg import OccupancyGrid

import math
import numpy as np
import random


class Navigation(Node):
    def __init__(self):
        super().__init__('velocity')
        self.laserSubscriber = self.create_subscription(LaserScan, "scan", self.laser_callback, 10)
        self.velPublisher = self.create_publisher(gm.Twist, 'cmd_vel', 10)

    def laser_callback(self,msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        ranges = msg.ranges
        angle_inc = msg.angle_increment

        vel = gm.Twist()


        straight_angle_index = int(abs(angle_min) / angle_inc)
        # left_index = int((math.pi/2) - angle_min / angle_inc)
        # right_index = int((-1 * math.pi/2) - angle_min / angle_inc)
        goStraight = True
        turnLeft = False
        turnRight = False

        
        for i in range(len(ranges)):
            if ranges[i] < .29 and i != straight_angle_index:
                if i > straight_angle_index:
                    turnRight = True
                else:
                    turnLeft = True
                
                goStraight = False

        if goStraight and (ranges[straight_angle_index] > .3 or math.isinf(ranges[straight_angle_index]) or math.isnan(ranges[straight_angle_index])):
            vel.linear.x = .1
            vel.angular.z = 0.0
        elif math.isnan(ranges[len(ranges)-1]) or math.isnan(ranges[0]):
            pass
        else:
            if turnRight or ranges[0] > ranges[len(ranges)-1]:
                #turn right
                vel.angular.z = -1.57
            else:
                #turn left
                vel.angular.z = 1.57



        self.velPublisher.publish(vel)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       


def main(args=None):
    rclpy.init(args=args)
    nav = Navigation()

    try:
        rclpy.spin(nav)
    except KeyboardInterrupt:
        pass

    nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


