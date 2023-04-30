from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

import tf2_ros
import geometry_msgs.msg as gm
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

import math
import numpy as np
import random
from p4pkg.disc_robot import *


class Navigation(Node):
    def __init__(self):
        super().__init__('velocity')
        self.laserSubscriber = self.create_subscription(
            LaserScan, "scan", self.laser_callback, 10)
        self.velPublisher = self.create_publisher(gm.Twist, 'cmd_vel', 10)

        self.robot = load_disc_robot("normal.robot")
        self.rad = self.robot['body']['radius']

        #Create list of points on the robot to check for collisions
        self.circle_list = []
        for angle in range(0, 181, 5):
            rads = math.radians(angle)

            x = self.rad * math.cos(rads)
            y = self.rad * math.sin(rads)

            self.circle_list.append((x, y))

    def laser_callback(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        ranges = msg.ranges
        angle_inc = msg.angle_increment

        vel = gm.Twist()

        goStraight = True
        turnLeft = False
        turnRight = False

        velocity = .3
        radius = self.rad

        #Collision Detection
        for i in range(len(ranges)):
            # Get current range
            r = ranges[i]
            # Get the Theta of Lidar
            theta = i * angle_inc + angle_min
            # Find the x/y distance to the obstacle from center
            ydist_to_obstacle = r * math.cos(abs(theta)) + radius * 0.5
            xdist_to_obstacle = r * math.sin(abs(theta))

            for point in self.circle_list:
                if point[0] < 0 and theta > 0 or point[0] > 0 and theta < 0:
                    future_x = abs(point[0])
                    future_y = point[1] + velocity
                    if future_y >= ydist_to_obstacle and future_x >= xdist_to_obstacle:
                        goStraight = False
                        if point[0] < 0:
                            turnRight = True
                        else:
                            turnLeft = True


        if goStraight:
            vel.linear.x = velocity
            vel.angular.z = 0.0
        elif math.isnan(ranges[len(ranges)-1]) or math.isnan(ranges[0]):
            pass
        else:
            if (turnRight and not turnLeft) or ranges[0] > ranges[len(ranges)-1]:
                # turn right
                vel.angular.z = -1.57/2
            else:
                # turn left
                vel.angular.z = 1.57/2

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
