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
from disc_robot import *


class Navigation(Node):
    def __init__(self):
        super().__init__('velocity')
        self.laserSubscriber = self.create_subscription(
            LaserScan, "scan", self.laser_callback, 10)
        self.velPublisher = self.create_publisher(gm.Twist, 'cmd_vel', 10)

        self.robot = load_disc_robot("normal.robot")
        self.rad = self.robot['body']['radius']

        self.circle_list = []
        for angle_degrees in range(0, 181, 5):
            angle_radians = math.radians(angle_degrees)

            x = self.rad * math.cos(angle_radians)
            y = self.rad * math.sin(angle_radians)

            self.circle_list.append((x, y))

    def laser_callback(self, msg):
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

        velocity = .1
        radius = self.rad
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

            # Step 1: Calculate X and Y
            # xVal = radius * math.sin(abs(theta))
            # yVal = radius * math.cos(abs(theta))

            # # Step 2: Calculate future X Y
            # newX = xVal + vel * math.cos(theta)
            # newY = yVal + vel * math.sin(theta)
            # # Step 3: Create a Vector From x, y to newX, newY
            # xToNew = (newX, newY) - (xVal, yVal)
            # # Step 4: Find the X and Y of the circle's origin based on the laser
            # circleX = -1.0 * (radius * 0.5 * math.cos(theta))
            # circleY = -1.0 * (radius * 0.5 * math.sin(theta))
            # # Step 5: Create a vector from circle origin to the newX and newY
            # circleToNew = (newX, newY) - (circleX, circleY)
            # Step 6: Project the origin -> new onto anglexy -> new

            # Step 7: Compare the Distances, if we find that Origin is longer then choose the next highest angle, otherwise move forward

            if ranges[i] < .29 and i != straight_angle_index:
                if i > straight_angle_index:
                    turnRight = True
                else:
                    turnLeft = True

                goStraight = False

        if goStraight and (ranges[straight_angle_index] > .3 or math.isinf(ranges[straight_angle_index]) or math.isnan(ranges[straight_angle_index])):
            vel.linear.x = velocity
            vel.angular.z = 0.0
        elif math.isnan(ranges[len(ranges)-1]) or math.isnan(ranges[0]):
            pass
        else:
            if turnRight or ranges[0] > ranges[len(ranges)-1]:
                # turn right
                vel.angular.z = -1.57
            else:
                # turn left
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
