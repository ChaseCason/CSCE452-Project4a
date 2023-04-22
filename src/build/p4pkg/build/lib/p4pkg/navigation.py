from p4pkg.disc_robot import load_disc_robot

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


