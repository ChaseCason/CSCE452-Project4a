
from p4pkg.disc_robot import load_disc_robot

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

import tf2_ros
import geometry_msgs.msg as gm
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import  Node

from nav_msgs.msg import OccupancyGrid

import math
import numpy as np
import random






class Velocity(Node):
    def __init__(self):
        super().__init__('velocity')
        self.velocitySubscriber = self.create_subscription(Twist, "cmd_vel", self.velocity_callback, 10)
        self.vlPublisher = self.create_publisher(Float64, 'vl', 10)
        self.vrPublisher = self.create_publisher(Float64, 'vr', 10)
    def velocity_callback(self,msg):
        v = msg.linear.x
        w = msg.angular.z
        l = 2
        vr = Float64()
        vl = Float64()
        vr.data = (2*v + w *l) / 2 
        vl.data = (2*v - w*l) / 2

        self.vlPublisher(vl)
        self.vrPublisher(vr)


def main(args=None):
    rclpy.init(args=args)
    sim = ()

    try:
        rclpy.spin(sim)
    except KeyboardInterrupt:
        pass