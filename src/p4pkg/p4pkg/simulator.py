from p4pkg.disc_robot import load_disc_robot
from p4pkg.world_reader import read_world
# from disc_robot import load_disc_robot
# from world_reader import read_world
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import tf2_ros
import geometry_msgs.msg as gm
import rclpy
import numpy as np
import random
from rclpy.node import  Node

from nav_msgs.msg import OccupancyGrid

import math



def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class Simulator(Node):

    def create_map(self, msg):
        self.mapPublisher.publish(self.occupancyGridMsg)

    def update_transforms(self):
        self.base_link.header.stamp = self.get_clock().now().to_msg()
        self.base_link.transform.translation.x = self.x
        self.base_link.transform.translation.y = self.y
        self.base_link_quat = quaternion_from_euler(0, 0, self.theta)
        self.base_link.transform.rotation.x = self.base_link_quat[0]
        self.base_link.transform.rotation.y = self.base_link_quat[1]
        self.base_link.transform.rotation.z = self.base_link_quat[2]
        self.base_link.transform.rotation.w = self.base_link_quat[3]
        self.base_link_broadcast.sendTransform(self.base_link)

    def vl_callback(self, msg):
        self.vl = msg.data
        self.update_position()

    def vr_callback(self, msg):
        self.vr = msg.data
        self.update_position()

    def update_position(self):
        dt =0.1 # time step
        v = (self.vr + self.vl) / 2
        w = (self.vr - self.vl) / self.l

        dx = v * dt * math.cos(self.theta)
        dy = v * dt * math.sin(self.theta)
        dtheta = w * dt

        self.x += dx * self.occupancyGridMsg.info.resolution
        self.y += dy * self.occupancyGridMsg.info.resolution

        self.theta += dtheta

        self.update_transforms()

    def __init__(self):
        super().__init__('simulator')


        self.robot = load_disc_robot("normal.robot")
        self.l = self.robot['wheels']['distance']  # distance between robot's wheels
        self.rad = self.robot['body']['radius']

        self.laser_rate = self.robot['laser']['rate']
        self.laser_count = self.robot['laser']['count']
        self.angle_min = self.robot['laser']['angle_min']
        self.angle_max = self.robot['laser']['angle_max']
        self.range_min = self.robot['laser']['range_min']
        self.range_max = self.robot['laser']['range_max']
        self.error = self.robot['laser']['error_variance']
        self.fail_prob = self.robot['laser']['fail_probability']

        self.world = read_world('brick.world')  # tuple (occupancyGridMsg, pose)
        self.occupancyGridMsg = self.world[0]
        self.initialPosition = self.world[1]

        self.x = self.initialPosition[0]
        self.y = self.initialPosition[1]
        self.theta = self.initialPosition[2]
        self.vl = 0.0
        self.vr = 0.0
        
        self.leftWheelSubscriber = self.create_subscription(Float64, "vl", self.vl_callback, 10)
        self.rightWheelSubscriber = self.create_subscription(Float64, "vr", self.vr_callback, 10)
        self.mapPublisher = self.create_publisher(OccupancyGrid, 'map', 10)
        self.laserPublisher = self.create_publisher(LaserScan, 'scan', 10)
        

        
        

        #self.mapPublisher.publish(self.occupancyGridMsg)
        self.tf_time  = self.create_timer(self.laser_rate, self.tf_callback)
        self.laser_time  = self.create_timer(self.laser_rate, self.laser_callback)
        
        # create_map(self, occupancyGridMsg)

    def tf_callback(self):
        # self.world_broadcast = tf2_ros.StaticTransformBroadcaster(self) #initialize broadcaster

        # self.world = gm.TransformStamped() #Setup transform world to base_link
        # self.world.header.stamp = self.get_clock().now().to_msg()
        # self.world.header.frame_id = ""
        # self.world.child_frame_id = "world" #WE NEED TO VERIFY THIS
        

        # self.world_broadcast.sendTransform(self.world)


        self.base_link_broadcast = tf2_ros.TransformBroadcaster(self) #initialize broadcaster
        
        self.base_link = gm.TransformStamped() #Setup transform world to base_link
        self.base_link.header.stamp = self.get_clock().now().to_msg()
        self.base_link.header.frame_id = "world"
        self.base_link.child_frame_id = "base_link" #WE NEED TO VERIFY THIS

        self.base_link_quat = quaternion_from_euler(0, 0, self.theta)

        self.base_link.transform.translation.x = self.x
        self.base_link.transform.translation.y = self.y

        self.base_link.transform.rotation.x = self.base_link_quat[0]
        self.base_link.transform.rotation.y = self.base_link_quat[1]
        self.base_link.transform.rotation.z = self.base_link_quat[2]
        self.base_link.transform.rotation.w = self.base_link_quat[3]
        
        self.base_link_broadcast.sendTransform(self.base_link)

        # self.world2laser_broadcast = tf2_ros.TransformBroadcaster(self) #initialize broadcaster
        
        # self.world2laser = gm.TransformStamped() #Setup transform world to world2laser
        # self.world2laser.header.stamp = self.get_clock().now().to_msg()
        # self.world2laser.header.frame_id = "world"
        # self.world2laser.child_frame_id = "laser" #WE NEED TO VERIFY THIS

        # self.world2laser_quat = quaternion_from_euler(0, 0, self.theta)

        # self.world2laser.transform.translation.x = self.x
        # self.world2laser.transform.translation.y = self.y

        # self.world2laser.transform.rotation.x = self.base_link_quat[0] + 0.5*self.rad
        # self.world2laser.transform.rotation.y = self.base_link_quat[1]
        # self.world2laser.transform.rotation.z = self.base_link_quat[2]
        # self.world2laser.transform.rotation.w = self.base_link_quat[3]
        
        # self.world2laser_broadcast.sendTransform(self.world2laser)

        self.laser_broadcast = tf2_ros.TransformBroadcaster(self) #initialize broadcaster
        
        self.laser = gm.TransformStamped() #Setup transform world to laser
        self.laser.header.stamp = self.get_clock().now().to_msg()
        self.laser.header.frame_id = "base_link"
        self.laser.child_frame_id = "laser" #WE NEED TO VERIFY THIS

        self.laser.transform.translation.x = 0.5*self.rad

        self.laser_broadcast.sendTransform(self.laser)

        self.occupancyGridMsg.header.stamp = self.get_clock().now().to_msg()
        self.mapPublisher.publish(self.occupancyGridMsg)

    def laser_callback(self):
        
        
        ranges = []
        #Defines how much range increases for each step in obstacle check
        #THIS VALUE MAY NEED TO CHANGE
        range_gap = .001
        angle_gap = (self.angle_max - self.angle_min) / self.laser_count
        for i in range(self.laser_count):
            if random.random() < self.fail_prob:
                ranges.append(float('nan'))
                continue

            #angle relative to laser frame 
            theta = self.theta + self.angle_min + (i * angle_gap)
            x = self.x + (0.5*self.rad * math.cos(theta))
            y = self.y + (0.5*self.rad * math.sin(theta))
            

            
            #angle relative to world (which one do we use?)
            #theta = math.atan2(math.sin(angle + self.laser.transform.rotation.z), math.cos(angle + self.laser.transform.rotation.z))
            curr_range = self.range_min
            hit_obstacle = False
            
            
            while curr_range < self.range_max:
                test_x = (x + curr_range*math.cos(theta)) 
                test_y = (y + curr_range*math.sin(theta)) 
                curr_x = int((x + curr_range*math.cos(theta)) / self.occupancyGridMsg.info.resolution)
                curr_y = int((y + curr_range*math.sin(theta)) / self.occupancyGridMsg.info.resolution)

                #check to make sure still inside grid
                if curr_x < 0 or curr_x >= self.occupancyGridMsg.info.width or curr_y < 0 or curr_y >= self.occupancyGridMsg.info.height:
                    #print("Outside grid")
                    break
                if self.occupancyGridMsg.data[curr_y * self.occupancyGridMsg.info.width + curr_x] == 100:
                    #print(i)
                    hit_obstacle = True
                    # out_range = math.sqrt((test_x-x)**2 + (test_y-y)**2) 
                    break

                curr_range += range_gap

            if(hit_obstacle):
                # print('------------------')
                # print(self.occupancyGridMsg.data[curr_y * self.occupancyGridMsg.info.width + curr_x])
                # print(curr_y * self.occupancyGridMsg.info.width + curr_x)
                # print(theta)
                ranges.append(curr_range + random.gauss(0,self.error))
            else:
                ranges.append(float('inf'))

        laser_msg = LaserScan()
        laser_msg.header.frame_id = "laser"
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.angle_min = self.angle_min
        laser_msg.angle_max = self.angle_max
        laser_msg.angle_increment = angle_gap
        laser_msg.scan_time = float(self.laser_rate)
        laser_msg.range_min = self.range_min
        laser_msg.range_max = self.range_max
        laser_msg.ranges = ranges
        self.laserPublisher.publish(laser_msg)

def main(args=None):
    rclpy.init(args=args)
    print("BEGIN")
    sim = Simulator()

    try:
        rclpy.spin(sim)
    except KeyboardInterrupt:
        pass

    sim.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
