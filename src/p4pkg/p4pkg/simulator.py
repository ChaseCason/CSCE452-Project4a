from p4pkg.disc_robot import load_disc_robot
from p4pkg.world_reader import read_world
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import tf2_ros
import geometry_msgs.msg as gm
import rclpy
from rclpy.node import  Node

from nav_msgs.msg import OccupancyGrid

import math

class Simulator(Node):

    def create_map(self, msg):
        self.mapPublisher.publish(self.occupancyGridMsg)

    def update_transforms(self):
        self.w_tf_transform.header.stamp = self.get_clock().now().to_msg()
        self.w_tf_transform.transform.translation.x = self.x
        self.w_tf_transform.transform.translation.y = self.y
        self.w_tf_transform.transform.rotation.z = math.sin(self.theta / 2)
        self.w_tf_transform.transform.rotation.w = math.cos(self.theta / 2)
        self.w_tf_broadcaster.sendTransform(self.w_tf_transform)
        self.bl_tf_broadcaster.sendTransform(self.bl_tf_transform)

    def vl_callback(self, msg):
        self.vl = msg.data
        self.update_position()

    def vr_callback(self, msg):
        self.vr = msg.data
        self.update_position()

    def update_position(self):
        dt = 0.1 # time step
        v = (self.vr + self.vl) / 2
        w = (self.vr - self.vl) / self.l

        dx = v * dt * math.cos(self.theta)
        dy = v * dt * math.sin(self.theta)
        dtheta = w * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta

        self.update_transforms()

    def __init__(self):

        self.robot = load_disc_robot("normal.robot")
        self.l = self.robot['wheels']['distance']  # distance between robot's wheels
        self.rad = self.robot['body']['radius']

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
        
        self.world_broadcast = tf2_ros.StaticTransformBroadcaster(self) #initialize broadcaster

        self.world = gm.TransformStamped() #Setup transform world to base_link
        self.world.header.stamp = self.get_clock().now().to_msg()
        self.world.header.frame_id = ""
        self.world.child_frame_id = "world" #WE NEED TO VERIFY THIS
        

        self.world_broadcast.sendTransform(self.world)


        self.base_link_broadcast = tf2_ros.TransformBroadcaster(self) #initialize broadcaster
        
        self.base_link = gm.TransformStamped() #Setup transform world to base_link
        self.base_link.header.stamp = self.get_clock().now().to_msg()
        self.base_link.header.frame_id = "world"
        self.base_link.child_frame_id = "base_link" #WE NEED TO VERIFY THIS

        self.base_link_quat = tf2_ros.transformations.quaternion_from_euler(0, 0, self.theta)
        self.base_link.transform.translation = gm.Vector3(self.x,self.y,0)
        self.base_link.transform.rotation.x = self.base_link_quat[0]
        self.base_link.transform.rotation.y = self.base_link_quat[1]
        self.base_link.transform.rotation.z = self.base_link_quat[2]
        self.base_link.transform.rotation.w = self.base_link_quat[3]
        
        self.base_link_broadcast.sendTransform(self.base_link)



        self.laser_broadcast = tf2_ros.TransformBroadcaster(self) #initialize broadcaster
        
        self.laser = gm.TransformStamped() #Setup transform world to laser
        self.laser.header.stamp = self.get_clock().now().to_msg()
        self.laser.header.frame_id = "base_link"
        self.laser.child_frame_id = "laser" #WE NEED TO VERIFY THIS

        self.laser.transform.translation = gm.Vector3(0.5*self.rad,0,0)

        self.laser_broadcast.sendTransform(self.laser)
        
        create_map(self, occupancyGridMsg)

def main(args=None):
    rclpy.init(args=args)

    sim = Simulator()

    try:
        rclpy.spin(sim)
    except KeyboardInterrupt:
        pass

    sim.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
