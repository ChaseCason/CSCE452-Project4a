from p4pkg.disc_robot import load_disc_robot
from p4pkg.world_reader import read_world
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import tf2_ros
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import  Node

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
        
        self.w_tf_transform = tf2_ros.TransformBroadcaster(self) #initialize broadcaster

        self.w_tf_transform = TransformStamped() #Setup transform world to base_link
        self.w_tf_transform.header.stamp = self.get_clock().now().to_msg()
        self.w_tf_transform.header.frame_id = "world"
        self.w_tf_transform.child_frame_id = "base_link"
        self.w_tf_transform.transform.translation.x = initialPosition[0]
        self.w_tf_transform.transform.translation.y = initialPosition[1]
        self.w_tf_transform.transform.translation.z = 0.0
        self.w_tf_transform.transform.rotation.x = 0.0
        self.w_tf_transform.transform.rotation.y = initialPosition[2]
        self.w_tf_transform.transform.rotation.z = 0.0
        self.w_tf_transform.transform.rotation.w = 1.0

        self.bl_tf_transform = tf2_ros.TransformBroadcaster(self) #initialize broadcaster

        self.bl_tf_transform = TransformStamped() #Setup transform base_link to laser
        self.bl_tf_transform.header.stamp = self.get_clock().now().to_msg()
        self.bl_tf_transform.header.frame_id = "base_link"
        self.bl_tf_transform.child_frame_id = "laser"
        self.bl_tf_transform.transform.translation.x = self.robot['body']['radius'] * 0.5
        self.bl_tf_transform.transform.translation.y = 0.0
        self.bl_tf_transform.transform.translation.z = 0.0
        self.bl_tf_transform.transform.rotation.x = 0.0
        self.bl_tf_transform.transform.rotation.y = 0.0
        self.bl_tf_transform.transform.rotation.z = 0.0
        self.bl_tf_transform.transform.rotation.w = 1.0

        self.w_tf_broadcaster.sendTransform(self.tf_transform)
        self.bl_tf_broadcaster.sendTransform(self.tf_transform)
        
        create_map(self, occupancyGridMsg)

def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node('simulator')
    
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
