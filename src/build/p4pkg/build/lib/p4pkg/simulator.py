from p4pkg.disc_robot import load_disc_robot
from p4pkg.world_reader import read_world
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import tf2_ros
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import  Node

class Simulator(Node):

    def create_map(self, msg):
        self.mapPublisher.publish(self.occupancyGridMsg)
        

    def vl_callback(self, msg):
        return

    def vr_callback(self, msg):
        return

    def __init__(self):

        #self.robot = load_disc_robot("normal.robot")
        #self.l = self.robot['wheels']['distance'] #distance between robot's wheels

        self.world = read_world('brick.world') #tuple (occupancyGridMsg, pose)
        self.occupancyGridMsg = self.world[0]
        self.initialPosition = self.world[1]
        """
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
        """
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
