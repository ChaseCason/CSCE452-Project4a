from p4pkg.disc_robot import load_disc_robot
from p4pkg.world_reader import read_world
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import tf2_ros
from geometry_msgs.msg import TransformStamped
import rclpy


class Simulator(Node):
    def __init__(self):

        self.robot = load_disc_robot("normal.robot")
        self.l = robot['wheels']['distance'] #distance between robot's wheels

        self.world = read_world('brick.world') #tuple (occupancyGridMsg, pose)
        self.occupancyGridMsg = world[0]
        self.initialPosition = world[1]

        self.leftWheelSubscriber = node.create_subscription(Float64, "vl", vl_callback)
        self.rightWheelSubscriber = node.create_subscription(Float64, "vr", vr_callback)

        self.mapPublisher = node.create_publisher(OccupancyGrid, 'map')
        self.laserPublisher = node.create_publisher(LaserScan, 'scan')
        
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
        self.bl_tf_transform.transform.translation.x = robot['body']['radius'] * 0.5
        self.bl_tf_transform.transform.translation.y = 0.0
        self.bl_tf_transform.transform.translation.z = 0.0
        self.bl_tf_transform.transform.rotation.x = 0.0
        self.bl_tf_transform.transform.rotation.y = 0.0
        self.bl_tf_transform.transform.rotation.z = 0.0
        self.bl_tf_transform.transform.rotation.w = 1.0

        self.w_tf_broadcaster.sendTransform(self.tf_transform)
        self.bl_tf_broadcaster.sendTransform(self.tf_transform)



    def v1_callback(self, msg):
        
        pass

    def vr_callback(self, msg):

        pass


def main(args=None):

    rclpy.init(args=args)
    node = Simulator()
    
    try:
        mapPublisher.publish(node.occupancyGridMsg)
        #laserPublisher.publish()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
