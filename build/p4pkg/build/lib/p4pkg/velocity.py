
from p4pkg.disc_robot import load_disc_robot

from std_msgs.msg import Float64

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import  Node

class Velocity(Node):
    def __init__(self):
        super().__init__('velocity')
        self.velocitySubscriber = self.create_subscription(Twist, "cmd_vel", self.velocity_callback, 10)
        self.vlPublisher = self.create_publisher(Float64, 'vl', 10)
        self.vrPublisher = self.create_publisher(Float64, 'vr', 10)
        self.robot = load_disc_robot("normal.robot")
        self.l = self.robot['wheels']['distance'] 
    def velocity_callback(self,msg):
        #Get input from cmd_vel
        v = msg.linear.x
        w = msg.angular.z

        #Distnace between robot wheels
        l = self.l
        vr = Float64()
        vl = Float64()

        #Translate anglular and linear velocity to vr and vl
        vr.data = (2*v + w *l) / 2 
        vl.data = (2*v - w*l) / 2
        
        #Publish wheel velocities to /vr and /vl
        self.vrPublisher.publish(vr)
        self.vlPublisher.publish(vl)

def main(args=None):
    rclpy.init(args=args)
    vel = Velocity()

    try:
        rclpy.spin(vel)
    except KeyboardInterrupt:
        pass

    vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()