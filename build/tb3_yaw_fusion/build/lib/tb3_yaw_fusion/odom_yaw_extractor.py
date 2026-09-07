import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion
from math import pi

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(
            Float64, 
            '/yaw/odom', 
            10)

    def listener_callback(self, msg):
        yaw = self.convert_to_yaw(msg)
        msg = Float64()
        msg.data = float(yaw)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%lf"' % msg.data)
        
    def convert_to_yaw(self, msg):
        quat = (
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        _ , _ , yaw = euler_from_quaternion(quat) # roll, pitch, yaw 
        if (yaw > pi):
            yaw = -pi + yaw%(pi)
        if (yaw < -pi):
            yaw = pi - yaw%(pi)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomNode()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()