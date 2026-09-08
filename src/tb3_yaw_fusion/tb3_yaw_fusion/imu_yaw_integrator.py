"""Integrate IMU angular velocity into a yaw estimate."""

from math import pi

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

class IMUIntegratorNode(Node):
    """Publish yaw integrated from IMU angular velocity."""

    def __init__(self):
        super().__init__('imu__integrator_node')
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(
            Float64,
            '/yaw/imu',
            10)
        self.time_p = 0
        self.theta_p = 0

    def listener_callback(self, msg):
        """Integrate an IMU message and publish the resulting yaw."""
        self.integrate_yaw(msg)
        msg = Float64()
        msg.data = float(self.theta_p)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def integrate_yaw(self, msg):
        """Update the integrated yaw using an IMU message timestamp."""
        time = (msg.header.stamp.nanosec / 1e9) + msg.header.stamp.sec
        if self.time_p == 0:
            self.time_p = time
            self.theta_p = 0
        theta = self.theta_p + msg.angular_velocity.z * (time - self.time_p)
        if theta > pi:
            theta = -pi + theta%(pi)
        if theta < -pi:
            theta = pi - theta%(pi)
        self.theta_p = theta
        self.time_p = time

def main(args=None):
    """Run the IMU yaw integrator node."""
    rclpy.init(args=args)
    imu_node = IMUIntegratorNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
