"""Extract yaw from odometry orientation messages."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from .utils import wrap



class FusedNode(Node):
    """Publish yaw extracted from odometry quaternion orientation."""

    def __init__(self):
        super().__init__("fused_node")
        self.declare_parameter("alpha", .98)
        self.yaw_imu_subscription = self.create_subscription(
            Float64, "yaw/imu", self.imu_listener_callback, 10
        )
        self.yaw_odom_subscription = self.create_subscription(
            Float64, "yaw/odom", self.odom_listener_callback, 10
        )
        self.publisher_ = self.create_publisher(Float64, "/yaw/fused", 10)
        self.imu_yaw = 0 
    def imu_listener_callback(self, msg):
        """Store the yaw from the IMU message."""
        self.imu_yaw = msg.data

        # new_alpha = rclpy.parameter.Parameter("alpha", rclpy.Parameter.Type.DOUBLE, 9.8)
        # all_new_parameters = [new_alpha]
        # self.set_parameters(all_new_parameters)

    def odom_listener_callback(self, msg):
        """Store the yaw from the odometry message."""
        alpha = self.get_parameter("alpha").get_parameter_value().double_value
        odom_yaw = msg.data
        fused_yaw = (alpha * self.imu_yaw )+ ((1 - alpha) * odom_yaw)
        fused_yaw = wrap(fused_yaw)

        #new_alpha = rclpy.parameter.Parameter("alpha", rclpy.Parameter.Type.DOUBLE, 9.8)
        # all_new_parameters = [new_alpha]
        # self.set_parameters(all_new_parameters)
        print(f"Alpha: {alpha}")

        msg = Float64()
        msg.data = float(fused_yaw)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    """Run the odometry yaw extractor node."""
    rclpy.init(args=args)
    fused_node = FusedNode()
    rclpy.spin(fused_node)
    fused_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
