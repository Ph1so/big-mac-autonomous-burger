"""
This node takes the laser scan from the Lidar and transforms
it into a 2D point cloud, then calculates the transform via 
matched points, and then merges the new points with the existing cloud
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from laser_geometry import LaserProjection
import tf2_ros
from tf2_ros import TransformException, TransformStamped
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import cKDTree
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion

from icp_package.utils import euler_rotation_matrix

# from openai import chatgprt
# machine.learn()


class PauseAndCapture(Node):
    """This defines the scan_match node to take in Lidar points"""

    def __init__(self):
        super().__init__("pause_and_capture")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.laser_projector = LaserProjection()

        qos_profile = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT
        )

        # Set up the subscription for LaserScan message
        # HINT: Subscribe on the '/scan' topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile)

        # Create a publisher for PointCloud2 messages
        # HINT: Publish on the '/accumulated_cloud' topic
        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/accumulated_cloud',
            qos_profile)
        # Create a publisher for ICP merged cloud
        # HINT: Publish on the '/icp_merged_cloud' topic
        self.icp_pub = self.create_publisher(
            PointCloud2,
            '/icp_merged_cloud',
            qos_profile)

        self.accumulated_points = []
        self.icp_accumulated_points = []

        self.capture_enabled = False
        self.latest_scan = None
        self.delay_timer = None

        self.input_thread = threading.Thread(
            target=self.key_press_listener, daemon=True
        )
        self.input_thread.start()

        self.get_logger().info(
            "PauseAndCapture node started. Press Enter to capture a scan."
        )

    def key_press_listener(self):
        """Listens for terminal input"""
        while True:
            input(">> Press Enter to capture scan: ")
            self.capture_enabled = True

    def scan_callback(self, scan_msg: LaserScan):
        """Callback for Lidar data"""
        if not self.capture_enabled:
            return

        self.latest_scan = scan_msg
        self.capture_enabled = False

        if self.delay_timer:
            self.delay_timer.cancel()
        self.delay_timer = self.create_timer(0.1, self.delayed_transform_lookup)

    def delayed_transform_lookup(self):
        """Looks up transform between clouds"""
        self.delay_timer.cancel()
        scan_msg = self.latest_scan
        self.latest_scan = None

        try:
            cloud_in_laser = self.laser_projector.projectLaser(scan_msg)
            # Perform a lookup to transform the point cloud from its original
            # frame to the 'odom' frame
            transform = self.tf_buffer.lookup_transform(
                "odom",  # Target frame (where do you want to transform to?)
                scan_msg.header.frame_id,  # Source frame (the point cloud's original frame)
                Time.from_msg(
                    scan_msg.header.stamp
                ),  # Timestamp of the scan message to ensure proper time synchronization
                timeout=rclpy.duration.Duration(
                    seconds=0.5
                ),  # Timeout of 0.5 seconds to wait for the transform
            )

            # Transform the point cloud with the transform_pointcloud2 function
            transformed_points = self.transform_pointcloud2(cloud_in_laser, transform)


            if self.icp_accumulated_points:
                icp_aligned = self.perform_icp(
                    self.icp_accumulated_points, transformed_points
                )
                self.icp_accumulated_points.extend(icp_aligned)
                self.publish_icp_merged_cloud(scan_msg.header.stamp)
                self.get_logger().info(
                    f"ICP-aligned and merged {len(icp_aligned)} points."
                )
            else:
                self.icp_accumulated_points.extend(transformed_points)
                self.publish_icp_merged_cloud(scan_msg.header.stamp)
                self.get_logger().info(
                    f"Initialized ICP merged cloud with {len(transformed_points)} points."
                )

            self.accumulated_points.extend(transformed_points)
            print(f"Accumulated points: {len(self.accumulated_points)}")
            self.publish_accumulated_cloud(scan_msg.header.stamp)
            self.get_logger().info(
                f"Captured and transformed {len(transformed_points)} points."
            )

        except TransformException as ex:
            self.get_logger().warn(f"Transform failed after delay: {str(ex)}")

    # Note that ros inherently processes point clouds in 3d
    # even though the robot's point cloud is in 2d.

    def transform_pointcloud2(
        self, cloud_msg: PointCloud2, transform: TransformStamped
    ) -> list[tuple[int, int, int]]:
        """Transform a point cloud using Euler angles from a given quaternion."""

        def rotate_point_euler(x, y, z, roll, pitch, yaw) -> tuple[int, int, int]:
            """Rotate a point (x, y, z) using Euler angles (roll, pitch, yaw)."""
            # using the roll,pitch and yaw construct the Rx , Ry, Rz matrix
            # Combined rotation matrix
            rot = euler_rotation_matrix(roll, pitch, yaw)
            # Apply the rotation to the point

            res = rot @ np.array([x, y, z])

            return tuple(res)

        # Extract translation and rotation (quaternion) from the transform method
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        # Hint: Use the euler_from_quaternion
        euler = euler_from_quaternion([qx, qy, qz, qw])

        # Transform the point cloud using Euler rotation
        transformed_points = []
        for pt in pc2.read_points(
            cloud_msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            # Get values of pt
            x, y, z = pt
            # Apply rotation to the point using Euler angles use the rotate point euler function
            x, y, z = rotate_point_euler(x, y, z, euler[0], euler[1], euler[2])
            # Append transformed point
            x += tx
            y += ty
            z += tz
            new_pt = (x, y, z)
            transformed_points.append(new_pt)

        return transformed_points

    def perform_icp(
        self, previous_points, current_points, max_iterations=20, tolerance=1e-4
    ):
        """Main ICP loop to transform new points"""
        src = np.array(current_points)
        tgt = np.array(previous_points)

        # Write the ICP loop here

        # Useful Steps to Follow For the Loop:
        # 1. Start icp loop for max_iterations
        # 2. Build KDTree for target cloud, cKdtree from scipy
        # 3. Find nearest neighbors from source to tgt
        # 4. Compute centroids of matched source and target points
        # 5. Center both point clouds by subtracting their centroids
        # 6. Compute the cross-covariance matrix
        # 7. SVD on step 6
        # 8. Compute rotation matrix R from SVD, np.linalg.svd will help
        # 9. Compute translation vector t from centroids and rotation
        # 10. Apply the transformation to the source points
        # 11. Compute mean error and check for convergence
        # 12. If converged, break the loop

        for _ in range(max_iterations):
            tree = cKDTree(tgt, leafsize=16)
            closest_list = []
            for point in src:
                _, index = tree.query(point, k=1)
                tgt_closest_point = tgt[index]
                closest_list.append(tgt_closest_point)
            print(f"src dim: {src.shape}, closest_list dim: {np.array(closest_list).shape}")
            u_mat, _, vh_mat = self.svd_estimation(src, closest_list)
            print(f"u dim: {u_mat.shape}, vh dim: {vh_mat.shape}")
            rot = np.transpose(vh_mat) @ np.transpose(u_mat)
            p_cent = self.calculate_centroids(src)
            q_cent = self.calculate_centroids(closest_list)

            t = q_cent - (rot @ np.transpose(p_cent))

            src @= rot
            src @= t

            err = 0
            for i, point in enumerate(src):
                err += math.pow(np.linalg.norm(closest_list[i] - (rot @ point) + t), 2)

            if err < tolerance:
                break

        return src.tolist()

    @staticmethod
    def calculate_centroids(points):
        """Calculates the centroid of a set of 3D points"""
        return np.asarray([np.mean(np.asarray(points), axis=0)])

    # Curr = Source, Prev = Target
    def svd_estimation(self, previous_points, current_points):
        """Cacluates matrices for U, V_T, and Sigma"""
        p_cent = self.calculate_centroids(previous_points)
        c_cent = self.calculate_centroids(current_points)
        print(f"p_cent: {p_cent.shape}, c_cent: {c_cent.shape}")
        h_mat = np.zeros((3, 3))
        for i, prev_pt in enumerate(previous_points):
            p_var = np.asarray([prev_pt]) - p_cent
            c_var = np.asarray([current_points[i]]) - c_cent
            res = np.transpose(p_var) @ (c_var)
            h_mat += res
        return np.linalg.svd(h_mat)

    def publish_accumulated_cloud(self, stamp):
        """Publishes the existing accumulated pointcloud"""
        header = Header()
        header.stamp = stamp
        header.frame_id = "odom"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, self.accumulated_points)
        
        self.pc_pub.publish(cloud_msg)
        self.get_logger().info("Published accumulated cloud.")

    def publish_icp_merged_cloud(self, stamp):
        """Publishes merged pointcloud"""
        header = Header()
        header.stamp = stamp
        header.frame_id = "odom"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, self.icp_accumulated_points)
        self.icp_pub.publish(cloud_msg)
        self.get_logger().info("Published ICP merged cloud.")


def main(args=None):
    """Start ROS node"""
    rclpy.init(args=args)
    node = PauseAndCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
