import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from laser_geometry import LaserProjection
import tf2_ros
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np
import threading
from scipy.spatial import cKDTree
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt

# ************************************************
#           IMPORTANT DETAILS BELOW             #
# ************************************************
"""
This starter code provides code base to implement occupancy grid mapping for HW5 Q3
Parts of this assignment depends on HW5a solutions

--- TBD --- indicates the section to fill out   
... indicates the specific place to type the code

There are 9 TBD sections

"""

class PauseAndCapture(Node):
    def __init__(self):
        super().__init__('pause_and_capture')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.laser_projector = LaserProjection()

        # ------------ TBD -------------------
        # write the appropriate QoS profile for the publishing MAP. this requires additional durability policy
        # refer to the lecture for more details

        map_qos = QoSProfile(
            ...
        )

        # Create a publisher for /map topic (with Occupancy Grid)
        self.map_pub = ...

        qos_profile = ...
        # Subscribe to the /scan topic
        self.subscription = ...

        # ---------------- TBD-END -------------

        self.pc_pub = self.create_publisher(PointCloud2, '/accumulated_cloud', 10)
        self.accumulated_points = []
        self.capture_enabled = False
        self.latest_scan = None
        self.delay_timer = None
        self.margin = 0.2  # padding outside the map
        self.visualize_normal = False  # set this to true to visualize the normals
        self.normal_simple = False  # use this to switch between a simple and more robust normal estimation

        # ----------------------- TBD -----------------------
        # initialize the variables
        self.resolution = 0.01
        self.width = ...  # measure the arena width and convert the metric value to pixel resolution, refer to the use class
        self.height = ...  # measure the arena length and convert the metric value to pixel resolution, refer to the use class
        self.origin_x = -...  # initialize to the half of the arena width/2 metric value. Should have negative sign
        self.origin_y = -...  # initialize to the half of the arena height/2 metric value. Should have negative sign
        self.l0 = ...  # initial probability of all cells will be 0.5. convert this to lof odds value
        self.lz_occ = ...  # use higher positive values [0, 1] for occupied cell
        self.lz_free = -...  # use lower negative values [0, 1] for empty cells. Should have negative sign
        self.log_odds_min = -...  # calculate log odds lower bounds for probability = 0.1. Should have negative sign
        self.log_odds_max = ...  # calculate log odds upper bounds for probability = 0.9
        # ----------------------- TBD-END -------------------

        self.log_odds_map = np.zeros((self.height, self.width), dtype=np.float32)
        self.is_first_scan = True
        self.input_thread = threading.Thread(target=self.key_press_listener, daemon=True)
        self.input_thread.start()
        self.map_origin_odom_x = None
        self.map_origin_odom_y = None
        self.use_icp = False
        self.visualize_normals = False  # enable this to visualize the computed normals
        self.get_logger().info("Occupancy Grid Mapping node started. Press Enter to capture a scan.")

    def key_press_listener(self):
        while True:
            input(">> Press Enter to capture scan: ")
            self.capture_enabled = True

    def scan_callback(self, scan_msg):
        if not self.capture_enabled:
            return

        self.latest_scan = scan_msg
        self.capture_enabled = False

        if self.delay_timer:
            self.delay_timer.cancel()
        self.delay_timer = self.create_timer(0.05, self.delayed_transform_lookup)

    def delayed_transform_lookup(self):
        self.delay_timer.cancel()
        scan_msg = self.latest_scan
        self.latest_scan = None

        try:
            # -------------- TBD -----------------
            cloud_in_laser = ... # laser projection of scan

            transform = self.tf_buffer.lookup_transform( # lookup transform between the accumulated map and the scan
                ...
            )

            sensor_origin = np.array([...]) # transform stamped translation planar components
            if self.use_icp:
                transformed_points = ... # use transform_pointcloud2 to transform laser projection points

                if len(self.accumulated_points) > 10:
                    transformed_points = self.icp_point_to_plane_(...) # pass point clouds and sensor origin
            else:
                transformed_points = ... # laser projection transform

            self.accumulated_points.extend(transformed_points)

            if self.map_origin_odom_x is None:
                self.compute_map_size_from_pointcloud()

            self.publish_accumulated_cloud(scan_msg.header.stamp)
            self.update_occupancy_grid([(x, y) for x, y, _ in transformed_points], transform)
            self.publish_occupancy_grid(scan_msg.header.stamp)
            self.get_logger().info(f"Captured and aligned {len(transformed_points)} points.")

            # ---------------- TBD END ------------------

        except TransformException as ex:
            self.get_logger().warn(f"Transform failed after delay: {str(ex)}")

    def compute_normals(self, points, sensor_origin):

        # --------------- TBD ---------------
        """"
        computes normal using three point normal. You can use your HW4 solution for this
        """
        normals = []
        for i in range(1, len(points) - 1):
            ...
        return np.array(normals)

        # --------------- TBD END ---------------

    def compute_normals_pca(self, points, tree, sensor_origin, k=10):
        """
        More sophisticated way to compute normals. Here using PCA
        This was not explicitly covered in class but similar to the concept of SVD principal directions
        Use this function to visualize normals and compare it to your method
        """
        normals = []
        for pt in points:
            _, idxs = tree.query(pt, k=k)
            neighbors = points[idxs]
            cov = np.cov(neighbors.T)
            eigvals, eigvecs = np.linalg.eigh(cov)
            normal = eigvecs[:, 0]
            normal /= np.linalg.norm(normal)
            to_sensor = sensor_origin - pt
            if np.dot(normal, to_sensor) < 0:
                normal = -normal
            normals.append(normal)
        return np.array(normals)

    def icp_point_to_plane_(self, source_points, target_points, target_sensor_origin, max_iterations=15):
        """
        Main function for Point2Plane ICP

        You can copy your implementation from HW4 into the relevant parts of the TBD

        """
        
        self.get_logger().warn("Using point2plane icp", )

        # --------------- TBD ---------------
        src = ...
        tgt = ...

        tree = ...

        if self.normal_simple:
            normals = ...
        else:
            normals = ...

        if self.visualize_normal:
            self.plot_normals(tgt, normals)

        for _ in range(max_iterations):
            ...
            # --------------- TBD END ---------------
        return [(p[0], p[1], 0.0) for p in src]

    def compute_map_size_from_pointcloud(self):
        """
        This function pads the generated map with value = self.margin
        """
        xs = [p[0] for p in self.accumulated_points]
        ys = [p[1] for p in self.accumulated_points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        min_x -= self.margin
        min_y -= self.margin
        max_x += self.margin
        max_y += self.margin

        self.origin_x = min_x
        self.origin_y = min_y
        self.map_origin_odom_x = min_x
        self.map_origin_odom_y = min_y

        self.width = math.ceil((max_x - min_x) / self.resolution)
        self.height = math.ceil((max_y - min_y) / self.resolution)

        self.log_odds_map = np.zeros((self.height, self.width), dtype=np.float32)

        self.get_logger().info(f"Map origin set to ({min_x:.2f}, {min_y:.2f})")
        self.get_logger().info(f"Map size: {self.width} x {self.height} cells")

    def transform_pointcloud2(self, cloud_msg, transform):
        # ------------ TBD ----------------
        """
        Scan and Merge
        You can copy your implementation from HW4 into the relevant parts of the TBD
        """
        ...

        transformed_points = []
        for pt in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            ...

        return transformed_points
    # -------------- TBD END -------------------

    def publish_accumulated_cloud(self, stamp):
        """
        Publish the merged point cloud
        """
        # -------------- TBD -------------
        ... # create a point cloud using pc2.create_cloud
        # ----------------- TBD END ---------------

    def world_to_map(self, x, y):
        """
        Transforms laser to map frame
        """
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        if 0 <= mx < self.width and 0 <= my < self.height:
            return mx, my
        return None, None

    def update_occupancy_grid(self, points, transform):
        """
        This is the main function for occupancy grid mapping
        """
        # Get robot's current position in map frame
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        # ---------------------- TBD ---------------------------------
        for x, y in points:
            # Convert robot and endpoint (scan hit) to map coordinates (cell indices)
            mx0, my0 = ...  # starting point for raycasting, use world_to_map()
            mx1, my1 = ...  # end point for raycasting, use world_to_map()

            # Skips update if either point is outside the map bounds
            if mx0 is None or mx1 is None:
                continue

            # Free space update (cells along the ray from robot to hit point)
            for cx, cy in self.ray_casting(mx0, my0, mx1, my1)[:-1]:  # excludes last point (occupied)
                lt_1 = ...  # previous log-odds value (lt−1)
                lt = ...  # log-odds update for free cell
                self.log_odds_map[cy, cx] = np.clip(lt, self.log_odds_min, self.log_odds_max)

            # Occupied cell update (the cell where the scan hits an obstacle)
            lt_1 = ...  # previous log-odds value (lt−1)
            lt = ... # log-odds update for occupied cell
            self.log_odds_map[my1, mx1] = np.clip(lt, self.log_odds_min, self.log_odds_max)
        # ---------------------- TBD-END ---------------------------------

    def ray_casting(self, x0, y0, x1, y1):
        """
        bresenham line algorithm
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    def publish_occupancy_grid(self, stamp):
        """
        publish the grid map for visualization
        """
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = stamp
        grid_msg.header.frame_id = "map"

        # ----------------- TBD ---------------------------------

        metadata = MapMetaData()
        metadata.resolution = self.resolution
        metadata.width = self.width
        metadata.height = self.height
        metadata.origin = Pose()
        metadata.origin.position.x = self.origin_x
        metadata.origin.position.y = self.origin_y

        grid_msg.info = metadata

        probs = ...  # calculate probability from the logodds (remember log)
        grid_data = ...  # ROS2 expects occupied grids to be = 100 * probability and int8
        grid_data[self.log_odds_map == 0] = ... # unexplored grids should have value of -1

        # ----------------- TBD END ---------------------
        grid_msg.data = grid_data.flatten().tolist()
        self.map_pub.publish(grid_msg)
        self.get_logger().info("Published occupancy grid.")

    def plot_normals(self, points, normals, title="Point Normals"):
        """
        plot the normals for debugging
        """
        points = np.array(points)
        normals = np.array(normals)
        plt.figure(figsize=(8, 8))
        plt.quiver(points[:, 0], points[:, 1], normals[:, 0], normals[:, 1],
                   angles='xy', scale_units='xy', scale=15, color='r', width=0.003, label='Normals')
        plt.plot(points[:, 0], points[:, 1], 'b.', markersize=2, label='Points')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title(title)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PauseAndCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

