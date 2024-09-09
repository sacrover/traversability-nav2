import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from nav_msgs.msg import OccupancyGrid, MapMetaData
from heightmap_msg.msg import GridMap16
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped
from rclpy.duration import Duration


class PointCloudToHeightmap(Node):
    def __init__(self):
        super().__init__("pointcloud_to_heightmap", namespace="traversability")
        self.declare_parameter("subscription_topic", "/ouster/points")
        self.declare_parameter("heightmap_topic", "heightmap")
        self.declare_parameter("gradientmap_topic", "height_gradient_map")
        self.declare_parameter("traversability_topic", "traversability")
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("max_range", 7.0)
        self.declare_parameter("robot_footprint", 1.0)
        self.declare_parameter("min_val_height", -3.0)
        self.declare_parameter("max_val_height", 0.5)
        self.declare_parameter("norm_max", 1000.0)
        self.declare_parameter("norm_min", 0.0)
        self.declare_parameter("gradient_threshold", 0.07)
        self.declare_parameter("sensor_frame", "os_sensor")
        self.declare_parameter("out_frame", "odom")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("timeout_duration", 0.2)

        # Retrieve the parameters
        subscription_topic = (
            self.get_parameter("subscription_topic").get_parameter_value().string_value
        )
        heightmap_topic = (
            self.get_parameter("heightmap_topic").get_parameter_value().string_value
        )
        self.resolution = (
            self.get_parameter("resolution").get_parameter_value().double_value
        )
        self.max_range = (
            self.get_parameter("max_range").get_parameter_value().double_value
        )
        self.robot_footprint = (
            self.get_parameter("robot_footprint").get_parameter_value().double_value
        )
        self.min_val_height = (
            self.get_parameter("min_val_height").get_parameter_value().double_value
        )
        self.max_val_height = (
            self.get_parameter("max_val_height").get_parameter_value().double_value
        )
        self.norm_max = (
            self.get_parameter("norm_max").get_parameter_value().double_value
        )
        self.norm_min = (
            self.get_parameter("norm_min").get_parameter_value().double_value
        )
        self.gradient_threshold = (
            self.get_parameter("gradient_threshold").get_parameter_value().double_value
        )
        self.sensor_frame = (
            self.get_parameter("sensor_frame").get_parameter_value().string_value
        )
        self.out_frame = (
            self.get_parameter("out_frame").get_parameter_value().string_value
        )
        self.robot_frame = (
            self.get_parameter("robot_frame").get_parameter_value().string_value
        )
        timeout_duration_sec = (
            self.get_parameter("timeout_duration").get_parameter_value().double_value
        )

        # Set up the subscriptions and publishers using the retrieved topics
        self.subscription = self.create_subscription(
            PointCloud2, subscription_topic, self.pointcloud_callback, 10
        )
        self.heightmap_publisher = self.create_publisher(GridMap16, heightmap_topic, 10)

        # Calculate the heightmap width and height based on max range and resolution
        self.width = int(2 * self.max_range // self.resolution)
        self.height = int(2 * self.max_range // self.resolution)

        # TF Buffer and Listener setup
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=Duration(seconds=timeout_duration_sec)
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_robot_position(self):
        try:
            # Lookup transform from the robot's base frame to the output frame
            transform = self.tf_buffer.lookup_transform(
                self.out_frame,
                self.robot_frame,
                rclpy.time.Time(),
            )
            return transform
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f"Transform lookup failed: {ex}")
            return None

    def transform_frame(self, points, in_frame):
        try:
            # Get the transform
            transform = self.tf_buffer.lookup_transform(
                self.out_frame, in_frame, rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().error(f"Transform error: {ex}")
            return

        # Apply the transformation to each point
        transformed_points = []

        for x, y, z in points:
            point = PointStamped()
            point.header.frame_id = self.out_frame
            point.point.x = x
            point.point.y = y
            point.point.z = z

            # Transform point to the map frame
            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)
            transformed_points.append(
                [
                    transformed_point.point.x,
                    transformed_point.point.y,
                    transformed_point.point.z,
                ]
            )

        return transformed_points

    def pub_occupancy_grid(self, grid_data, publisher):

        # Create OccupancyGrid message
        occupancy_grid_msg = GridMap16()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = self.sensor_frame

        # Create MapMetaData
        map_meta_data = MapMetaData()
        map_meta_data.resolution = self.resolution
        map_meta_data.width = self.width
        map_meta_data.height = self.height

        # Set the origin to the robot's current position
        map_meta_data.origin.position.x = -self.max_range
        map_meta_data.origin.position.y = -self.max_range
        map_meta_data.origin.position.z = 0.0
        map_meta_data.origin.orientation.x = 0.0
        map_meta_data.origin.orientation.y = 0.0
        map_meta_data.origin.orientation.z = 0.0
        map_meta_data.origin.orientation.w = 1.0

        occupancy_grid_msg.info = map_meta_data

        # Flatten heightmap and assign to data
        occupancy_grid_msg.data = grid_data.flatten().tolist()

        # Publish heightmap
        publisher.publish(occupancy_grid_msg)

    def pointcloud_callback(self, msg):

        points = read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        if len(points) == 0:
            return

        points_2d_arr = np.array(points.tolist())

        heightmap = (
            np.ones((self.width, self.height), dtype=np.float32) * -100.0
        )  # init with some low number

        x_mask = np.abs(points_2d_arr[:, 0]) < self.max_range
        y_mask = np.abs(points_2d_arr[:, 1]) < self.max_range

        z_mask = (points_2d_arr[:, 2] >= self.min_val_height) & (
            points_2d_arr[:, 2] <= self.max_val_height
        )

        distance_squared = points_2d_arr[:, 0] ** 2 + points_2d_arr[:, 1] ** 2
        footprint_mask = distance_squared >= self.robot_footprint**2

        valid_mask = x_mask & y_mask & z_mask & footprint_mask

        filtered_points = points_2d_arr[valid_mask]

        # Compute the indices and update the heightmap
        x_indices = (
            (-filtered_points[:, 0] + self.max_range) // self.resolution - 0.5
        ).astype(int)
        y_indices = (
            (-filtered_points[:, 1] + self.max_range) // self.resolution - 0.5
        ).astype(int)

        heightmap[y_indices, x_indices] = np.minimum(
            np.maximum(heightmap[y_indices, x_indices], filtered_points[:, 2]), 1
        )

        heightmap_normalized = (heightmap - self.min_val_height) / (
            self.max_val_height - self.min_val_height
        ) * (self.norm_max - self.norm_min) + self.norm_min

        heightmap_normalized[heightmap == -100.0] = -1.0

        heightmap_normalized = heightmap_normalized.astype(np.int16)

        self.pub_occupancy_grid(heightmap_normalized, self.heightmap_publisher)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToHeightmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
