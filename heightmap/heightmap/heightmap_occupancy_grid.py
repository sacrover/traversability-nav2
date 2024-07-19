import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped
from rclpy.duration import Duration


class PointCloudToHeightmap(Node):
    def __init__(self):
        super().__init__("pointcloud_to_heightmap")
        self.subscription = self.create_subscription(
            PointCloud2, "/ouster/points", self.pointcloud_callback, 20
        )

        self.heightmap_publisher = self.create_publisher(OccupancyGrid, "heightmap", 20)
        self.gradientmap_publisher = self.create_publisher(
            OccupancyGrid, "height_gradient_map", 20
        )
        # Define the resolution and size of the heightmap
        self.resolution = 0.1  # meters per cell
        self.max_range = 8.0  # max range to consider in the heightmap

        self.width = int(2 * self.max_range // self.resolution)
        self.height = int(2 * self.max_range // self.resolution)

        # Set the heightmap bounds
        self.min_val_height = -1.0
        self.max_val_height = 1.0

        # Set the gradient threshold
        self.gradient_threshold = 0.07

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=10.0)
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sensor_frame = "os_sensor"
        self.out_frame = "odom"
        self.robot_frame = "base_link"
        self.timeout_duration = Duration(seconds=1.5)

    def get_robot_position(self):
        try:
            # Lookup transform from the robot's base frame to the output frame
            transform = self.tf_buffer.lookup_transform(
                self.out_frame,
                self.robot_frame,
                rclpy.time.Time(),
                self.timeout_duration,
            )
            self.get_logger().info(
                f"Transform received at time: {transform.header.stamp.sec}.{transform.header.stamp.nanosec} rclpyTime: {rclpy.time.Time()}"
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
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = self.sensor_frame

        # Get robot's current position
        # robot_position_transform = self.get_robot_position()
        # if robot_position_transform is None:
        # return

        # Create MapMetaData
        map_meta_data = MapMetaData()
        map_meta_data.resolution = self.resolution
        map_meta_data.width = self.width
        map_meta_data.height = self.height

        # Calculate origin position
        # origin_x = (
        #     robot_position_transform.transform.translation.x
        #     - (self.width * self.resolution) / 2.0
        # )
        # origin_y = (
        #     robot_position_transform.transform.translation.y
        #     - (self.height * self.resolution) / 2.0
        # )

        # OR use ROS 2 logging
        # self.get_logger().info(f"Calculated origin x: {origin_x:.4f} y: {origin_y:.4f}")

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

        self.get_logger().debug(
            f"lidar_max x & y: {np.max(points_2d_arr[:, 0])}, {np.max(points_2d_arr[:, 1])}"
        )

        # transform frame
        # transformed_points = self.transform_frame(
        #     points_2d_arr, in_frame=msg.header.frame_id
        # )

        # if transformed_points is None:
        #     return

        # points_2d_arr = np.array(transformed_points)

        # Populate the heightmap with the maximum z values

        heightmap = (
            np.ones((self.width, self.height), dtype=np.float32) * -100.0
        )  # init with some low number

        for x, y, z in points_2d_arr:
            if (
                abs(x) < self.max_range
                and abs(y) < self.max_range
                and z >= self.min_val_height
                and z <= self.max_val_height
            ):
                x_idx = int((-x + self.max_range) // self.resolution - 0.5)
                y_idx = int((-y + self.max_range) // self.resolution - 0.5)
                heightmap[y_idx, x_idx] = min(max(heightmap[y_idx, x_idx], z), 1)

        heightmap_normalized = (
            (heightmap - self.min_val_height)
            / (self.max_val_height - self.min_val_height)
            * 100.0
        )

        heightmap_normalized[heightmap == -100.0] = 0.0
        heightmap_normalized[0, 0] = 100.0

        heightmap_normalized = heightmap_normalized.astype(np.uint8)

        self.pub_occupancy_grid(heightmap_normalized, self.heightmap_publisher)

        # Compute the height gradient map using the raw data

        gradient_map = np.zeros_like(heightmap)
        for i in range(1, self.height - 1):
            for j in range(1, self.width - 1):
                if heightmap[i, j] > -np.inf:
                    dz_dx = 0.0
                    dz_dy = 0.0

                    if heightmap[i, j + 1] > -100.0 and heightmap[i, j - 1] > -100.0:
                        dz_dx = (heightmap[i, j] - heightmap[i, j - 1]) / (
                            self.resolution
                        )

                    if heightmap[i + 1, j] > -100.0 and heightmap[i - 1, j] > -100.0:
                        dz_dy = (heightmap[i, j] - heightmap[i - 1, j]) / (
                            self.resolution
                        )

                    gradient_map[i, j] = np.sqrt(dz_dx**2 + dz_dy**2)

        gradient_mask = gradient_map >= self.gradient_threshold

        # Normalize the gradient map for visualization
        gradient_map_normalized = (
            (gradient_map - self.gradient_threshold)
            / (np.max(gradient_map) - self.gradient_threshold)
        ).astype(np.int8)
        gradient_map_normalized[~gradient_mask] = -1

        # Publish gradient map
        self.pub_occupancy_grid(gradient_map_normalized, self.gradientmap_publisher)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToHeightmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
