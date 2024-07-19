import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point
import numpy as np
import tf_transformations


class OccupancyGridTransformer(Node):
    def __init__(self):
        super().__init__("occupancy_grid_transformer")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.grid_sub = self.create_subscription(
            OccupancyGrid, "/heightmap", self.grid_callback, 10
        )

        self.grid_pub = self.create_publisher(OccupancyGrid, "/odom/heightmap_tr", 10)
        self.test_pose_pub_tr_ct = self.create_publisher(
            PoseStamped, "/transform_pose_center", 10
        )
        self.test_pose_pub_tr = self.create_publisher(
            PoseStamped, "/transform_pose", 10
        )
        self.test_pose_pub_or = self.create_publisher(PoseStamped, "/robot_pose", 10)

        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_msg = None

    def grid_callback(self, msg):
        self.last_msg = msg

        if self.last_msg is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", self.last_msg.header.frame_id, rclpy.time.Time()
            )
            self.get_logger().info(
                f"Transform timer: {self.last_msg.header.frame_id} {self.last_msg.header.stamp} rclpy_time: {rclpy.time.Time()}"
            )
            self.transform_grid(self.last_msg, transform)
        except tf2_ros.LookupException as e:
            self.get_logger().error(f"Transform lookup error: {e}")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f"Transform extrapolation error: {e}")

    def transform_grid(self, grid, transform):
        if grid is None:
            self.get_logger().error(f"No OccupancyGrid message received")
            return

        resolution = grid.info.resolution
        width = grid.info.width
        height = grid.info.height

        # Transform the origin
        origin_pose = PoseStamped()
        origin_pose.header = grid.header
        origin_pose.header.stamp = self.get_clock().now().to_msg()
        origin_pose.pose = grid.info.origin
        origin_pose.pose.position.y += height * resolution / 2.0
        origin_pose.pose.position.x += width * resolution / 2.0

        transformed_center_pose = PoseStamped()
        transformed_center_pose.header = grid.header
        transformed_center_pose.header.frame_id = "odom"
        transformed_center_pose.header.stamp = self.get_clock().now().to_msg()

        transformed_center_pose.pose = tf2_geometry_msgs.do_transform_pose(
            origin_pose.pose, transform
        )

        self.test_pose_pub_or.publish(origin_pose)
        self.test_pose_pub_tr_ct.publish(transformed_center_pose)

        # Create a new OccupancyGrid message
        transformed_grid = OccupancyGrid()
        transformed_grid.header = grid.header
        transformed_grid.header.stamp = self.get_clock().now().to_msg()
        transformed_grid.header.frame_id = "odom"
        transformed_grid.info = grid.info

        # Update the pose of the origin
        transformed_grid.info.origin = transformed_center_pose.pose
        transformed_grid.info.origin.position = transformed_center_pose.pose.position
        transformed_grid.info.origin.position.x -= height * resolution / 2.0
        transformed_grid.info.origin.position.y -= width * resolution / 2.0
        transformed_grid.info.origin.orientation.x = 0.0
        transformed_grid.info.origin.orientation.y = 0.0
        transformed_grid.info.origin.orientation.z = 0.0
        transformed_grid.info.origin.orientation.w = 1.0

        # Initialize the transformed grid data
        transformed_grid.data = [0] * len(grid.data)

        transformed_origin_pose = PoseStamped()
        transformed_origin_pose.header = grid.header
        transformed_origin_pose.header.frame_id = "odom"
        transformed_origin_pose.header.stamp = self.get_clock().now().to_msg()
        transformed_origin_pose.pose = transformed_grid.info.origin
        self.test_pose_pub_tr.publish(transformed_origin_pose)

        # TODO: update transformed_grid.data by first transforming
        # the grid.data (x, y) and then checking where it falls
        # in the transformed_grid.data (x, y)

        # CODE GOES HERE
        # transformed_grid.data = grid.data
        # self.grid_pub.publish(transformed_grid)
        # Initialize the transformed grid data
        transformed_grid_data = [0] * (
            width * height
        )  # Use -1 to indicate unknown areas

        # Initialize the transformed grid data
        # Create the transformation matrix
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Convert quaternion to rotation matrix
        q = [rotation.x, rotation.y, rotation.z, rotation.w]
        rotation_matrix = tf_transformations.quaternion_matrix(q)

        transform_matrix = np.eye(4)
        transform_matrix[:3, 3] = [translation.x, translation.y, translation.z]
        transform_matrix = np.dot(transform_matrix, rotation_matrix)

        # Convert grid data to x, y coordinates
        max_range = width * resolution / 2.0
        y_indices, x_indices = np.indices((height, width))
        x_coords = -max_range + (x_indices.flatten() + 0.5) * resolution
        y_coords = -max_range + (y_indices.flatten() + 0.5) * resolution

        # Create array of points in the sensor frame
        points = np.vstack(
            (x_coords, y_coords, np.zeros_like(x_coords), np.ones_like(x_coords))
        )

        # Transform points from sensor frame to odom frame
        transformed_points = np.dot(transform_matrix, points)

        # Calculate new indices in the odom frame
        new_x_coords = (
            (transformed_points[0] - transformed_grid.info.origin.position.x)
            / resolution
        ).astype(int)
        new_y_coords = (
            (transformed_points[1] - transformed_grid.info.origin.position.y)
            / resolution
        ).astype(int)

        # Ensure indices are within grid bounds
        valid_indices = (
            (0 <= new_x_coords)
            & (new_x_coords < width)
            & (0 <= new_y_coords)
            & (new_y_coords < height)
        )

        # Map valid transformed coordinates to the new grid
        for old_idx, valid in enumerate(valid_indices):
            if valid:
                new_idx = new_y_coords[old_idx] * width + new_x_coords[old_idx]
                if 0 <= new_idx < width * height:
                    transformed_grid_data[new_idx] = grid.data[old_idx]

        transformed_grid.data = transformed_grid_data

        self.grid_pub.publish(transformed_grid)


def main(args=None):
    rclpy.init(args=args)
    transformer = OccupancyGridTransformer()
    rclpy.spin(transformer)
    transformer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
