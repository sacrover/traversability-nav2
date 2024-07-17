import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose


class OccupancyGridTransformer(Node):
    def __init__(self):
        super().__init__("occupancy_grid_transformer")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.grid_sub = self.create_subscription(
            OccupancyGrid, "/heightmap", self.grid_callback, 10
        )

        self.grid_pub = self.create_publisher(OccupancyGrid, "/odom/heightmap", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_msg = None

    def grid_callback(self, msg):
        self.last_msg = msg

    def timer_callback(self):
        if self.last_msg is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", self.last_msg.header.frame_id, rclpy.time.Time()
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

        transformed_grid = OccupancyGrid()
        transformed_grid.header = grid.header
        transformed_grid.header.frame_id = "odom"
        transformed_grid.info = grid.info

        # Perform the transformation on the origin
        origin_pose = Pose()
        origin_pose = grid.info.origin

        transformed_origin_pose = tf2_geometry_msgs.do_transform_pose(
            origin_pose, transform
        )
        transformed_grid.info.origin = transformed_origin_pose

        # Copy the data from the original grid
        transformed_grid.data = grid.data

        self.grid_pub.publish(transformed_grid)


def main(args=None):
    rclpy.init(args=args)
    transformer = OccupancyGridTransformer()
    rclpy.spin(transformer)
    transformer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
