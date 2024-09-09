import rclpy
from rclpy.node import Node
from heightmap_msg.msg import SurfaceMap, Quad
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
import numpy as np
from heightmap_msg.msg import GridMap16  # Replace with the actual package


class SurfaceMapNode(Node):
    def __init__(self):
        super().__init__("surface_map_node")

        self.subscription = self.create_subscription(
            GridMap16, "/heightmap_fused_odom", self.map_callback, 10
        )
        self.frame_id = None
        self.publisher = self.create_publisher(SurfaceMap, "/surface_map", 10)
        self.original_min = -3.0
        self.original_max = 0.5
        self.norm_min = 0.0  # Set this according to your data
        self.norm_max = 1000.0  # Set this according to your data

    def map_callback(self, msg):

        if self.frame_id is None:
            self.frame_id = msg.header.frame_id
        # Convert GridMap16 data to a 2D numpy array
        grid_data = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width
        )

        # Denormalize the grid values
        z_values = self.denormalize_grid(grid_data)

        # Create the grid points for x and y axes
        x_grid, y_grid = self.create_grid(msg.info)
        self.origin = msg.info.origin

        # Create the SurfaceMap message
        surface_map_msg = self.create_surface_map_msg(x_grid, y_grid, z_values)
        self.publisher.publish(surface_map_msg)

    def denormalize_grid(self, grid_data):
        denorm_grid = np.full(grid_data.shape, np.nan)
        valid_mask = grid_data != -1

        denorm_grid[valid_mask] = (grid_data[valid_mask] - self.norm_min) / (
            self.norm_max - self.norm_min
        ) * (self.original_max - self.original_min) + self.original_min

        return denorm_grid

    def create_grid(self, info):
        x_edges = np.linspace(0, info.width * info.resolution, info.width)
        y_edges = np.linspace(0, info.height * info.resolution, info.height)
        x_grid, y_grid = np.meshgrid(x_edges, y_edges)
        return x_grid, y_grid

    def create_surface_map_msg(self, x_grid, y_grid, z_values):
        surface_map = SurfaceMap()
        surface_map.origin = self.origin
        surface_map.header = Header()
        surface_map.header.stamp = self.get_clock().now().to_msg()
        surface_map.header.frame_id = self.frame_id

        for i in range(x_grid.shape[0] - 1):
            for j in range(y_grid.shape[1] - 1):
                vertices = np.array(
                    [
                        [x_grid[i, j], y_grid[i, j], z_values[i, j]],
                        [x_grid[i, j + 1], y_grid[i, j + 1], z_values[i, j + 1]],
                        [
                            x_grid[i + 1, j + 1],
                            y_grid[i + 1, j + 1],
                            z_values[i + 1, j + 1],
                        ],
                        [x_grid[i + 1, j], y_grid[i + 1, j], z_values[i + 1, j]],
                    ]
                )

                if not np.any(np.isnan(vertices[:, 2])):
                    quad_msg = Quad()
                    for k in range(4):
                        point = Point32()
                        point.x = vertices[k][0]
                        point.y = vertices[k][1]
                        point.z = vertices[k][2]

                        quad_msg.vertices[k] = point
                    surface_map.quads.append(quad_msg)
        return surface_map


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
