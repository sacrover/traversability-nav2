import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np
import cv2
import random


class PointCloudToHeightmap(Node):
    def __init__(self):
        super().__init__("pointcloud_to_heightmap")
        self.subscription = self.create_subscription(
            PointCloud2, "/ouster/points", self.pointcloud_callback, 10
        )
        self.heightmap_publisher = self.create_publisher(Image, "heightmap", 10)
        self.gradientmap_publisher = self.create_publisher(
            Image, "height_gradient_map", 10
        )

    def pointcloud_callback(self, msg):
        points = read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        if len(points) == 0:
            return

        points_2d_arr = np.array(points.tolist())

        self.get_logger().debug(
            f"lidar_max x & y: {np.max(points_2d_arr[:, 0])}, {np.max(points_2d_arr[:, 1])}"
        )

        # Filter points based on z-value
        points_2d_arr = points_2d_arr[
            (points_2d_arr[:, 2] >= -1.0) & (points_2d_arr[:, 2] <= 0.0)
        ]

        if len(points_2d_arr) == 0:
            return

        # Define the resolution and size of the heightmap
        resolution = 0.1  # meters per cell
        max_range = 8  # max range to consider in the heightmap
        width = int(2 * max_range // resolution)
        height = int(2 * max_range // resolution)

        # Populate the heightmap with the maximum z values
        heightmap = np.ones((width, height), dtype=np.float32) * -100.0
        for x, y, z in points_2d_arr:
            if abs(x) < max_range and abs(y) < max_range:
                x_idx = int((x + max_range) // resolution) - 1
                y_idx = int((y + max_range) // resolution) - 1
                heightmap[y_idx, x_idx] = min(max(heightmap[y_idx, x_idx], z), 1)

        # Normalize the heightmap for visualization
        # min_val, max_val = np.min(heightmap[heightmap > -100]), np.max(heightmap)
        min_val, max_val = -1.5, 1.0

        heightmap_normalized = (
            (heightmap - min_val) / (max_val - min_val) * 255
        ).astype(np.uint8)

        # Convert the heightmap to a PointCloud2 message and publish it (or publish as an image)
        header = msg.header
        header.stamp = self.get_clock().now().to_msg()

        # Publish heightmap as an image (for visualization purposes)
        # Apply a colormap
        heightmap_colored = cv2.applyColorMap(heightmap_normalized, cv2.COLORMAP_JET)

        valid_mask = heightmap > -100.0
        heightmap_colored[~valid_mask] = [0, 0, 0]  # Set invalid points to black

        # Rotate the image by 180 degrees and flip vertically
        # heightmap_colored = cv2.rotate(heightmap_colored, cv2.ROTATE_180)
        heightmap_colored = cv2.flip(heightmap_colored, 0)
        # heightmap_msg = self.cv2_to_imgmsg(heightmap_normalized, encoding="mono8")
        heightmap_msg = self.cv2_to_imgmsg(heightmap_colored, encoding="bgr8")
        self.heightmap_publisher.publish(heightmap_msg)

        # Compute the height gradient map using the raw data
        gradient_map = np.zeros_like(heightmap)
        for i in range(1, height - 1):
            for j in range(1, width - 1):
                if heightmap[i, j] > -100.0:
                    dz_dx = 0.0
                    dz_dy = 0.0

                    if heightmap[i, j + 1] > -100.0 and heightmap[i, j - 1] > -100.0:
                        dz_dx = (heightmap[i, j] - heightmap[i, j - 1]) / (resolution)

                    if heightmap[i + 1, j] > -100.0 and heightmap[i - 1, j] > -100.0:
                        dz_dy = (heightmap[i, j] - heightmap[i - 1, j]) / (resolution)

                    gradient_map[i, j] = np.sqrt(dz_dx**2 + dz_dy**2)

        min_grad, max_grad = np.min(gradient_map), np.max(gradient_map)
        self.get_logger().info(f"min gradient: {min_grad}, max gradient: {max_grad}")

        gradient_threshold = 0.07
        gradient_mask = gradient_map >= gradient_threshold

        # Normalize the gradient map for visualization
        gradient_map_normalized = cv2.normalize(
            gradient_map, None, 0, 255, cv2.NORM_MINMAX
        )
        gradient_map_normalized = gradient_map_normalized.astype(np.uint8)

        # Apply a colormap to the gradient map
        gradient_colored = cv2.applyColorMap(gradient_map_normalized, cv2.COLORMAP_JET)

        # Set invalid points to black in the gradient map
        gradient_colored[~valid_mask] = [0, 0, 0]
        gradient_colored[~gradient_mask] = [0, 0, 0]

        ## Add text annotation DEBUG
        # random.seed(42)
        # num_points_to_annotate = 0
        # for i in range(height):
        #     for j in range(width):
        #         if num_points_to_annotate > 7:
        #             break

        #         if (
        #             gradient_mask[i, j] and random.random() < 0.05
        #         ):  # Check if the point is above the threshold
        #             # Define the text to be displayed (format as needed)
        #             text = f"{gradient_map[i, j]:.2f}"

        #             # Calculate the position for placing the text
        #             text_position = (
        #                 j + 2,
        #                 i + 2,
        #             )  # Adjust position based on your needs

        #             # Add text to the image
        #             cv2.putText(
        #                 gradient_colored,
        #                 text,
        #                 text_position,
        #                 cv2.FONT_HERSHEY_SIMPLEX,
        #                 0.3,
        #                 (255, 255, 255),
        #                 1,
        #             )

        #             num_points_to_annotate += 1

        # Flip the gradient map vertically
        gradient_colored = cv2.flip(gradient_colored, 0)

        # Convert the colored gradient map to an Image message and publish it
        gradient_msg = self.cv2_to_imgmsg(gradient_colored, encoding="bgr8")
        self.gradientmap_publisher.publish(gradient_msg)

    def cv2_to_imgmsg(self, cv_image, encoding="passthrough"):
        image_msg = Image()
        image_msg.height = cv_image.shape[0]
        image_msg.width = cv_image.shape[1]
        image_msg.encoding = encoding
        image_msg.is_bigendian = False
        image_msg.step = cv_image.shape[1]
        image_msg.data = cv_image.flatten().tolist()
        return image_msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToHeightmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
