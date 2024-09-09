import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from heightmap_msg.msg import GridMap16
from std_msgs.msg import Header
import numpy as np
import pickle


class KalmanFilter:
    def __init__(self, process_noise, measurement_noise):
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def predict(self, state, covariance):
        predicted_state = state
        predicted_covariance = covariance + self.process_noise
        return predicted_state, predicted_covariance

    def update(self, predicted_state, predicted_covariance, measurement):
        kalman_gain = predicted_covariance / (
            predicted_covariance + self.measurement_noise
        )
        state = predicted_state + kalman_gain * (measurement - predicted_state)
        covariance = (1 - kalman_gain) * predicted_covariance
        return state, covariance


class FusedTraversabilityMap(Node):
    def __init__(self):
        super().__init__("fused_traversability_map", namespace="traversability")

        # Declare parameters with defaults
        self.declare_parameter("process_noise", 0.04)
        self.declare_parameter("measurement_noise", 0.01)
        self.declare_parameter("max_height", 2.0)
        self.declare_parameter("min_height", -2.0)
        self.declare_parameter("max_gradient", 5.0)
        self.declare_parameter("min_gradient", 0.0)
        self.declare_parameter("traversability_max_gradient", 0.9)
        self.declare_parameter("traversability_min_gradient", 0.5)
        self.declare_parameter("clear_map_after", 7)
        self.declare_parameter("norm_max", 1000.0)
        self.declare_parameter("norm_min", 0.0)
        self.declare_parameter("save_map", True)
        self.declare_parameter("publish_fused_heightmap", True)
        self.declare_parameter("save_map_interval", 5.0)
        self.declare_parameter("publish_map_interval", 2.0)
        self.declare_parameter("out_frame", "odom")
        self.declare_parameter("subscription_topic", "heightmap_odom")
        self.declare_parameter("global_heightmap_topic", "global_heightmap")
        self.declare_parameter("fused_heightmap_topic", "heightmap_fused_odom")
        self.declare_parameter("traversability_topic", "traversability_odom")

        # Get parameters
        process_noise = self.get_parameter("process_noise").value
        measurement_noise = self.get_parameter("measurement_noise").value
        self.max_height = self.get_parameter("max_height").value
        self.min_height = self.get_parameter("min_height").value
        self.max_gradient = self.get_parameter("max_gradient").value
        self.min_gradient = self.get_parameter("min_gradient").value
        self.traversability_max_gradient = self.get_parameter(
            "traversability_max_gradient"
        ).value
        self.traversability_min_gradient = self.get_parameter(
            "traversability_min_gradient"
        ).value
        self.clear_map_after = self.get_parameter("clear_map_after").value
        self.norm_max = self.get_parameter("norm_max").value
        self.norm_min = self.get_parameter("norm_min").value
        self.save_map = self.get_parameter("save_map").value
        self.publish_fused_heightmap = self.get_parameter(
            "publish_fused_heightmap"
        ).value
        self.out_frame = self.get_parameter("out_frame").value
        subscription_topic = (
            self.get_parameter("subscription_topic").get_parameter_value().string_value
        )
        traversability_topic = (
            self.get_parameter("traversability_topic")
            .get_parameter_value()
            .string_value
        )
        fused_heightmap_topic = (
            self.get_parameter("fused_heightmap_topic")
            .get_parameter_value()
            .string_value
        )

        self.subscription = self.create_subscription(
            GridMap16, subscription_topic, self.map_callback, 10
        )
        self.publisher = self.create_publisher(OccupancyGrid, traversability_topic, 10)
        self.heightmap_fused_publisher = self.create_publisher(
            OccupancyGrid, fused_heightmap_topic, 10
        )
        self.global_map_kf = None
        self.denorm_global_map = None
        self.global_covariance = None
        self.map_info = None
        self.old_map_kf = None
        self.incoming_map_info = None

        self.map_buffer = []  # Buffer to store the incoming maps
        self.map_to_publish = None

        self.kalman_filter = KalmanFilter(
            process_noise=process_noise, measurement_noise=measurement_noise
        )

        # Initialize min and max extents
        self.min_x = float("inf")
        self.min_y = float("inf")
        self.max_x = float("-inf")
        self.max_y = float("-inf")
        self.old_width = 0
        self.old_height = 0
        self.offset_x = None
        self.offset_y = None
        self.resolution = None

        self.save_id = 0

        # Set up a timer to save the map every 5 seconds
        self.timer = self.create_timer(5.0, self.map_saver)

    def map_saver(self):
        if self.global_map_kf is not None:
            if self.save_map:
                self.save_id += 1
                np.save(f"{self.save_id}_global_map.npy", self.map_to_publish)
                with open(f"{self.save_id}_global_map_info.pkl", "wb") as f:
                    pickle.dump(self.map_info, f)
                self.get_logger().info("Map saved")

    def initialize_global_map(self, info):
        self.resolution = info.resolution

        self.global_map_kf = np.full((info.height, info.width), -1, dtype=float)
        self.old_map_kf = self.global_map_kf.copy()
        self.global_covariance = np.zeros((info.height, info.width), dtype=float)
        self.map_info = info
        self.old_height = info.height
        self.old_width = info.width

        # Initialize extents with current map info
        self.update_extents(info)

    def update_extents(self, info):
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        end_x = origin_x + info.width * resolution
        end_y = origin_y + info.height * resolution

        self.min_x = min(self.min_x, origin_x)
        self.min_y = min(self.min_y, origin_y)
        self.max_x = max(self.max_x, end_x)
        self.max_y = max(self.max_y, end_y)

    def map_callback(self, msg):

        if self.clear_map_after == -1:
            self.merge_maps([msg])
            return

        self.map_buffer.append(msg)

        if len(self.map_buffer) > self.clear_map_after:
            self.global_map_kf = None
            self.min_x = float("inf")
            self.min_y = float("inf")
            self.max_x = float("-inf")
            self.max_y = float("-inf")
            self.merge_maps(self.map_buffer[1:])
            self.map_buffer.pop(0)
            return

    def merge_maps(self, maps):
        for map in maps:
            if self.global_map_kf is None:
                self.initialize_global_map(map.info)
            self.incoming_map_info = map.info
            self.update_extents(map.info)
            self.expand_global_map(map.info)
            self.update_map_data(map)

        # After merging, calculate the gradient map
        self.calculate_gradient_map()

    def denormalize_global_map(self):

        # Denormalize the global map using the normalization parameters
        original_min = self.min_height
        original_max = self.max_height

        # do not normalize if the value is -1
        self.denorm_global_map = np.full(self.global_map_kf.shape, np.nan)
        valid_mask = self.global_map_kf != -1

        self.denorm_global_map[valid_mask] = (
            self.global_map_kf[valid_mask] - self.norm_min
        ) / (self.norm_max - self.norm_min) * (
            original_max - original_min
        ) + original_min

    def terrain_impedance(self, gradient_map, max_gradient=0.8, min_gradient=0.3):
        impedance = np.ones_like(gradient_map) * -1.0
        valid_mask = ~np.isnan(gradient_map)
        impedance[valid_mask] = (
            np.clip(
                (gradient_map[valid_mask] - min_gradient)
                / (max_gradient - min_gradient),
                0.0,
                1.0,
            )
            * 100.0
        )

        return impedance

    def calculate_gradient_map(self):

        self.denormalize_global_map()
        global_map_gradient = np.full(self.denorm_global_map.shape, np.nan)
        heightmap = self.denorm_global_map
        height, width = self.denorm_global_map.shape

        for i in range(1, height - 1):
            for j in range(1, width - 1):
                if not np.isnan(heightmap[i, j]):
                    dz_dx = 0.0
                    dz_dy = 0.0

                    if not np.isnan(heightmap[i, j - 1]):
                        dz_dx = (
                            heightmap[i, j] - heightmap[i, j - 1]
                        ) / self.resolution

                    if not np.isnan(heightmap[i - 1, j]):
                        dz_dy = (
                            heightmap[i, j] - heightmap[i - 1, j]
                        ) / self.resolution

                    global_map_gradient[i, j] = np.sqrt(dz_dx**2 + dz_dy**2)

        # Normalize the gradient map for visualization
        nan_mask = np.isnan(global_map_gradient)
        global_map_gradient_normalized = (
            np.clip(
                (global_map_gradient - self.min_gradient)
                / (self.max_gradient - self.min_gradient),
                0.0,
                1.0,
            )
            * (self.norm_max - self.norm_min)
            + self.norm_min
        )
        global_map_gradient_normalized[nan_mask] = -1.0

        traversability_map = self.terrain_impedance(
            global_map_gradient,
            max_gradient=self.traversability_max_gradient,
            min_gradient=self.traversability_min_gradient,
        )

        if self.publish_fused_heightmap:
            self.map_publisher(
                self.global_map_kf,
                self.heightmap_fused_publisher,
                isGridMap16=False,
                factor=0.1,
            )

        self.map_publisher(traversability_map, self.publisher, isGridMap16=False)

    def map_publisher(self, map, publisher, isGridMap16=True, factor=1.0):

        if isGridMap16:
            map = map.astype(np.int16)
            msg = GridMap16()
        else:
            map = map * factor
            map = map.astype(np.int8)
            msg = OccupancyGrid()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.out_frame
        msg.info = self.map_info
        msg.data = map.flatten().tolist()
        publisher.publish(msg)

    def expand_global_map(self, new_info):
        resolution = self.map_info.resolution
        new_origin_x = self.min_x
        new_origin_y = self.min_y
        new_width = int((self.max_x - self.min_x) / resolution)
        new_height = int((self.max_y - self.min_y) / resolution)

        # Create new global map and covariance with expanded dimensions
        new_global_map_kf = np.full((new_height, new_width), -1.0, dtype=float)
        new_global_covariance = np.zeros((new_height, new_width), dtype=float)

        # Calculate offsets to place old map into the new map
        offset_x = new_width - self.old_width
        offset_y = new_height - self.old_height

        # Ensure offsets are within bounds
        offset_x = max(0, offset_x)
        offset_y = max(0, offset_y)

        if new_info.origin.position.x > self.map_info.origin.position.x:
            offset_x = 0
        if new_info.origin.position.y > self.map_info.origin.position.y:
            offset_y = 0

        new_global_map_kf[
            offset_y : offset_y + self.global_map_kf.shape[0],
            offset_x : offset_x + self.global_map_kf.shape[1],
        ] = np.where(
            self.old_map_kf != -1,
            self.old_map_kf,
            new_global_map_kf[
                offset_y : offset_y + self.global_map_kf.shape[0],
                offset_x : offset_x + self.global_map_kf.shape[1],
            ],
        )

        # Place old covariance data into the new map
        new_global_covariance[
            max(0, offset_y) : max(0, offset_y) + self.global_covariance.shape[0],
            max(0, offset_x) : max(0, offset_x) + self.global_covariance.shape[1],
        ] = self.global_covariance

        # Update map info
        self.global_map_kf = new_global_map_kf
        self.global_covariance = new_global_covariance

        # Update the origin and size of the map_info
        self.map_info.origin.position.x = new_origin_x
        self.map_info.origin.position.y = new_origin_y
        self.map_info.width = new_width
        self.map_info.height = new_height

        self.old_height = new_height
        self.old_width = new_width

    def update_map_data(self, msg):
        resolution = self.map_info.resolution
        new_origin_x = msg.info.origin.position.x
        new_origin_y = msg.info.origin.position.y
        offset_x = int(
            (new_origin_x - self.map_info.origin.position.x) / resolution - 0.5
        )
        offset_y = int(
            (new_origin_y - self.map_info.origin.position.y) / resolution - 0.5
        )

        self.offset_x = offset_x
        self.offset_y = offset_y

        # Ensure offsets are within bounds
        offset_x = max(0, offset_x)
        offset_y = max(0, offset_y)

        # Calculate size of the overlap area
        size_x = min(self.global_map_kf.shape[1] - offset_x, msg.info.width)
        size_y = min(self.global_map_kf.shape[0] - offset_y, msg.info.height)

        # Slice the incoming data and update the global map
        incoming_data = np.array(msg.data, dtype=float).reshape(
            (msg.info.height, msg.info.width)
        )

        for y in range(size_y):
            for x in range(size_x):
                incoming_value = incoming_data[y, x]
                global_x = offset_x + x
                global_y = offset_y + y
                old_value_kf = -1

                if (
                    0 <= global_y < self.old_map_kf.shape[0]
                    and 0 <= global_x < self.old_map_kf.shape[1]
                ):
                    old_value_kf = (
                        self.old_map_kf[global_y, global_x]
                        if self.old_map_kf[global_y, global_x] is not None
                        else -1
                    )
                if old_value_kf != -1 and incoming_value != -1:
                    predicted_state, predicted_covariance = self.kalman_filter.predict(
                        old_value_kf, self.global_covariance[global_y, global_x]
                    )
                    updated_state, updated_covariance = self.kalman_filter.update(
                        predicted_state, predicted_covariance, incoming_value
                    )
                    self.global_map_kf[global_y, global_x] = updated_state
                    self.global_covariance[global_y, global_x] = updated_covariance
                elif incoming_value != -1:
                    self.global_map_kf[global_y, global_x] = incoming_value
                    self.global_covariance[global_y, global_x] = (
                        self.kalman_filter.process_noise
                    )

        self.old_map_kf = self.global_map_kf.copy()


def main(args=None):
    rclpy.init(args=args)
    fused_traversability_map = FusedTraversabilityMap()
    rclpy.spin(fused_traversability_map)
    fused_traversability_map.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
