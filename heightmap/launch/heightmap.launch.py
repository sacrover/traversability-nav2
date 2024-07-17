from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    heightmap_occupancy_grid_node = Node(
        package="heightmap",
        executable="heightmap_occupancy_grid",
        name="pointcloud_to_heightmap",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    heightmap_transform_node = Node(
        package="heightmap",
        executable="heightmap_transform",
        name="occupancy_grid_transformer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),
            heightmap_occupancy_grid_node,
            heightmap_transform_node,
        ]
    )
