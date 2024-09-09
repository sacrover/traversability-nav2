from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    package_share_directory = get_package_share_directory("traversability")

    # Define the path to the YAML file inside the package
    params_file_path = os.path.join(
        package_share_directory, "config", "traversability_params.yaml"
    )

    pcl_to_heightmap_node = Node(
        package="traversability",
        executable="pcl_to_heightmap",
        name="pointcloud_to_heightmap",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    heightmap_transform_node = Node(
        package="traversability",
        executable="grid_transform",
        name="grid_transform",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    fused_traversability_node = Node(
        package="traversability",
        executable="fused_traversability_map",
        name="traversability_filtering",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, params_file_path],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),
            pcl_to_heightmap_node,
            heightmap_transform_node,
            fused_traversability_node,
        ]
    )
