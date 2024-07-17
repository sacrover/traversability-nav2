#include "nav2_heightmap_plugin/heightmap_layer.hpp"

namespace nav2_heightmap_plugin
{
HeightmapLayer::HeightmapLayer()
{
}

void HeightmapLayer::resetRange()
{
  min_x_ = min_y_ =  std::numeric_limits<double>::max();
  max_x_ = max_y_ = -std::numeric_limits<double>::max();
}

void HeightmapLayer::onInitialize()
{
  auto node = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  costmap_ = layered_costmap_->getCostmap();

  occupancy_grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/heightmap", 10, std::bind(&HeightmapLayer::occupancyGridCallback, this, std::placeholders::_1));

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  
  need_recalculation_ = true;
  current_ = true;

  rolling_window_ = layered_costmap_->isRolling();
}

void HeightmapLayer::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  latest_occupancy_grid_ = msg;
}

void HeightmapLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  if (latest_occupancy_grid_) {
    if (rolling_window_) {
    updateOrigin(robot_x - 5.0 / 2, robot_y - 5.0 / 2);
  }
    // *min_x = 0.0;
    // *min_y = 0.0;
    // *max_x = latest_occupancy_grid_->info.width * latest_occupancy_grid_->info.resolution;
    // *max_y = latest_occupancy_grid_->info.height * latest_occupancy_grid_->info.resolution;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);
}

void HeightmapLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  if (!enabled_ || !latest_occupancy_grid_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tfBuffer_->lookupTransform("map", latest_occupancy_grid_->header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(rclcpp::get_logger("nav2_heightmap_plugin"), "Could not transform %s", ex.what());
    return;
  }

  unsigned char* costmap_array = costmap_->getCharMap();
  for (unsigned int i = 0; i < latest_occupancy_grid_->info.width; ++i) {
    for (unsigned int j = 0; j < latest_occupancy_grid_->info.height; ++j) {
      int index = i + j * latest_occupancy_grid_->info.width;

      // Transform point from os_sensor frame to map frame

      geometry_msgs::msg::PointStamped point_in, point_out;
      point_in.header.frame_id = latest_occupancy_grid_->header.frame_id;
      point_in.point.x = i * latest_occupancy_grid_->info.resolution;
      point_in.point.y = j * latest_occupancy_grid_->info.resolution;
      point_in.point.z = 0.0;

      tf2::doTransform(point_in, point_out, transform);

      unsigned int map_x, map_y;
      // costmap_->worldToMap(i * latest_occupancy_grid_->info.resolution, j * latest_occupancy_grid_->info.resolution, map_x, map_y);
       if (costmap_->worldToMap(point_out.point.x, point_out.point.y, map_x, map_y)) {
        int costmap_index = costmap_->getIndex(map_x, map_y);
        costmap_array[costmap_index] = latest_occupancy_grid_->data[index];
       }
    }
  }

  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

void HeightmapLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

void HeightmapLayer::reset()
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  latest_occupancy_grid_.reset();
}
}  // namespace nav2_heightmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_heightmap_plugin::HeightmapLayer, nav2_costmap_2d::Layer)

