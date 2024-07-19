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
  declareParameter("threshold", rclcpp::ParameterValue(0)); // Define the threshold parameter
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "threshold", threshold_);

  costmap_ = layered_costmap_->getCostmap();

  occupancy_grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/heightmap_odom", 10, std::bind(&HeightmapLayer::occupancyGridCallback, this, std::placeholders::_1));

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

void HeightmapLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/, double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!enabled_ || !latest_occupancy_grid_) {
        return;
    }

    if (layered_costmap_->isRolling()) {
      updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }

    // Get the dimensions and resolution of the occupancy grid
    unsigned int width = latest_occupancy_grid_->info.width;
    unsigned int height = latest_occupancy_grid_->info.height;
    double resolution = latest_occupancy_grid_->info.resolution;

    // Get the origin of the occupancy grid
    double origin_x = latest_occupancy_grid_->info.origin.position.x;
    double origin_y = latest_occupancy_grid_->info.origin.position.y;

    // Update the bounds of the costmap to include the entire occupancy grid
    double grid_min_x = origin_x;
    double grid_min_y = origin_y;
    double grid_max_x = 4*(origin_x + width * resolution);
    double grid_max_y = 4*(origin_y + height * resolution);

    *min_x = std::min(*min_x, grid_min_x);
    *min_y = std::min(*min_y, grid_min_y);
    *max_x = std::max(*max_x, grid_max_x);
    *max_y = std::max(*max_y, grid_max_y);
}

void HeightmapLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  if (!enabled_ || !latest_occupancy_grid_) {
    return;
  }

  // Get dimensions of the occupancy grid
  const auto& info = latest_occupancy_grid_->info;
  int width = info.width;
  int height = info.height;
  double resolution = info.resolution;
  double origin_x = info.origin.position.x;
  double origin_y = info.origin.position.y;

  // Get the pointer to the master costmap array
  unsigned char* costmap_array = master_grid.getCharMap();

  // Iterate through the specified window
  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      // Convert costmap coordinates to occupancy grid coordinates
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      int mx = static_cast<int>((wx - origin_x) / resolution);
      int my = static_cast<int>((wy - origin_y) / resolution);

      // Check bounds
      if (mx >= 0 && mx < width && my >= 0 && my < height) {
        int index = my * width + mx;
        int8_t value = latest_occupancy_grid_->data[index];

        // Convert occupancy grid value to costmap cost
        if (value == -1) {
          continue; // Unknown value, skip
        }

        unsigned char cost = nav2_costmap_2d::FREE_SPACE;
        if (value >= 0 && value <= 100) {
          cost = static_cast<unsigned char>((value / 100.0) * nav2_costmap_2d::LETHAL_OBSTACLE);
        }

        // Calculate the index in the master costmap array
        int costmap_index = master_grid.getIndex(i, j);
        // Set the cost in the master costmap array
        costmap_array[costmap_index] = cost;
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

