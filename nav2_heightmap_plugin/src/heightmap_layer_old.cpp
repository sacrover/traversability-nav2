#include "nav2_heightmap_plugin/heightmap_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_heightmap_plugin
{

HeightmapLayer::HeightmapLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

void HeightmapLayer::resetRange()
{
  min_x_ = min_y_ =  std::numeric_limits<double>::max();
  max_x_ = max_y_ = -std::numeric_limits<double>::max();
}
// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
HeightmapLayer::onInitialize()
{
  RCLCPP_INFO(rclcpp::get_logger(
      "nav2_costmap_2d"), ">>>>>>>>>>>>>>started onInitialize");
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  need_recalculation_ = true;
  current_ = true;

  costmap_ = layered_costmap_->getCostmap();

  // Subscriber for PointCloud2
  point_cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/ouster/points", 10, std::bind(&HeightmapLayer::pointCloudCallback, this, std::placeholders::_1));

  height_threshold_ = -0.5; // Height threshold for filtering points
  resetRange();
  RCLCPP_INFO(rclcpp::get_logger(
      "nav2_costmap_2d"), ">>>>>>>>>>>>>>finished onInitialize");
}

// Callback function for PointCloud2 subscription
void HeightmapLayer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  heightmap_.clear();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (*iter_z >= height_threshold_) {
      geometry_msgs::msg::Point32 point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      heightmap_.push_back(point);
    }
  }
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
HeightmapLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{

  RCLCPP_INFO(rclcpp::get_logger(
      "nav2_costmap_2d"), ">>>>>>>>>>>>>>started updateBounds");
   if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::numeric_limits<double>::max();
    *min_y = std::numeric_limits<double>::max();
    *max_x = -std::numeric_limits<double>::max();
    *max_y = -std::numeric_limits<double>::max();

    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;

    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }

  RCLCPP_INFO(rclcpp::get_logger(
      "nav2_costmap_2d"), "Updated bounds - last_min_x_: %f, last_min_y_: %f", last_min_x_, last_min_y_);
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
HeightmapLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
void
HeightmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i, int max_j)
{

  RCLCPP_INFO(rclcpp::get_logger(
      "nav2_costmap_2d"), ">>>>>>>>>>>>>>started updateCosts");

  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> guard(data_mutex_);
  unsigned char * costmap_array = costmap_->getCharMap();
  unsigned int size_x = costmap_->getSizeInCellsX(), size_y = costmap_->getSizeInCellsY();

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  for (const auto& point : heightmap_) {
    double x = point.x, y = point.y;
    if (x < 0 || x >= size_x * costmap_->getResolution() || y < 0 || y >= size_y * costmap_->getResolution()) {
      continue;
    }

    unsigned int map_x, map_y;
    costmap_->worldToMap(x, y, map_x, map_y);
    int index = costmap_->getIndex(map_x, map_y);

    if (point.z >= height_threshold_) {
      costmap_array[index] = LETHAL_OBSTACLE;
    } else {
      costmap_array[index] = NO_INFORMATION;
    }
  }

  // Use the appropriate update method
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  RCLCPP_INFO(rclcpp::get_logger(
      "nav2_costmap_2d"), ">>>>>>>>>>>>>>finished updateCosts");
}



}  // namespace custom_nav2_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_heightmap_plugin::HeightmapLayer, nav2_costmap_2d::Layer)
