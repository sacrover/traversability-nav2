// heightmap_layer.hpp
#ifndef NAV2_HEIGHTMAP_PLUGIN__HEIGHTMAP_LAYER_HPP_
#define NAV2_HEIGHTMAP_PLUGIN__HEIGHTMAP_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

namespace nav2_heightmap_plugin
{
class HeightmapLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  HeightmapLayer();
  virtual void onInitialize() override;
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  virtual void onFootprintChanged() override;

  void resetRange();
  virtual void reset() override;
  virtual bool isClearable() {return false;}

private:
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  
  bool enabled_;
  bool need_recalculation_;
  bool rolling_window_;
  double min_x_, min_y_, max_x_, max_y_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_occupancy_grid_;
  std::mutex data_mutex_;
  nav2_costmap_2d::Costmap2D * costmap_;
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;

  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

};
}  // namespace nav2_heightmap_plugin

#endif  // NAV2_HEIGHTMAP_PLUGIN__HEIGHTMAP_LAYER_HPP_