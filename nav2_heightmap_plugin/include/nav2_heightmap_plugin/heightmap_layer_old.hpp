// heightmap_layer.hpp
#ifndef NAV2_HEIGHTMAP_PLUGIN__HEIGHTMAP_LAYER_HPP_
#define NAV2_HEIGHTMAP_PLUGIN__HEIGHTMAP_LAYER_HPP_

#include "geometry_msgs/msg/point32.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include <vector>
#include <mutex>


// TODO: Try subscribing to occupancy grid and process from there 

namespace nav2_heightmap_plugin
{

class HeightmapLayerOld : public nav2_costmap_2d::CostmapLayer
{
public:
  HeightmapLayerOld();

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  virtual void onFootprintChanged() override;

  void resetRange();
  virtual void reset() override;
  virtual bool isClearable() {return false;}

protected:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  std::vector<geometry_msgs::msg::Point32> heightmap_;
  double height_threshold_;
  std::mutex data_mutex_;
  bool need_recalculation_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  double min_x_, min_y_, max_x_, max_y_;
  nav2_costmap_2d::Costmap2D * costmap_;
};

}  // namespace custom_nav2_costmap_plugin

#endif  // NAV2_HEIGHTMAP_PLUGIN__HEIGHTMAP_LAYER_HPP_
