#ifndef Z_SURFACE_RVIZ_PLUGIN_HPP
#define Z_SURFACE_RVIZ_PLUGIN_HPP

#include <rviz_common/display.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <OgreManualObject.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace z_surface_rviz_plugin
{

class MyDisplay : public rviz_common::MessageFilterDisplay<sensor_msgs::msg::PointCloud2>
{
  Q_OBJECT
public:
  MyDisplay();
  ~MyDisplay() override;

  void onInitialize() override;

protected:
  void createGridSurface(const std::vector<float>& x_values, const std::vector<float>& y_values, const std::vector<float>& z_values);
  void processMessage(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) override;

private Q_SLOTS:
  void updateTopic();

private:
  Ogre::ManualObject* manual_object_ = nullptr;
  std::vector<float> x_points_;
  std::vector<float> y_points_;
  std::vector<float> z_points_;
};

}  // namespace z_surface_rviz_plugin

#endif  // Z_SURFACE_RVIZ_PLUGIN_HPP
