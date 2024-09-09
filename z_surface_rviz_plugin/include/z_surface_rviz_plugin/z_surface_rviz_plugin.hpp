#ifndef Z_SURFACE_RVIZ_PLUGIN_HPP
#define Z_SURFACE_RVIZ_PLUGIN_HPP

#include <rviz_common/display.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <OgreManualObject.h>
#include <rclcpp/rclcpp.hpp>
#include <quad_surface_msg/msg/quad_surface.hpp>  // Include the QuadSurface message header

namespace z_surface_rviz_plugin
{

class MyDisplay : public rviz_common::MessageFilterDisplay<quad_surface_msg::msg::QuadSurface>
{
  Q_OBJECT
public:
  MyDisplay();
  ~MyDisplay() override;

  void onInitialize() override;

protected:
  void createGridSurface(const quad_surface_msg::msg::QuadSurface::ConstSharedPtr quad_surface);
  void processMessage(const quad_surface_msg::msg::QuadSurface::ConstSharedPtr msg) override;  // Update the processMessage method

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
