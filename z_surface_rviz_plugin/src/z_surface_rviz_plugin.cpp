#include "z_surface_rviz_plugin/z_surface_rviz_plugin.hpp"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreMaterial.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace z_surface_rviz_plugin
{

MyDisplay::MyDisplay()
{
}

MyDisplay::~MyDisplay()
{
  // Destructor implementation
}

void MyDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateTopic();
}

void MyDisplay::createGridSurface(const std::vector<float>& x_values, const std::vector<float>& y_values, const std::vector<float>& z_values)
{
  Ogre::SceneManager* scene_manager = context_->getSceneManager();

  // Remove the old manual object if it exists
  if (manual_object_)
  {
    std::cout << "Destroying old manual object" << std::endl;
    scene_manager->destroyManualObject(manual_object_);
    manual_object_ = nullptr;
  }

  // Create a ManualObject to hold the grid surface
  manual_object_ = scene_manager->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);

  // Create a material for the grid surface
  Ogre::MaterialPtr surface_material = Ogre::MaterialManager::getSingleton().getByName("GridSurfaceMaterial");
  if (!surface_material)
  {
    std::cout << "Creating new material GridSurfaceMaterial" << std::endl;

    // Create a material for the grid surface
    surface_material = Ogre::MaterialManager::getSingleton().create(
        "GridSurfaceMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    surface_material->setDiffuse(Ogre::ColourValue(0.0, 1.0, 0.0, 1.0));  // Semi-transparent green
    surface_material->setAmbient(Ogre::ColourValue(0.0, 1.0, 0.0, 0.5));
    surface_material->setSelfIllumination(Ogre::ColourValue(0.0, 1.0, 0.0, 0.5));

    Ogre::Technique* surface_tech = surface_material->getTechnique(0);
    Ogre::Pass* surface_pass = surface_tech->getPass(0);
    surface_pass->setDepthCheckEnabled(true);   // Enable depth checking
    surface_pass->setCullingMode(Ogre::CULL_NONE); // Disable face culling

    // Add an additional pass for wireframe
    Ogre::Pass* wireframe_pass = surface_tech->createPass();
    wireframe_pass->setPolygonMode(Ogre::PM_WIREFRAME);
    wireframe_pass->setDiffuse(Ogre::ColourValue(0.0, 0.0, 0.0, 1.0));  // Black wireframe
    wireframe_pass->setAmbient(Ogre::ColourValue(0.0, 0.0, 0.0, 1.0));
    wireframe_pass->setSelfIllumination(Ogre::ColourValue(0.0, 0.0, 0.0, 1.0));
    wireframe_pass->setSceneBlending(Ogre::SBT_REPLACE);
    wireframe_pass->setCullingMode(Ogre::CULL_NONE); // Disable face culling
  }
  // Begin creating the grid surface
  manual_object_->begin("GridSurfaceMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // Creating the grid
  std::vector<std::array<float, 3>> vertices;
  float cell_size = 1.0;  // Adjust the cell size as needed

  float min_x = *std::min_element(x_values.begin(), x_values.end()) + 0.5f * cell_size;
  float max_x = *std::max_element(x_values.begin(), x_values.end()) + 0.5f * cell_size;
  float min_y = *std::min_element(y_values.begin(), y_values.end()) + 0.5f * cell_size;
  float max_y = *std::max_element(y_values.begin(), y_values.end()) + 0.5f * cell_size;

  std::vector<float> x_edges, y_edges;
  for (float x = min_x; x <= max_x + cell_size; x += cell_size)
    x_edges.push_back(x);
  for (float y = min_y; y <= max_y + cell_size; y += cell_size)
    y_edges.push_back(y);

  std::vector<std::vector<float>> x_grid(y_edges.size() - 1, std::vector<float>(x_edges.size() - 1));
  std::vector<std::vector<float>> y_grid(y_edges.size() - 1, std::vector<float>(x_edges.size() - 1));
  std::vector<std::vector<float>> z_grid(y_edges.size() - 1, std::vector<float>(x_edges.size() - 1, 0.0f));

  for (size_t i = 0; i < y_edges.size() - 1; ++i)
  {
    for (size_t j = 0; j < x_edges.size() - 1; ++j)
    {
      x_grid[i][j] = x_edges[j];
      y_grid[i][j] = y_edges[i];
    }
  }

  for (size_t i = 0; i < x_values.size(); ++i)
  {
    size_t x_idx = static_cast<size_t>((x_values[i] - min_x) / cell_size);
    size_t y_idx = static_cast<size_t>((y_values[i] - min_y) / cell_size);
    if (x_idx < x_grid[0].size() && y_idx < y_grid.size())
    {
      z_grid[y_idx][x_idx] = z_values[i];
    }
  }

  for (size_t i = 0; i < y_grid.size() - 1; ++i)
  {
    for (size_t j = 0; j < x_grid[0].size() - 1; ++j)
    {
      manual_object_->position(x_grid[i][j], y_grid[i][j], z_grid[i][j]);         // Bottom-left
      manual_object_->position(x_grid[i][j + 1], y_grid[i][j + 1], z_grid[i][j + 1]); // Bottom-right
      manual_object_->position(x_grid[i + 1][j + 1], y_grid[i + 1][j + 1], z_grid[i + 1][j + 1]); // Top-right
      manual_object_->position(x_grid[i + 1][j], y_grid[i + 1][j], z_grid[i + 1][j]); // Top-left

      int base_index = (i * (x_grid[0].size() - 1) + j) * 4;
      manual_object_->quad(base_index, base_index + 1, base_index + 2, base_index + 3);
    }
  }

  manual_object_->end();
  manual_object_->setVisible(true);
}

void MyDisplay::processMessage(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::cout << "Processing message" << std::endl;

  x_points_.clear();
  y_points_.clear();
  z_points_.clear();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    x_points_.push_back(*iter_x);
    y_points_.push_back(*iter_y);
    z_points_.push_back(*iter_z);
  }

  createGridSurface(x_points_, y_points_, z_points_);
}


void MyDisplay::updateTopic()
{
  this->resetSubscription(); // Ensure previous subscription is reset
  this->subscribe(); // Set up the new subscription
}

}  // namespace z_surface_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(z_surface_rviz_plugin::MyDisplay, rviz_common::Display)
