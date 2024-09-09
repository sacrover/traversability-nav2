#include "z_surface_rviz_plugin/z_surface_rviz_plugin.hpp"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreMaterial.h>
#include <OgrePass.h>
#include <OgreTechnique.h>

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

void MyDisplay::createGridSurface(const quad_surface_msg::msg::QuadSurface::ConstSharedPtr quad_surface)
{
    const auto& origin = quad_surface->origin.position;
    Ogre::SceneManager* scene_manager = context_->getSceneManager();

    // Remove the old manual object if it exists
    if (manual_object_)
    {
        std::cout << "Destroying old manual object" << std::endl;
        scene_manager->destroyManualObject(manual_object_);
        manual_object_ = nullptr;
    }

    // Create a new ManualObject to hold the grid surface
    manual_object_ = scene_manager->createManualObject();
    manual_object_->setDynamic(true);
    scene_node_->attachObject(manual_object_);

    // Create or retrieve the material for the grid surface
    Ogre::MaterialPtr surface_material = Ogre::MaterialManager::getSingleton().getByName("GridSurfaceMaterial");
    if (!surface_material)
    {
        std::cout << "Creating new material GridSurfaceMaterial" << std::endl;
        surface_material = Ogre::MaterialManager::getSingleton().create(
            "GridSurfaceMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        
        surface_material->setReceiveShadows(false);  // Disable shadows for clarity

        Ogre::Technique* surface_tech = surface_material->getTechnique(0);
        Ogre::Pass* surface_pass = surface_tech->getPass(0);
        surface_pass->setLightingEnabled(false);
        surface_pass->setVertexColourTracking(Ogre::TVC_DIFFUSE); // Use vertex colors
        surface_pass->setDepthCheckEnabled(true);
        surface_pass->setCullingMode(Ogre::CULL_NONE);

        // Add an additional pass for wireframe
        Ogre::Pass* wireframe_pass = surface_tech->createPass();
        wireframe_pass->setPolygonMode(Ogre::PM_WIREFRAME);
        wireframe_pass->setDiffuse(Ogre::ColourValue(0.0, 0.0, 0.0, 1.0));  // Black wireframe
        wireframe_pass->setAmbient(Ogre::ColourValue(0.0, 0.0, 0.0, 1.0));
        wireframe_pass->setSelfIllumination(Ogre::ColourValue(0.0, 0.0, 0.0, 1.0));
        wireframe_pass->setSceneBlending(Ogre::SBT_REPLACE);
        wireframe_pass->setCullingMode(Ogre::CULL_NONE);
    }

    // Begin creating the grid surface using quads
    manual_object_->begin("GridSurfaceMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Find min and max height for normalization
    float min_height = std::numeric_limits<float>::max();
    float max_height = std::numeric_limits<float>::lowest();
    for (const auto& quad : quad_surface->quads)
    {
        for (const auto& vertex : quad.vertices)
        {
            if (vertex.z < min_height) min_height = vertex.z;
            if (vertex.z > max_height) max_height = vertex.z;
        }
    }

    // Iterate over the quads in the QuadSurface message and create the grid
    for (const auto& quad : quad_surface->quads)
    {
        // Extract the four vertices of the quad
        auto& v0 = quad.vertices[0];
        auto& v1 = quad.vertices[1];
        auto& v2 = quad.vertices[2];
        auto& v3 = quad.vertices[3];

        // Function to map height to a color
        auto getColorFromHeight = [min_height, max_height](float height) -> Ogre::ColourValue {
            float normalized_height = (height - min_height) / (max_height - min_height);
            return Ogre::ColourValue(normalized_height, 1.0f - normalized_height, 0.0f, 1.0f); // Gradient from red to green
        };

        // Add positions and colors for the quad vertices
        manual_object_->position(v0.x + origin.x, v0.y + origin.y, v0.z + origin.z);
        manual_object_->colour(getColorFromHeight(v0.z));
            
        manual_object_->position(v1.x + origin.x, v1.y + origin.y, v1.z + origin.z);
        manual_object_->colour(getColorFromHeight(v1.z));
            
        manual_object_->position(v2.x + origin.x, v2.y + origin.y, v2.z + origin.z);
        manual_object_->colour(getColorFromHeight(v2.z));
            
        manual_object_->position(v3.x + origin.x, v3.y + origin.y, v3.z + origin.z);
        manual_object_->colour(getColorFromHeight(v3.z));


        // Create the quad by specifying the vertex indices
        int base_index = manual_object_->getCurrentVertexCount() - 4;
        manual_object_->quad(base_index, base_index + 1, base_index + 2, base_index + 3);
    }

    manual_object_->end();
    manual_object_->setVisible(true);
}

void MyDisplay::processMessage(const quad_surface_msg::msg::QuadSurface::ConstSharedPtr msg)
{
    std::cout << "Processing QuadSurface message" << std::endl;

    // Directly pass the QuadSurface message to createGridSurface
    createGridSurface(msg);
}

void MyDisplay::updateTopic()
{
  this->resetSubscription(); // Ensure previous subscription is reset
  this->subscribe(); // Set up the new subscription
}

}  // namespace z_surface_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(z_surface_rviz_plugin::MyDisplay, rviz_common::Display)
