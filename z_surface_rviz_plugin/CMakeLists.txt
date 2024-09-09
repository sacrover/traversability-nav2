cmake_minimum_required(VERSION 3.8)
project(z_surface_rviz_plugin)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable MOC processing
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quad_surface_msg REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Include directories
include_directories(
  include
  ${PROJECT_NAME}/include
  ${rviz_common_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${quad_surface_msg_INCLUDE_DIRS}  # Add this line
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Wrap MOC files
set(MOC_HEADERS
  include/z_surface_rviz_plugin/z_surface_rviz_plugin.hpp
)

# Generate MOC files
qt5_wrap_cpp(MOC_FILES ${MOC_HEADERS})

# Add library
add_library(${PROJECT_NAME} SHARED
  src/z_surface_rviz_plugin.cpp
  ${MOC_FILES}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${Ogre_LIBRARIES}
  ${rviz_common_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${quad_surface_msg_LIBRARIES}  
  ${Qt5Widgets_LIBRARIES}
)

# Export libraries and dependencies
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rviz_common rclcpp sensor_msgs heightmap_msg)  # Add heightmap_msg here

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export plugin description
pluginlib_export_plugin_description_file(rviz_common rviz_plugin.xml)

ament_package()
