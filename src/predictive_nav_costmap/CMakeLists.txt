cmake_minimum_required(VERSION 3.8)
project(predictive_nav_costmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(predictive_nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  nav2_util
  pluginlib
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  predictive_nav_msgs
)

# The library name must match the <library path="..."> in
# predictive_nav_costmap_plugins.xml.
add_library(predictive_nav_costmap_layers SHARED
  src/predicted_obstacle_layer.cpp
)
target_include_directories(predictive_nav_costmap_layers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(predictive_nav_costmap_layers ${dependencies})

install(TARGETS predictive_nav_costmap_layers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

pluginlib_export_plugin_description_file(nav2_costmap_2d predictive_nav_costmap_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(predictive_nav_costmap_layers)
ament_export_dependencies(${dependencies})

ament_package()
