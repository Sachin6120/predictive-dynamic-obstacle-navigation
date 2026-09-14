cmake_minimum_required(VERSION 3.8)
project(predictive_nav_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(predictive_nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(lidar_obstacle_tracker_node src/lidar_obstacle_tracker_node.cpp)
target_include_directories(lidar_obstacle_tracker_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(lidar_obstacle_tracker_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  predictive_nav_msgs
)

install(TARGETS
  lidar_obstacle_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/stage4g1_static_probe.py
  scripts/stage4g1_nearwall_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Include production TU without changing its encapsulation/API for tests.
  add_executable(stage4g2_lifecycle test/stage4g2_lifecycle.cpp)
  target_compile_options(stage4g2_lifecycle PRIVATE -fno-access-control)
  target_include_directories(stage4g2_lifecycle PRIVATE include ${EIGEN3_INCLUDE_DIR})
  ament_target_dependencies(stage4g2_lifecycle rclcpp sensor_msgs nav_msgs geometry_msgs
    tf2 tf2_ros tf2_geometry_msgs visualization_msgs predictive_nav_msgs)
  add_test(NAME stage4g2_lifecycle COMMAND stage4g2_lifecycle)
  set_tests_properties(stage4g2_lifecycle PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=94" TIMEOUT 20)

  # Stage-4G3: deblending against recorded Gazebo merge/occlusion geometry.
  add_executable(stage4g3_deblend test/stage4g3_deblend.cpp)
  target_compile_options(stage4g3_deblend PRIVATE -fno-access-control)
  target_include_directories(stage4g3_deblend PRIVATE include ${EIGEN3_INCLUDE_DIR})
  ament_target_dependencies(stage4g3_deblend rclcpp sensor_msgs nav_msgs geometry_msgs
    tf2 tf2_ros tf2_geometry_msgs visualization_msgs predictive_nav_msgs)
  add_test(NAME stage4g3_deblend COMMAND stage4g3_deblend)
  set_tests_properties(stage4g3_deblend PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=95" TIMEOUT 20)

  # Stage-4G4: bounded-motion reachability mathematics and age handling.
  add_executable(stage4g4_reachability test/stage4g4_reachability.cpp)
  target_compile_options(stage4g4_reachability PRIVATE -fno-access-control)
  target_include_directories(stage4g4_reachability PRIVATE include ${EIGEN3_INCLUDE_DIR})
  ament_target_dependencies(stage4g4_reachability rclcpp sensor_msgs nav_msgs geometry_msgs
    tf2 tf2_ros tf2_geometry_msgs visualization_msgs predictive_nav_msgs)
  add_test(NAME stage4g4_reachability COMMAND stage4g4_reachability)
  set_tests_properties(stage4g4_reachability PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=96" TIMEOUT 30)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
