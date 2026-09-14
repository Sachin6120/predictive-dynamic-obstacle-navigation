cmake_minimum_required(VERSION 3.8)
project(predictive_nav_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  models
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/publish_initial_pose.py
  scripts/obstacle_mover.py
  scripts/evaluate_tracking.py
  scripts/evaluate_stage4c.py
  scripts/evaluate_stage4d.py
  scripts/stage4e_trial.py
  scripts/run_stage4e_ab.py
  scripts/summarize_stage4e.py
  scripts/make_stage4f_world.py
  scripts/stage4f_trial.py
  scripts/run_stage4f_bench.py
  scripts/summarize_stage4f.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
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
