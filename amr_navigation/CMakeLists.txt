cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

if(POLICY CMP0009)
  cmake_policy(SET CMP0009 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(ramp_cost_layer SHARED src/ramp_cost_layer.cpp)
target_include_directories(ramp_cost_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ramp_cost_layer
  nav2_costmap_2d
  nav2_core
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(
  TARGETS ramp_cost_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/send_concurrent_goals.py
    scripts/ramp_region_markers.py
    scripts/peer_scan_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(ramp_cost_layer)
ament_export_dependencies(nav2_costmap_2d nav2_core pluginlib rclcpp)

ament_package()
