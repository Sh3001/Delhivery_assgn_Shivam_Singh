cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch config models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/random_walk_obstacle.py
    scripts/scan_frame_fixer.py
    scripts/sensor_bsp_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # flake8 is run explicitly so a config file can be supplied; see
  # flake8.ini for why generate_warehouse_world.py is exempt from E501.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/flake8.ini)
endif()

ament_package()
