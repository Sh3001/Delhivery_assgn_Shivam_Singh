cmake_minimum_required(VERSION 3.8)
project(amr_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/stop_stack.py
    scripts/send_goal.py
    scripts/fleet_ready.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
