cmake_minimum_required(VERSION 3.8)
project(pr_controller)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/pen_mount_broadcaster.py
  scripts/inverse_kinematics_control.py
  scripts/draw_text_demo.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
