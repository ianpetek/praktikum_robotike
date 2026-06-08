cmake_minimum_required(VERSION 3.8)
project(pr_calibration)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/generate_calibration_sheet.py
  scripts/calibrate_camera.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
