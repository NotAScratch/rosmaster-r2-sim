cmake_minimum_required(VERSION 3.8)
project(rosmaster_r2_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds launch config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/phase1_check.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
