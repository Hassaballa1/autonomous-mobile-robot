cmake_minimum_required(VERSION 3.8)
project(amr_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch rviz config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/patrol.py DESTINATION lib/${PROJECT_NAME})

ament_package()
