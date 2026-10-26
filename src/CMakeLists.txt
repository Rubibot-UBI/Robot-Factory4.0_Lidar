cmake_minimum_required(VERSION 3.5)
project(my_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY description launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/lidar_listener.py DESTINATION lib/${PROJECT_NAME})

ament_package()
