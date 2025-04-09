cmake_minimum_required(VERSION 3.8)
project(json_mqtt_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(beeyonders_msgs REQUIRED)

ament_package()
