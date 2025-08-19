cmake_minimum_required(VERSION 3.8)
project(testbed_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY
  launch
  config
  params
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()