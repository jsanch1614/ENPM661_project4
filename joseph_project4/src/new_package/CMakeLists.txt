cmake_minimum_required(VERSION 3.8)
project(new_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(pick_move src/pick_move.cpp)

ament_target_dependencies(pick_move
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
)

target_compile_features(pick_move PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  pick_move
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
