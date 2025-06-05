cmake_minimum_required(VERSION 3.8)
project(motion-control-mecanum-pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(include)

add_library(motion_control_mecanum
  src/motion-control-mecanum/motion_controller.cpp
  src/motion-control-mecanum/motor_controller.cpp
)
ament_target_dependencies(motion_control_mecanum
  rclcpp
  geometry_msgs
)

add_executable(motion_controller_node
  src/motion-control-mecanum/motion_controller_node.cpp
  src/motion-control-mecanum/motion_controller_entrypoint.cpp
)
target_link_libraries(motion_controller_node motion_control_mecanum)
ament_target_dependencies(motion_controller_node
  rclcpp
  geometry_msgs
)

install(TARGETS motion_control_mecanum motion_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)



ament_package()
